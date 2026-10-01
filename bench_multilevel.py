#!/usr/bin/env python3
"""Benchmark MultiLevel (iterated Dijkstra) against RA*pex on random queries.

Generates N random source,target pairs, runs both solvers over the same query
file (one process each, so the graph is loaded once), then compares per-query
runtime and the set of returned cost vectors.

py -3 bench_multilevel.py -n 20 --seed 7 -j 4
"""
import argparse
import csv
import json
import os
import random
import re
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path


def graph_size(gr):
    with gr.open() as f:
        for line in f:
            if line.startswith('p'):
                return int(line.split()[2])
    raise ValueError(f'no problem line in {gr}')


def run(exe, maps, queries, rules, algo, cutoff, out_dir, dll_dir, extra=()):
    out_dir.mkdir(parents=True, exist_ok=True)
    stats = out_dir / f'{algo}_stats.txt'
    cmd = [str(exe), *extra, '-m', *map(str, maps), '-q', str(queries), '-r', str(rules),
           '-a', algo, '-t', str(cutoff), '-o', str(stats)]
    print(f'> {algo} on {queries.name}', flush=True)
    start = time.perf_counter()
    env = dict(os.environ)
    if dll_dir:  # make sure the exe picks up the runtime DLLs it was built with
        env['PATH'] = str(dll_dir) + os.pathsep + env.get('PATH', '')
    proc = subprocess.run(cmd, capture_output=True, text=True, errors='replace', env=env)
    wall = time.perf_counter() - start
    (out_dir / f'{algo}_console.txt').write_text(proc.stdout + '\n' + proc.stderr, encoding='utf-8')
    if proc.returncode != 0:
        raise RuntimeError(f'{algo} exited with {proc.returncode}; see {algo}_console.txt')
    rows = [l.split('\t') for l in stats.read_text().splitlines() if l.strip()]
    return rows, proc.stdout, wall


def multilevel_results(rows):
    # MultiLevel  src  dst  #sol  seconds  #edges  edges_path  solutions_path  status
    res = []
    for r in rows:
        sols = Path(r[7])
        vecs = {tuple(map(int, l.split())) for l in sols.read_text().splitlines() if l.strip()} if sols.exists() else set()
        res.append(dict(seconds=float(r[4]), status=r[8].strip(), vectors=vecs, edges=int(r[5])))
    return res


def rapex_results(rows, console, targets, cutoff):
    # RApex-variant  src  dst  num_gen  num_exp  #sol  seconds
    blocks = re.split(r'^Started Query: \d+/\d+\s*$', console, flags=re.M)[1:]
    res = []
    for r, block, target in zip(rows, blocks, targets):
        vecs = set()
        for line in block.splitlines():
            i = line.find('{')
            if i < 0:
                continue
            try:
                obj = json.loads(line[i:])
            except ValueError:
                continue
            if obj.get('id') == target and isinstance(obj.get('full_cost'), list):
                vecs.add(tuple(obj['full_cost']))
        secs = float(r[6])
        res.append(dict(seconds=secs, status='TIMEOUT' if secs >= cutoff else 'COMPLETE', vectors=vecs,
                        generated=int(r[3]), expanded=int(r[4])))
    return res


def main():
    d = Path('resources/dataset')
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('-n', '--num-queries', type=int, default=10)
    p.add_argument('--seed', type=int, default=None, help='random seed (default: random)')
    p.add_argument('--exe', type=Path, default=Path('build/multiobj.exe'))
    p.add_argument('--maps', nargs=3, type=Path, default=[d / f'USA-road-{x}.BAY.gr' for x in 'dt3'])
    p.add_argument('--rules', type=Path, default=d / 'fork_rules.txt')
    p.add_argument('--cutoff', type=int, default=300)
    p.add_argument('--out', type=Path, default=Path('bench'))
    p.add_argument('-j', '--jobs', type=int, default=1,
                   help='solver processes to run at once; queries are split into this many chunks per algorithm')
    p.add_argument('--dll-dir', type=Path, default=Path('C:/msys64/ucrt64/bin'),
                   help='toolchain bin dir prepended to PATH (empty string to disable)')
    a = p.parse_args()

    seed = a.seed if a.seed is not None else random.randrange(1 << 30)
    rng = random.Random(seed)
    n_vertices = graph_size(a.maps[0])
    pairs = []
    while len(pairs) < a.num_queries:
        s, t = rng.randint(1, n_vertices), rng.randint(1, n_vertices)
        if s != t:
            pairs.append((s, t))

    out = (a.out / f'seed{seed}_n{a.num_queries}').resolve()
    out.mkdir(parents=True, exist_ok=True)
    qfile = out / 'queries.txt'
    qfile.write_text(''.join(f'{s},{t}\n' for s, t in pairs))
    print(f'seed={seed}  queries={len(pairs)}  rules={a.rules}  -> {out}')

    exe, maps, rules = a.exe.resolve(), [m.resolve() for m in a.maps], a.rules.resolve()
    # Split queries into contiguous chunks; every (algorithm, chunk) is one solver process.
    k = max(1, min(a.jobs, len(pairs)))
    bounds = [(i * len(pairs) // k, (i + 1) * len(pairs) // k) for i in range(k)]
    jobs = []
    for c, (lo, hi) in enumerate(bounds):
        cdir = out / f'chunk{c}' if k > 1 else out
        cdir.mkdir(parents=True, exist_ok=True)
        cq = cdir / 'queries.txt'
        cq.write_text(''.join(f'{s},{t}\n' for s, t in pairs[lo:hi]))
        jobs.append(('MultiLevel', c, cq, cdir, ()))
        jobs.append(('RApex', c, cq, cdir, ('--merge', 'RANDOM')))
    if a.jobs > 1:
        print(f'running {len(jobs)} solver processes, {a.jobs} at a time '
              f'(timings are noisier when processes share the CPU)')
    bench_start = time.perf_counter()
    with ThreadPoolExecutor(max_workers=max(1, a.jobs)) as pool:
        futures = {(algo, c): pool.submit(run, exe, maps, cq, rules, algo, a.cutoff, cdir, a.dll_dir, list(extra))
                   for algo, c, cq, cdir, extra in jobs}
        done = {key: f.result() for key, f in futures.items()}
    bench_wall = time.perf_counter() - bench_start
    ml, ra = [], []
    for c, (lo, hi) in enumerate(bounds):
        ml += multilevel_results(done['MultiLevel', c][0])
        ra += rapex_results(done['RApex', c][0], done['RApex', c][1], [t for _, t in pairs[lo:hi]], a.cutoff)
    ml_wall = sum(w for (algo, _), (_, _, w) in done.items() if algo == 'MultiLevel')
    ra_wall = sum(w for (algo, _), (_, _, w) in done.items() if algo == 'RApex')
    if len(ml) != len(pairs) or len(ra) != len(pairs):
        raise RuntimeError(f'expected {len(pairs)} results, got MultiLevel={len(ml)} RApex={len(ra)}')

    fields = ['query', 'source', 'target', 'ml_status', 'ml_seconds', 'ml_vectors',
              'rapex_status', 'rapex_seconds', 'rapex_vectors', 'ml_kept_edges', 'rapex_generated',
              'rapex_expanded', 'match', 'missing_in_rapex', 'extra_in_rapex']
    mism = 0
    with (out / 'summary.csv').open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for i, ((s, t), m, r) in enumerate(zip(pairs, ml, ra), 1):
            match = m['vectors'] == r['vectors']
            both_complete = m['status'] == 'COMPLETE' and r['status'] == 'COMPLETE'
            mism += both_complete and not match
            w.writerow(dict(query=i, source=s, target=t,
                            ml_status=m['status'], ml_seconds=f"{m['seconds']:.4f}", ml_vectors=len(m['vectors']),
                            rapex_status=r['status'], rapex_seconds=f"{r['seconds']:.4f}", rapex_vectors=len(r['vectors']),
                            ml_kept_edges=m['edges'], rapex_generated=r['generated'], rapex_expanded=r['expanded'],
                            match=match, missing_in_rapex=sorted(m['vectors'] - r['vectors']),
                            extra_in_rapex=sorted(r['vectors'] - m['vectors'])))
            print(f"q{i:<3} {s:>7}->{t:<7} ML {m['seconds']:8.3f}s {len(m['vectors']):3}v  "
                  f"RApex {r['seconds']:8.3f}s {len(r['vectors']):3}v  {'OK' if match else 'DIFF'}"
                  f"{'' if both_complete else ' (incomplete)'}")

    ml_t = sum(m['seconds'] for m in ml)
    ra_t = sum(r['seconds'] for r in ra)
    print(f'\nTotal solver time: MultiLevel {ml_t:.3f}s  RApex {ra_t:.3f}s  (speedup x{ra_t / ml_t if ml_t else float("inf"):.2f})')
    print(f'Process wall time: MultiLevel {ml_wall:.1f}s  RApex {ra_wall:.1f}s (summed over processes, includes graph loading)')
    print(f'Bench elapsed: {bench_wall:.1f}s with -j {a.jobs}')
    print(f'Set mismatches on completed queries: {mism}/{len(pairs)}')
    print(f'Details: {out / "summary.csv"}')


if __name__ == '__main__':
    main()
