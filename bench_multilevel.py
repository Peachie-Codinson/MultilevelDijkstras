#!/usr/bin/env python3
"""Benchmark MultiLevel (iterated Dijkstra) against RA*pex on random queries.

Generates N random source,target pairs and solves each one with both
algorithms under both rulebooks: fork (r1 > {r2, r3}) and chain (r1 > r2 > r3).
Per-query runtimes are complete: RA*pex = heuristic setup + search, MultiLevel =
its whole solve. Each solver's one-time graph build is recorded separately.
Writes summary_fork.csv, summary_chain.csv and runtime.png to the output folder.

py -3 bench_multilevel.py --graph BAY -n 100 --seed 7 -j 6
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

DATA = Path('resources/dataset')
GRAPHS = {
    'BAY': [DATA / f'USA-road-{x}.BAY.gr' for x in 'dt3'],
    'grid-distinct': [Path(f'resources/random/grid500x500_distinct_s1/r{k}.gr') for k in (1, 2, 3)],
    'grid-violation': [Path(f'resources/random/grid500x500_violation_s1/r{k}.gr') for k in (1, 2, 3)],
}
RULEBOOKS = {'fork': DATA / 'fork_rules.txt', 'chain': DATA / 'chain_rules.txt'}


def graph_size(gr):
    with gr.open() as f:
        for line in f:
            if line.startswith('p'):
                return int(line.split()[2])
    raise ValueError(f'no problem line in {gr}')


def make_queries(n_vertices, n, seed):
    rng, pairs = random.Random(seed), []
    while len(pairs) < n:
        s, t = rng.randint(1, n_vertices), rng.randint(1, n_vertices)
        if s != t:
            pairs.append((s, t))
    return pairs


def run(exe, maps, queries, rules, algo, cutoff, out_dir, dll_dir, extra=()):
    out_dir.mkdir(parents=True, exist_ok=True)
    stats = out_dir / f'{algo}_stats.txt'
    cmd = [str(exe), *extra, '-m', *map(str, maps), '-q', str(queries), '-r', str(rules),
           '-a', algo, '-t', str(cutoff), '-o', str(stats)]
    start = time.perf_counter()
    env = dict(os.environ)
    if dll_dir:  # make sure the exe picks up the runtime DLLs it was built with
        env['PATH'] = str(dll_dir) + os.pathsep + env.get('PATH', '')
    proc = subprocess.run(cmd, capture_output=True, text=True, errors='replace', env=env)
    wall = time.perf_counter() - start
    (out_dir / f'{algo}_console.txt').write_text(proc.stdout + '\n' + proc.stderr, encoding='utf-8')
    if proc.returncode != 0:
        raise RuntimeError(f'{algo} exited with {proc.returncode}; see {out_dir / (algo + "_console.txt")}')
    rows = [l.split('\t') for l in stats.read_text().splitlines() if l.strip()]
    return rows, proc.stdout, wall


def multilevel_results(rows):
    # MultiLevel  src  dst  #sol  seconds  #edges  edges_path  solutions_path  status
    #             pass_edges "total>p1>p2.."  pass_vertices  labels "created/rejected"
    res = []
    for r in rows:
        sols = Path(r[7])
        vecs = {tuple(map(int, l.split())) for l in sols.read_text().splitlines() if l.strip()} if sols.exists() else set()
        extra = [x.strip() for x in r[9:12]] + [''] * (12 - max(len(r), 9))
        res.append(dict(seconds=float(r[4]), status=r[8].strip(), vectors=vecs, edges=int(r[5]),
                        pass_edges=extra[0], pass_vertices=extra[1], labels=extra[2]))
    return res


def rapex_results(rows, console, targets, cutoff):
    # RApex-variant  src  dst  num_gen  num_exp  #sol  search_seconds  heuristic_seconds
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
        search, heuristic = float(r[6]), float(r[7])
        res.append(dict(search=search, heuristic=heuristic, seconds=search + heuristic,
                        status='TIMEOUT' if search >= cutoff else 'COMPLETE', vectors=vecs,
                        generated=int(r[3]), expanded=int(r[4])))
    return res


def setup_seconds(console, label):
    m = re.search(rf'^{re.escape(label)} graph build: ([\d.eE+-]+)s', console, re.M)
    return float(m.group(1)) if m else float('nan')


FIELDS = ['query', 'source', 'target',
          'ml_status', 'ml_seconds', 'ml_vectors',
          'rapex_status', 'rapex_seconds', 'rapex_search_seconds', 'rapex_heuristic_seconds', 'rapex_vectors',
          'match', 'missing_in_rapex', 'extra_in_rapex',
          'rapex_generated', 'rapex_expanded', 'ml_pass_edges', 'ml_pass_vertices', 'ml_labels']


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--graph', choices=list(GRAPHS), default='BAY', help='graph preset (or give --maps)')
    p.add_argument('--maps', nargs=3, type=Path, help='three .gr files (r1 r2 r3); overrides --graph')
    p.add_argument('-n', '--num-queries', type=int, default=10)
    p.add_argument('--seed', type=int, default=None, help='random seed (default: random)')
    p.add_argument('--rulebooks', nargs='+', choices=list(RULEBOOKS), default=list(RULEBOOKS))
    p.add_argument('--cutoff', type=int, default=300, help='per-query search limit in seconds')
    p.add_argument('-j', '--jobs', type=int, default=1, help='solver processes to run at once')
    p.add_argument('--tag', help='output folder prefix (default: graph name)')
    p.add_argument('--out', type=Path, default=Path('bench'))
    p.add_argument('--exe', type=Path, default=Path('build/multiobj.exe'))
    p.add_argument('--dll-dir', type=Path, default=Path('C:/msys64/ucrt64/bin'),
                   help='toolchain bin dir prepended to PATH (empty string to disable)')
    p.add_argument('--keep-edges', action='store_true', help='keep the per-query optimal-subgraph .edges files')
    p.add_argument('--no-plot', action='store_true')
    a = p.parse_args()

    maps = [m.resolve() for m in (a.maps or GRAPHS[a.graph])]
    graph = 'custom' if a.maps else a.graph
    seed = a.seed if a.seed is not None else random.randrange(1 << 30)
    pairs = make_queries(graph_size(maps[0]), a.num_queries, seed)
    out = (a.out / f'{a.tag or graph}_seed{seed}_n{len(pairs)}').resolve()
    out.mkdir(parents=True, exist_ok=True)
    (out / 'queries.txt').write_text(''.join(f'{s},{t}\n' for s, t in pairs))
    print(f'graph={graph} seed={seed} queries={len(pairs)} rulebooks={a.rulebooks} -> {out}', flush=True)

    # One process per (rulebook, algorithm, chunk of queries).
    k = max(1, min(max(1, a.jobs // 2), len(pairs)))
    bounds = [(i * len(pairs) // k, (i + 1) * len(pairs) // k) for i in range(k)]
    jobs = []
    for rb in a.rulebooks:
        for c, (lo, hi) in enumerate(bounds):
            cdir = out / rb / f'chunk{c}'
            cdir.mkdir(parents=True, exist_ok=True)
            q = cdir / 'queries.txt'
            q.write_text(''.join(f'{s},{t}\n' for s, t in pairs[lo:hi]))
            jobs.append((rb, 'MultiLevel', c, q, cdir, ()))
            jobs.append((rb, 'RApex', c, q, cdir, ('--merge', 'RANDOM')))
    print(f'running {len(jobs)} solver processes, {a.jobs} at a time', flush=True)
    start = time.perf_counter()
    with ThreadPoolExecutor(max_workers=max(1, a.jobs)) as pool:
        futures = {(rb, algo, c): pool.submit(run, a.exe.resolve(), maps, q, RULEBOOKS[rb].resolve(), algo,
                                              a.cutoff, cdir, a.dll_dir, list(extra))
                   for rb, algo, c, q, cdir, extra in jobs}
        done = {key: f.result() for key, f in futures.items()}
    print(f'all solvers finished in {time.perf_counter() - start:.0f}s', flush=True)

    setup = {'MultiLevel': [], 'RA*pex': []}
    for (rb, algo, c), (_, console, _) in done.items():
        if algo == 'MultiLevel':
            setup['MultiLevel'].append(setup_seconds(console, 'MultiLevel'))
        else:
            setup['RA*pex'].append(setup_seconds(console, 'RA*pex'))
    setup = {k_: sum(v) / len(v) for k_, v in setup.items() if v}
    meta = dict(graph=graph, seed=seed, queries=len(pairs), cutoff=a.cutoff, jobs=a.jobs,
                graph_build_seconds=setup, maps=[str(m) for m in maps])
    (out / 'meta.json').write_text(json.dumps(meta, indent=2))

    for rb in a.rulebooks:
        ml, ra = [], []
        for c, (lo, hi) in enumerate(bounds):
            ml += multilevel_results(done[rb, 'MultiLevel', c][0])
            ra += rapex_results(done[rb, 'RApex', c][0], done[rb, 'RApex', c][1], [t for _, t in pairs[lo:hi]], a.cutoff)
        if len(ml) != len(pairs) or len(ra) != len(pairs):
            raise RuntimeError(f'{rb}: expected {len(pairs)} results, got MultiLevel={len(ml)} RApex={len(ra)}')
        mism = complete = 0
        with (out / f'summary_{rb}.csv').open('w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=FIELDS)
            w.writeheader()
            for i, ((s, t), m, r) in enumerate(zip(pairs, ml, ra), 1):
                both = m['status'] == r['status'] == 'COMPLETE'
                match = m['vectors'] == r['vectors']
                complete += both
                mism += both and not match
                w.writerow(dict(query=i, source=s, target=t,
                                ml_status=m['status'], ml_seconds=f"{m['seconds']:.4f}", ml_vectors=len(m['vectors']),
                                rapex_status=r['status'], rapex_seconds=f"{r['seconds']:.4f}",
                                rapex_search_seconds=f"{r['search']:.4f}", rapex_heuristic_seconds=f"{r['heuristic']:.4f}",
                                rapex_vectors=len(r['vectors']), match=match,
                                missing_in_rapex=sorted(m['vectors'] - r['vectors']),
                                extra_in_rapex=sorted(r['vectors'] - m['vectors']),
                                rapex_generated=r['generated'], rapex_expanded=r['expanded'],
                                ml_pass_edges=m['pass_edges'], ml_pass_vertices=m['pass_vertices'], ml_labels=m['labels']))
        ml_t, ra_t = sum(m['seconds'] for m in ml), sum(r['seconds'] for r in ra)
        print(f"[{rb}] MultiLevel {ml_t:.2f}s  RA*pex {ra_t:.2f}s (incl. {sum(r['heuristic'] for r in ra):.2f}s heuristic)  "
              f"speedup x{ra_t / ml_t if ml_t else float('inf'):.2f}  RA*pex faster on "
              f"{sum(r['seconds'] < m['seconds'] for m, r in zip(ml, ra))}/{len(pairs)}  "
              f"completed: MultiLevel {sum(m['status'] == 'COMPLETE' for m in ml)}, RA*pex "
              f"{sum(r['status'] == 'COMPLETE' for r in ra)}, both {complete}  mismatches {mism}/{complete}", flush=True)
    print('graph build, once per graph: ' + ', '.join(f'{k_} {v:.2f}s' for k_, v in setup.items()))

    if not a.keep_edges:
        for e in out.rglob('*.edges'):
            e.unlink()
    if not a.no_plot:
        from plot_bench import plot
        plot(out)


if __name__ == '__main__':
    main()
