#!/usr/bin/env python3
"""Compare how much of the graph MultiLevel's first pass (r1) prunes under the
chain rulebook (r1 > r2 > r3) and the fork rulebook (r1 > {r2, r3}), query by
query, on BAY and the two random grids. Only MultiLevel is run.

The fork case can time out in its (r2, r3) Pareto search; that does not matter
here because pass 1 finishes (and is recorded) before that search starts.

py -3 pruning_compare.py -n 100 --seed 42 -j 8
"""
import argparse
import csv
import random
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import matplotlib.pyplot as plt

from bench_multilevel import graph_size, multilevel_results, run

DATA = Path('resources/dataset')
GRAPHS = {
    'BAY': [DATA / f'USA-road-{x}.BAY.gr' for x in 'dt3'],
    'grid distinct': [Path(f'resources/random/grid500x500_distinct_s1/r{k}.gr') for k in (1, 2, 3)],
    'grid violation': [Path(f'resources/random/grid500x500_violation_s1/r{k}.gr') for k in (1, 2, 3)],
}
RULEBOOKS = {'chain': DATA / 'chain_rules.txt', 'fork': DATA / 'fork_rules.txt'}


def queries(n_vertices, n, seed):
    # Same generator as bench_multilevel.py, so the same seed gives the same queries.
    rng, pairs = random.Random(seed), []
    while len(pairs) < n:
        s, t = rng.randint(1, n_vertices), rng.randint(1, n_vertices)
        if s != t:
            pairs.append((s, t))
    return pairs


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('-n', '--num-queries', type=int, default=100)
    p.add_argument('--seed', type=int, default=42)
    p.add_argument('-j', '--jobs', type=int, default=8)
    p.add_argument('--cutoff', type=int, default=10, help='per-query limit; pass 1 is recorded even on timeout')
    p.add_argument('--exe', type=Path, default=Path('build/multiobj.exe'))
    p.add_argument('--dll-dir', type=Path, default=Path('C:/msys64/ucrt64/bin'))
    p.add_argument('--out', type=Path, default=Path('bench'))
    p.add_argument('--plot-only', action='store_true', help='skip solving; replot from the saved pruning.csv')
    p.add_argument('--graphs', nargs='+', choices=list(GRAPHS), help='only plot these graphs')
    a = p.parse_args()

    out = (a.out / f'pruning_seed{a.seed}_n{a.num_queries}').resolve()
    if a.plot_only:
        with (out / 'pruning.csv').open(newline='') as f:
            table = [{k: (v if k in ('graph', 'fork_status') else int(v)) for k, v in r.items()} for r in csv.DictReader(f)]
        replot(table, out, a.graphs)
        return
    jobs, pairs_by_graph = [], {}
    for g, maps in GRAPHS.items():
        pairs = queries(graph_size(maps[0]), a.num_queries, a.seed)
        pairs_by_graph[g] = pairs
        chunks = max(1, a.jobs // 2)
        for rb, rules in RULEBOOKS.items():
            for c in range(chunks):
                lo, hi = c * len(pairs) // chunks, (c + 1) * len(pairs) // chunks
                d = out / g.replace(' ', '_') / rb / f'chunk{c}'
                d.mkdir(parents=True, exist_ok=True)
                q = d / 'queries.txt'
                q.write_text(''.join(f'{s},{t}\n' for s, t in pairs[lo:hi]))
                jobs.append((g, rb, c, q, d, rules.resolve(), [m.resolve() for m in maps]))

    start = time.perf_counter()
    with ThreadPoolExecutor(max_workers=a.jobs) as pool:
        futs = {(g, rb, c): pool.submit(run, a.exe.resolve(), maps, q, rules, 'MultiLevel', a.cutoff, d, a.dll_dir)
                for g, rb, c, q, d, rules, maps in jobs}
        rows = {k: multilevel_results(f.result()[0]) for k, f in futs.items()}
    print(f'ran {len(jobs)} processes in {time.perf_counter() - start:.0f}s')

    # Collect per-query pass sizes in query order.
    table = []
    for g, pairs in pairs_by_graph.items():
        per_rb = {rb: [] for rb in RULEBOOKS}
        for rb in RULEBOOKS:
            for c in range(max(1, a.jobs // 2)):
                per_rb[rb] += rows[g, rb, c]
        for i, (s, t) in enumerate(pairs):
            ch = list(map(int, per_rb['chain'][i]['pass_edges'].split('>')))
            fk = list(map(int, per_rb['fork'][i]['pass_edges'].split('>')))
            table.append(dict(graph=g, query=i + 1, source=s, target=t, total_edges=ch[0],
                              chain_pass1=ch[1], fork_pass1=fk[1], chain_final=ch[-1],
                              fork_status=per_rb['fork'][i]['status']))
    with (out / 'pruning.csv').open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(table[0]))
        w.writeheader()
        w.writerows(table)
    replot(table, out, a.graphs)


def replot(table, out, graphs):
    if graphs:
        table = [r for r in table if r['graph'] in graphs]
    name = 'pruning_pass1' + ('_' + '_'.join(g.replace(' ', '_') for g in graphs) if graphs else '')
    plot(table, out / f'{name}.png')


def plot(table, path):
    plt.rcParams.update({'font.size': 12, 'font.family': 'serif'})
    graphs = list(dict.fromkeys(r['graph'] for r in table))
    fig, axes = plt.subplots(len(graphs), 1, figsize=(15, 4.2 * len(graphs)), constrained_layout=True, squeeze=False)
    axes = axes[:, 0]
    for ax, g in zip(axes, graphs):
        rs = [r for r in table if r['graph'] == g]
        x = [r['query'] for r in rs]
        pct = lambda k: [100 * r[k] / r['total_edges'] for r in rs]
        chain1, fork1, final = pct('chain_pass1'), pct('fork_pass1'), pct('chain_final')
        ax.plot(x, chain1, 'o', ms=9, mfc='none', mec='#1f3fd1', mew=1.8, label='r1 > r2 > r3: after pass 1')
        ax.plot(x, fork1, 'x', ms=7, color='#f28e1c', mew=2, label='r1 > {r2, r3}: after pass 1')
        ax.plot(x, final, '.', ms=6, color='#2ca02c', label='r1 > r2 > r3: after pass 3 (final)')
        ax.set_yscale('log')
        ax.set_ylabel('% of graph edges left')
        ax.grid(True, which='both', alpha=0.3)
        ax.set_xlim(0, len(rs) + 1)
        same = sum(r['chain_pass1'] == r['fork_pass1'] for r in rs)
        mean = lambda v: sum(v) / len(v)
        ax.axhline(mean(chain1), color='#1f3fd1', ls='--', lw=1.2, alpha=0.7,
                   label=f'average left after pass 1: {mean(chain1):.3f}%')
        ax.set_title(f'{g} ({rs[0]["total_edges"]:,} edges): pass 1 prunes on average {100 - mean(chain1):.3f}% '
                     f'(r1 > r2 > r3) and {100 - mean(fork1):.3f}% (r1 > {{r2, r3}}); '
                     f'same edge count on {same}/{len(rs)} queries', fontsize=12, fontweight='bold')
        ax.legend(loc='best', framealpha=0.95, fontsize=10)
    axes[-1].set_xlabel('Query (same seed, so query i is the same start/goal pair in both rulebooks)')
    fig.savefig(path, dpi=180)
    fig.savefig(path.with_suffix('.pdf'))
    print(f'Wrote {path}')


if __name__ == '__main__':
    main()
