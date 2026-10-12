#!/usr/bin/env python3
"""Run RA*pex with an approximation factor epsilon on the queries of an earlier
bench_multilevel.py run, and check its answers against that run's exact
MultiLevel solutions.

The check is the paper's guarantee (Definition 6): every exact solution must be
epsilon-rule-dominated (Definition 5) by at least one RA*pex solution. Only
queries MultiLevel finished exactly are checked.

py -3 bench_epsilon.py bench/grid-violation-t600_seed7_n24 --epsilon 0.01 --rulebook fork -j 8
"""
import argparse
import csv
import json
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

from bench_multilevel import RULEBOOKS, multilevel_results, rapex_results, run

# Rule hierarchy per rulebook, as (topological order of rules, successors of each rule).
HIERARCHY = {'fork': ([0, 1, 2], {0: {1, 2}, 1: set(), 2: set()}),
             'chain': ([0, 1, 2], {0: {1, 2}, 1: {2}, 2: set()})}


def eps_dominates(x, y, eps, rulebook):
    """Definition 5: any rule where x exceeds (1+eps) y must be compensated by a
    higher rule where x is below (1+eps) y."""
    order, succ = HIERARCHY[rulebook]
    overridden = set()
    for r in order:
        if r in overridden:
            continue
        if x[r] > (1 + eps) * y[r]:
            return False
        if x[r] < (1 + eps) * y[r]:
            overridden |= succ[r]
    return True


def write_rules(src, dst, eps):
    lines = src.read_text().split('\n')
    n = int(lines[0])
    lines[1] = ' '.join([str(eps)] * n)
    dst.write_text('\n'.join(lines))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('reference', type=Path, help='bench_multilevel.py output folder with exact MultiLevel results')
    p.add_argument('--epsilon', type=float, default=0.01)
    p.add_argument('--rulebook', choices=list(HIERARCHY), default='fork')
    p.add_argument('--cutoff', type=int, default=None, help='default: the reference run\'s cutoff')
    p.add_argument('-j', '--jobs', type=int, default=8)
    p.add_argument('--exe', type=Path, default=Path('build/multiobj.exe'))
    p.add_argument('--dll-dir', type=Path, default=Path('C:/msys64/ucrt64/bin'))
    a = p.parse_args()

    ref = a.reference.resolve()
    meta = json.loads((ref / 'meta.json').read_text())
    cutoff = a.cutoff or meta['cutoff']
    maps = [Path(m) for m in meta['maps']]
    pairs = [tuple(map(int, l.split(','))) for l in (ref / 'queries.txt').read_text().split()]
    out = ref / f'eps{a.epsilon:g}_{a.rulebook}'
    out.mkdir(exist_ok=True)
    rules = out / 'rules.txt'
    write_rules(RULEBOOKS[a.rulebook].resolve(), rules, a.epsilon)

    # Exact reference: the MultiLevel results of the reference run, in query order.
    exact = []
    chunks = sorted((ref / a.rulebook).glob('chunk*'), key=lambda d: int(d.name[5:]))
    for d in chunks:
        exact += multilevel_results([l.split('\t') for l in (d / 'MultiLevel_stats.txt').read_text().splitlines() if l.strip()])
    with (ref / f'summary_{a.rulebook}.csv').open(newline='') as f:
        ref_rows = list(csv.DictReader(f))

    k = max(1, min(a.jobs, len(pairs)))
    bounds = [(i * len(pairs) // k, (i + 1) * len(pairs) // k) for i in range(k)]
    jobs = []
    for c, (lo, hi) in enumerate(bounds):
        d = out / f'chunk{c}'
        d.mkdir(exist_ok=True)
        q = d / 'queries.txt'
        q.write_text(''.join(f'{s},{t}\n' for s, t in pairs[lo:hi]))
        jobs.append((c, q, d))
    print(f'RA*pex eps={a.epsilon} {a.rulebook} on {len(pairs)} queries from {ref.name}, cutoff {cutoff}s', flush=True)
    start = time.perf_counter()
    with ThreadPoolExecutor(max_workers=a.jobs) as pool:
        futs = {c: pool.submit(run, a.exe.resolve(), maps, q, rules, 'RApex', cutoff, d, a.dll_dir, ['--merge', 'RANDOM'])
                for c, q, d in jobs}
        done = {c: f.result() for c, f in futs.items()}
    print(f'finished in {time.perf_counter() - start:.0f}s', flush=True)
    approx = []
    for c, (lo, hi) in enumerate(bounds):
        approx += rapex_results(done[c][0], done[c][1], [t for _, t in pairs[lo:hi]], cutoff)

    rows = []
    for i, ((s, t), e, r, rr) in enumerate(zip(pairs, exact, approx, ref_rows), 1):
        checked = e['status'] == 'COMPLETE' and r['status'] == 'COMPLETE'
        uncovered = [y for y in e['vectors'] if not any(eps_dominates(x, y, a.epsilon, a.rulebook) for x in r['vectors'])]
        rows.append(dict(query=i, source=s, target=t,
                         exact_status=e['status'], exact_seconds=e['seconds'], exact_size=len(e['vectors']),
                         rapex_exact_status=rr['rapex_status'], rapex_exact_seconds=float(rr['rapex_seconds']),
                         eps_status=r['status'], eps_seconds=r['seconds'], eps_search_seconds=r['search'],
                         eps_size=len(r['vectors']), covered=(not uncovered) if checked else '',
                         uncovered=len(uncovered) if checked else ''))
    with (out / 'summary.csv').open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    checked = [r for r in rows if r['covered'] != '']
    print(f"RA*pex eps={a.epsilon}: completed {sum(r['eps_status'] == 'COMPLETE' for r in rows)}/{len(rows)} "
          f"(exact RA*pex {sum(r['rapex_exact_status'] == 'COMPLETE' for r in rows)}, "
          f"MultiLevel {sum(r['exact_status'] == 'COMPLETE' for r in rows)})")
    print(f"coverage guarantee holds on {sum(r['covered'] is True for r in checked)}/{len(checked)} checkable queries")
    for r in rows:
        check = '' if r['covered'] == '' else 'covered' if r['covered'] else f"{r['uncovered']} UNCOVERED"
        print(f"q{r['query']:<3} exact {r['exact_status'][:4]} {r['exact_seconds']:7.1f}s {r['exact_size']:5}  |  "
              f"eps {r['eps_status'][:4]} {r['eps_seconds']:7.1f}s {r['eps_size']:4}  {check}")
    plot(rows, a.epsilon, out / 'epsilon.png', meta, cutoff)


def plot(rows, eps, path, meta, cutoff):
    plt.rcParams.update({'font.size': 12, 'font.family': 'serif'})
    fig, (left, right) = plt.subplots(1, 2, figsize=(16, 6), constrained_layout=True)
    rows = sorted(rows, key=lambda r: r['exact_size'])
    x = list(range(1, len(rows) + 1))
    series = [('exact_seconds', 'exact_status', '#1f3fd1', 'o', 'MultiLevel (exact)'),
              ('rapex_exact_seconds', 'rapex_exact_status', '#f28e1c', 's', 'RA*pex, eps = 0 (exact)'),
              ('eps_seconds', 'eps_status', '#2ca02c', 'D', f'RA*pex, eps = {eps:g}')]
    for key, status, color, m, label in series:
        ys = [max(r[key], 1e-3) for r in rows]
        left.plot(x, ys, '-' + m, color=color, lw=1.8, ms=6, label=label)
        tx = [i for i, r in zip(x, rows) if r[status] != 'COMPLETE']
        left.plot(tx, [ys[i - 1] for i in tx], 'x', color='#c0392b', ms=11, mew=2.5)
    left.set_yscale('log')
    left.set_ylabel('Runtime per query (seconds)')
    left.set_xlabel('Query (sorted by exact solution set size)')
    left.grid(True, which='both', alpha=0.3)
    left.legend(handles=left.get_lines()[0::2] + [Line2D([], [], color='#c0392b', marker='x', ls='', mew=2.5,
                label=f'timed out ({cutoff}s)')], loc='upper left', fontsize=10)
    left.set_title('Runtime (RA*pex includes its heuristic setup)')

    right.plot(x, [r['exact_size'] for r in rows], '-o', color='#1f3fd1', lw=1.8, ms=6, label='exact (MultiLevel)')
    right.plot(x, [r['eps_size'] for r in rows], '-D', color='#2ca02c', lw=1.8, ms=6, label=f'RA*pex, eps = {eps:g}')
    bad = [i for i, r in zip(x, rows) if r['covered'] is False]
    if bad:
        right.plot(bad, [rows[i - 1]['eps_size'] for i in bad], 'x', color='#c0392b', ms=12, mew=3, label='coverage fails')
    right.set_yscale('log')
    right.set_ylabel('Solution set size')
    right.set_xlabel('Query (sorted by exact solution set size)')
    right.grid(True, which='both', alpha=0.3)
    right.legend(loc='upper left', fontsize=10)
    checked = [r for r in rows if r['covered'] != '']
    right.set_title(f"Solution set size  |  coverage holds on {sum(r['covered'] is True for r in checked)}/{len(checked)} checked queries")
    fig.suptitle(f"{meta['graph']}, r1 > {{r2, r3}}, {len(rows)} queries (seed {meta['seed']}): exact vs eps = {eps:g}",
                 fontsize=14, fontweight='bold')
    fig.savefig(path, dpi=180)
    fig.savefig(path.with_suffix('.pdf'))
    print(f'Wrote {path}')


if __name__ == '__main__':
    main()
