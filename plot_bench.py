#!/usr/bin/env python3
"""Plot a bench_multilevel.py run: r1 > {r2, r3} on the left, r1 > r2 > r3 on the right.

Each panel shows, per query, the complete runtime of MultiLevel and of RA*pex
(heuristic setup + search) on a log axis, and the solution set size on a second
axis. Queries are sorted by RA*pex runtime; timed-out queries are marked with x.

py -3 plot_bench.py bench/BAY_seed7_n100
"""
import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.ticker import MaxNLocator

BLUE, ORANGE, GREY, RED = '#1f3fd1', '#f28e1c', '#7f7f7f', '#c0392b'
PANELS = [('fork', 'r1 > {r2, r3}'), ('chain', 'r1 > r2 > r3')]


def load(path):
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        for k in ('ml_seconds', 'rapex_seconds'):
            r[k] = float(r[k])
        for k in ('ml_vectors', 'rapex_vectors'):
            r[k] = int(r[k])
    return sorted(rows, key=lambda r: r['rapex_seconds'])


def panel(ax, rows, label):
    floor = 1e-3  # keep tiny runtimes visible on the log axis
    x = list(range(1, len(rows) + 1))
    ml = [max(r['ml_seconds'], floor) for r in rows]
    ra = [max(r['rapex_seconds'], floor) for r in rows]
    ax.plot(x, ml, '-o', color=BLUE, lw=2, ms=6, mfc='white', mew=1.8)
    ax.plot(x, ra, '--s', color=ORANGE, lw=2, ms=5)
    for key, ys in (('ml_status', ml), ('rapex_status', ra)):
        tx = [i for i, r in zip(x, rows) if r[key] != 'COMPLETE']
        if tx:
            ax.plot(tx, [ys[i - 1] for i in tx], 'x', color=RED, ms=11, mew=2.5)
    ax.set_yscale('log')
    ax.set_xlabel('Query (sorted by RA*pex runtime)')
    ax.set_ylabel('Runtime per query (seconds)')
    ax.grid(True, which='major', alpha=0.35)
    ax.grid(True, which='minor', ls=':', alpha=0.2)
    ax.xaxis.set_major_locator(MaxNLocator(integer=True))

    size = ax.twinx()
    size.plot(x, [r['ml_vectors'] for r in rows], '-^', color=GREY, lw=1.5, ms=6, alpha=0.85)
    size.set_ylabel('Solution set size')
    size.yaxis.set_major_locator(MaxNLocator(integer=True))
    size.set_ylim(0, max(r['ml_vectors'] for r in rows) * 1.15 + 1)

    handles = [Line2D([], [], color=BLUE, marker='o', mfc='white', lw=2, label='MultiLevel runtime'),
               Line2D([], [], color=ORANGE, marker='s', ls='--', lw=2, label='RA*pex runtime (heuristic + search)'),
               Line2D([], [], color=GREY, marker='^', lw=1.5, label='Solution set size')]
    if any(r['ml_status'] != 'COMPLETE' or r['rapex_status'] != 'COMPLETE' for r in rows):
        handles.append(Line2D([], [], color=RED, marker='x', ls='', mew=2.5, ms=10, label='timed out'))
    ax.legend(handles=handles, loc='upper left', framealpha=0.95, fontsize=10)

    both = [r for r in rows if r['ml_status'] == r['rapex_status'] == 'COMPLETE']
    same = sum(r['match'] == 'True' for r in both)
    speedup = sum(r['rapex_seconds'] for r in rows) / max(sum(r['ml_seconds'] for r in rows), 1e-9)
    faster = sum(r['rapex_seconds'] < r['ml_seconds'] for r in rows)
    ax.set_title(f'{label}\nidentical solution sets on {same}/{len(both)} queries both completed\n'
                 f'MultiLevel x{speedup:.1f} faster in total  |  RA*pex faster on {faster}/{len(rows)} queries',
                 fontsize=12)


def plot(folder, out=None):
    folder = Path(folder)
    meta = json.loads((folder / 'meta.json').read_text()) if (folder / 'meta.json').exists() else {}
    panels = [(rb, label) for rb, label in PANELS if (folder / f'summary_{rb}.csv').exists()]
    if not panels:
        raise SystemExit(f'no summary_fork.csv or summary_chain.csv in {folder}')
    plt.rcParams.update({'font.size': 12, 'axes.linewidth': 1.2, 'font.family': 'serif'})
    fig, axes = plt.subplots(1, len(panels), figsize=(8 * len(panels), 6), constrained_layout=True, squeeze=False)
    n = 0
    for ax, (rb, label) in zip(axes[0], panels):
        rows = load(folder / f'summary_{rb}.csv')
        n = len(rows)
        panel(ax, rows, label)
    build = meta.get('graph_build_seconds', {})
    build_txt = ', '.join(f'{k} {v:.2f}s' for k, v in build.items())
    fig.suptitle(f"{meta.get('graph', folder.name)}: {n} queries, seed {meta.get('seed', '?')}"
                 + (f"  |  one-time graph build (not per query): {build_txt}" if build_txt else ''),
                 fontsize=14, fontweight='bold')
    out = Path(out) if out else folder / 'runtime.png'
    fig.savefig(out, dpi=180)
    fig.savefig(out.with_suffix('.pdf'))
    plt.close(fig)
    print(f'Wrote {out}')


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('folder', type=Path, help='a bench_multilevel.py output folder')
    p.add_argument('-o', '--out', type=Path, help='output image (default: <folder>/runtime.png)')
    a = p.parse_args()
    plot(a.folder, a.out)


if __name__ == '__main__':
    main()
