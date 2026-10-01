#!/usr/bin/env python3
"""Plot a bench_multilevel.py summary.csv in the style of Fig. 2 (setting S2).

Left:  per-instance runtime (log scale) of MultiLevel and RA*pex, with the
       solution set size on a second axis.
Right: RA*pex nodes explored/expanded (x10^6) with runtime on a second axis.
Instances are sorted by RA*pex runtime so the trend reads left to right.

py -3 plot_bench.py bench/seed42_n12/summary.csv
"""
import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator

BLUE, ORANGE, GREY = '#1f3fd1', '#f28e1c', '#7f7f7f'


def load(path):
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        for k in ('ml_seconds', 'rapex_seconds'):
            r[k] = float(r[k])
        for k in ('ml_vectors', 'rapex_vectors', 'rapex_generated', 'rapex_expanded'):
            r[k] = int(r[k])
    return sorted(rows, key=lambda r: r['rapex_seconds'])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('summary', type=Path)
    p.add_argument('-o', '--out', type=Path, help='output image (default: next to the csv)')
    a = p.parse_args()
    rows = load(a.summary)
    x = list(range(1, len(rows) + 1))
    floor = 1e-4  # keep zero runtimes visible on a log axis

    plt.rcParams.update({'font.size': 13, 'axes.linewidth': 1.2, 'font.family': 'serif'})
    fig, (left, right) = plt.subplots(1, 2, figsize=(15, 5.8), constrained_layout=True)

    # Left: runtimes + solution set size
    ml_t = [max(r['ml_seconds'], floor) for r in rows]
    ra_t = [max(r['rapex_seconds'], floor) for r in rows]
    l1, = left.plot(x, ml_t, '-o', color=BLUE, lw=2.4, ms=8, mfc='white', mew=2, label='MultiLevel runtime')
    l2, = left.plot(x, ra_t, '--s', color=ORANGE, lw=2.4, ms=8, label='RA*pex runtime')
    left.set_yscale('log')
    left.set_xlabel('Instance (sorted by RA*pex runtime)')
    left.set_ylabel('Runtime (seconds)')
    left.grid(True, which='major', ls='-', alpha=0.35)
    left.grid(True, which='minor', ls=':', alpha=0.2)
    left.xaxis.set_major_locator(MaxNLocator(integer=True))
    size_ax = left.twinx()
    l3, = size_ax.plot(x, [r['ml_vectors'] for r in rows], '-^', color=GREY, lw=2, ms=9, label='Solution set size')
    mism = [i for i, r in zip(x, rows) if r['match'] != 'True']
    if mism:
        size_ax.plot(mism, [rows[i - 1]['rapex_vectors'] for i in mism], 'rx', ms=14, mew=3, label='Set mismatch')
    size_ax.set_ylabel('Solution set size')
    size_ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    size_ax.set_ylim(0, max(r['ml_vectors'] for r in rows) + 1)
    left.legend(handles=[l1, l2, l3], loc='upper left', framealpha=0.95)
    left.set_title('Runtime and solution set size (r1 > r2, r1 > r3)', fontsize=13)

    # Right: RA*pex search effort + both runtimes
    gen = [r['rapex_generated'] / 1e6 for r in rows]
    exp = [r['rapex_expanded'] / 1e6 for r in rows]
    r1, = right.plot(x, gen, '-o', color=BLUE, lw=2.4, ms=8, mfc='white', mew=2, label='Nodes explored (RA*pex)')
    r2, = right.plot(x, exp, '-^', color=GREY, lw=2.4, ms=8, label='Nodes expanded (RA*pex)')
    right.set_xlabel('Instance (sorted by RA*pex runtime)')
    right.set_ylabel(r'Number of nodes ($\times 10^6$)')
    right.grid(True, alpha=0.35)
    right.xaxis.set_major_locator(MaxNLocator(integer=True))
    rt = right.twinx()
    r3, = rt.plot(x, ra_t, '--s', color=ORANGE, lw=2.4, ms=8, label='RA*pex runtime')
    r4, = rt.plot(x, ml_t, ':D', color='#2ca02c', lw=2.4, ms=7, label='MultiLevel runtime')
    rt.set_yscale('log')
    rt.set_ylabel('Runtime (seconds)')
    right.legend(handles=[r1, r2, r3, r4], loc='upper left', framealpha=0.95)
    right.set_title('RA*pex search effort per instance', fontsize=13)

    speedup = sum(r['rapex_seconds'] for r in rows) / max(sum(r['ml_seconds'] for r in rows), 1e-9)
    fig.suptitle(f'{len(rows)} BAY queries  |  identical solution sets: '
                 f'{sum(r["match"] == "True" for r in rows)}/{len(rows)}  |  total speedup x{speedup:.2f}',
                 fontsize=14, fontweight='bold')
    out = a.out or a.summary.with_name('fig2_style.png')
    fig.savefig(out, dpi=200)
    fig.savefig(out.with_suffix('.pdf'))
    print(f'Wrote {out} and {out.with_suffix(".pdf")}')


if __name__ == '__main__':
    main()
