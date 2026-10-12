#!/usr/bin/env python3
"""Generate a random 3-objective grid graph as three DIMACS .gr files (r1, r2, r3).

Vertices sit on a W x H grid and every pair of neighbours gets two arcs (one per
direction, same costs), like a road map. Cost profiles:
  distinct   r1, r2, r3 uniform in 1..1000 -> ties on r1 are rare
  violation  r1 = 1 with probability --p-violation, else 0 (a rarely broken rule);
             r2, r3 uniform in 1..100      -> many routes tie on r1

py -3 gen_random_graph.py --width 500 --height 500 --profile violation --seed 1
"""
import argparse
import random
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--width', type=int, default=500)
    p.add_argument('--height', type=int, default=500)
    p.add_argument('--profile', choices=['distinct', 'violation'], default='distinct')
    p.add_argument('--p-violation', type=float, default=0.1)
    p.add_argument('--seed', type=int, default=1)
    p.add_argument('--out', type=Path, default=Path('resources/random'))
    a = p.parse_args()

    rng = random.Random(a.seed)
    W, H = a.width, a.height
    vid = lambda x, y: y * W + x + 1  # DIMACS ids are 1-based

    def costs():
        if a.profile == 'distinct':
            return rng.randint(1, 1000), rng.randint(1, 1000), rng.randint(1, 1000)
        return int(rng.random() < a.p_violation), rng.randint(1, 100), rng.randint(1, 100)

    arcs = []
    for y in range(H):
        for x in range(W):
            for nx, ny in ((x + 1, y), (x, y + 1)):
                if nx < W and ny < H:
                    c = costs()
                    arcs.append((vid(x, y), vid(nx, ny), c))
                    arcs.append((vid(nx, ny), vid(x, y), c))

    name = f'grid{W}x{H}_{a.profile}_s{a.seed}'
    folder = a.out / name
    folder.mkdir(parents=True, exist_ok=True)
    for k in range(3):
        with (folder / f'r{k + 1}.gr').open('w', newline='\n') as f:
            f.write(f'c random grid {W}x{H}, profile {a.profile}, seed {a.seed}, objective r{k + 1}\n')
            f.write(f'p sp {W * H} {len(arcs)}\n')
            f.writelines(f'a {s} {t} {c[k]}\n' for s, t, c in arcs)
    print(f'{folder}: {W * H} vertices, {len(arcs)} arcs')


if __name__ == '__main__':
    main()
