#!/usr/bin/env python3
"""Animate MultiLevel Dijkstra and RA*pex searching a grid graph, from the JSON
traces written by `multiobj --trace` (one GIF per algorithm).

MultiLevel: pass 1 forward Dijkstra on r1, backward A* that forms the optimal
r1 subgraph, BOA*'s heuristic sweeps (r2, r3) on that subgraph, then the BOA*
search for the (r2, r3) trade-offs, then the solution paths.
RA*pex: its heuristic (backward Dijkstra on r1, r2, r3 over the whole graph),
then its search, then the solution paths.

Whole-graph phases are drawn on the full grid; the searches are drawn zoomed in
on the area they touch. Heuristic sweep orders are recomputed here with the same
Dijkstra the solvers run (they are not in the traces).

py -3 animate_search.py bench/animation/trace.MultiLevel.113678-109622.json \
    bench/animation/trace.RApex.113678-109622.json --grid resources/random/grid500x500_violation_s1
"""
import argparse
import heapq
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

BG = (17, 19, 26)
CELL = (38, 42, 54)
FORWARD = (64, 104, 196)
BACKWARD = (38, 168, 160)
HEUR = [(64, 104, 196), (120, 90, 200), (38, 168, 160)]
EDGE_KEPT = (150, 158, 172)
EDGE_CUT = (150, 45, 45)
PATH = (255, 80, 200)
START, GOAL = (70, 225, 110), (255, 95, 95)
HEAT_LO, HEAT_HI = np.array([255, 224, 90], float), np.array([214, 40, 40], float)
TEXT, DIM = (235, 238, 245), (150, 156, 170)


def font(size, bold=False):
    for name in (('arialbd.ttf' if bold else 'arial.ttf'), 'DejaVuSans.ttf'):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default()


def load_graph(folder):
    """Arc list and per-rule costs from r1.gr, r2.gr, r3.gr."""
    src = dst = None
    costs = []
    for k in (1, 2, 3):
        s, d, c = [], [], []
        with open(Path(folder) / f'r{k}.gr') as f:
            for line in f:
                if line.startswith('a'):
                    _, a, b, w = line.split()
                    s.append(int(a)); d.append(int(b)); c.append(int(w))
        if src is None:
            src, dst = np.array(s), np.array(d)
        costs.append(np.array(c))
    return src, dst, costs


def dijkstra_order(n, src, dst, cost, start, reverse, mask=None):
    """Vertices in the order Dijkstra settles them (from start, over reversed arcs if reverse)."""
    a, b = (dst, src) if reverse else (src, dst)
    idx = np.arange(len(a)) if mask is None else np.nonzero(mask)[0]
    order_idx = idx[np.argsort(a[idx], kind='stable')]
    starts = np.searchsorted(a[order_idx], np.arange(n + 2))
    heads, ws = b[order_idx].tolist(), cost[order_idx].tolist()
    starts = starts.tolist()
    dist = {start: 0}
    done, out, heap = set(), [], [(0, start)]
    while heap:
        d, v = heapq.heappop(heap)
        if v in done:
            continue
        done.add(v)
        out.append(v)
        for k in range(starts[v], starts[v + 1]):
            u, nd = heads[k], d + ws[k]
            if nd < dist.get(u, math.inf):
                dist[u] = nd
                heapq.heappush(heap, (nd, u))
    return out


class Canvas:
    """Full-grid view (1 cell = 1 px, scaled) and a zoomed view of a crop with edges."""

    def __init__(self, width, height, crop, pitch, banner=86):
        self.W, self.H = width, height
        self.x0, self.x1, self.y0, self.y1 = crop
        self.pitch, self.banner = pitch, banner
        self.cw, self.ch = (self.x1 - self.x0 + 1), (self.y1 - self.y0 + 1)
        self.size = (max(self.cw * pitch, 560), self.ch * pitch + banner)
        self.f_title, self.f_text, self.f_small = font(24, True), font(16), font(14)

    def xy(self, v):
        return (v - 1) % self.W, (v - 1) // self.W

    def frame(self, title, lines, body):
        img = Image.new('RGB', self.size, BG)
        img.paste(body, ((self.size[0] - body.width) // 2, self.banner))
        d = ImageDraw.Draw(img)
        d.text((14, 8), title, fill=TEXT, font=self.f_title)
        for i, (t, c) in enumerate(lines):
            d.text((14, 40 + 21 * i), t, fill=c, font=self.f_text if i == 0 else self.f_small)
        return img

    # --- full view
    def full(self, layers, rect=True):
        side = self.size[1] - self.banner
        img = np.zeros((self.H, self.W, 3), np.uint8)
        img[:] = CELL
        for mask, color in layers:
            img[mask] = color
        pil = Image.fromarray(img).resize((side * self.W // max(self.W, self.H), side * self.H // max(self.W, self.H)),
                                          Image.NEAREST)
        if rect:
            s = pil.width / self.W
            ImageDraw.Draw(pil).rectangle([self.x0 * s, self.y0 * s, (self.x1 + 1) * s, (self.y1 + 1) * s],
                                          outline=(255, 255, 255), width=2)
        return pil

    # --- zoomed view
    def zoom(self, cell_rgb, edges=None, paths=(), marks=()):
        p = self.pitch
        img = Image.new('RGB', (self.cw * p, self.ch * p), BG)
        d = ImageDraw.Draw(img)
        if edges:
            for (cx, cy, dx, dy), color in edges.items():
                x, y = (cx - self.x0) * p, (cy - self.y0) * p
                if dx:
                    d.rectangle([x + p - 2, y + 2, x + p + 1, y + p - 3], fill=color)
                else:
                    d.rectangle([x + 2, y + p - 2, x + p - 3, y + p + 1], fill=color)
        for cy in range(self.ch):
            for cx in range(self.cw):
                x, y = cx * p, cy * p
                d.rectangle([x + 1, y + 1, x + p - 2, y + p - 2], fill=tuple(int(c) for c in cell_rgb[cy, cx]))
        for pts, color, width in paths:
            d.line([((px - self.x0) * p + p / 2, (py - self.y0) * p + p / 2) for px, py in pts], fill=color, width=width)
        for (px, py), color in marks:
            x, y = (px - self.x0) * p, (py - self.y0) * p
            d.rectangle([x - 2, y - 2, x + p + 1, y + p + 1], fill=color, outline=(255, 255, 255))
        return img

    def in_crop(self, x, y):
        return self.x0 <= x <= self.x1 and self.y0 <= y <= self.y1


def steps(n_events, n_frames):
    """Event counts shown in each of n_frames frames (ends with all events)."""
    n_frames = max(1, min(n_frames, n_events))
    return [math.ceil(n_events * (i + 1) / n_frames) for i in range(n_frames)]


def heat(counts):
    rgb = np.zeros(counts.shape + (3,), float)
    rgb[:] = CELL
    hit = counts > 0
    if hit.any():
        t = np.log1p(counts[hit]) / math.log1p(counts.max())
        rgb[hit] = HEAT_LO + (HEAT_HI - HEAT_LO) * t[:, None]
    return rgb


def sweep_frames(cv, title, phases, base_layers, n=18):
    """Full-view frames for a list of (label, vertex order, color) sweeps."""
    frames, layers = [], list(base_layers)
    for label, order, color, note in phases:
        mask = np.zeros((cv.H, cv.W), bool)
        arr = np.array(order) - 1
        for k in steps(len(order), n):
            mask.flat[arr[:k]] = True
            frames.append((cv.frame(title, [(label, TEXT), (f'{k:,} / {len(order):,} vertices settled  -  {note}', DIM)],
                                    cv.full(layers + [(mask.copy(), color)])), 90))
        layers.append((mask, color))
    return frames, layers


def search_frames(cv, title, label, events, target, n, edges=None, note=''):
    counts = np.zeros((cv.ch, cv.cw))
    xs = [cv.xy(v) for v in events]
    frames, found, prev = [], 0, 0
    for k in steps(len(events), n):
        for x, y in xs[prev:k]:
            if cv.in_crop(x, y):
                counts[y - cv.y0, x - cv.x0] += 1
        found += sum(1 for v in events[prev:k] if v == target)
        prev = k
        frames.append((cv.frame(title, [(label, TEXT), (f'{k:,} / {len(events):,} expansions   |   solutions found: '
                                                         f'{found}   {note}', DIM)],
                                cv.zoom(heat(counts), edges)), 80))
    return frames, counts


def final_frames(cv, title, counts, solutions, src, dst, edges, summary):
    paths = [([cv.xy(v) for v in s['path']], PATH, 2) for s in solutions]
    marks = [(cv.xy(src), START), (cv.xy(dst), GOAL)]
    img = cv.zoom(heat(counts) * 0.45 + np.array(CELL) * 0.55, edges, paths, marks)
    return [(cv.frame(title, summary, img), 4000)]


def save(frames, path):
    imgs = [f.convert('P', palette=Image.ADAPTIVE, colors=96) for f, _ in frames]
    imgs[0].save(path, save_all=True, append_images=imgs[1:], duration=[d for _, d in frames], loop=0, disposal=1)
    print(f'Wrote {path} ({len(imgs)} frames, {Path(path).stat().st_size / 1e6:.1f} MB)')


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('mld', type=Path)
    p.add_argument('rapex', type=Path)
    p.add_argument('--grid', type=Path, required=True, help='folder with r1.gr r2.gr r3.gr of a W x H grid')
    p.add_argument('--width', type=int, default=500)
    p.add_argument('--height', type=int, default=500)
    p.add_argument('--margin', type=int, default=10)
    p.add_argument('--pitch', type=int, default=7, help='pixels per cell in the zoomed view')
    p.add_argument('--ml-seconds', type=float, help='measured MultiLevel runtime for the summary')
    p.add_argument('--rapex-seconds', nargs=2, type=float, metavar=('HEURISTIC', 'SEARCH'))
    p.add_argument('--out', type=Path, default=None)
    a = p.parse_args()

    m, r = json.loads(a.mld.read_text()), json.loads(a.rapex.read_text())
    src, dst = m['source'], m['target']
    W, H = a.width, a.height
    xy = lambda v: ((v - 1) % W, (v - 1) // W)
    touched = set(m['pareto']) | set(r['expanded']) | {v for s in m['solutions'] for v in s['path']}
    xs, ys = zip(*(xy(v) for v in touched))
    crop = (max(0, min(xs) - a.margin), min(W - 1, max(xs) + a.margin),
            max(0, min(ys) - a.margin), min(H - 1, max(ys) + a.margin))
    cv = Canvas(W, H, crop, a.pitch)
    out = a.out or a.mld.parent
    n = W * H

    print('loading graph and recomputing heuristic sweeps...', flush=True)
    gs, gd, costs = load_graph(a.grid)
    kept_pairs = list(zip(m['kept'][0][0::2], m['kept'][0][1::2]))
    kept_set = set(kept_pairs)
    in_sub = np.array([(int(u), int(v)) in kept_set for u, v in zip(gs, gd)])

    # Edges inside the crop (one per neighbouring pair), coloured by whether pass 1 kept them.
    def crop_edges(keep):
        e = {}
        for u, v, w1 in zip(gs.tolist(), gd.tolist(), costs[0].tolist()):
            (x1, y1), (x2, y2) = xy(u), xy(v)
            if (x2, y2) < (x1, y1) or not (cv.in_crop(x1, y1) and cv.in_crop(x2, y2)):
                continue
            k = (x1, y1, x2 - x1, y2 - y1)
            e[k] = EDGE_KEPT if keep((u, v)) or keep((v, u)) else EDGE_CUT
        return e
    sub_edges = crop_edges(lambda uv: uv in kept_set)

    # ---------------- MultiLevel
    t = 'MultiLevel Dijkstra   (r1 > {r2, r3})'
    frames, layers = sweep_frames(cv, t, [
        ('Pass 1: forward Dijkstra on r1 from the start, stopped once it passes the best r1 cost',
         m['forward'][0], FORWARD, 'best r1 = 0: every rule-free vertex is reached (ties broken by vertex id)'),
        ('Pass 1: backward A* on r1 from the goal, guided by the forward distances',
         m['backward'][0], BACKWARD, 'settles only vertices on r1-optimal routes (ties by vertex id)'),
    ], [])
    # subgraph forming in the zoomed view: kept edges appear in the order the backward search found them
    shown, edges = 0, {}
    crop_kept = [(u, v) for u, v in kept_pairs if cv.in_crop(*xy(u)) and cv.in_crop(*xy(v))]
    base = np.zeros((cv.ch, cv.cw, 3)); base[:] = CELL
    for k in steps(len(crop_kept), 22):
        for u, v in crop_kept[shown:k]:
            (x1, y1), (x2, y2) = sorted([xy(u), xy(v)])
            edges[(x1, y1, x2 - x1, y2 - y1)] = EDGE_KEPT
        shown = k
        frames.append((cv.frame(t, [('Optimal r1 subgraph forming (zoomed in): edges on r1-optimal routes',
                                     TEXT), (f'{len(kept_pairs):,} of {len(gs):,} edges kept in the whole graph',
                                             DIM)], cv.zoom(base, edges)), 90))
    frames.append((cv.frame(t, [('Optimal r1 subgraph: grey edges kept, red edges cut (they break rule r1)', TEXT),
                                (f'{len(kept_pairs):,} of {len(gs):,} edges kept ({100 * len(kept_pairs) / len(gs):.0f}%)',
                                 DIM)], cv.zoom(base, sub_edges)), 1800))
    h2 = dijkstra_order(n, gs, gd, costs[1], dst, True, in_sub)
    h3 = dijkstra_order(n, gs, gd, costs[2], dst, True, in_sub)
    hf, _ = sweep_frames(cv, t, [
        ('BOA* heuristic: backward Dijkstra on r2 within the subgraph', h2, HEUR[1], 'exact r2 distance to the goal'),
        ('BOA* heuristic: backward Dijkstra on r3 within the subgraph', h3, HEUR[2], 'exact r3 distance to the goal'),
    ], [], n=12)
    frames += hf
    sf, counts = search_frames(cv, t, 'BOA* on the subgraph: (r2, r3) trade-offs (red = expanded many times)',
                               m['pareto'], dst, 60, sub_edges)
    frames += sf
    ml_note = f'measured runtime {a.ml_seconds:.2f}s in total' if a.ml_seconds else ''
    frames += final_frames(cv, t, counts, m['solutions'], src, dst, sub_edges,
                           [(f'{len(m["solutions"])} Pareto-optimal routes (all with r1 = {m["solutions"][0]["cost"][0]})   green = start, red = goal',
                             TEXT), (f'{len(m["pareto"]):,} BOA* expansions   {ml_note}', DIM)])
    save(frames, out / 'multilevel.gif')

    # ---------------- RA*pex
    t = 'RA*pex   (r1 > {r2, r3}, exact)'
    order = [dijkstra_order(n, gs, gd, costs[k], dst, True) for k in range(3)]
    frames, _ = sweep_frames(cv, t, [
        (f'Heuristic: backward Dijkstra on r{k + 1} over the whole graph', order[k], HEUR[k],
         f'exact r{k + 1} distance to the goal') for k in range(3)], [], n=12)
    all_edges = crop_edges(lambda uv: True)  # RA*pex searches the whole graph, rule-breaking edges included
    sf, counts = search_frames(cv, t, 'Search over all three costs at once (red = expanded many times)',
                               r['expanded'], dst, 60, all_edges)
    frames += sf
    ra_note = (f'measured {a.rapex_seconds[0]:.2f}s heuristic + {a.rapex_seconds[1]:.2f}s search'
               if a.rapex_seconds else '')
    frames += final_frames(cv, t, counts, r['solutions'], src, dst, all_edges,
                           [(f'{len(r["solutions"])} Pareto-optimal routes, the same as MultiLevel   green = start, red = goal', TEXT),
                            (f'{len(r["expanded"]):,} expansions on {len(set(r["expanded"])):,} vertices   {ra_note}', DIM)])
    save(frames, out / 'rapex.gif')


if __name__ == '__main__':
    main()
