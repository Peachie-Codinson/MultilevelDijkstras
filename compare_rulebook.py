#!/usr/bin/env python3
"""Run one-query fork-rulebook comparisons from the Rulebook_approximation root.

Python 3.9+: py -3 compare_rulebook.py --query resources/dataset/one_query.txt
"""
import argparse
import csv
import json
import re
import subprocess
import sys
import time
from pathlib import Path

COST_KEYS = ('cost_until_now', 'cost', 'full_cost')


def read_query(path):
    rows = [line.strip() for line in path.read_text().splitlines()
            if line.strip() and not line.lstrip().startswith('#')]
    if len(rows) != 1:
        raise ValueError('Query file must contain exactly one source,target line')
    a, b = (int(x.strip()) for x in rows[0].split(','))
    return a, b


def json_vectors(log, target):
    vectors = set()
    for line in log.splitlines():
        # The existing solvers print goal records as JSON, sometimes after a prefix.
        start = line.find('{')
        if start < 0:
            continue
        try:
            obj = json.loads(line[start:])
        except (ValueError, TypeError):
            continue
        if not isinstance(obj, dict) or obj.get('id') != target:
            continue
        for key in COST_KEYS:
            val = obj.get(key)
            if isinstance(val, list) and len(val) == 3:
                vectors.add(tuple(int(v) for v in val))
                break
    return vectors


def multilevel_vectors(path):
    vectors = set()
    for line in path.read_text().splitlines():
        values = line.split()
        if len(values) != 3 or not all(re.fullmatch(r'\d+', v) for v in values):
            raise ValueError(f'Unexpected MultiLevel solution line: {line!r}')
        vectors.add(tuple(map(int, values)))
    return vectors


def run(args, name, source, target, root):
    out = root / name
    out.mkdir(parents=True, exist_ok=True)
    stats = out / 'stats.txt'
    command = [str(args.exe.resolve()), '-m', *(str(p.resolve()) for p in args.maps),
               '-q', str(args.query.resolve()), '-r', str(args.rules.resolve()),
               '-a', name, '-t', str(args.cutoff), '-o', str(stats.resolve())]
    if name == 'RApex':
        command[1:1] = ['--merge', 'RANDOM']
    start = time.perf_counter()
    try:
        proc = subprocess.run(command, capture_output=True, text=True,
                              errors='replace', timeout=args.cutoff + 90)
        code = proc.returncode
        log = proc.stdout + '\n' + proc.stderr
    except subprocess.TimeoutExpired as exc:
        code = 'PROCESS_TIMEOUT'
        log = (exc.stdout or b'').decode(errors='replace') if isinstance(exc.stdout, bytes) else (exc.stdout or '')
        log += '\nPROCESS TIMEOUT: cutoff plus 90 seconds elapsed\n'
    wall = time.perf_counter() - start
    (out / 'console.txt').write_text(log, encoding='utf-8')
    vectors = set()
    status = 'UNKNOWN'
    graph_edges = ''
    if name == 'MultiLevel' and stats.exists():
        records = [x for x in stats.read_text().splitlines() if x.strip()]
        if records:
            fields = records[-1].split('\t')
            if len(fields) >= 9:
                status = fields[-1].strip()
                graph_edges = fields[6 - 1]  # sixth field: retained edge count
                solutions = Path(fields[7])
                if not solutions.is_absolute():
                    solutions = Path.cwd() / solutions
                if solutions.exists():
                    vectors = multilevel_vectors(solutions)
    else:
        vectors = json_vectors(log, target)
        # The original solver's output format does not promise a completion marker.
        # Even exit 0 can mean the algorithm hit its own -t cutoff.
        status = 'CHECK_LOG'
    if code != 0:
        status = f'ERROR({code})'
    if re.search(r'\b(time\s*out|timed\s*out|cutoff)\b', log, re.I):
        status = 'TIMEOUT_OR_CUTOFF'
    (out / 'vectors.txt').write_text(''.join(' '.join(map(str, v)) + '\n' for v in sorted(vectors)), encoding='utf-8')
    return dict(algorithm=name, source=source, target=target, status=status,
                wall_seconds=f'{wall:.3f}', vector_count=len(vectors),
                r1_edges=graph_edges, return_code=code, vectors=vectors)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--exe', type=Path, default=Path('build/multiobj.exe'))
    p.add_argument('--maps', nargs=3, type=Path, default=[Path(f'resources/dataset/USA-road-{x}.BAY.gr') for x in ('d', 't', '3')])
    p.add_argument('--query', type=Path, default=Path('resources/dataset/one_query.txt'))
    p.add_argument('--rules', type=Path, default=Path('resources/dataset/fork_rules.txt'))
    p.add_argument('--cutoff', type=int, default=300)
    p.add_argument('--out', type=Path, default=Path('comparison'))
    a = p.parse_args()
    for path in [a.exe, *a.maps, a.query, a.rules]:
        if not path.is_file():
            p.error(f'Missing file: {path}')
    source, target = read_query(a.query)
    a.out.mkdir(parents=True, exist_ok=True)
    results = [run(a, name, source, target, a.out) for name in ('MultiLevel', 'RBExact', 'RApex')]
    with (a.out / 'summary.csv').open('w', newline='', encoding='utf-8') as file:
        writer = csv.DictWriter(file, fieldnames=['algorithm', 'source', 'target', 'status', 'wall_seconds', 'vector_count', 'r1_edges', 'return_code'])
        writer.writeheader()
        writer.writerows({k: v for k, v in row.items() if k != 'vectors'} for row in results)
    exact = results[0]['vectors']
    with (a.out / 'differences.txt').open('w', encoding='utf-8') as file:
        file.write('Set comparisons against MultiLevel; interpret only after verifying complete runs.\n')
        for row in results[1:]:
            file.write(f"\n{row['algorithm']}:\n")
            file.write('  Missing: ' + repr(sorted(exact - row['vectors'])) + '\n')
            file.write('  Extra:   ' + repr(sorted(row['vectors'] - exact)) + '\n')
    for row in results:
        print(f"{row['algorithm']:10} {row['status']:18} {row['wall_seconds']:>9}s  {row['vector_count']:>5} vectors")
    print(f"Details: {a.out.resolve()}")
    if any(row['status'] != 'COMPLETE' for row in results):
        print('Verify RBExact/RApex completion in their console and stats files before interpreting set differences.')

if __name__ == '__main__':
    main()
