#!/usr/bin/env python3

from itertools import product
from os import makedirs
from sys import argv, stderr

import numpy as np

if len(argv) < 2:
    print('Give a data number.')
    raise SystemExit

data_dir = f'data{argv[1]}'

parameters = {
    'duration': [1],
    'left wheel speed': np.linspace(-10, 10, 41),
    'right wheel speed': np.linspace(-10, 10, 41),
    'strut extension percent': [0.0, 0.25, 0.5, 0.75, 1.0],
    'friction coefficient': [0.9],
}

sweep = product(*list(parameters.values()))

# Create friction dirs
for fc in parameters['friction coefficient']:
    makedirs(f'{data_dir}/f{fc}', exist_ok=True)

# Create list of jobs
for params in sweep:
    fname = '_'.join(str(p) for p in params)
    # vis_fname = f'fsm/fsm_vis-{fname}.json'
    vis_fname = '/dev/null'
    csv_fname = f'{data_dir}/f{fc}/sweep-{fname}.csv'

    args = fname.replace('_', ' ')
    cmd = f'../bin/sweep {args} 1> {vis_fname} 2> {csv_fname}'
    print(cmd)

print(f'1. {argv[0]} # > job_list.txt', file=stderr)
print('2. wc -l job_list.txt', file=stderr)
print('3. parallel --bar < job_list.txt', file=stderr)
