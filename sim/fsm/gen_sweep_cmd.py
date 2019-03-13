#!/usr/bin/env python3

from itertools import product

strut_count = [3, 5, 7]
fsm_deg = [5, 10, 20]
wheel_speed = [8, 16]
turn_factor = [0.25, 0.5, 0.75, 1.0]
strut_extension_percent = [0.5, 1.0]
friction_coeff = [0.7, 0.8, 0.9, 1.0]

sweep = product(strut_count,
                fsm_deg,
                wheel_speed,
                turn_factor,
                strut_extension_percent,
                friction_coeff)

total = len(strut_count) \
      * len(fsm_deg) \
      * len(wheel_speed) \
      * len(turn_factor) \
      * len(strut_extension_percent) \
      * len(friction_coeff)

i = 0
print('#!/usr/bin/env bash\n')
print('mkdir -p data2')
print('echo "Starting..."')
for sc, fd, ws, tf, se, fc in sweep:
    fname = f'{sc}-{fd}-{ws}-{tf}-{se}-{fc}'
    # vis_fname = f'fsm/fsm_vis-{fname}.json'
    vis_fname = '/dev/null'
    csv_fname = f'data2/fsm-{fname}.csv'

    args = fname.replace('-', ' ')
    cmd = f'../bin/fsm {args} 1> {vis_fname} 2> {csv_fname}'
    print(cmd)

    i += 1
    print(f'echo "Completed {i:03} of {total}"')

