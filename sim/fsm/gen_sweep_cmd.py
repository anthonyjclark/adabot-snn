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


print('#!/usr/bin/env bash\n')

data_dir = 'data4'

# Create friction dirs
for fc in friction_coeff:
    print('mkdir -p', f'{data_dir}/f{fc}')
print()

# Create array of jobs
print('jobs=(')
for i, (sc, fd, ws, tf, se, fc) in enumerate(sweep):
    fname = f'{sc}-{fd}-{ws}-{tf}-{se}-{fc}'
    # vis_fname = f'fsm/fsm_vis-{fname}.json'
    vis_fname = '/dev/null'
    csv_fname = f'{data_dir}/f{fc}/fsm-{fname}.csv'

    args = fname.replace('-', ' ')
    cmd = f'"../bin/fsm {args} {i+1}/{total} 1> {vis_fname} 2> {csv_fname}"'
    print(cmd)
    if i >= 9:
        break
print(')')

# Function to run experiment
print(f'''
run_sim () {{
    echo "$1"
    eval "$1"
}}''')

# Run in batches on N
N = 4
print(f'''
echo "Starting..."

N={N}
(
for job in "${{jobs[@]}}"; do
    ((i=i%N)); ((i++==0)) && wait
    run_sim "$job" &
done
)

echo "Done."''')
