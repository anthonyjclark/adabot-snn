#!/usr/bin/env bash

mkdir -p data3/f0.7
mkdir -p data3/f0.8
mkdir -p data3/f0.9
mkdir -p data3/f1.0

jobs=(
"../bin/fsm 3 5 8 0.25 0.5 0.7 1/576 1> /dev/null 2> data3/f0.7/fsm-3-5-8-0.25-0.5-0.7.csv"
"../bin/fsm 3 5 8 0.25 0.5 0.8 2/576 1> /dev/null 2> data3/f0.8/fsm-3-5-8-0.25-0.5-0.8.csv"
"../bin/fsm 3 5 8 0.25 0.5 0.9 3/576 1> /dev/null 2> data3/f0.9/fsm-3-5-8-0.25-0.5-0.9.csv"
"../bin/fsm 3 5 8 0.25 0.5 1.0 4/576 1> /dev/null 2> data3/f1.0/fsm-3-5-8-0.25-0.5-1.0.csv"
"../bin/fsm 3 5 8 0.25 1.0 0.7 5/576 1> /dev/null 2> data3/f0.7/fsm-3-5-8-0.25-1.0-0.7.csv"
"../bin/fsm 3 5 8 0.25 1.0 0.8 6/576 1> /dev/null 2> data3/f0.8/fsm-3-5-8-0.25-1.0-0.8.csv"
"../bin/fsm 3 5 8 0.25 1.0 0.9 7/576 1> /dev/null 2> data3/f0.9/fsm-3-5-8-0.25-1.0-0.9.csv"
"../bin/fsm 3 5 8 0.25 1.0 1.0 8/576 1> /dev/null 2> data3/f1.0/fsm-3-5-8-0.25-1.0-1.0.csv"
"../bin/fsm 3 5 8 0.5 0.5 0.7 9/576 1> /dev/null 2> data3/f0.7/fsm-3-5-8-0.5-0.5-0.7.csv"
"../bin/fsm 3 5 8 0.5 0.5 0.8 10/576 1> /dev/null 2> data3/f0.8/fsm-3-5-8-0.5-0.5-0.8.csv"
)

run_sim () {
    echo "$1"
    eval "$1"
}

echo "Starting..."

N=4
(
for job in "${jobs[@]}"; do
    ((i=i%N)); ((i++==0)) && wait
    run_sim "$job" &
done
)

echo "Done."
