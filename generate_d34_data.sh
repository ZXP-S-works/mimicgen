#!/usr/bin/bash

tasks=(three_piece_assembly square coffee threading)
n_runs=${#tasks[@]}

cd mimicgen/scripts

python generate_core_configs.py

for var in 3 4;
do
for ((i=0; i<n_runs; i++));
do
  task=${tasks[$i]}
  echo "Starting generation for ${task}."
  python generate_dataset.py --config /tmp/core_configs/demo_src_${task}_task_D${var}.json --auto-remove-exp 2>&1 & sleep 1s
done
done

wait
