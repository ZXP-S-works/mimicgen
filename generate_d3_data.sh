#!/usr/bin/bash

tasks=(three_piece_assembly_d3 square_d3 coffee_d3 threading_d3)
n_runs=${#tasks[@]}

cd mimicgen/scripts

#python generate_core_configs.py

for ((i=0; i<n_runs; i++));
do
  task=${tasks[$i]}
  echo "Starting generation for ${task}."
  python generate_dataset.py --config /tmp/core_configs/demo_src_${task}_task_D3.json --auto-remove-exp 2>&1 & sleep 1s
done

wait

#for ((i=0; i<n_runs; i++));
#do
#  task = ${tasks[$i]}
#  echo "Starting pcd generation for ${task}."
#  python generate_dataset.py --config /tmp/core_configs/demo_src_${task}_task_D3.json --auto-remove-exp 2>&1 & sleep 1s
#done
