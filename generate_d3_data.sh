#!/usr/bin/bash

tasks=(three_piece_assembly_d3 square_d3 coffee_d3 threading_d3)
n_runs=${#task[@]}

python mimicgen/scripts/generate_core_configs.py

for ((i=0; i<n_runs; i++));
do
  task = ${tasks[$i]}
  echo "Starting generation for ${tasks[$i]}."
  nohup python generate_dataset.py --config /tmp/core_configs/demo_src_${task}_task_D3.json --auto-remove-exp 2>&1 & sleep 1m
done

wait

for ((i=0; i<n_runs; i++));
do
  task = ${tasks[$i]}
  echo "Starting pcd generation for ${tasks[$i]}."
  nohup python generate_dataset.py --config /tmp/core_configs/demo_src_${task}_task_D3.json --auto-remove-exp 2>&1 & sleep 1m
done
