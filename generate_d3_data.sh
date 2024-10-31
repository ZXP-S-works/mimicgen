#!/usr/bin/bash

tasks=(three_piece_assembly square coffee threading)
n_runs=${#tasks[@]}

cd mimicgen/scripts

python generate_core_configs.py

for ((i=0; i<n_runs; i++));
do
  task=${tasks[$i]}
  echo "Starting generation for ${task}."
  python generate_dataset.py --config /tmp/core_configs/demo_src_${task}_task_D3.json --auto-remove-exp 2>&1 & sleep 1s
done

wait

## in equidiffpo
#
#for ((i=0; i<n_runs; i++));
#do
#  task=${tasks[$i]}
#  echo "Starting img voxel pcd generation for ${task}."
#  cp /tmp/core_datasets/${task}/demo_src_${task}_task_D3/demo.hdf5 data/robomimic/datasets/${task}_d3/${task}_d3.hdf5
#  python equi_diffpo/scripts/dataset_states_to_obs.py --input data/robomimic/datasets/${task}_d3/${task}_d3.hdf5 \
#  --output data/robomimic/datasets/${task}_d3/${task}_d3_pc.hdf5 --num_workers=12 && \
#  python equi_diffpo/scripts/robomimic_dataset_conversion.py -i data/robomimic/datasets/${task}_d3/${task}_d3.hdf5 \
#  -o data/robomimic/datasets/${task}_d3/${task}_d3_abs.hdf5 -n 12  && \
#  python equi_diffpo/scripts/robomimic_dataset_conversion.py -i data/robomimic/datasets/${task}_d3/${task}_d3_pc.hdf5 \
#  -o data/robomimic/datasets/${task}_d3/${task}_d3_pc_abs.hdf5 -n 12 \
#   2>&1 & sleep 1s
#done
#
#wait