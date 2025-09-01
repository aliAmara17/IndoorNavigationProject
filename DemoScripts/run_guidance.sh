#!/bin/bash
# Demo script: Run live guidance (must run run_dataset.sh first in another terminal)

cd $HOME/ORB_SLAM3/Path-Recording-Replay

python3 tools/live_guidance.py \
  --csv results/fr1_xyz_poses.csv \
  --goal results/fr1_xyz_goal.json \
  --live /tmp/LivePose.txt \
  --rate_hz 10 \
  --lookahead 0.2 \
  --goal_radius 0.1

