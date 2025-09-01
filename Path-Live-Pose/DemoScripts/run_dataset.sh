#!/bin/bash
# Demo script: Run ORB-SLAM3 on TUM dataset and produce trajectory

VOC=$HOME/ORB_SLAM3/Vocabulary/ORBvoc.txt
DATA=$HOME/Dev/TUM_dataset/rgbd_dataset_freiburg1_xyz
EXE=$HOME/ORB_SLAM3/Examples/Monocular/mono_tum
CFG=$HOME/ORB_SLAM3/Examples/Monocular/TUM1.yaml

echo "Running ORB-SLAM3 on dataset..."
$EXE $VOC $CFG $DATA

