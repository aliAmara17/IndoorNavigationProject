# Report (2–4 pages)

## Abstract
Monocular ORB-SLAM3 used to record, convert, and replay camera trajectory. No GPS; relative metric poses.

## System Setup
Ubuntu 20.04 VM, Python 3.8, ORB-SLAM3, OpenCV, Pangolin; TUM RGB-D datasets.

## Data Flow
RGB → ORB-SLAM3 → TUM trajectory (CameraTrajectory.txt / KeyFrameTrajectory.txt) → CSV → plots & MP4 replay.

## Pose Format
timestamp, tx ty tz qx qy qz qw (meters, unit quaternion). Top-down uses X vs Z; 3D optional.

## Replay Method
parse_tum_to_csv.py (TUM→CSV). replay_trajectory.py (PNGs + MP4).

## Results
Show first/second runs (PNGs + MP4) and comment on stability/length.

## Limitations
Monocular scale/drift; failures in low texture; relative frame (no GPS).

## Future Work
Live ROS (/tf), real camera input, smoothing/denoising, loop closures.
