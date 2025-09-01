# ORB-SLAM3 Path Recording & Replay (Monocular RGB)

This project extends **ORB-SLAM3** to record, convert, and replay camera trajectories.  
It adds tools to:
- Save trajectories in TUM format (from ORB-SLAM3).
- Convert TUM → CSV.
- Replay trajectories with top-down and 3D visualizations.
- Export MP4 videos of the replay.

Camera input is **monocular RGB only (no GPS)**.  
All trajectories are **relative in meters**.

---

## 1. Clone Repositories

### Clone this project (Path-Recording-Replay)
```bash
git clone https://github.com/waelzidan7/Path-Recording-Replay.git
cd Path-Recording-Replay
```
Clone and build ORB-SLAM3
```bash
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ~/Dev/ORB_SLAM3
cd ~/Dev/ORB_SLAM3
chmod +x build.sh
./build.sh
```
## 2. Dependencies

Make sure you have the following installed:

Ubuntu 20.04 (tested in VM)

C++14 compiler (gcc/g++)

CMake ≥ 3.10

Pangolin (for ORB-SLAM3 visualization)

OpenCV ≥ 4.4

Eigen ≥ 3.3.7

Python ≥ 3.8 with:

numpy

matplotlib

opencv-python

Install Python dependencies from this repo:
```bash
python3 -m pip install -r requirements.txt
```
## 3. Dataset
We use the **[TUM RGB-D dataset](https://vision.in.tum.de/data/datasets/rgbd-dataset)**.
Download any sequence (e.g. rgbd_dataset_freiburg1_xyz, rgbd_dataset_freiburg1_desk2, …)
and extract them under:
```bash
~/Dev/TUM_dataset/
```
## 4. Usage
Step 1 – Run ORB-SLAM3 on a TUM sequence

From your ORB-SLAM3 repo:
```bash
cd ~/Dev/ORB_SLAM3

# Pick ANY extracted sequence folder under ~/Dev/TUM_dataset
DATA=~/Dev/TUM_dataset/<your_sequence_folder>   # e.g. rgbd_dataset_freiburg1_xyz

# Run monocular ORB-SLAM3 (choose correct YAML file: TUM1.yaml for fr1_*, TUM2.yaml for fr2_*, TUM3.yaml for fr3_*)
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml "$DATA"

# In the viewer, press 's' to save the trajectory before shutdown.
# ORB-SLAM3 will write KeyFrameTrajectory.txt (sometimes CameraTrajectory.txt).
```
Step 2 – Convert trajectory (TUM → CSV)
From this repo (Path-Recording-Replay):
```bash
cd ~/Path-Recording-Replay

# Name this run after the dataset folder
RUN=$(basename "$DATA")

# Use KeyFrameTrajectory.txt if available; otherwise CameraTrajectory.txt
SRC=~/Dev/ORB_SLAM3/KeyFrameTrajectory.txt
[ -f "$SRC" ] || SRC=~/Dev/ORB_SLAM3/CameraTrajectory.txt

# Convert to CSV
python3 tools/parse_tum_to_csv.py --tum "$SRC" --out "results/${RUN}_poses.csv"
```
Step 3 – Replay & visualize (PNG + MP4)
```bash
python3 tools/replay_trajectory.py --csv "results/${RUN}_poses.csv" --center --make3d --animate

# Rename outputs
mv -f results/path_topdown.png  "results/${RUN}_topdown.png"
mv -f results/path_3d.png       "results/${RUN}_3d.png"
mv -f videos/path_replay.mp4    "videos/${RUN}_replay.mp4"
```
## 5. Results

After running, you’ll find in the results/ and videos/ folders:

CSV of poses (*_poses.csv)

Top-down trajectory plot (*_topdown.png)

3D trajectory plot (*_3d.png)

Replay video (*_replay.mp4)
6. Preview Results

To preview images:
```bash
eog results/first_topdown.png
```
To play the replay video:
```bash
mpv videos/first_replay.mp4
# or
vlc videos/first_replay.mp4
```
