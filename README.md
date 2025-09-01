# IndoorNavigationProject

# Indoor Navigation without GPS (Monocular RGB)

This project extends **[ORB-SLAM3]** to support live pose export and integrates with **[Path-Recording-Replay](https://github.com/waelzidan7/Path-Recording-Replay)** for trajectory recording, conversion, and replay.

The system enables **indoor navigation without GPS** using only a monocular RGB camera:
- **Run ORB-SLAM3** on TUM datasets or live camera.
- **Save trajectories** in TUM format (live pose export).
- **Convert** TUM → CSV for easier processing.
- **Replay trajectories** in 2D top-down or 3D.
- **Export** trajectory replays as images or MP4 videos.
- **Guidance mode**: use recorded trajectories for navigation.

---

## 📂 Repository Structure
```
IndoorNavigationProject/
├─ ORB_SLAM3/               # Submodule: modified ORB-SLAM3 fork (pose export)
├─ Path-Recording-Replay/   # Submodule: Wael Zidan’s replay & guidance tools
├─ DemoScripts/             # Helper scripts to run datasets and guidance
├─ docs/                    # (Optional) Documentation / slides
├─ examples/                # Sample results (images, videos)
└─ README.md                # This file
```

---

## 🚀 Quick Start

### 1. Clone with Submodules
```bash
git clone --recurse-submodules https://github.com/aliAmara17/IndoorNavigationProject.git
cd IndoorNavigationProject
```

If you forgot `--recurse-submodules`:
```bash
git submodule update --init --recursive
```

---

### 2. Dependencies
- **Ubuntu 20.04** (tested in VM)
- **C++14 compiler** (gcc/g++)
- **CMake ≥ 3.10**
- **OpenCV ≥ 4.4**
- **Eigen ≥ 3.3.7**
- **Pangolin** (for ORB-SLAM3 visualization)
- **Python ≥ 3.6** with packages:
  ```bash
  pip install -r Path-Recording-Replay/requirements.txt
  ```
- **ffmpeg** for MP4 export:
  ```bash
  sudo apt-get install ffmpeg
  ```

---

### 3. Build ORB-SLAM3
```bash
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```

---

### 4. Run Dataset Example
```bash
# Example: run TUM dataset with monocular RGB
./DemoScripts/run_dataset.sh
```

---

### 5. Replay / Guidance
After ORB-SLAM3 finishes, convert & replay trajectory:
```bash
./DemoScripts/run_guidance.sh
```

Outputs:
- Top-down PNG
- 3D PNG
- Replay MP4

---

## 📊 Examples

### From this project
Sample outputs from **fr1/room** dataset:

- **Top-down replay**  
  ![Top-down](examples/fr1_room_topdown.png)

- **3D trajectory**  
  ![3D](examples/fr1_room_3d.png)

- **Replay video**  
  Stored in [`videos/fr1_room_replay.mp4`](videos/fr1_room_replay.mp4)

### More examples
Additional examples are available in  
➡️ [waelzidan7/Path-Recording-Replay – Examples](https://github.com/waelzidan7/Path-Recording-Replay/tree/main/examples)

---

## 🧑‍🤝‍🧑 Credits
- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) – base SLAM system  
- [aliAmara17/ORB_SLAM3](https://github.com/aliAmara17/ORB_SLAM3) – fork with live pose export  
- [Path-Recording-Replay](https://github.com/waelzidan7/Path-Recording-Replay) – replay & guidance tools  

---

## 📜 License
This repo follows the original licenses of [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) and [Path-Recording-Replay](https://github.com/waelzidan7/Path-Recording-Replay).
