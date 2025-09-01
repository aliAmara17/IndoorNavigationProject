# Path Recording & Replay for ORB-SLAM3 (Monocular, RGB-only)

## 1) Goal & Constraints
- Use **ORB-SLAM3** with a single **RGB** camera (no GPS).
- Provide a small feature that helps analyze/communicate trajectories.

## 2) Our Feature (what we added)
**Path Recording & Replay** toolchain:
- Input: **TUM** trajectory (`CameraTrajectory.txt` / `KeyFrameTrajectory.txt`) or **CSV** with [timestamp, tx, ty, tz, qx, qy, qz, qw].
- Output: **topdown.png**, **3d.png**, **replay.mp4** (animated path).
- Options: `--center`, `--downsample k`, `--make3d`, `--animate`.

## 3) Pipeline (how to use)
1. Run ORB-SLAM3 on a TUM sequence; in the viewer press **s** to save the trajectory.  
2. Convert TUM → CSV:  
   `python3 tools/parse_tum_to_csv.py --tum <path> --out results/<run>_poses.csv`  
3. Replay & export:  
   `python3 tools/replay_trajectory.py --csv results/<run>_poses.csv --center --make3d --animate`  
4. Artifacts appear under `results/` and `videos/`. We include example outputs in `examples/<run>/`.

## 4) Implementation Highlights
- TUM parser with validation; CSV loader with required columns.
- `--center` normalization to compare runs from a common origin.
- 3D plot with equalized axes; MP4 via Matplotlib + ffmpeg.
- Small CLI; easy to reuse with other sequences.

## 5) Results (in this repo)
- **room** → `examples/room/{topdown.png, 3d.png, replay.mp4}`
- **xyz**  → `examples/xyz/{topdown.png, 3d.png, replay.mp4}`

## 6) Limitations
- Quality depends on SLAM tracking; noisy poses → noisy path.
- MP4 export requires `ffmpeg` installed.

## 7) Repro quick note
YAML hint when running ORB-SLAM3: `fr1_* → TUM1.yaml`, `fr2_* → TUM2.yaml`, `fr3_* → TUM3.yaml`.

## 8) Acknowledgments / Academic Integrity
- ORB-SLAM3 by UZ-SLAMLab; TUM RGB-D dataset (fr1_room, fr1_xyz).
- ChatGPT helped with refactoring and documentation; commands and outputs were verified locally.
