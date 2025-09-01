#!/usr/bin/env bash
set -e

# === CONFIG (change if your folders are different) ===
PROJECT_DIR="$HOME/orbslam3_path_replay"
ORB_DIR="$HOME/Dev/ORB_SLAM3"

usage() {
  echo "Usage: $0 <run_name> [--no3d] [--nocenter] [--downsample N] [--csv existing_csv]"
  echo "Examples:"
  echo "  $0 fr1_room_long"
  echo "  $0 fr2_desk --downsample 2"
  echo "  $0 custom_run --csv results/custom_poses.csv"
  exit 1
}

[ -z "$1" ] && usage
RUN="$1"; shift

MAKE3D=1
CENTER=1
DOWNSAMPLE=""
EXISTING_CSV=""

# Parse optional flags
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no3d) MAKE3D=0; shift ;;
    --nocenter) CENTER=0; shift ;;
    --downsample) DOWNSAMPLE="$2"; shift 2 ;;
    --csv) EXISTING_CSV="$2"; shift 2 ;;
    *) echo "Unknown option: $1"; usage ;;
  esac
done

mkdir -p "$PROJECT_DIR/results" "$PROJECT_DIR/videos"

cd "$PROJECT_DIR"

# 1) Decide CSV source
CSV_OUT="results/${RUN}_poses.csv"
if [ -n "$EXISTING_CSV" ]; then
  echo "[i] Using existing CSV: $EXISTING_CSV"
  cp -f "$EXISTING_CSV" "$CSV_OUT"
else
  # 2) Find trajectory file dumped by ORB-SLAM3 (press 's' before running this script)
  SRC_TRAJ=""
  if [ -f "$ORB_DIR/CameraTrajectory.txt" ]; then
    SRC_TRAJ="$ORB_DIR/CameraTrajectory.txt"
  elif [ -f "$ORB_DIR/KeyFrameTrajectory.txt" ]; then
    SRC_TRAJ="$ORB_DIR/KeyFrameTrajectory.txt"
  else
    echo "[!] No trajectory found in $ORB_DIR (CameraTrajectory.txt / KeyFrameTrajectory.txt)."
    echo "    Run ORB-SLAM3, click Map Viewer, press 's', then re-run this script."
    exit 1
  fi

  # 3) Copy trajectory with unique name and convert to CSV
  TRAJ_COPY="results/${RUN}_CameraTrajectory.txt"
  cp -f "$SRC_TRAJ" "$TRAJ_COPY"
  echo "[i] Copied trajectory -> $TRAJ_COPY"

  python3 tools/parse_tum_to_csv.py --tum "$TRAJ_COPY" --out "$CSV_OUT"
  echo "[i] Wrote CSV -> $CSV_OUT (rows: $(($(wc -l < "$CSV_OUT")-1)))"
fi

# 4) Build plotting/animation args
CENTER_FLAG=""
[ "$CENTER" -eq 1 ] && CENTER_FLAG="--center"
DOWNSAMPLE_FLAG=""
[ -n "$DOWNSAMPLE" ] && DOWNSAMPLE_FLAG="--downsample $DOWNSAMPLE"

# 5) Make plots (always top-down, optional 3D)
python3 tools/replay_trajectory.py --csv "$CSV_OUT" $CENTER_FLAG $DOWNSAMPLE_FLAG --make3d || true
# rename outputs to run-specific
[ -f results/path_topdown.png ] && mv -f results/path_topdown.png "results/${RUN}_topdown.png"
[ -f results/path_3d.png ]      && mv -f results/path_3d.png      "results/${RUN}_3d.png"

# 6) Make animation MP4
python3 tools/replay_trajectory.py --csv "$CSV_OUT" $CENTER_FLAG $DOWNSAMPLE_FLAG --animate
mv -f videos/path_replay.mp4 "videos/${RUN}_replay.mp4"

# 7) Summary + quick open hints
echo ""
echo "=== DONE ==="
ls -lh "results/${RUN}_CameraTrajectory.txt" 2>/dev/null || true
ls -lh "results/${RUN}_poses.csv"
ls -lh "results/${RUN}_topdown.png" 2>/dev/null || true
ls -lh "results/${RUN}_3d.png"      2>/dev/null || true
ls -lh "videos/${RUN}_replay.mp4"
echo ""
echo "View:"
echo "  eog results/${RUN}_topdown.png"
echo "  eog results/${RUN}_3d.png            # if exists"
echo "  mpv videos/${RUN}_replay.mp4"
