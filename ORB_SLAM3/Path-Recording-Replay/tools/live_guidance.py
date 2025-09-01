#!/usr/bin/env python3
import argparse, time, math, os, numpy as np, pandas as pd

def load_path(csv_path:str):
    df = pd.read_csv(csv_path)
    for cand in [('tx','ty','tz'), ('x','y','z')]:
        if all(c in df.columns for c in cand):
            df = df.rename(columns={cand[0]:'tx', cand[1]:'ty', cand[2]:'tz'})
            break
    if not {'tx','ty','tz'}.issubset(df.columns):
        raise ValueError("CSV must contain tx,ty,tz columns")
    xy = df[['tx','ty']].values.astype(float)
    # cumulative arclength
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    return xy, s

def forward_heading(xy, i):
    if i<=0: v = xy[1]-xy[0]
    elif i>=len(xy)-1: v = xy[-1]-xy[-2]
    else: v = xy[i+1]-xy[i-1]
    n = np.linalg.norm(v) or 1e-9
    return v/n

def nearest_index(xy, p):
    d2 = np.sum((xy - p)**2, axis=1)
    return int(np.argmin(d2))

def lookahead_index(s, i_near, lookahead_dist):
    target_s = s[i_near] + lookahead_dist
    j = np.searchsorted(s, target_s, side="left")
    return min(max(j, i_near), len(s)-1)

def heading_error_deg(h_curr, vec_to_target):
    if np.linalg.norm(vec_to_target) < 1e-9: return 0.0
    dot = np.clip(np.dot(h_curr, vec_to_target) / (np.linalg.norm(vec_to_target)), -1.0, 1.0)
    ang = math.degrees(math.acos(dot))
    cross_z = h_curr[0]*vec_to_target[1] - h_curr[1]*vec_to_target[0]
    return ang if cross_z > 0 else -ang

def read_live_pose(path:str):
    try:
        with open(path, "r") as f:
            line = f.read().strip()
        if not line: return None
        parts = line.split()
        # ts tx ty tz qx qy qz qw
        if len(parts) < 4: return None
        tx, ty = float(parts[1]), float(parts[2])
        return np.array([tx, ty], dtype=float)
    except Exception:
        return None

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True, help="results/<run>_poses.csv")
    ap.add_argument("--goal", required=True, help="results/<run>_goal.json")
    ap.add_argument("--live", default="/tmp/LivePose.txt")
    ap.add_argument("--rate_hz", type=float, default=10.0)
    ap.add_argument("--lookahead", type=float, default=0.2)
    ap.add_argument("--goal_radius", type=float, default=0.1)
    args = ap.parse_args()

    import json
    xy, s = load_path(args.csv)
    with open(args.goal, "r") as f:
        goal = json.load(f)
    g = np.array([goal["t"][0], goal["t"][1]], dtype=float)

    period = 1.0 / max(args.rate_hz, 1e-3)
    print(f"[live_guidance] reading poses from {args.live}")
    print("[live_guidance] Ctrl+C to stop.\n")

    last_msg = ""
    try:
        while True:
            p = read_live_pose(args.live)
            if p is None:
                time.sleep(period)
                continue

            i_near = nearest_index(xy, p)
            j = lookahead_index(s, i_near, args.lookahead)
            t = xy[j]
            h = forward_heading(xy, i_near)
            vec = t - p
            he = heading_error_deg(h, vec)              # deg, +left / -right
            dist_goal = float(np.linalg.norm(p - g))    # same units as path (SLAM units)
            forward_dist = float(np.linalg.norm(vec))

            # terse guidance text
            turn = "left" if he > 1e-3 else "right"
            msg = (f"idx={i_near:03d}  turn {turn} {abs(he):.1f}°  "
                   f"forward {forward_dist:.2f}  dist_to_goal {dist_goal:.2f}")
            # only print when changed, to keep screen readable
            if msg != last_msg:
                print(msg)
                last_msg = msg

            if dist_goal <= args.goal_radius:
                print("✅ goal reached (within radius).")
                break

            time.sleep(period)
    except KeyboardInterrupt:
        print("\n[live_guidance] stopped by user.")

if __name__ == "__main__":
    main()

