#!/usr/bin/env python3
import argparse, time, json
import numpy as np
import pandas as pd
from pathlib import Path

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True, help="results/<run>_poses.csv")
    ap.add_argument("--out", default="/tmp/LivePose.txt")
    ap.add_argument("--rate_hz", type=float, default=10.0)
    ap.add_argument("--loop", action="store_true", help="loop forever over the CSV")
    args = ap.parse_args()

    df = pd.read_csv(args.csv)
    # normalize column names
    for cand in [('tx','ty','tz'), ('x','y','z')]:
        if all(c in df.columns for c in cand):
            df = df.rename(columns={cand[0]:'tx', cand[1]:'ty', cand[2]:'tz'})
            break
    if 'timestamp' not in df.columns:
        df['timestamp'] = np.arange(len(df), dtype=float)

    out = Path(args.out)
    period = 1.0 / max(args.rate_hz, 1e-3)

    print(f"[pose_feeder] Streaming {len(df)} poses → {out} at {args.rate_hz:.1f} Hz")
    try:
        while True:
            for _, r in df.iterrows():
                # format: ts tx ty tz qx qy qz qw  (qw/qxyz optional)
                tx, ty, tz = float(r['tx']), float(r['ty']), float(r['tz'])
                ts = float(r['timestamp'])
                if all(c in df.columns for c in ('qx','qy','qz','qw')):
                    qx, qy, qz, qw = float(r['qx']), float(r['qy']), float(r['qz']), float(r['qw'])
                else:
                    # fallback: no orientation in CSV → set identity quaternion
                    qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
                line = f"{ts:.6f} {tx:.6f} {ty:.6f} {tz:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n"
                out.write_text(line)
                time.sleep(period)
            if not args.loop:
                break
    except KeyboardInterrupt:
        print("\n[pose_feeder] Stopped.")

if __name__ == "__main__":
    main()

