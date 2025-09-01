#!/usr/bin/env python3
import argparse, json, pandas as pd

ap = argparse.ArgumentParser()
ap.add_argument("--csv", required=True)
ap.add_argument("--out", default=None)
args = ap.parse_args()

df = pd.read_csv(args.csv)
for cand in [('tx','ty','tz'), ('x','y','z')]:
    if all(c in df.columns for c in cand):
        df = df.rename(columns={cand[0]:'tx', cand[1]:'ty', cand[2]:'tz'})
        break
if not {'tx','ty','tz'}.issubset(df.columns):
    raise ValueError(f"CSV must have tx,ty,tz columns. Found: {df.columns}")

row = df.iloc[-1]  # choose last pose as goal
goal = {
    "idx": int(len(df)-1),
    "timestamp": float(row.get('timestamp', len(df)-1)),
    "t": [float(row['tx']), float(row['ty']), float(row['tz'])]
}
if all(c in df.columns for c in ('qx','qy','qz','qw')):
    goal["q"] = [float(row['qx']), float(row['qy']), float(row['qz']), float(row['qw'])]

out_path = args.out or args.csv.replace("_poses.csv", "_goal.json")
with open(out_path, "w") as f:
    json.dump(goal, f, indent=2)
print(f"Saved goal → {out_path}")

