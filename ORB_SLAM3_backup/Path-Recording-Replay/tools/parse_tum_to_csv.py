import argparse, os, sys, csv

def parse_tum_line(line):
    parts = line.strip().split()
    if len(parts) != 8:
        return None
    ts, tx, ty, tz, qx, qy, qz, qw = parts
    return {
        "timestamp": float(ts),
        "tx": float(tx), "ty": float(ty), "tz": float(tz),
        "qx": float(qx), "qy": float(qy), "qz": float(qz), "qw": float(qw)
    }

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tum", required=True, help="Path to CameraTrajectory.txt (TUM format)")
    ap.add_argument("--out", default="results/poses.csv", help="Output CSV path")
    args = ap.parse_args()

    os.makedirs(os.path.dirname(args.out), exist_ok=True)

    rows = []
    with open(args.tum, "r") as f:
        for line in f:
            line=line.strip()
            if not line or line.startswith("#"):
                continue
            rec = parse_tum_line(line)
            if rec is not None:
                rows.append(rec)

    if not rows:
        print("No valid TUM lines found.", file=sys.stderr)
        sys.exit(2)

    with open(args.out, "w", newline="") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=["timestamp","tx","ty","tz","qx","qy","qz","qw"])
        writer.writeheader()
        writer.writerows(rows)
    print(f"Wrote {len(rows)} poses to {args.out}")

if __name__ == "__main__":
    main()
