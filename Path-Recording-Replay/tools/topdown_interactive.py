#!/usr/bin/env python3
import argparse, json, os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def load_df(csv_path: str) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    for cand in [('tx','ty','tz'), ('x','y','z')]:
        if all(c in df.columns for c in cand):
            df = df.rename(columns={cand[0]:'tx', cand[1]:'ty', cand[2]:'tz'})
            break
    assert {'tx','ty','tz'}.issubset(df.columns), f"CSV must contain tx,ty,tz. Found: {df.columns}"
    if 'timestamp' not in df.columns and 't' in df.columns:
        df = df.rename(columns={'t':'timestamp'})
    if 'timestamp' not in df.columns:
        df['timestamp'] = np.arange(len(df))
    return df

def compute_view(df_xy, centered=True):
    x = df_xy[:,0].copy(); y = df_xy[:,1].copy()
    mx, my = float(np.mean(x)), float(np.mean(y))
    if centered:
        x -= mx; y -= my
    xmin, xmax = float(x.min()), float(x.max())
    ymin, ymax = float(y.min()), float(y.max())
    return (x, y, mx, my, xmin, xmax, ymin, ymax, centered)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True, help="results/<run>_poses.csv")
    ap.add_argument("--out", default=None, help="results/<run>_goal.json")
    ap.add_argument("--save_preview", action='store_true')
    args = ap.parse_args()

    df = load_df(args.csv)
    xy = df[['tx','ty']].values.astype(np.float32)
    x,y,mx,my,xmin,xmax,ymin,ymax,centered = compute_view(xy, centered=True)

    fig, ax = plt.subplots(figsize=(8,8))
    ax.set_aspect('equal', adjustable='box')
    ax.plot(x, y, '-', lw=2, color='tab:blue')
    ax.plot(x[0], y[0], 'o', color='tab:green', ms=8, label='start')
    ax.plot(x[-1], y[-1], 'o', color='tab:red', ms=8, label='end')
    ax.set_title("Click a goal on the path, then press Enter. (Esc/close to cancel)")
    ax.grid(True, ls=':', alpha=0.4)
    ax.set_xlim(xmin-(xmax-xmin)*0.05, xmax+(xmax-xmin)*0.05)
    ax.set_ylim(ymin-(ymax-ymin)*0.05, ymax+(ymax-ymin)*0.05)

    clicked = {'pt': None, 'marker': None, 'nn': None}
    path = np.stack([x,y], axis=1)

    def on_click(event):
        if not event.inaxes:
            return
        cx, cy = event.xdata, event.ydata
        d2 = (path[:,0]-cx)**2 + (path[:,1]-cy)**2
        idx = int(np.argmin(d2))
        clicked['pt'] = (cx, cy)
        clicked['nn'] = idx
        # update marker
        if clicked['marker'] is not None:
            clicked['marker'].remove()
        clicked['marker'], = ax.plot(path[idx,0], path[idx,1], 'o', color='black', ms=10, mfc='none', mew=2)
        fig.canvas.draw_idle()

    cid = fig.canvas.mpl_connect('button_press_event', on_click)

    # block until user presses Enter or closes
    print(">>> Click to pick a goal, then press Enter in the plot window to confirm.")
    confirmed = {'ok': False}
    def on_key(event):
        if event.key == 'enter':
            confirmed['ok'] = True
            plt.close(fig)
    kid = fig.canvas.mpl_connect('key_press_event', on_key)

    plt.show()  # interactive window

    if not confirmed['ok'] or clicked['nn'] is None:
        print("No goal selected. Exiting.")
        return

    goal_idx = clicked['nn']
    row = df.iloc[goal_idx]
    # convert back to world coordinates (undo centering)
    gx = float(path[goal_idx,0] + mx)
    gy = float(path[goal_idx,1] + my)
    goal = {
        "idx": int(goal_idx),
        "timestamp": float(row.get('timestamp', goal_idx)),
        "t": [float(row['tx']), float(row['ty']), float(row['tz'])]
    }
    # include orientation if present
    for qset in (('qx','qy','qz','qw'), ('qw','qx','qy','qz')):
        if all(q in df.columns for q in qset):
            goal["q"] = [float(row['qx']), float(row['qy']), float(row['qz']), float(row['qw'])]
            break

    out_path = args.out or os.path.join(os.path.dirname(args.csv), "goal.json")
    with open(out_path, "w") as f:
        json.dump(goal, f, indent=2)
    print(f"Saved goal → {out_path}")

    if args.save_preview:
        # save a simple PNG of the figure with the selected goal highlighted
        fig2, ax2 = plt.subplots(figsize=(8,8))
        ax2.set_aspect('equal', adjustable='box')
        ax2.plot(x, y, '-', lw=2, color='tab:blue')
        ax2.plot(x[0], y[0], 'o', color='tab:green', ms=8)
        ax2.plot(x[-1], y[-1], 'o', color='tab:red', ms=8)
        ax2.plot(path[goal_idx,0], path[goal_idx,1], 'o', color='black', ms=10, mfc='none', mew=2)
        ax2.grid(True, ls=':', alpha=0.4)
        ax2.set_xlim(ax.get_xlim()); ax2.set_ylim(ax.get_ylim())
        png_path = out_path.replace(".json", "_preview.png")
        fig2.savefig(png_path, dpi=140, bbox_inches='tight')
        plt.close(fig2)
        print(f"Saved preview → {png_path}")

if __name__ == "__main__":
    main()

