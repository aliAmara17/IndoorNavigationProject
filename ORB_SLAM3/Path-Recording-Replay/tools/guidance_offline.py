#!/usr/bin/env python3
import argparse, json, os, math
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")  # headless
import matplotlib.pyplot as plt
from matplotlib import animation

def load_df(csv_path: str) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    # normalize column names for positions
    for cand in [('tx','ty','tz'), ('x','y','z')]:
        if all(c in df.columns for c in cand):
            df = df.rename(columns={cand[0]:'tx', cand[1]:'ty', cand[2]:'tz'})
            break
    if not {'tx','ty','tz'}.issubset(df.columns):
        raise ValueError(f"CSV must have tx,ty,tz columns. Found: {df.columns}")
    # timestamp fallback
    if 'timestamp' not in df.columns and 't' in df.columns:
        df = df.rename(columns={'t':'timestamp'})
    if 'timestamp' not in df.columns:
        df['timestamp'] = np.arange(len(df), dtype=float)
    return df

def path_arclength(xy):
    dif = np.diff(xy, axis=0)
    seg = np.linalg.norm(dif, axis=1)
    s = np.zeros(len(xy))
    s[1:] = np.cumsum(seg)
    return s

def forward_heading(xy, i):
    """Estimate heading (unit vector) at index i using local tangent."""
    if i <= 0:
        v = xy[1] - xy[0]
    elif i >= len(xy)-1:
        v = xy[-1] - xy[-2]
    else:
        v = xy[i+1] - xy[i-1]
    n = np.linalg.norm(v) + 1e-9
    return v / n

def nearest_index(xy, p):
    d2 = np.sum((xy - p)**2, axis=1)
    return int(np.argmin(d2))

def lookahead_index_by_distance(s, i_near, lookahead_dist):
    """Given cumulative arclength s and nearest index, find index ~ lookahead_dist ahead."""
    target_s = s[i_near] + lookahead_dist
    # find smallest j where s[j] >= target_s
    j = np.searchsorted(s, target_s, side="left")
    j = min(max(j, i_near), len(s)-1)
    return j

def heading_error_deg(h_curr, vec_to_target):
    """Signed heading error (deg), positive => target to the left."""
    # angle between
    dot = np.clip(np.dot(h_curr, vec_to_target)/(np.linalg.norm(vec_to_target)+1e-9), -1.0, 1.0)
    ang = math.degrees(math.acos(dot))
    # sign via 2D cross product (z-component)
    cross_z = h_curr[0]*vec_to_target[1] - h_curr[1]*vec_to_target[0]
    return ang if cross_z > 0 else -ang

def cross_track_error(p, a, b):
    """Signed lateral distance from point p to segment a->b."""
    ap = p - a
    ab = b - a
    L2 = np.dot(ab, ab)
    if L2 < 1e-12:  # degenerate
        proj = a
    else:
        t = np.clip(np.dot(ap, ab)/L2, 0.0, 1.0)
        proj = a + t*ab
    # sign via left/right of segment direction (2D cross)
    left = np.array([-ab[1], ab[0]])
    sign = 1.0 if np.dot(ap, left) > 0 else -1.0
    d = np.linalg.norm(p - proj)
    return sign*d

def make_topdown_axes(xy):
    x, y = xy[:,0], xy[:,1]
    xmin, xmax = x.min(), x.max()
    ymin, ymax = y.min(), y.max()
    dx, dy = xmax - xmin, ymax - ymin
    pad_x, pad_y = 0.05*dx if dx>0 else 1.0, 0.05*dy if dy>0 else 1.0
    fig, ax = plt.subplots(figsize=(6,6))
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, ls=':', alpha=0.4)
    ax.set_xlim(xmin - pad_x, xmax + pad_x)
    ax.set_ylim(ymin - pad_y, ymax + pad_y)
    ax.plot(x, y, '-', lw=2, color='tab:blue', alpha=0.8, label='path')
    ax.plot(x[0], y[0], 'o', color='tab:green', ms=8, label='start')
    ax.plot(x[-1], y[-1], 'o', color='tab:red', ms=8, label='end')
    ax.legend(loc='best', fontsize=8)
    return fig, ax

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True, help="results/<run>_poses.csv")
    ap.add_argument("--goal", required=True, help="results/<run>_goal.json")
    ap.add_argument("--lookahead", type=float, default=0.2, help="lookahead distance (units of path scale)")
    ap.add_argument("--goal_radius", type=float, default=0.1, help="stop when within this distance to goal")
    ap.add_argument("--out", default=None, help="output mp4 path")
    ap.add_argument("--csv_out", default=None, help="output guidance CSV path")
    args = ap.parse_args()

    df = load_df(args.csv)
    xy = df[['tx','ty']].values.astype(np.float32)

    with open(args.goal, 'r') as f:
        goal = json.load(f)
    g_xy = np.array([goal['t'][0], goal['t'][1]], dtype=np.float32)

    s = path_arclength(xy)
    # indices to simulate "live" along the whole route
    indices = np.arange(len(xy), dtype=int)

    # prepare plotting
    fig, ax = make_topdown_axes(xy)
    live_pt, = ax.plot([], [], 'o', color='k', ms=6)
    target_pt, = ax.plot([], [], 'o', color='tab:orange', ms=8, mfc='none', mew=2)
    goal_pt, = ax.plot([g_xy[0]], [g_xy[1]], 'x', color='k', ms=8, mew=2)
    text_box = ax.text(0.02, 0.98, '', transform=ax.transAxes, va='top', ha='left', fontsize=9,
                       bbox=dict(boxstyle="round,pad=0.3", fc="w", ec="0.5", alpha=0.9))

    # compute once for efficiency
    guidance_rows = []

    def init():
        live_pt.set_data([], [])
        target_pt.set_data([], [])
        text_box.set_text('')
        return live_pt, target_pt, text_box

    def update(k):
        i = indices[k]
        p = xy[i]
        live_pt.set_data([p[0]], [p[1]])

        # nearest to path is i itself (since we simulate), but compute target via lookahead
        j = lookahead_index_by_distance(s, i, args.lookahead)
        t = xy[j]
        target_pt.set_data([t[0]], [t[1]])

        # current heading (from path tangent)
        h = forward_heading(xy, i)
        vec = t - p
        if np.linalg.norm(vec) < 1e-9:
            he = 0.0
        else:
            he = heading_error_deg(h, vec)

        # cross-track error to segment [i, i+1] (or last segment)
        a = xy[i]
        b = xy[min(i+1, len(xy)-1)]
        cte = cross_track_error(p, a, b)

        dist_goal = np.linalg.norm(p - g_xy)
        stop = dist_goal < args.goal_radius

        text = (f"idx={i}\n"
                f"heading_err={he:+.1f}°\n"
                f"cross_track={cte:+.3f}\n"
                f"dist_to_goal={dist_goal:.3f}\n"
                f"{'GOAL REACHED' if stop else ''}")
        text_box.set_text(text)

        guidance_rows.append({
            "idx": int(i),
            "x": float(p[0]), "y": float(p[1]),
            "target_x": float(t[0]), "target_y": float(t[1]),
            "heading_err_deg": float(he),
            "cross_track": float(cte),
            "dist_to_goal": float(dist_goal)
        })
        return live_pt, target_pt, text_box

    Writer = animation.writers['ffmpeg']
    writer = Writer(fps=15, metadata=dict(artist='guidance_offline'), bitrate=1800)

    ani = animation.FuncAnimation(fig, update, frames=len(indices), init_func=init, blit=False, interval=50)
    out_mp4 = args.out or os.path.join("videos", "guidance.mp4")
    os.makedirs(os.path.dirname(out_mp4), exist_ok=True)
    ani.save(out_mp4, writer=writer, dpi=140)

    # write guidance CSV
    out_csv = args.csv_out or os.path.join("results", "guidance_metrics.csv")
    pd.DataFrame(guidance_rows).to_csv(out_csv, index=False)

    print(f"Saved guidance video → {out_mp4}")
    print(f"Saved guidance metrics → {out_csv}")

if __name__ == "__main__":
    main()

