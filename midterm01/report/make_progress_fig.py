"""Kaggle score progression of the three tracks in one figure.

Usage: python3 make_progress_fig.py [output.png]   (needs matplotlib)
Filled markers = versions on the main line; hollow markers = side experiments that were not adopted.
All scores are Kaggle public scores (lower is better).
"""
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

GREEN, ORANGE, GRAY = "#1aae7b", "#e8683a", "#8a8984"

TRACKS = [
    {
        "title": "Track1",
        "main": [("Common\nbase", 1.56), ("Yaw-only\ninit search", 1.62), ("+ yaw meas.\nσ 4°", 1.52), ("+ GNSS\nprior", 1.13)],
        "side": [(1, 1.62, "floor 0.6 + xy grid", 20), (3, 1.17, "GNSS xy only", 4), (3, 1.47, "GNSS yaw only", 0)],
        "below": [3],
        "baseline": (1.79, "baseline 1.79"),
        "ylim": (1.0, 1.95),
    },
    {
        "title": "Track2",
        "main": [("Converged version\n+ wall prior, σ 4°", 1.46), ("+ GNSS\nprior", 0.77)],
        "side": [(1, 0.78, "GNSS xy only", 6), (1, 1.38, "GNSS yaw only", 0)],
        "below": [1],
        "baseline": (1.92, "baseline 1.92"),
        "ylim": (0.6, 2.05),
    },
    {
        "title": "Track3  (final: public 1.74 / private 1.36)",
        "main": [
            ("Old\n(scale 8.5,\nno ZUPT)", 3.98),
            ("scale 7.5\n+ ZUPT", 2.46),
            ("+ init search\n+ yaw freeze", 2.01),
            ("+ accel-bias\nnoise 0.002", 2.01),
            ("+ trail\nheading", 1.87),
            ("+ σ 4°,\nstronger trail", 1.74),
        ],
        "side": [
            (3, 1.93, "floor 0.6 + hold", -14),
            (5, 1.92, "trail while stopped", 14),
            (5, 1.85, "σ 4° only", 0),
            (5, 1.84, "lateral push", -14),
        ],
        "below": [5],
        "baseline": (1.29, "public baseline 1.29"),
        "ylim": (1.15, 4.2),
    },
]

out = sys.argv[1] if len(sys.argv) > 1 else "figures/all_tracks_progress.png"
fig, axes = plt.subplots(1, 3, figsize=(16, 4.8), gridspec_kw={"width_ratios": [4, 2.6, 6]})
for ax, t in zip(axes, TRACKS):
    xs = list(range(len(t["main"])))
    ys = [v for _, v in t["main"]]
    ax.plot(xs, ys, "-o", color=GREEN, lw=2.2, ms=7, zorder=3)
    for x, y in zip(xs, ys):
        dy = -17 if x in t["below"] else 9
        ax.annotate(f"{y:.2f}", (x, y), textcoords="offset points", xytext=(0, dy), ha="center", fontsize=10)
    for x, y, label, dy in t["side"]:
        ax.plot([x], [y], "o", mfc="white", mec=ORANGE, mew=2, ms=7, zorder=3)
        ax.annotate(f"{label} {y:.2f}", (x, y), textcoords="offset points", xytext=(10, dy - 3), ha="left", fontsize=8, color="#555")
    base, base_label = t["baseline"]
    ax.axhline(base, color=GRAY, ls="--", lw=1.3)
    ax.text(len(xs) + 0.5, base, base_label, ha="right", va="bottom", fontsize=9, color=GRAY)
    ax.set_xticks(xs)
    ax.set_xticklabels([l for l, _ in t["main"]], fontsize=8)
    ax.set_xlim(-0.4, len(xs) + 0.6)
    ax.set_ylim(*t["ylim"])
    ax.set_title(t["title"], fontsize=12, loc="left")
    ax.grid(axis="y", color="#e5e5e5")
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
axes[0].set_ylabel("Kaggle public score (lower is better)")
fig.suptitle("Kaggle score progression (filled = adopted on the main line; hollow = side experiments, not adopted)", fontsize=12, x=0.01, ha="left")
fig.tight_layout(rect=(0, 0, 1, 0.94))
fig.savefig(out, dpi=170)
print("wrote", out)
