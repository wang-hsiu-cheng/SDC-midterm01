"""Kaggle score progression of the three tracks in ONE chart.

Usage: python3 make_progress_fig.py [output.png]   (needs matplotlib)
x axis = development step (the step names of each track are listed in the report table);
y axis = Kaggle public score (lower is better); dashed lines = the baseline of each track.
Only the versions on the main line are drawn; side experiments are discussed in the report text.
"""
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

BLUE, ORANGE, GREEN, GRAY = "#2a7de1", "#e8683a", "#1aae7b", "#8a8984"

# (label, color, scores per step, baseline, value-label placement per step: +1 above, -1 below)
TRACKS = [
    ("Track1", BLUE, [1.56, 1.62, 1.52, 1.13], 1.79, [(0, 10, "center"), (0, 10, "center"), (0, 10, "center"), (0, -18, "center")], "baseline 1.79", "left"),
    ("Track2", ORANGE, [1.46, 0.77], 1.92, [(0, -20, "center"), (0, -20, "center")], "baseline 1.92", "left"),
    ("Track3 (public)", GREEN, [3.98, 2.46, 2.01, 2.01, 1.87, 1.74], 1.29, [(0, 10, "center"), (12, 4, "left"), (0, 10, "center"), (0, 10, "center"), (0, 10, "center"), (0, -18, "center")], "public baseline 1.29", "right"),
]

out = sys.argv[1] if len(sys.argv) > 1 else "figures/all_tracks_progress.png"
fig, ax = plt.subplots(figsize=(9, 5.4))
for name, color, ys, base, place, base_label, base_side in TRACKS:
    xs = list(range(1, len(ys) + 1))
    ax.plot(xs, ys, "-o", color=color, lw=2.4, ms=7, label=name, zorder=3)
    for x, y, (dx, dy, ha) in zip(xs, ys, place):
        ax.annotate(f"{y:.2f}", (x, y), textcoords="offset points", xytext=(dx, dy),
                    ha=ha, fontsize=9, color=color, fontweight="bold")
    ax.axhline(base, color=color, ls="--", lw=1.2, alpha=0.8)
    if base_side == "left":
        ax.text(0.75, base, base_label, ha="left", va="bottom", fontsize=8, color=color)
    else:
        ax.text(6.45, base, base_label, ha="right", va="bottom", fontsize=8, color=color)

ax.set_xticks(range(1, 7))
ax.set_xticklabels([f"Step {i}" for i in range(1, 7)])
ax.set_xlim(0.7, 6.5)
ax.set_ylim(0.5, 4.2)
ax.set_xlabel("Development step (step names: see the table in the report)")
ax.set_ylabel("Kaggle public score (lower is better)")
ax.grid(axis="y", color="#e5e5e5")
for side in ("top", "right"):
    ax.spines[side].set_visible(False)
ax.legend(loc="upper right", frameon=False)
ax.set_title("Kaggle score progression of the three tracks (main-line versions; dashed = baseline)", fontsize=11, loc="left")
fig.tight_layout()
fig.savefig(out, dpi=170)
print("wrote", out)
