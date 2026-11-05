#!/usr/bin/env python3
"""Render PNG previews of the compact marker-free WEMT strap mount."""

from __future__ import annotations

import json
import struct
from pathlib import Path
from typing import Optional

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyBboxPatch, Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


ROOT = Path(__file__).resolve().parents[1]
ASSET_DIR = ROOT / "assets" / "wmet_strap_mount"


def load_binary_stl(path: Path) -> np.ndarray:
    with path.open("rb") as stream:
        stream.read(80)
        triangle_count = struct.unpack("<I", stream.read(4))[0]
        dtype = np.dtype(
            [
                ("normal", "<f4", (3,)),
                ("vertices", "<f4", (3, 3)),
                ("attribute", "<u2"),
            ]
        )
        records = np.fromfile(stream, dtype=dtype, count=triangle_count)
    if len(records) != triangle_count:
        raise RuntimeError(f"incomplete binary STL: {path}")
    return records["vertices"].astype(float)


def cuboid_triangles(center, size) -> np.ndarray:
    cx, cy, cz = center
    sx, sy, sz = (value / 2.0 for value in size)
    vertices = np.asarray(
        [
            [cx - sx, cy - sy, cz - sz],
            [cx + sx, cy - sy, cz - sz],
            [cx + sx, cy + sy, cz - sz],
            [cx - sx, cy + sy, cz - sz],
            [cx - sx, cy - sy, cz + sz],
            [cx + sx, cy - sy, cz + sz],
            [cx + sx, cy + sy, cz + sz],
            [cx - sx, cy + sy, cz + sz],
        ],
        dtype=float,
    )
    faces = [
        (0, 2, 1), (0, 3, 2),
        (4, 5, 6), (4, 6, 7),
        (0, 1, 5), (0, 5, 4),
        (1, 2, 6), (1, 6, 5),
        (2, 3, 7), (2, 7, 6),
        (3, 0, 4), (3, 4, 7),
    ]
    return vertices[np.asarray(faces)]


def add_mesh(ax, triangles, color, edge="#1f2933", alpha=1.0, width=0.08):
    ax.add_collection3d(
        Poly3DCollection(
            triangles,
            facecolor=color,
            edgecolor=edge,
            linewidth=width,
            alpha=alpha,
        )
    )


def set_equal_limits(ax, arrays, *, z_floor=0.0, padding=0.10):
    points = np.concatenate([array.reshape(-1, 3) for array in arrays], axis=0)
    minimum = points.min(axis=0)
    maximum = points.max(axis=0)
    center = (minimum + maximum) / 2.0
    span = float((maximum - minimum).max()) * (1.0 + padding)
    half = span / 2.0
    ax.set_xlim(center[0] - half, center[0] + half)
    ax.set_ylim(center[1] - half, center[1] + half)
    ax.set_zlim(z_floor, max(maximum[2] + 4.0, center[2] + half * 0.55))
    ax.set_box_aspect((1.0, 1.0, 0.60))


def save_3d_view(
    output: Path,
    carrier: np.ndarray,
    *,
    title: str,
    elev: float,
    azim: float,
    receiver: Optional[np.ndarray] = None,
    pin_line=None,
    carrier_alpha: float = 1.0,
):
    fig = plt.figure(figsize=(10, 8), facecolor="white")
    ax = fig.add_subplot(111, projection="3d")
    add_mesh(ax, carrier, "#4c8fc7", alpha=carrier_alpha)
    arrays = [carrier]
    if receiver is not None:
        add_mesh(ax, receiver, "#ef8a3c", edge="#8a3c11", width=0.18)
        arrays.append(receiver)
    if pin_line is not None:
        start, end = np.asarray(pin_line[0]), np.asarray(pin_line[1])
        ax.plot(
            [start[0], end[0]],
            [start[1], end[1]],
            [start[2], end[2]],
            color="#d62828",
            linewidth=4.0,
            label="1.75 mm retaining pin",
        )
        ax.legend(loc="upper right")
    set_equal_limits(ax, arrays)
    ax.set_title(title, fontsize=15, pad=16)
    ax.set_xlabel("X / mm")
    ax.set_ylabel("Y / mm")
    ax.set_zlabel("Z / mm")
    ax.set_proj_type("ortho")
    ax.grid(True, alpha=0.28)
    ax.view_init(elev=elev, azim=azim)
    fig.tight_layout()
    fig.savefig(output, dpi=170, bbox_inches="tight")
    plt.close(fig)


def save_strap_diagram(output: Path, p: dict):
    fig, ax = plt.subplots(figsize=(8, 10), facecolor="white")
    width = p["base_width"]
    length = p["base_length"]
    base = FancyBboxPatch(
        (-width / 2.0, -length / 2.0),
        width,
        length,
        boxstyle="round,pad=0,rounding_size=3",
        facecolor="#4c8fc7",
        edgecolor="#17324d",
        linewidth=2.0,
    )
    ax.add_patch(base)

    channel_width = p["receiver_width"] + 2.0 * p["side_clearance"]
    ax.add_patch(
        Rectangle(
            (-channel_width / 2.0, -length / 2.0),
            channel_width,
            p["receiver_length"] + p["distal_edge_recess"],
            facecolor="#ef8a3c",
            edgecolor="#8a3c11",
            alpha=0.75,
            label="receiver/channel",
        )
    )

    for x in (-p["strap_slot_x"], p["strap_slot_x"]):
        for y in (-p["strap_slot_y"], p["strap_slot_y"]):
            ax.add_patch(
                Rectangle(
                    (
                        x - p["strap_slot_width"] / 2.0,
                        y - p["strap_slot_length"] / 2.0,
                    ),
                    p["strap_slot_width"],
                    p["strap_slot_length"],
                    facecolor="#20252b",
                    edgecolor="none",
                )
            )

    for y, label in ((p["strap_slot_y"], "proximal strap"),
                     (-p["strap_slot_y"], "distal strap")):
        ax.annotate(
            label + " wraps behind the arm",
            xy=(p["strap_slot_x"], y),
            xytext=(width / 2.0 + 13.0, y),
            va="center",
            arrowprops={"arrowstyle": "->", "linewidth": 1.6},
        )

    ax.annotate(
        "+Y PROXIMAL\nupper arm: shoulder / forearm: elbow",
        xy=(0.0, length / 2.0),
        xytext=(0.0, length / 2.0 + 12.0),
        ha="center",
        va="bottom",
        arrowprops={"arrowstyle": "->", "linewidth": 2.0},
    )
    ax.text(
        0.0,
        -length / 2.0 - 9.0,
        "-Y DISTAL / receiver insertion end",
        ha="center",
        va="top",
    )
    ax.set_xlim(-width / 2.0 - 8.0, width / 2.0 + 50.0)
    ax.set_ylim(-length / 2.0 - 16.0, length / 2.0 + 26.0)
    ax.set_aspect("equal")
    ax.set_title("Outward-face view and two-strap routing", pad=14)
    ax.set_xlabel("X / mm")
    ax.set_ylabel("Y / mm")
    ax.grid(True, alpha=0.18)
    fig.tight_layout()
    fig.savefig(output, dpi=170, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    manifest = json.loads((ASSET_DIR / "parameters.json").read_text())
    p = manifest["parameters"]
    d = manifest["derived"]
    carrier = load_binary_stl(ASSET_DIR / "wmet_receiver_strap_mount.stl")
    coupon = load_binary_stl(ASSET_DIR / "wmet_receiver_fit_coupon.stl")

    receiver_center = (
        0.0,
        (d["receiver_distal_y"] + d["receiver_proximal_y"]) / 2.0,
        d["receiver_face_z"] + p["receiver_thickness"] / 2.0,
    )
    receiver = cuboid_triangles(
        receiver_center,
        (p["receiver_width"], p["receiver_length"], p["receiver_thickness"]),
    )
    pin_y = d["distal_base_edge_y"] + 1.2
    pin_z = d["receiver_face_z"] + p["receiver_thickness"] / 2.0
    pin = ((-17.0, pin_y, pin_z), (17.0, pin_y, pin_z))

    save_3d_view(
        ASSET_DIR / "preview_01_isometric.png",
        carrier,
        title="Compact marker-free WEMT strap mount — body side",
        elev=28,
        azim=-55,
    )
    save_3d_view(
        ASSET_DIR / "preview_02_body_side_with_receiver.png",
        carrier,
        title="Receiver seated between two foam-covered body rails",
        elev=72,
        azim=-90,
        receiver=receiver,
        pin_line=pin,
        carrier_alpha=0.34,
    )
    save_3d_view(
        ASSET_DIR / "preview_03_distal_slide_end.png",
        carrier,
        title="Distal insertion end and retaining-pin location",
        elev=7,
        azim=-90,
        receiver=receiver,
        pin_line=pin,
        carrier_alpha=0.34,
    )
    save_strap_diagram(ASSET_DIR / "preview_04_strap_routing.png", p)
    save_3d_view(
        ASSET_DIR / "preview_05_fit_coupon.png",
        coupon,
        title="20 mm fit coupon — print this before the full mount",
        elev=24,
        azim=-55,
    )
    print(f"Wrote previews to {ASSET_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
