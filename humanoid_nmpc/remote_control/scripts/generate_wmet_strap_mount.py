#!/usr/bin/env python3
"""Generate a compact, marker-free WEMT arm strap mount.

The carrier keeps the proven slide-channel geometry from the visual-marker
mount, but removes the ArUco plate.  Two 25 mm hook-and-loop straps pass
through four slots and clamp two foam-covered support rails to the arm.

Coordinate convention:

* +Y is proximal (towards the shoulder on the upper arm, towards the elbow on
  the forearm).
* -Y is distal and is the receiver insertion/charging-port end.
* Z=0 is the outward face placed on the print bed; body-contact features are
  at +Z.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path

try:
    import cadquery as cq
except ImportError as exc:  # pragma: no cover - depends on local CAD tooling
    raise SystemExit(
        "CadQuery is required. Install it in a separate CAD environment with "
        "`python3 -m pip install cadquery`, then run this script again."
    ) from exc


@dataclass(frozen=True)
class StrapMountParameters:
    # Compact carrier body.
    base_width: float = 68.0
    base_length: float = 84.0
    base_thickness: float = 2.6
    base_corner_radius: float = 3.0

    # Measured WEMT enclosure, millimetres.
    receiver_width: float = 23.0
    receiver_thickness: float = 13.0
    receiver_length: float = 70.0

    # Sliding fit.  side_clearance is applied independently to each side.
    side_clearance: float = 0.35
    thickness_clearance: float = 0.40
    # The switch protrudes about 1 mm; this leaves about 1.5 mm real clearance.
    switch_face_gap: float = 2.5
    wall_thickness: float = 2.4
    lower_lip_overhang: float = 0.8
    upper_lip_overhang: float = 1.3
    lip_thickness: float = 2.0

    # Receiver insertion and removable non-metal retaining pin.
    distal_edge_recess: float = 3.0
    length_clearance: float = 0.5
    retaining_pin_diameter: float = 2.1

    # Two independent 25 mm hook-and-loop straps.
    strap_width: float = 25.0
    strap_slot_length: float = 27.0
    strap_slot_width: float = 3.5
    strap_slot_x: float = 27.0
    strap_slot_y: float = 22.0

    # Printed rails are covered with 3 mm EVA.  The foam contact surface sits
    # above the receiver so the enclosure and switch never press on the skin.
    support_rail_width: float = 8.0
    support_rail_length: float = 70.0
    support_rail_x: float = 17.0
    support_rail_printed_height: float = 21.0
    recommended_foam_thickness: float = 3.0

    # Short sliding-fit coupon.
    coupon_width: float = 38.0
    coupon_length: float = 20.0


def _rounded_base(width: float, length: float, thickness: float, radius: float):
    base = cq.Workplane("XY").box(
        width, length, thickness, centered=(True, True, False)
    )
    return base.edges("|Z").fillet(radius)


def _channel_dimensions(p: StrapMountParameters) -> dict[str, float]:
    inner_width = p.receiver_width + 2.0 * p.side_clearance
    receiver_face_z = p.base_thickness + p.switch_face_gap
    receiver_outer_z = receiver_face_z + p.receiver_thickness
    upper_lip_under_z = receiver_outer_z + p.thickness_clearance
    channel_top_z = upper_lip_under_z + p.lip_thickness

    distal_base_edge_y = -p.base_length / 2.0
    receiver_distal_y = distal_base_edge_y + p.distal_edge_recess
    receiver_proximal_y = receiver_distal_y + p.receiver_length
    stop_inner_y = receiver_proximal_y + p.length_clearance
    stop_outer_y = stop_inner_y + p.wall_thickness
    channel_center_y = (distal_base_edge_y + stop_outer_y) / 2.0
    channel_length = stop_outer_y - distal_base_edge_y

    return {
        "inner_width": inner_width,
        "receiver_face_z": receiver_face_z,
        "receiver_outer_z": receiver_outer_z,
        "upper_lip_under_z": upper_lip_under_z,
        "channel_top_z": channel_top_z,
        "distal_base_edge_y": distal_base_edge_y,
        "receiver_distal_y": receiver_distal_y,
        "receiver_proximal_y": receiver_proximal_y,
        "stop_inner_y": stop_inner_y,
        "stop_outer_y": stop_outer_y,
        "channel_center_y": channel_center_y,
        "channel_length": channel_length,
    }


def _add_slide_channel(base, p: StrapMountParameters, *, coupon: bool):
    dims = _channel_dimensions(p)
    inner_width = dims["inner_width"]

    if coupon:
        channel_length = p.coupon_length
        channel_center_y = 0.0
        include_stop = False
    else:
        channel_length = dims["channel_length"]
        channel_center_y = dims["channel_center_y"]
        include_stop = True

    wall_height = dims["channel_top_z"] - p.base_thickness
    wall_x = inner_width / 2.0 + p.wall_thickness / 2.0
    for sign in (-1.0, 1.0):
        wall = (
            cq.Workplane("XY")
            .box(
                p.wall_thickness,
                channel_length,
                wall_height,
                centered=(True, True, False),
            )
            .translate((sign * wall_x, channel_center_y, p.base_thickness))
        )
        base = base.union(wall)

        # Support only the enclosure edges; keep its central switch face clear.
        shelf_x = sign * (inner_width / 2.0 - p.lower_lip_overhang / 2.0)
        lower_shelf = (
            cq.Workplane("XY")
            .box(
                p.lower_lip_overhang,
                channel_length,
                p.switch_face_gap,
                centered=(True, True, False),
            )
            .translate((shelf_x, channel_center_y, p.base_thickness))
        )
        base = base.union(lower_shelf)

        upper_lip_x = sign * (
            inner_width / 2.0 - p.upper_lip_overhang / 2.0
        )
        upper_lip = (
            cq.Workplane("XY")
            .box(
                p.upper_lip_overhang,
                channel_length,
                p.lip_thickness,
                centered=(True, True, False),
            )
            .translate(
                (upper_lip_x, channel_center_y, dims["upper_lip_under_z"])
            )
        )
        base = base.union(upper_lip)

    if include_stop:
        stop = (
            cq.Workplane("XY")
            .box(
                inner_width + 2.0 * p.wall_thickness,
                p.wall_thickness,
                wall_height,
                centered=(True, True, False),
            )
            .translate(
                (
                    0.0,
                    (dims["stop_inner_y"] + dims["stop_outer_y"]) / 2.0,
                    p.base_thickness,
                )
            )
        )
        base = base.union(stop)

        # A 1.75 mm filament pin lies just outside the receiver's distal face.
        pin_y = dims["distal_base_edge_y"] + 1.2
        pin_z = dims["receiver_face_z"] + p.receiver_thickness / 2.0
        pin_hole = (
            cq.Workplane("YZ")
            .center(pin_y, pin_z)
            .circle(p.retaining_pin_diameter / 2.0)
            .extrude(inner_width + 2.0 * p.wall_thickness + 2.0, both=True)
        )
        base = base.cut(pin_hole)

    return base


def _cut_strap_slots(base, p: StrapMountParameters):
    for x in (-p.strap_slot_x, p.strap_slot_x):
        for y in (-p.strap_slot_y, p.strap_slot_y):
            slot = (
                cq.Workplane("XY")
                .workplane(offset=-0.5)
                .center(x, y)
                .rect(p.strap_slot_width, p.strap_slot_length)
                .extrude(p.base_thickness + 1.0)
            )
            base = base.cut(slot)
    return base


def _add_support_rails(model, p: StrapMountParameters):
    for x in (-p.support_rail_x, p.support_rail_x):
        rail = (
            cq.Workplane("XY")
            .box(
                p.support_rail_width,
                p.support_rail_length,
                p.support_rail_printed_height,
                centered=(True, True, False),
            )
            .translate((x, 0.0, p.base_thickness))
        )
        rail = rail.edges("|Y").fillet(1.5)
        model = model.union(rail)
    return model


def _add_proximal_arrow(model, p: StrapMountParameters):
    points = [
        (-3.0, -2.0),
        (0.0, 3.0),
        (3.0, -2.0),
        (1.0, -2.0),
        (1.0, -5.0),
        (-1.0, -5.0),
        (-1.0, -2.0),
    ]
    arrow = (
        cq.Workplane("XY")
        .workplane(offset=p.base_thickness)
        .center(0.0, p.base_length / 2.0 - 3.5)
        .polyline(points)
        .close()
        .extrude(0.7)
    )
    return model.union(arrow)


def make_fit_coupon(p: StrapMountParameters):
    base = _rounded_base(
        p.coupon_width, p.coupon_length, p.base_thickness, radius=2.0
    )
    return _add_slide_channel(base, p, coupon=True)


def make_strap_mount(p: StrapMountParameters):
    base = _rounded_base(
        p.base_width,
        p.base_length,
        p.base_thickness,
        p.base_corner_radius,
    )
    base = _cut_strap_slots(base, p)
    model = _add_slide_channel(base, p, coupon=False)
    model = _add_support_rails(model, p)
    return _add_proximal_arrow(model, p)


def _export(shape, output_dir: Path, stem: str) -> None:
    cq.exporters.export(shape, str(output_dir / f"{stem}.step"))
    cq.exporters.export(
        shape,
        str(output_dir / f"{stem}.stl"),
        tolerance=0.05,
        angularTolerance=0.1,
    )


def _write_manifest(output_dir: Path, p: StrapMountParameters) -> None:
    dims = _channel_dimensions(p)
    manifest = {
        "units": "mm",
        "model": "compact marker-free WEMT arm strap mount",
        "coordinate_convention": {
            "+Y": "proximal: shoulder on upper arm, elbow on forearm",
            "-Y": "distal receiver insertion and charging-port end",
            "Z=0": "outward face placed on print bed",
            "+Z": "body-contact side",
        },
        "parameters": asdict(p),
        "derived": dims,
        "overall": {
            "printed_width": p.base_width,
            "printed_length": p.base_length,
            "printed_height": p.base_thickness + p.support_rail_printed_height,
            "height_with_uncompressed_foam": (
                p.base_thickness
                + p.support_rail_printed_height
                + p.recommended_foam_thickness
            ),
        },
        "printing": {
            "recommended_material": "PETG",
            "prototype_material": "PLA",
            "layer_height": 0.2,
            "perimeters": 3,
            "infill_percent": "30-40",
            "orientation": "print flat outward base face on build plate",
            "supports": "none expected",
        },
    }
    (output_dir / "parameters.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def _validate(shape, name: str) -> None:
    solid = shape.val()
    if not solid.isValid():
        raise RuntimeError(f"{name} is not a valid solid")
    if len(shape.solids().vals()) != 1:
        raise RuntimeError(f"{name} must contain exactly one solid")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    default_output = (
        Path(__file__).resolve().parents[1]
        / "assets"
        / "wmet_strap_mount"
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=default_output,
        help=f"output directory (default: {default_output})",
    )
    parser.add_argument(
        "--side-clearance",
        type=float,
        default=StrapMountParameters.side_clearance,
        help="clearance on each receiver side in mm",
    )
    parser.add_argument(
        "--thickness-clearance",
        type=float,
        default=StrapMountParameters.thickness_clearance,
        help="clearance above the receiver in mm",
    )
    args = parser.parse_args()

    p = StrapMountParameters(
        side_clearance=args.side_clearance,
        thickness_clearance=args.thickness_clearance,
    )
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    coupon = make_fit_coupon(p)
    mount = make_strap_mount(p)
    _validate(coupon, "fit coupon")
    _validate(mount, "strap mount")
    _export(coupon, output_dir, "wmet_receiver_fit_coupon")
    _export(mount, output_dir, "wmet_receiver_strap_mount")
    _write_manifest(output_dir, p)

    bbox = mount.val().BoundingBox()
    print(f"Generated CAD files in {output_dir}")
    print("  wmet_receiver_fit_coupon.step/.stl")
    print("  wmet_receiver_strap_mount.step/.stl")
    print("  parameters.json")
    print(
        "  printed bounding box: "
        f"{bbox.xlen:.1f} x {bbox.ylen:.1f} x {bbox.zlen:.1f} mm"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
