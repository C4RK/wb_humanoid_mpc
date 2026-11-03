#!/usr/bin/env python3
"""Generate a printable WEMT receiver + ArUco marker carrier.

The model is intentionally parameterized so the receiver clearance can be
adjusted after printing the small fit coupon.  CadQuery is used because it can
export both STEP solids (for SolidWorks/FreeCAD) and STL meshes (for slicing).

Coordinate convention used by the model:

* +Y is proximal (towards shoulder for ID10, towards elbow for ID20).
* -Y is distal and is the receiver insertion/charging-port end.
* Z=0 is the flat marker face; receiver and body-contact features are at +Z.
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
class MountParameters:
    # Printed marker and carrier plate.
    marker_size: float = 77.5
    plate_width: float = 96.0
    plate_length: float = 90.0
    plate_thickness: float = 2.6
    plate_corner_radius: float = 4.0

    # The measured WEMT enclosure dimensions in millimetres.
    receiver_width: float = 23.0
    receiver_thickness: float = 13.0
    receiver_length: float = 70.0

    # Sliding fit.  side_clearance is applied independently to each side.
    side_clearance: float = 0.35
    thickness_clearance: float = 0.40
    # The button protrudes about 1 mm, leaving roughly 1.5 mm of real clearance.
    switch_face_gap: float = 2.5
    wall_thickness: float = 2.4
    lower_lip_overhang: float = 0.8
    upper_lip_overhang: float = 1.3
    lip_thickness: float = 2.0

    # The receiver sits 3 mm inside the distal plate edge.  A piece of 1.75 mm
    # printer filament through the aligned holes is the removable retaining pin.
    distal_edge_recess: float = 3.0
    length_clearance: float = 0.5
    retaining_pin_diameter: float = 2.1

    # Two independent 25 mm hook-and-loop straps use four slots.
    strap_width: float = 25.0
    strap_slot_length: float = 27.0
    strap_slot_width: float = 3.5
    strap_slot_x: float = 43.5
    strap_slot_y: float = 23.0

    # Printed rails receive 3 mm EVA foam after printing.  Their final contact
    # surface is above the receiver, so the enclosure does not press on skin.
    support_rail_width: float = 13.0
    support_rail_length: float = 70.0
    support_rail_x: float = 31.0
    support_rail_printed_height: float = 18.0
    recommended_foam_thickness: float = 3.0

    # A short coupon is printed before the full carrier.
    coupon_width: float = 38.0
    coupon_length: float = 20.0


def _rounded_plate(width: float, length: float, thickness: float, radius: float):
    plate = cq.Workplane("XY").box(
        width, length, thickness, centered=(True, True, False)
    )
    return plate.edges("|Z").fillet(radius)


def _channel_dimensions(p: MountParameters) -> dict[str, float]:
    inner_width = p.receiver_width + 2.0 * p.side_clearance
    receiver_face_z = p.plate_thickness + p.switch_face_gap
    receiver_outer_z = receiver_face_z + p.receiver_thickness
    upper_lip_under_z = receiver_outer_z + p.thickness_clearance
    channel_top_z = upper_lip_under_z + p.lip_thickness

    distal_plate_edge_y = -p.plate_length / 2.0
    receiver_distal_y = distal_plate_edge_y + p.distal_edge_recess
    receiver_proximal_y = receiver_distal_y + p.receiver_length
    stop_inner_y = receiver_proximal_y + p.length_clearance
    stop_outer_y = stop_inner_y + p.wall_thickness
    channel_center_y = (distal_plate_edge_y + stop_outer_y) / 2.0
    channel_length = stop_outer_y - distal_plate_edge_y

    return {
        "inner_width": inner_width,
        "receiver_face_z": receiver_face_z,
        "receiver_outer_z": receiver_outer_z,
        "upper_lip_under_z": upper_lip_under_z,
        "channel_top_z": channel_top_z,
        "distal_plate_edge_y": distal_plate_edge_y,
        "receiver_distal_y": receiver_distal_y,
        "receiver_proximal_y": receiver_proximal_y,
        "stop_inner_y": stop_inner_y,
        "stop_outer_y": stop_outer_y,
        "channel_center_y": channel_center_y,
        "channel_length": channel_length,
    }


def _add_slide_channel(base, p: MountParameters, *, coupon: bool):
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

    wall_height = dims["channel_top_z"] - p.plate_thickness
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
            .translate(
                (
                    sign * wall_x,
                    channel_center_y,
                    p.plate_thickness,
                )
            )
        )
        base = base.union(wall)

        # A narrow lower shelf supports only the enclosure edges.  The central
        # switch face stays clear while the 2 mm button travels through the slot.
        shelf_x = sign * (inner_width / 2.0 - p.lower_lip_overhang / 2.0)
        lower_shelf = (
            cq.Workplane("XY")
            .box(
                p.lower_lip_overhang,
                channel_length,
                p.switch_face_gap,
                centered=(True, True, False),
            )
            .translate((shelf_x, channel_center_y, p.plate_thickness))
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
                (
                    upper_lip_x,
                    channel_center_y,
                    dims["upper_lip_under_z"],
                )
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
                    p.plate_thickness,
                )
            )
        )
        base = base.union(stop)

        # Aligned holes accept a short 1.75 mm filament retaining pin after the
        # receiver is inserted.  The pin sits distal to the receiver end face.
        pin_y = dims["distal_plate_edge_y"] + 1.2
        pin_z = (
            dims["receiver_face_z"]
            + p.receiver_thickness / 2.0
        )
        pin_hole = (
            cq.Workplane("YZ")
            .center(pin_y, pin_z)
            .circle(p.retaining_pin_diameter / 2.0)
            .extrude(inner_width + 2.0 * p.wall_thickness + 2.0, both=True)
        )
        base = base.cut(pin_hole)

    return base


def make_fit_coupon(p: MountParameters):
    base = _rounded_plate(
        p.coupon_width,
        p.coupon_length,
        p.plate_thickness,
        radius=2.0,
    )
    return _add_slide_channel(base, p, coupon=True)


def _cut_strap_slots(plate, p: MountParameters):
    for x in (-p.strap_slot_x, p.strap_slot_x):
        for y in (-p.strap_slot_y, p.strap_slot_y):
            slot = (
                cq.Workplane("XY")
                .workplane(offset=-0.5)
                .center(x, y)
                .rect(p.strap_slot_width, p.strap_slot_length)
                .extrude(p.plate_thickness + 1.0)
            )
            plate = plate.cut(slot)
    return plate


def _add_support_rails(model, p: MountParameters):
    for x in (-p.support_rail_x, p.support_rail_x):
        rail = (
            cq.Workplane("XY")
            .box(
                p.support_rail_width,
                p.support_rail_length,
                p.support_rail_printed_height,
                centered=(True, True, False),
            )
            .translate((x, 0.0, p.plate_thickness))
        )
        # Round the body-facing longitudinal edges; EVA foam is added later.
        rail = rail.edges("|Y").fillet(2.0)
        model = model.union(rail)
    return model


def _add_proximal_arrow(model, p: MountParameters):
    # Raised arrow on the body side, outside the receiver stop.  It identifies
    # +Y (proximal) even after the printed paper marker is attached.
    points = [
        (-3.5, -2.0),
        (0.0, 3.5),
        (3.5, -2.0),
        (1.2, -2.0),
        (1.2, -6.0),
        (-1.2, -6.0),
        (-1.2, -2.0),
    ]
    arrow = (
        cq.Workplane("XY")
        .workplane(offset=p.plate_thickness)
        .center(0.0, p.plate_length / 2.0 - 5.5)
        .polyline(points)
        .close()
        .extrude(0.7)
    )
    return model.union(arrow)


def make_full_mount(p: MountParameters):
    plate = _rounded_plate(
        p.plate_width,
        p.plate_length,
        p.plate_thickness,
        p.plate_corner_radius,
    )
    plate = _cut_strap_slots(plate, p)
    model = _add_slide_channel(plate, p, coupon=False)
    model = _add_support_rails(model, p)
    model = _add_proximal_arrow(model, p)
    return model


def _export(shape, output_dir: Path, stem: str) -> None:
    cq.exporters.export(shape, str(output_dir / f"{stem}.step"))
    cq.exporters.export(
        shape,
        str(output_dir / f"{stem}.stl"),
        tolerance=0.05,
        angularTolerance=0.1,
    )


def _write_manifest(output_dir: Path, p: MountParameters) -> None:
    dims = _channel_dimensions(p)
    manifest = {
        "units": "mm",
        "coordinate_convention": {
            "+Y": "proximal: shoulder for ID10, elbow for ID20",
            "-Y": "distal and receiver insertion/charging-port end",
            "Z=0": "flat ArUco marker face",
        },
        "parameters": asdict(p),
        "derived": dims,
        "printing": {
            "recommended_material": "PETG",
            "prototype_material": "PLA",
            "layer_height": 0.2,
            "perimeters": 3,
            "infill_percent": "25-35",
            "orientation": "print flat marker face on the build plate",
            "supports": "none expected",
        },
    }
    (output_dir / "parameters.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    default_output = (
        Path(__file__).resolve().parents[1]
        / "assets"
        / "wmet_marker_mount"
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
        default=MountParameters.side_clearance,
        help="clearance on each receiver side in mm",
    )
    parser.add_argument(
        "--thickness-clearance",
        type=float,
        default=MountParameters.thickness_clearance,
        help="clearance above the receiver in mm",
    )
    args = parser.parse_args()

    p = MountParameters(
        side_clearance=args.side_clearance,
        thickness_clearance=args.thickness_clearance,
    )
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    coupon = make_fit_coupon(p)
    mount = make_full_mount(p)
    _export(coupon, output_dir, "wmet_receiver_fit_coupon")
    _export(mount, output_dir, "wmet_aruco_arm_mount")
    _write_manifest(output_dir, p)

    print(f"Generated CAD files in {output_dir}")
    print("  wmet_receiver_fit_coupon.step/.stl")
    print("  wmet_aruco_arm_mount.step/.stl")
    print("  parameters.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
