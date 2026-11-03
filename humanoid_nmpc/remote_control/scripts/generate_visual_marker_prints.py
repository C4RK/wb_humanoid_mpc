#!/usr/bin/env python3
"""Generate print-ready visual-reference targets for the two-phone protocol.

The generated files use OpenCV DICT_4X4_50 and are laid out on physical A4
pages.  The 320 x 240 mm ChArUco pattern is split exactly at the middle
vertical square boundary so it can be assembled from two A4 sheets.  ArUco
IDs 10 and 20 are placed together on a separate A4 page; each marker has a
60 mm black outer square and an 80 x 80 mm cutting area.

Run this script in the ``unitree-rl`` conda environment, which supplies the
OpenCV aruco module used by the rest of the visual-validation tools.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


MM_PER_INCH = 25.4
A4_WIDTH_MM = 210.0
A4_HEIGHT_MM = 297.0

CHARUCO_SQUARES_X = 8
CHARUCO_SQUARES_Y = 6
CHARUCO_SQUARE_MM = 40.0
CHARUCO_MARKER_MM = 30.0

BODY_MARKER_MM = 60.0
BODY_CUT_SQUARE_MM = 80.0
BODY_MARKER_IDS = (10, 20)


def mm_to_px(length_mm: float, dpi: int) -> int:
    return int(round(length_mm / MM_PER_INCH * dpi))


def load_font(size_px: int) -> ImageFont.ImageFont:
    candidates = (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf",
    )
    for path in candidates:
        if Path(path).exists():
            return ImageFont.truetype(path, size_px)
    return ImageFont.load_default()


def centered_text(draw: ImageDraw.ImageDraw, xy: tuple[int, int], text: str,
                  font: ImageFont.ImageFont, fill: int = 0) -> None:
    bbox = draw.textbbox((0, 0), text, font=font)
    width = bbox[2] - bbox[0]
    draw.text((xy[0] - width // 2, xy[1]), text, font=font, fill=fill)


def draw_crop_marks(draw: ImageDraw.ImageDraw, box: tuple[int, int, int, int],
                    dpi: int) -> None:
    """Draw crop marks outside a box without touching the calibration image."""
    x0, y0, x1, y1 = box
    gap = mm_to_px(2.0, dpi)
    length = mm_to_px(8.0, dpi)
    line_width = max(1, mm_to_px(0.25, dpi))
    gray = 110
    for x in (x0, x1):
        draw.line((x, y0 - gap - length, x, y0 - gap), fill=gray,
                  width=line_width)
        draw.line((x, y1 + gap, x, y1 + gap + length), fill=gray,
                  width=line_width)
    for y in (y0, y1):
        draw.line((x0 - gap - length, y, x0 - gap, y), fill=gray,
                  width=line_width)
        draw.line((x1 + gap, y, x1 + gap + length, y), fill=gray,
                  width=line_width)


def save_a4_pdf(page: Image.Image, path: Path, dpi: int) -> None:
    page.convert("RGB").save(path, "PDF", resolution=float(dpi))


def make_charuco(dictionary, output_dir: Path, dpi: int) -> None:
    board_width_mm = CHARUCO_SQUARES_X * CHARUCO_SQUARE_MM
    board_height_mm = CHARUCO_SQUARES_Y * CHARUCO_SQUARE_MM
    board_width_px = mm_to_px(board_width_mm, dpi)
    board_height_px = mm_to_px(board_height_mm, dpi)

    board = cv2.aruco.CharucoBoard(
        (CHARUCO_SQUARES_X, CHARUCO_SQUARES_Y),
        CHARUCO_SQUARE_MM,
        CHARUCO_MARKER_MM,
        dictionary,
    )
    board_image = board.generateImage(
        (board_width_px, board_height_px), marginSize=0, borderBits=1
    )
    full = Image.fromarray(board_image)
    full.save(output_dir / "charuco_8x6_320x240mm_full_preview.png", dpi=(dpi, dpi))

    a4_width_px = mm_to_px(A4_WIDTH_MM, dpi)
    a4_height_px = mm_to_px(A4_HEIGHT_MM, dpi)
    half_width_px = board_width_px // 2
    font = load_font(mm_to_px(4.0, dpi))
    small_font = load_font(mm_to_px(3.0, dpi))

    for half_index, label in enumerate(("LEFT", "RIGHT")):
        page = Image.new("L", (a4_width_px, a4_height_px), 255)
        draw = ImageDraw.Draw(page)
        x0 = (a4_width_px - half_width_px) // 2
        y0 = (a4_height_px - board_height_px) // 2
        x1 = x0 + half_width_px
        y1 = y0 + board_height_px
        source_x0 = half_index * half_width_px
        half = full.crop((source_x0, 0, source_x0 + half_width_px,
                          board_height_px))
        page.paste(half, (x0, y0))
        draw_crop_marks(draw, (x0, y0, x1, y1), dpi)
        centered_text(
            draw,
            (a4_width_px // 2, mm_to_px(5.0, dpi)),
            f"ChArUco 8x6 - {label} HALF - PRINT AT 100%",
            font,
        )
        centered_text(
            draw,
            (a4_width_px // 2, a4_height_px - mm_to_px(10.0, dpi)),
            "Each square must measure 40.0 mm",
            small_font,
        )
        filename = f"charuco_A4_{label.lower()}.pdf"
        save_a4_pdf(page, output_dir / filename, dpi)


def make_body_markers(dictionary, output_dir: Path, dpi: int) -> None:
    a4_width_px = mm_to_px(A4_WIDTH_MM, dpi)
    a4_height_px = mm_to_px(A4_HEIGHT_MM, dpi)
    cut_px = mm_to_px(BODY_CUT_SQUARE_MM, dpi)
    marker_px = mm_to_px(BODY_MARKER_MM, dpi)
    gap_px = mm_to_px(20.0, dpi)
    combined_width = cut_px * 2 + gap_px
    first_x = (a4_width_px - combined_width) // 2
    y0 = (a4_height_px - cut_px) // 2
    font = load_font(mm_to_px(5.0, dpi))
    small_font = load_font(mm_to_px(3.2, dpi))

    page = Image.new("L", (a4_width_px, a4_height_px), 255)
    draw = ImageDraw.Draw(page)

    for marker_index, marker_id in enumerate(BODY_MARKER_IDS):
        x0 = first_x + marker_index * (cut_px + gap_px)
        x1 = x0 + cut_px
        y1 = y0 + cut_px
        marker = cv2.aruco.generateImageMarker(
            dictionary, marker_id, marker_px, borderBits=1
        )
        marker_image = Image.fromarray(marker)
        marker_x = x0 + (cut_px - marker_px) // 2
        marker_y = y0 + (cut_px - marker_px) // 2
        page.paste(marker_image, (marker_x, marker_y))
        draw_crop_marks(draw, (x0, y0, x1, y1), dpi)
        centered_text(
            draw,
            ((x0 + x1) // 2, y0 - mm_to_px(13.0, dpi)),
            f"ID {marker_id}",
            font,
        )
        role = "UPPER ARM" if marker_id == 10 else "FOREARM"
        centered_text(
            draw,
            ((x0 + x1) // 2, y1 + mm_to_px(5.0, dpi)),
            role,
            small_font,
        )
        marker_image.save(
            output_dir / f"aruco_id{marker_id}_60mm.png", dpi=(dpi, dpi)
        )

    centered_text(
        draw,
        (a4_width_px // 2, mm_to_px(18.0, dpi)),
        "DICT_4X4_50 - PRINT AT 100% / ACTUAL SIZE",
        font,
    )
    centered_text(
        draw,
        (a4_width_px // 2, a4_height_px - mm_to_px(20.0, dpi)),
        "Black marker square: 60.0 mm   Cut area: 80 x 80 mm",
        small_font,
    )
    save_a4_pdf(page, output_dir / "aruco_ID10_ID20_A4.pdf", dpi)


def verify_markers(dictionary, output_dir: Path) -> None:
    detector = cv2.aruco.ArucoDetector(
        dictionary, cv2.aruco.DetectorParameters()
    )
    found = []
    for marker_id in BODY_MARKER_IDS:
        image = cv2.imread(str(output_dir / f"aruco_id{marker_id}_60mm.png"))
        # A standalone marker PNG ends exactly at the black outer border.
        # Add a synthetic white quiet zone for the detector self-check; the
        # print-ready A4 page already supplies a physical 10 mm quiet zone.
        quiet = max(16, image.shape[0] // 6)
        check_image = cv2.copyMakeBorder(
            image, quiet, quiet, quiet, quiet,
            cv2.BORDER_CONSTANT, value=(255, 255, 255),
        )
        _corners, ids, _rejected = detector.detectMarkers(check_image)
        detected = [] if ids is None else ids.reshape(-1).tolist()
        if marker_id not in detected:
            raise RuntimeError(
                f"self-check failed: ID {marker_id} was not detected"
            )
        found.extend(detected)

    board_image = cv2.imread(
        str(output_dir / "charuco_8x6_320x240mm_full_preview.png")
    )
    _corners, ids, _rejected = detector.detectMarkers(board_image)
    board_count = 0 if ids is None else len(ids)
    if board_count < 20:
        raise RuntimeError(
            f"self-check failed: only {board_count} ChArUco markers detected"
        )
    print(f"Self-check passed: body IDs {sorted(set(found))}; "
          f"ChArUco markers detected={board_count}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=(Path(__file__).resolve().parent.parent / "assets" /
                 "visual_markers"),
    )
    parser.add_argument("--dpi", type=int, default=300)
    args = parser.parse_args()
    if args.dpi < 200:
        parser.error("--dpi must be at least 200 for reliable printing")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    make_charuco(dictionary, args.output_dir, args.dpi)
    make_body_markers(dictionary, args.output_dir, args.dpi)
    verify_markers(dictionary, args.output_dir)
    print(f"Print files written to {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
