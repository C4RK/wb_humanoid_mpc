#!/usr/bin/env python3
"""Generate a clearly labelled synthetic EM/vision comparison example."""

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


WIDTH, HEIGHT = 1600, 1260
FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
OUT_PATH = (
    Path(__file__).resolve().parents[1]
    / "docs"
    / "figures"
    / "em_visual_comparison_example.png"
)

COLORS = {
    "text": "#202124",
    "muted": "#5f6368",
    "grid": "#dfe3e7",
    "border": "#9aa0a6",
    "vision": "#1a73e8",
    "em": "#d93025",
    "upper": "#7b1fa2",
    "forearm": "#f57c00",
    "elbow": "#188038",
    "warning": "#b3261e",
}


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    path = (
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
        if bold
        else FONT_PATH
    )
    if not Path(path).exists():
        path = FONT_PATH
    return ImageFont.truetype(path, size)


def centered(draw: ImageDraw.ImageDraw, xy: tuple[int, int], text: str, fnt, fill):
    box = draw.textbbox((0, 0), text, font=fnt)
    draw.text((xy[0] - (box[2] - box[0]) / 2, xy[1]), text, font=fnt, fill=fill)


def plot_panel(
    draw: ImageDraw.ImageDraw,
    top: int,
    title: str,
    y_max: float,
    series: list[tuple[str, str, list[float]]],
) -> None:
    left, right = 150, 1530
    plot_top, plot_bottom = top + 45, top + 245
    draw.text((left, top), title, font=font(25, True), fill=COLORS["text"])
    draw.rectangle((left, plot_top, right, plot_bottom), fill="#fafafa", outline=COLORS["border"], width=2)

    ticks = [0, y_max / 2, y_max]
    for value in ticks:
        y = plot_bottom - (value / y_max) * (plot_bottom - plot_top)
        draw.line((left, y, right, y), fill=COLORS["grid"], width=2)
        label = f"{value:g}"
        box = draw.textbbox((0, 0), label, font=font(18))
        draw.text((left - 15 - (box[2] - box[0]), y - 12), label, font=font(18), fill=COLORS["muted"])

    for _, color, values in series:
        points = []
        for index, value in enumerate(values):
            x = left + index * (right - left) / (len(values) - 1)
            y = plot_bottom - (value / y_max) * (plot_bottom - plot_top)
            points.append((x, y))
        draw.line(points, fill=color, width=5, joint="curve")

    draw.text((35, (plot_top + plot_bottom) / 2 - 12), "角度 (°)", font=font(19), fill=COLORS["text"])


def main() -> None:
    image = Image.new("RGB", (WIDTH, HEIGHT), "white")
    draw = ImageDraw.Draw(image)

    centered(draw, (WIDTH // 2, 18), "EM 与双手机视觉参考的同步曲线示例", font(36, True), COLORS["text"])
    centered(
        draw,
        (WIDTH // 2, 66),
        "合成示意数据——不是实际实验结果，不得作为论文测量值",
        font(23, True),
        COLORS["warning"],
    )

    draw.line((930, 112, 985, 112), fill=COLORS["vision"], width=6)
    draw.text((995, 95), "双手机视觉参考", font=font(20), fill=COLORS["text"])
    draw.line((1245, 112, 1300, 112), fill=COLORS["em"], width=6)
    draw.text((1310, 95), "EM估计", font=font(20), fill=COLORS["text"])

    upper_visual = [0, 0, 15, 30, 30, 60, 60, 30, 30, 0, 0]
    upper_em = [5, 5, 18, 35, 35, 67, 66, 37, 35, 6, 5]
    forearm_visual = [0, 0, 20, 40, 40, 70, 70, 50, 50, 10, 0]
    forearm_em = [8, 8, 28, 50, 49, 82, 80, 60, 59, 19, 8]
    elbow_visual = [90, 90, 90, 90, 90, 60, 60, 30, 30, 90, 90]
    elbow_em = [93, 93, 94, 94, 95, 64, 65, 35, 34, 94, 93]

    plot_panel(
        draw,
        130,
        "A  大臂抬举角",
        90,
        [("视觉", COLORS["vision"], upper_visual), ("EM", COLORS["em"], upper_em)],
    )
    plot_panel(
        draw,
        395,
        "B  前臂方向角",
        90,
        [("视觉", COLORS["vision"], forearm_visual), ("EM", COLORS["em"], forearm_em)],
    )
    plot_panel(
        draw,
        660,
        "C  肘关节屈曲角",
        120,
        [("视觉", COLORS["vision"], elbow_visual), ("EM", COLORS["em"], elbow_em)],
    )

    errors = [
        ("大臂三维方向误差", COLORS["upper"], [6, 5, 8, 7, 6, 10, 9, 8, 7, 6, 5]),
        ("前臂三维方向误差", COLORS["forearm"], [9, 8, 12, 11, 10, 14, 13, 12, 10, 9, 8]),
        ("肘角绝对误差", COLORS["elbow"], [3, 3, 4, 4, 5, 4, 5, 5, 4, 4, 3]),
    ]
    plot_panel(draw, 925, "D  三维方向/肘角绝对误差", 25, errors)

    legend_x = 760
    for label, color, _ in errors:
        draw.line((legend_x, 975, legend_x + 50, 975), fill=color, width=5)
        draw.text((legend_x + 60, 958), label, font=font(18), fill=COLORS["text"])
        legend_x += 275

    left, right = 150, 1530
    for index, seconds in enumerate(range(0, 51, 5)):
        x = left + index * (right - left) / 10
        centered(draw, (int(x), 1184), str(seconds), font(18), COLORS["muted"])
    centered(draw, (WIDTH // 2, 1217), "时间 (s)", font(21), COLORS["text"])

    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    image.save(OUT_PATH, dpi=(180, 180))
    print(OUT_PATH)


if __name__ == "__main__":
    main()
