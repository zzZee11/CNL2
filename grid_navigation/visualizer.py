"""
grid_navigation/visualizer.py
-----------------------------
Clean visualization generator for 2D grid navigation missions.
Exports navigation map as PNG image (using matplotlib if available, or Pillow fallback)
and telemetry data as companion JSON.
"""

import os
import json
import math
from typing import List, Tuple, Optional

try:
    from .config import DEFAULT_PLOT_PATH, ROBOT_RADIUS_M, WHEEL_BASE_M
except ImportError:
    from config import DEFAULT_PLOT_PATH, ROBOT_RADIUS_M, WHEEL_BASE_M


def _render_with_matplotlib(
    trajectory: List[Tuple[float, float]],
    planned_path: List[Tuple[float, float]],
    obstacles: List[Tuple[float, float]],
    final_pose: Tuple[float, float, float],
    total_dist: float,
    status: str,
    output_path: str
) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    fig, ax = plt.subplots(figsize=(8, 8), dpi=120)
    ax.set_facecolor("#fcfcfc")
    ax.grid(True, linestyle="--", alpha=0.5, color="#d0d0d0")

    # 1. Obstacles with inflation
    for ox, oy in obstacles:
        halo = Circle(
            (ox, oy),
            radius=ROBOT_RADIUS_M,
            facecolor="#ff4444",
            edgecolor="#cc0000",
            alpha=0.25,
            linestyle="--",
            linewidth=1.0,
            zorder=2
        )
        ax.add_patch(halo)
        ax.plot(ox, oy, "ro", markersize=5, zorder=3)

    # 2. Planned Path
    if planned_path and len(planned_path) > 1:
        px = [p[0] for p in planned_path]
        py = [p[1] for p in planned_path]
        ax.plot(px, py, "b--o", markersize=4, linewidth=1.8, alpha=0.7, label="Planned Route", zorder=4)

    # 3. Driven Trajectory
    if trajectory and len(trajectory) > 1:
        tx = [p[0] for p in trajectory]
        ty = [p[1] for p in trajectory]
        ax.plot(tx, ty, "g-", linewidth=2.5, alpha=0.9, label="Driven Path", zorder=5)

    # 4. Start Point Indicator
    ax.plot(0.0, 0.0, "o", color="#0066cc", markersize=9, label="Start (0, 0)", zorder=6)

    # 5. Goal Point Indicator
    if planned_path:
        goal_pt = planned_path[-1]
        ax.plot(goal_pt[0], goal_pt[1], "s", color="#ff8800", markersize=10, label=f"Goal ({goal_pt[0]:.1f}, {goal_pt[1]:.1f})", zorder=6)

    # 6. Final Robot Pose & Heading Arrow
    fx, fy, fth = final_pose
    arrow_len = 0.20
    dx = arrow_len * math.cos(fth)
    dy = arrow_len * math.sin(fth)
    ax.arrow(fx, fy, dx, dy, head_width=0.06, head_length=0.06, fc="#333333", ec="#111111", linewidth=2.0, zorder=7)
    ax.plot(fx, fy, "o", color="#333333", markersize=6, zorder=7)

    ax.set_aspect("equal", "box")
    ax.set_xlabel("X Forward (meters)", fontsize=11, fontweight="bold")
    ax.set_ylabel("Y Lateral (meters)", fontsize=11, fontweight="bold")
    ax.set_title(
        f"Grid Navigation Map — {status}\nDistance: {total_dist:.2f} m | Final Heading: {math.degrees(fth):+.1f}°",
        fontsize=12,
        fontweight="bold"
    )
    ax.legend(loc="upper left", framealpha=0.9)

    all_x = [0.0, fx] + [p[0] for p in trajectory] + [p[0] for p in obstacles]
    all_y = [0.0, fy] + [p[1] for p in trajectory] + [p[1] for p in obstacles]
    pad = 0.6
    ax.set_xlim(min(all_x) - pad, max(all_x) + pad)
    ax.set_ylim(min(all_y) - pad, max(all_y) + pad)

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    plt.tight_layout()
    plt.savefig(output_path, dpi=120)
    plt.close(fig)


def _render_with_pillow(
    trajectory: List[Tuple[float, float]],
    planned_path: List[Tuple[float, float]],
    obstacles: List[Tuple[float, float]],
    final_pose: Tuple[float, float, float],
    total_dist: float,
    status: str,
    output_path: str
) -> None:
    from PIL import Image, ImageDraw

    width, height = 800, 800
    img = Image.new("RGBA", (width, height), (252, 252, 252, 255))
    overlay = Image.new("RGBA", (width, height), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)
    draw_overlay = ImageDraw.Draw(overlay)

    fx, fy, fth = final_pose
    all_x = [0.0, fx] + [p[0] for p in trajectory] + [p[0] for p in obstacles]
    all_y = [0.0, fy] + [p[1] for p in trajectory] + [p[1] for p in obstacles]
    if planned_path:
        all_x += [p[0] for p in planned_path]
        all_y += [p[1] for p in planned_path]

    pad = 0.5
    min_x, max_x = min(all_x) - pad, max(all_x) + pad
    min_y, max_y = min(all_y) - pad, max(all_y) + pad

    span_x = max(0.8, max_x - min_x)
    span_y = max(0.8, max_y - min_y)
    span = max(span_x, span_y)

    mid_x = (min_x + max_x) / 2.0
    mid_y = (min_y + max_y) / 2.0
    min_x, max_x = mid_x - span / 2.0, mid_x + span / 2.0
    min_y, max_y = mid_y - span / 2.0, mid_y + span / 2.0

    margin = 70
    plot_dim = width - 2 * margin
    scale = plot_dim / span

    def to_pixel(wx: float, wy: float) -> Tuple[int, int]:
        px = int(margin + (wx - min_x) * scale)
        py = int(height - margin - (wy - min_y) * scale)
        return px, py

    # Draw coordinate frame border
    draw.rectangle([(margin, margin), (width - margin, height - margin)], outline=(180, 180, 180), width=2)

    # Draw light grid lines
    grid_spacing = 0.2 if span <= 2.0 else 0.5
    gx = math.floor(min_x / grid_spacing) * grid_spacing
    while gx <= max_x:
        px, _ = to_pixel(gx, 0)
        if margin <= px <= width - margin:
            draw.line([(px, margin), (px, height - margin)], fill=(235, 235, 235), width=1)
        gx += grid_spacing

    gy = math.floor(min_y / grid_spacing) * grid_spacing
    while gy <= max_y:
        _, py = to_pixel(0, gy)
        if margin <= py <= height - margin:
            draw.line([(margin, py), (width - margin, py)], fill=(235, 235, 235), width=1)
        gy += grid_spacing

    # 1. Obstacles with inflation halos
    r_px = int(ROBOT_RADIUS_M * scale)
    for ox, oy in obstacles:
        opx, opy = to_pixel(ox, oy)
        # Red semi-transparent halo
        draw_overlay.ellipse(
            [(opx - r_px, opy - r_px), (opx + r_px, opy + r_px)],
            fill=(255, 68, 68, 60),
            outline=(204, 0, 0, 180),
            width=1
        )
        # Obstacle center
        draw.ellipse([(opx - 4, opy - 4), (opx + 4, opy + 4)], fill=(204, 0, 0))

    # 2. Planned route
    if planned_path and len(planned_path) > 1:
        pts = [to_pixel(p[0], p[1]) for p in planned_path]
        draw.line(pts, fill=(30, 100, 220), width=2)
        for pt in pts:
            draw.ellipse([(pt[0] - 3, pt[1] - 3), (pt[0] + 3, pt[1] + 3)], fill=(30, 100, 220))

    # 3. Driven trajectory
    if trajectory and len(trajectory) > 1:
        t_pts = [to_pixel(p[0], p[1]) for p in trajectory]
        draw.line(t_pts, fill=(34, 160, 40), width=3)

    # 4. Start point
    spx, spy = to_pixel(0.0, 0.0)
    draw.ellipse([(spx - 6, spy - 6), (spx + 6, spy + 6)], fill=(0, 102, 204), outline=(0, 50, 150), width=2)

    # 5. Goal point
    if planned_path:
        gpx, gpy = to_pixel(planned_path[-1][0], planned_path[-1][1])
        draw.rectangle([(gpx - 6, gpy - 6), (gpx + 6, gpy + 6)], fill=(255, 136, 0), outline=(180, 90, 0), width=2)

    # 6. Final robot pose and heading arrow
    fpx, fpy = to_pixel(fx, fy)
    arrow_len = 25
    apx = int(fpx + arrow_len * math.cos(fth))
    apy = int(fpy - arrow_len * math.sin(fth))
    draw.line([(fpx, fpy), (apx, apy)], fill=(20, 20, 20), width=3)
    draw.ellipse([(fpx - 5, fpy - 5), (fpx + 5, fpy + 5)], fill=(30, 30, 30))

    # Composite overlay
    img = Image.alpha_composite(img, overlay).convert("RGB")
    final_draw = ImageDraw.Draw(img)

    # Headers & Text
    final_draw.text((margin, 20), f"Grid Navigation Map - {status}", fill=(20, 20, 20))
    final_draw.text(
        (margin, 40),
        f"Distance: {total_dist:.2f} m | Final Heading: {math.degrees(fth):+.1f} deg | Pose: ({fx:.2f}, {fy:.2f}) m",
        fill=(70, 70, 70)
    )

    # Legend text at bottom
    legend_y = height - 40
    final_draw.ellipse([(margin, legend_y - 4), (margin + 8, legend_y + 4)], fill=(0, 102, 204))
    final_draw.text((margin + 14, legend_y - 6), "Start (0,0)", fill=(30, 30, 30))

    final_draw.rectangle([(margin + 110, legend_y - 4), (margin + 118, legend_y + 4)], fill=(255, 136, 0))
    final_draw.text((margin + 124, legend_y - 6), "Goal", fill=(30, 30, 30))

    final_draw.line([(margin + 175, legend_y), (margin + 195, legend_y)], fill=(34, 160, 40), width=3)
    final_draw.text((margin + 201, legend_y - 6), "Driven Path", fill=(30, 30, 30))

    final_draw.line([(margin + 300, legend_y), (margin + 320, legend_y)], fill=(30, 100, 220), width=2)
    final_draw.text((margin + 326, legend_y - 6), "Planned Route", fill=(30, 30, 30))

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    img.save(output_path, "PNG")


def export_navigation_map(
    trajectory: List[Tuple[float, float]],
    planned_path: List[Tuple[float, float]],
    obstacles: List[Tuple[float, float]],
    final_pose: Tuple[float, float, float],
    total_dist: float,
    status: str = "MISSION FINISHED",
    output_path: str = DEFAULT_PLOT_PATH
) -> str:
    """
    Renders high-resolution map of trajectory, planned waypoints, and obstacles,
    and saves a companion JSON telemetry file.
    Prefers matplotlib if installed; falls back cleanly to Pillow.
    """
    try:
        import matplotlib
        _render_with_matplotlib(
            trajectory=trajectory,
            planned_path=planned_path,
            obstacles=obstacles,
            final_pose=final_pose,
            total_dist=total_dist,
            status=status,
            output_path=output_path
        )
    except ImportError:
        _render_with_pillow(
            trajectory=trajectory,
            planned_path=planned_path,
            obstacles=obstacles,
            final_pose=final_pose,
            total_dist=total_dist,
            status=status,
            output_path=output_path
        )

    # Save companion JSON
    fx, fy, fth = final_pose
    json_path = os.path.splitext(output_path)[0] + "_telemetry.json"
    telemetry = {
        "status": status,
        "final_pose": {"x": fx, "y": fy, "theta_deg": round(math.degrees(fth), 2)},
        "distance_travelled_m": round(total_dist, 3),
        "obstacle_count": len(obstacles),
        "obstacles": [{"x": ox, "y": oy} for ox, oy in obstacles],
        "trajectory_points": len(trajectory)
    }
    with open(json_path, "w") as jf:
        json.dump(telemetry, jf, indent=2)

    print(f"[Visualizer] Navigation map saved: {output_path}")
    print(f"[Visualizer] Telemetry JSON saved:  {json_path}")
    return output_path

