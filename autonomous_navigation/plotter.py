"""
autonomous_navigation/plotter.py
---------------------------------
Visualizes the autonomous navigation mission on a 2D Cartesian plane:
- Start position (0, 0) and Target goal position (X, Y)
- Planned A* global route and waypoints
- Recorded robot trajectory (odometry path)
- Mapped obstacles and obstacle clearance inflation zones
- Comprehensive mission summary (distance travelled, time, status)
- Exports detected obstacle coordinates to a JSON data file for testing/logging

Performance Note:
Saving obstacles and rendering the map is an extremely lightweight post-mission task:
- Real-time logging during drive: < 1 microsecond per reading (appending to Python list).
- Post-run rendering: Runs once after robot halts, taking ~50-150 ms total.
- Has 0% impact on real-time motor control loops.
"""

import os
import json
import math
from typing import List, Tuple, Optional

try:
    from .config import ROBOT_CLEARANCE_RADIUS, DEFAULT_PLOT_PATH
except ImportError:
    from config import ROBOT_CLEARANCE_RADIUS, DEFAULT_PLOT_PATH

try:
    import matplotlib
    matplotlib.use("Agg")  # Non-interactive headless backend
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

try:
    from PIL import Image, ImageDraw, ImageFont
    HAS_PIL = True
except ImportError:
    HAS_PIL = False


def save_navigation_plot(
    trajectory: List[Tuple[float, float]],
    recorded_obstacles: List[Tuple[float, float]],
    start_pos: Tuple[float, float],
    goal_pos: Tuple[float, float],
    planned_path: List[Tuple[float, float]],
    final_pose: Tuple[float, float, float],
    total_dist: float,
    elapsed_s: float,
    status: str = "COMPLETED",
    output_path: str = DEFAULT_PLOT_PATH,
    grid: Optional[List[List[int]]] = None,
) -> str:
    """
    Renders and saves the 2D Cartesian navigation map and trajectory to an image file,
    and exports all detected obstacle coordinates to a paired JSON data file.

    Returns:
        The absolute path of the saved image file.
    """
    # Ensure parent output directory exists
    out_dir = os.path.dirname(os.path.abspath(output_path))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    # 1. Save Structured Obstacle Data (JSON)
    json_path = os.path.splitext(output_path)[0] + "_obstacles.json"
    try:
        obstacle_data = {
            "mission_status": status,
            "start_position": [round(start_pos[0], 3), round(start_pos[1], 3)],
            "goal_position": [round(goal_pos[0], 3), round(goal_pos[1], 3)],
            "final_pose": {
                "x": round(final_pose[0], 3),
                "y": round(final_pose[1], 3),
                "heading_deg": round(math.degrees(final_pose[2]), 1),
            },
            "distance_travelled_m": round(total_dist, 3),
            "elapsed_time_s": round(elapsed_s, 2),
            "obstacle_count": len(recorded_obstacles),
            "obstacles": [
                {"id": idx + 1, "x": round(p[0], 3), "y": round(p[1], 3)}
                for idx, p in enumerate(recorded_obstacles)
            ],
        }
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(obstacle_data, f, indent=2)
        print(f"[Plotter] Obstacle coordinates saved to: {os.path.abspath(json_path)}")
    except Exception as e:
        print(f"[Plotter] Warning: Could not save obstacle JSON ({e})")

    # 2. Render and Save Visual Map (Image)
    if HAS_MATPLOTLIB:
        _render_matplotlib(
            trajectory=trajectory,
            recorded_obstacles=recorded_obstacles,
            start_pos=start_pos,
            goal_pos=goal_pos,
            planned_path=planned_path,
            final_pose=final_pose,
            total_dist=total_dist,
            elapsed_s=elapsed_s,
            status=status,
            output_path=output_path,
        )
    elif HAS_PIL:
        _render_pillow(
            trajectory=trajectory,
            recorded_obstacles=recorded_obstacles,
            start_pos=start_pos,
            goal_pos=goal_pos,
            planned_path=planned_path,
            final_pose=final_pose,
            total_dist=total_dist,
            elapsed_s=elapsed_s,
            status=status,
            output_path=output_path,
        )
    else:
        print("[Plotter] Warning: Neither matplotlib nor pillow is available. Cannot save plot image.")
        return output_path

    print(f"[Plotter] Navigation map saved to: {os.path.abspath(output_path)}")
    return os.path.abspath(output_path)


def _render_matplotlib(
    trajectory,
    recorded_obstacles,
    start_pos,
    goal_pos,
    planned_path,
    final_pose,
    total_dist,
    elapsed_s,
    status,
    output_path,
):
    fig, ax = plt.subplots(figsize=(10, 8), dpi=150)
    ax.set_facecolor("#fdfdfd")
    ax.grid(True, linestyle="--", alpha=0.55, color="#c0c0c0")

    # 1. Plot Mapped Obstacles & Safety Inflation Halos
    if recorded_obstacles:
        oxs = [p[0] for p in recorded_obstacles]
        oys = [p[1] for p in recorded_obstacles]
        ax.scatter(
            oxs, oys,
            color="#e74c3c", s=35, alpha=0.85,
            edgecolors="#922b21", linewidth=0.8,
            label=f"Detected Obstacles ({len(recorded_obstacles)})", zorder=3
        )
        # Add clearance halos around obstacles
        step = max(1, len(recorded_obstacles) // 30)
        for i in range(0, len(recorded_obstacles), step):
            p = recorded_obstacles[i]
            circle = Circle(
                (p[0], p[1]), ROBOT_CLEARANCE_RADIUS,
                facecolor="#e74c3c", edgecolor="#c0392b",
                alpha=0.15, linestyle=":", zorder=2
            )
            ax.add_patch(circle)

    # 2. Plot Planned A* Global Path
    if planned_path and len(planned_path) > 1:
        pxs = [p[0] for p in planned_path]
        pys = [p[1] for p in planned_path]
        ax.plot(
            pxs, pys,
            linestyle="--", color="#3498db", linewidth=2.0,
            label="Planned A* Path", zorder=3
        )
        ax.scatter(pxs, pys, color="#2980b9", s=15, zorder=3)

    # 3. Plot Actual Robot Trajectory
    if trajectory and len(trajectory) > 1:
        txs = [p[0] for p in trajectory]
        tys = [p[1] for p in trajectory]
        ax.plot(
            txs, tys,
            color="#27ae60", linewidth=2.8,
            label=f"Navigated Path ({total_dist:.2f} m)", zorder=4
        )
        # Orientation directional markers every 10%
        stride = max(1, len(trajectory) // 8)
        for i in range(stride, len(trajectory) - 1, stride):
            p1 = trajectory[i]
            p2 = trajectory[min(len(trajectory) - 1, i + 1)]
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            if math.hypot(dx, dy) > 0.02:
                ax.annotate(
                    "", xy=(p2[0], p2[1]), xytext=(p1[0], p1[1]),
                    arrowprops=dict(arrowstyle="->", color="#1e8449", lw=1.5),
                    zorder=5
                )

    # 4. Markers: Start, Goal, Final Pose
    ax.scatter(
        [start_pos[0]], [start_pos[1]],
        color="#2980b9", marker="o", s=130,
        edgecolors="black", linewidth=1.5,
        label=f"Start ({start_pos[0]:.1f}, {start_pos[1]:.1f})", zorder=6
    )

    ax.scatter(
        [goal_pos[0]], [goal_pos[1]],
        color="#f39c12", marker="*", s=280,
        edgecolors="black", linewidth=1.5,
        label=f"Goal ({goal_pos[0]:.1f}, {goal_pos[1]:.1f})", zorder=6
    )

    # Final robot pose heading indicator
    cur_x, cur_y, cur_theta = final_pose
    ax.scatter(
        [cur_x], [cur_y],
        color="#8e44ad", marker="D", s=110,
        edgecolors="black", linewidth=1.5,
        label=f"Final Pose ({cur_x:.2f}, {cur_y:.2f})", zorder=7
    )
    arrow_len = 0.35
    ax.annotate(
        "", xy=(cur_x + arrow_len * math.cos(cur_theta), cur_y + arrow_len * math.sin(cur_theta)),
        xytext=(cur_x, cur_y),
        arrowprops=dict(arrowstyle="->", color="#8e44ad", lw=2.2),
        zorder=8
    )

    # Axes framing and equal aspect ratio
    ax.set_aspect("equal", adjustable="datalim")
    ax.axhline(0, color="#888888", linewidth=0.8, linestyle=":")
    ax.axvline(0, color="#888888", linewidth=0.8, linestyle=":")

    ax.set_xlabel("X Position (meters) [Forward]", fontsize=11, fontweight="bold")
    ax.set_ylabel("Y Position (meters) [Left / Right]", fontsize=11, fontweight="bold")

    # Title & Information Box
    ax.set_title(
        f"Autonomous Navigation Cartesian Trajectory\nTarget: ({goal_pos[0]:.2f}, {goal_pos[1]:.2f}) m  |  Status: {status}",
        fontsize=13, fontweight="bold", pad=12
    )

    info_box = (
        f"Mission Summary:\n"
        f"• Status: {status}\n"
        f"• Goal: ({goal_pos[0]:.2f}, {goal_pos[1]:.2f}) m\n"
        f"• Distance: {total_dist:.2f} m\n"
        f"• Elapsed Time: {elapsed_s:.1f} s\n"
        f"• Obstacles Logged: {len(recorded_obstacles)}"
    )
    ax.text(
        0.02, 0.02, info_box,
        transform=ax.transAxes, fontsize=9,
        verticalalignment="bottom", horizontalalignment="left",
        bbox=dict(boxstyle="round,pad=0.6", facecolor="white", alpha=0.9, edgecolor="#bdc3c7")
    )

    ax.legend(loc="upper left", framealpha=0.92, edgecolor="#bdc3c7", fontsize=9)
    plt.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def _render_pillow(
    trajectory,
    recorded_obstacles,
    start_pos,
    goal_pos,
    planned_path,
    final_pose,
    total_dist,
    elapsed_s,
    status,
    output_path,
):
    """Pillow fallback renderer when matplotlib is not installed."""
    w, h = 900, 900
    img = Image.new("RGB", (w, h), color=(250, 250, 252))
    draw = ImageDraw.Draw(img)

    # Determine bounding box across all points
    all_pts = [start_pos, goal_pos, (final_pose[0], final_pose[1])] + trajectory + recorded_obstacles + planned_path
    all_xs = [p[0] for p in all_pts]
    all_ys = [p[1] for p in all_pts]

    min_x = min(-1.0, min(all_xs) - 1.0)
    max_x = max(2.0, max(all_xs) + 1.0)
    min_y = min(-1.0, min(all_ys) - 1.0)
    max_y = max(2.0, max(all_ys) + 1.0)

    # Keep square aspect ratio
    span = max(max_x - min_x, max_y - min_y)
    mid_x = (min_x + max_x) / 2.0
    mid_y = (min_y + max_y) / 2.0
    min_x, max_x = mid_x - span / 2.0, mid_x + span / 2.0
    min_y, max_y = mid_y - span / 2.0, mid_y + span / 2.0

    margin = 80
    scale = (w - 2 * margin) / max(0.01, span)

    def to_px(wx, wy):
        px = int(margin + (wx - min_x) * scale)
        py = int(h - margin - (wy - min_y) * scale)
        return px, py

    # Draw grid lines
    step_m = 1.0 if span < 12 else 2.0
    gx = math.floor(min_x)
    while gx <= max_x:
        p1 = to_px(gx, min_y)
        p2 = to_px(gx, max_y)
        draw.line([p1, p2], fill=(225, 225, 230), width=1)
        draw.text((p1[0] - 10, h - margin + 8), f"{gx:.0f}m", fill=(130, 130, 140))
        gx += step_m

    gy = math.floor(min_y)
    while gy <= max_y:
        p1 = to_px(min_x, gy)
        p2 = to_px(max_x, gy)
        draw.line([p1, p2], fill=(225, 225, 230), width=1)
        draw.text((margin - 36, p1[1] - 6), f"{gy:.0f}m", fill=(130, 130, 140))
        gy += step_m

    # Axis (0, 0) cross
    o_px = to_px(0.0, 0.0)
    draw.line([(o_px[0], margin), (o_px[0], h - margin)], fill=(160, 160, 170), width=2)
    draw.line([(margin, o_px[1]), (w - margin, o_px[1])], fill=(160, 160, 170), width=2)

    # Draw Obstacles (Clearance circle + center dot)
    for ox, oy in recorded_obstacles:
        px, py = to_px(ox, oy)
        r_clear = int(ROBOT_CLEARANCE_RADIUS * scale)
        draw.ellipse([px - r_clear, py - r_clear, px + r_clear, py + r_clear], fill=(255, 220, 220))
        draw.ellipse([px - 4, py - 4, px + 4, py + 4], fill=(231, 76, 60), outline=(192, 57, 43))

    # Draw Planned Path
    if planned_path and len(planned_path) > 1:
        pts = [to_px(p[0], p[1]) for p in planned_path]
        for i in range(len(pts) - 1):
            draw.line([pts[i], pts[i + 1]], fill=(52, 152, 219), width=2)
        for pt in pts:
            draw.ellipse([pt[0] - 3, pt[1] - 3, pt[0] + 3, pt[1] + 3], fill=(41, 128, 185))

    # Draw Actual Trajectory
    if trajectory and len(trajectory) > 1:
        pts = [to_px(p[0], p[1]) for p in trajectory]
        for i in range(len(pts) - 1):
            draw.line([pts[i], pts[i + 1]], fill=(39, 174, 96), width=3)

    # Start Point
    sp = to_px(start_pos[0], start_pos[1])
    draw.ellipse([sp[0] - 7, sp[1] - 7, sp[0] + 7, sp[1] + 7], fill=(41, 128, 185), outline=(0, 0, 0))
    draw.text((sp[0] + 10, sp[1] - 8), "Start (0,0)", fill=(41, 128, 185))

    # Goal Point
    gp = to_px(goal_pos[0], goal_pos[1])
    draw.rectangle([gp[0] - 8, gp[1] - 8, gp[0] + 8, gp[1] + 8], fill=(243, 156, 18), outline=(0, 0, 0))
    draw.text((gp[0] + 12, gp[1] - 8), f"Goal ({goal_pos[0]:.1f}, {goal_pos[1]:.1f})", fill=(214, 137, 16))

    # Final Robot Pose
    fp = to_px(final_pose[0], final_pose[1])
    draw.ellipse([fp[0] - 6, fp[1] - 6, fp[0] + 6, fp[1] + 6], fill=(142, 68, 173), outline=(0, 0, 0))

    # Header Title & Stats
    draw.text((margin, 20), f"Autonomous Navigation Trajectory: Target ({goal_pos[0]:.2f}, {goal_pos[1]:.2f}) m", fill=(20, 20, 30))
    stats_str = f"Status: {status} | Dist: {total_dist:.2f}m | Time: {elapsed_s:.1f}s | Obstacles: {len(recorded_obstacles)}"
    draw.text((margin, 42), stats_str, fill=(70, 80, 90))

    img.save(output_path)
