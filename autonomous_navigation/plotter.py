"""
autonomous_navigation/plotter.py
---------------------------------
Visualizes the autonomous navigation mission on a 2D Cartesian plane:
- Continuous obstacles: Renders obstacles as connected barrier walls with smooth safety clearance envelopes
- Robot physical width: Renders continuous swept-path corridor (13.4 cm) and oriented body footprints (18×13.4 cm)
- Target orientation at Goal: Visualizes requested final heading with oriented footprint and directional arrow
- Start position (0, 0) and Target goal position (X, Y)
- Planned A* global route and waypoints
- Recorded robot trajectory (odometry path)
- Comprehensive mission summary (distance travelled, time, status, heading accuracy)
- Exports detected obstacle coordinates to a JSON data file for testing/logging
"""

import os
import json
import math
from typing import List, Tuple, Optional

try:
    from .config import (
        ROBOT_CLEARANCE_RADIUS, DEFAULT_PLOT_PATH,
        WHEEL_BASE_M, ROBOT_LENGTH_M, ROBOT_WIDTH_M,
        OBSTACLE_CHAIN_MAX_DIST_M
    )
except ImportError:
    from config import (
        ROBOT_CLEARANCE_RADIUS, DEFAULT_PLOT_PATH,
        WHEEL_BASE_M, ROBOT_LENGTH_M, ROBOT_WIDTH_M,
        OBSTACLE_CHAIN_MAX_DIST_M
    )

try:
    import matplotlib
    matplotlib.use("Agg")  # Non-interactive headless backend
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon, Patch
    from matplotlib.lines import Line2D
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

try:
    from PIL import Image, ImageDraw, ImageFont
    HAS_PIL = True
except ImportError:
    HAS_PIL = False


def cluster_obstacles(
    obstacles: List[Tuple[float, float]],
    max_link_dist: float = OBSTACLE_CHAIN_MAX_DIST_M
) -> List[List[Tuple[float, float]]]:
    """
    Groups discrete obstacle points into continuous chains/barriers.
    Points within max_link_dist are grouped together and sorted into an ordered polyline.
    Returns a list of chains, where each chain is a list of (x, y) coordinates.
    """
    if not obstacles:
        return []

    # Deduplicate while preserving order
    pts = list(dict.fromkeys(obstacles))
    n = len(pts)
    if n == 1:
        return [[pts[0]]]

    # Build adjacency list for connected components
    adj = {i: [] for i in range(n)}
    for i in range(n):
        for j in range(i + 1, n):
            d = math.hypot(pts[i][0] - pts[j][0], pts[i][1] - pts[j][1])
            if d <= max_link_dist:
                adj[i].append(j)
                adj[j].append(i)

    visited = set()
    clusters = []
    for i in range(n):
        if i in visited:
            continue
        comp = []
        q = [i]
        visited.add(i)
        while q:
            curr = q.pop()
            comp.append(curr)
            for neighbor in adj[curr]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    q.append(neighbor)

        comp_pts = [pts[idx] for idx in comp]
        if len(comp_pts) <= 2:
            clusters.append(comp_pts)
        else:
            # Order the cluster along the chain (diameter traversal)
            max_d = -1.0
            start_idx = 0
            for a in range(len(comp_pts)):
                for b in range(a + 1, len(comp_pts)):
                    d = math.hypot(comp_pts[a][0] - comp_pts[b][0], comp_pts[a][1] - comp_pts[b][1])
                    if d > max_d:
                        max_d = d
                        start_idx = a

            chain = [comp_pts[start_idx]]
            remaining = set(range(len(comp_pts))) - {start_idx}
            curr = comp_pts[start_idx]
            while remaining:
                next_idx = min(
                    remaining,
                    key=lambda idx: math.hypot(comp_pts[idx][0] - curr[0], comp_pts[idx][1] - curr[1])
                )
                chain.append(comp_pts[next_idx])
                remaining.remove(next_idx)
                curr = comp_pts[next_idx]
            clusters.append(chain)

    return clusters


def compute_swept_path(
    trajectory: List[Tuple[float, float]],
    width: float = WHEEL_BASE_M
) -> Tuple[List[Tuple[float, float]], List[Tuple[float, float]], List[Tuple[float, float]]]:
    """
    Computes the 2D swept-path corridor (polygon) and left/right wheel tracks along the trajectory.
    Returns: (polygon_pts, left_track, right_track)
    """
    if len(trajectory) < 2:
        return [], [], []

    half_w = width / 2.0
    left_track = []
    right_track = []
    n = len(trajectory)
    last_normal = (0.0, 1.0)

    for i in range(n):
        if i == 0:
            dx = trajectory[1][0] - trajectory[0][0]
            dy = trajectory[1][1] - trajectory[0][1]
        elif i == n - 1:
            dx = trajectory[n - 1][0] - trajectory[n - 2][0]
            dy = trajectory[n - 1][1] - trajectory[n - 2][1]
        else:
            dx = trajectory[i + 1][0] - trajectory[i - 1][0]
            dy = trajectory[i + 1][1] - trajectory[i - 1][1]

        length = math.hypot(dx, dy)
        if length > 1e-4:
            normal = (-dy / length, dx / length)
            last_normal = normal
        else:
            normal = last_normal

        px, py = trajectory[i]
        left_track.append((px + half_w * normal[0], py + half_w * normal[1]))
        right_track.append((px - half_w * normal[0], py - half_w * normal[1]))

    polygon = left_track + right_track[::-1]
    return polygon, left_track, right_track


def get_robot_box_corners(
    cx: float,
    cy: float,
    theta: float,
    length: float = ROBOT_LENGTH_M,
    width: float = ROBOT_WIDTH_M
) -> List[Tuple[float, float]]:
    """
    Computes the 4 corners of an oriented rectangular robot footprint centered at (cx, cy).
    """
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    hl = length / 2.0
    hw = width / 2.0
    local_corners = [(hl, hw), (-hl, hw), (-hl, -hw), (hl, -hw)]
    world_corners = []
    for lx, ly in local_corners:
        wx = cx + (lx * cos_t - ly * sin_t)
        wy = cy + (lx * sin_t + ly * cos_t)
        world_corners.append((wx, wy))
    return world_corners


def save_navigation_plot(
    trajectory: List[Tuple[float, float]],
    recorded_obstacles: List[Tuple[float, float]],
    start_pos: Tuple[float, float],
    goal_pos: Tuple[float, float],
    planned_path: List[Tuple[float, float]],
    final_pose: Tuple[float, float, float],
    total_dist: float,
    elapsed_s: float,
    target_theta: Optional[float] = None,
    status: str = "COMPLETED",
    output_path: str = DEFAULT_PLOT_PATH,
    grid: Optional[List[List[int]]] = None,
) -> str:
    """
    Renders and saves the 2D Cartesian navigation map and trajectory to an image file,
    and exports all detected obstacle coordinates to a paired JSON data file.
    Includes continuous obstacles, robot width corridor, and target orientation.

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
            "target_orientation_deg": round(math.degrees(target_theta), 1) if target_theta is not None else None,
            "final_pose": {
                "x": round(final_pose[0], 3),
                "y": round(final_pose[1], 3),
                "heading_deg": round(math.degrees(final_pose[2]), 1),
            },
            "distance_travelled_m": round(total_dist, 3),
            "elapsed_time_s": round(elapsed_s, 2),
            "robot_dimensions": {
                "length_m": ROBOT_LENGTH_M,
                "width_m": ROBOT_WIDTH_M,
                "wheel_base_m": WHEEL_BASE_M,
            },
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
            target_theta=target_theta,
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
            target_theta=target_theta,
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
    target_theta,
    status,
    output_path,
):
    fig, ax = plt.subplots(figsize=(10.5, 8.5), dpi=150)
    ax.set_facecolor("#fdfdfd")
    ax.grid(True, linestyle="--", alpha=0.55, color="#c0c0c0")

    # 1. Plot Continuous Obstacles & Smooth Clearance Envelopes
    clusters = cluster_obstacles(recorded_obstacles, max_link_dist=OBSTACLE_CHAIN_MAX_DIST_M)
    has_continuous = False

    for chain in clusters:
        if len(chain) == 1:
            # Isolated obstacle point & circular clearance halo
            p = chain[0]
            circle = Circle(
                (p[0], p[1]), ROBOT_CLEARANCE_RADIUS,
                facecolor="#e74c3c", edgecolor="#c0392b",
                alpha=0.18, linestyle=":", zorder=2
            )
            ax.add_patch(circle)
            ax.scatter([p[0]], [p[1]], color="#e74c3c", s=36, edgecolors="#78281f", linewidth=0.8, zorder=5)
        else:
            has_continuous = True
            # Continuous clearance envelope capsule along each segment
            for i in range(len(chain) - 1):
                p1 = chain[i]
                p2 = chain[i + 1]
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                L = math.hypot(dx, dy)
                if L > 1e-4:
                    nx = -dy / L
                    ny = dx / L
                    r = ROBOT_CLEARANCE_RADIUS
                    capsule_pts = [
                        (p1[0] + r * nx, p1[1] + r * ny),
                        (p2[0] + r * nx, p2[1] + r * ny),
                        (p2[0] - r * nx, p2[1] - r * ny),
                        (p1[0] - r * nx, p1[1] - r * ny),
                    ]
                    ax.add_patch(Polygon(capsule_pts, facecolor="#e74c3c", edgecolor="none", alpha=0.16, zorder=2))
                # Add circular cap at vertices
                ax.add_patch(Circle((p1[0], p1[1]), ROBOT_CLEARANCE_RADIUS, facecolor="#e74c3c", edgecolor="none", alpha=0.16, zorder=2))
                ax.add_patch(Circle((p2[0], p2[1]), ROBOT_CLEARANCE_RADIUS, facecolor="#e74c3c", edgecolor="none", alpha=0.16, zorder=2))

            # Continuous physical barrier line
            c_xs = [p[0] for p in chain]
            c_ys = [p[1] for p in chain]
            ax.plot(c_xs, c_ys, color="#b03a2e", linewidth=3.4, solid_capstyle='round', zorder=4)
            ax.scatter(c_xs, c_ys, color="#e74c3c", s=32, edgecolors="#78281f", linewidth=0.8, zorder=5)

    # 2. Plot Planned A* Global Path
    if planned_path and len(planned_path) > 1:
        pxs = [p[0] for p in planned_path]
        pys = [p[1] for p in planned_path]
        ax.plot(
            pxs, pys,
            linestyle="--", color="#3498db", linewidth=1.8,
            label="Planned A* Path", zorder=3
        )
        ax.scatter(pxs, pys, color="#2980b9", s=14, zorder=3)

    # 3. Plot Robot Physical Width (Swept-Path Corridor) & Actual Trajectory
    if trajectory and len(trajectory) > 1:
        txs = [p[0] for p in trajectory]
        tys = [p[1] for p in trajectory]

        # Swept-path swath (width of the robot: 13.4 cm)
        swept_poly, left_t, right_t = compute_swept_path(trajectory, width=WHEEL_BASE_M)
        if swept_poly:
            ax.add_patch(Polygon(
                swept_poly,
                facecolor="#2ecc71", edgecolor="none", alpha=0.20,
                label=f"Robot Swath ({WHEEL_BASE_M * 100:.1f} cm Width)", zorder=2
            ))
            # Left and right wheel tracks
            ax.plot([p[0] for p in left_t], [p[1] for p in left_t], color="#27ae60", linestyle=":", linewidth=0.9, alpha=0.6, zorder=3)
            ax.plot([p[0] for p in right_t], [p[1] for p in right_t], color="#27ae60", linestyle=":", linewidth=0.9, alpha=0.6, zorder=3)

        # Trajectory Centerline
        ax.plot(
            txs, tys,
            color="#1e8449", linewidth=2.4,
            label=f"Navigated Path ({total_dist:.2f} m)", zorder=4
        )
        # Orientation directional markers along trajectory
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

    # 4. Markers & Oriented Robot Body Footprints
    # Start Pose Footprint & Marker
    start_theta = 0.0
    if trajectory and len(trajectory) > 1:
        start_theta = math.atan2(trajectory[1][1] - trajectory[0][1], trajectory[1][0] - trajectory[0][0])
    start_box = get_robot_box_corners(start_pos[0], start_pos[1], start_theta)
    ax.add_patch(Polygon(
        start_box,
        facecolor="#3498db", edgecolor="#1b4f72", alpha=0.25,
        linewidth=1.4, linestyle="-",
        label=f"Start Pose ({start_pos[0]:.1f}, {start_pos[1]:.1f})", zorder=6
    ))
    ax.scatter(
        [start_pos[0]], [start_pos[1]],
        color="#2980b9", marker="o", s=80,
        edgecolors="black", linewidth=1.2, zorder=7
    )

    # Target Goal Marker & Optional Target Orientation
    ax.scatter(
        [goal_pos[0]], [goal_pos[1]],
        color="#f39c12", marker="*", s=260,
        edgecolors="black", linewidth=1.4,
        label=f"Goal ({goal_pos[0]:.1f}, {goal_pos[1]:.1f})", zorder=8
    )

    if target_theta is not None:
        goal_box = get_robot_box_corners(goal_pos[0], goal_pos[1], target_theta)
        ax.add_patch(Polygon(
            goal_box,
            facecolor="#f39c12", edgecolor="#d35400", alpha=0.22,
            linewidth=1.6, linestyle="--",
            label=f"Target Pose ({math.degrees(target_theta):+.0f}°)", zorder=6
        ))
        arrow_len = 0.35
        ax.annotate(
            "",
            xy=(goal_pos[0] + arrow_len * math.cos(target_theta), goal_pos[1] + arrow_len * math.sin(target_theta)),
            xytext=(goal_pos[0], goal_pos[1]),
            arrowprops=dict(arrowstyle="-|>", color="#d35400", lw=2.4),
            zorder=8
        )

    # Final Robot Pose Footprint & Marker
    cur_x, cur_y, cur_theta = final_pose
    final_box = get_robot_box_corners(cur_x, cur_y, cur_theta)
    ax.add_patch(Polygon(
        final_box,
        facecolor="#9b59b6", edgecolor="#512e5f", alpha=0.32,
        linewidth=1.6, linestyle="-",
        label=f"Final Pose ({cur_x:.2f}, {cur_y:.2f})", zorder=7
    ))
    ax.scatter(
        [cur_x], [cur_y],
        color="#8e44ad", marker="D", s=90,
        edgecolors="black", linewidth=1.2, zorder=8
    )
    arrow_len = 0.35
    ax.annotate(
        "", xy=(cur_x + arrow_len * math.cos(cur_theta), cur_y + arrow_len * math.sin(cur_theta)),
        xytext=(cur_x, cur_y),
        arrowprops=dict(arrowstyle="->", color="#512e5f", lw=2.2),
        zorder=9
    )

    # Add custom legend handles for Continuous Obstacles & Clearance
    handles, labels = ax.get_legend_handles_labels()
    if recorded_obstacles:
        obs_label = f"Continuous Obstacles ({len(recorded_obstacles)} pts)" if has_continuous else f"Obstacles ({len(recorded_obstacles)} pts)"
        handles.insert(0, Line2D([0], [0], color="#b03a2e", lw=3.2, label=obs_label))
        handles.insert(1, Patch(facecolor="#e74c3c", edgecolor="#c0392b", alpha=0.18, linestyle=":", label=f"Safety Clearance ({ROBOT_CLEARANCE_RADIUS*100:.0f} cm)"))

    # Axes framing and equal aspect ratio
    ax.set_aspect("equal", adjustable="datalim")
    ax.axhline(0, color="#888888", linewidth=0.8, linestyle=":")
    ax.axvline(0, color="#888888", linewidth=0.8, linestyle=":")

    ax.set_xlabel("X Position (meters) [Forward]", fontsize=11, fontweight="bold")
    ax.set_ylabel("Y Position (meters) [Left / Right]", fontsize=11, fontweight="bold")

    # Title & Information Box
    target_orient_str = f" @ {math.degrees(target_theta):+.1f}°" if target_theta is not None else ""
    ax.set_title(
        f"Autonomous Navigation Cartesian Trajectory\nTarget: ({goal_pos[0]:.2f}, {goal_pos[1]:.2f}) m{target_orient_str}  |  Status: {status}",
        fontsize=12, fontweight="bold", pad=12
    )

    heading_info = f"• Final Heading: {math.degrees(cur_theta):.1f}°\n"
    if target_theta is not None:
        target_deg = math.degrees(target_theta)
        hdg_err = math.degrees(math.atan2(math.sin(target_theta - cur_theta), math.cos(target_theta - cur_theta)))
        heading_info = (
            f"• Target Heading: {target_deg:+.1f}°\n"
            f"• Final Heading: {math.degrees(cur_theta):+.1f}° (Err: {hdg_err:+.1f}°)\n"
        )

    info_box = (
        f"Mission Summary:\n"
        f"• Status: {status}\n"
        f"• Goal: ({goal_pos[0]:.2f}, {goal_pos[1]:.2f}) m\n"
        f"{heading_info}"
        f"• Distance: {total_dist:.2f} m\n"
        f"• Robot Dimensions: 18×13.4 cm\n"
        f"• Obstacles Logged: {len(recorded_obstacles)}\n"
        f"• Elapsed Time: {elapsed_s:.1f} s"
    )
    ax.text(
        0.02, 0.02, info_box,
        transform=ax.transAxes, fontsize=8.5,
        verticalalignment="bottom", horizontalalignment="left",
        bbox=dict(boxstyle="round,pad=0.5", facecolor="white", alpha=0.92, edgecolor="#bdc3c7")
    )

    ax.legend(handles=handles, loc="upper left", framealpha=0.92, edgecolor="#bdc3c7", fontsize=8.5)
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
    target_theta,
    status,
    output_path,
):
    """Pillow fallback renderer when matplotlib is not installed."""
    w, h = 950, 950
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

    # 1. Draw Continuous Obstacles & Clearance Zones
    clusters = cluster_obstacles(recorded_obstacles, max_link_dist=OBSTACLE_CHAIN_MAX_DIST_M)
    r_clear = int(ROBOT_CLEARANCE_RADIUS * scale)

    for chain in clusters:
        if len(chain) == 1:
            px, py = to_px(chain[0][0], chain[0][1])
            draw.ellipse([px - r_clear, py - r_clear, px + r_clear, py + r_clear], fill=(255, 225, 225))
            draw.ellipse([px - 4, py - 4, px + 4, py + 4], fill=(231, 76, 60), outline=(146, 43, 33))
        else:
            # Continuous clearance capsules
            for i in range(len(chain) - 1):
                p1 = chain[i]
                p2 = chain[i + 1]
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                L = math.hypot(dx, dy)
                if L > 1e-4:
                    nx = -dy / L
                    ny = dx / L
                    r = ROBOT_CLEARANCE_RADIUS
                    c_poly = [
                        to_px(p1[0] + r * nx, p1[1] + r * ny),
                        to_px(p2[0] + r * nx, p2[1] + r * ny),
                        to_px(p2[0] - r * nx, p2[1] - r * ny),
                        to_px(p1[0] - r * nx, p1[1] - r * ny),
                    ]
                    draw.polygon(c_poly, fill=(255, 225, 225))
                px1, py1 = to_px(p1[0], p1[1])
                px2, py2 = to_px(p2[0], p2[1])
                draw.ellipse([px1 - r_clear, py1 - r_clear, px1 + r_clear, py1 + r_clear], fill=(255, 225, 225))
                draw.ellipse([px2 - r_clear, py2 - r_clear, px2 + r_clear, py2 + r_clear], fill=(255, 225, 225))

            # Continuous barrier lines
            px_chain = [to_px(p[0], p[1]) for p in chain]
            for i in range(len(px_chain) - 1):
                draw.line([px_chain[i], px_chain[i + 1]], fill=(176, 58, 46), width=4)
            for pt in px_chain:
                draw.ellipse([pt[0] - 4, pt[1] - 4, pt[0] + 4, pt[1] + 4], fill=(231, 76, 60), outline=(120, 40, 31))

    # 2. Draw Swept-Path Corridor (Robot Width)
    if trajectory and len(trajectory) > 1:
        swept_poly, left_t, right_t = compute_swept_path(trajectory, width=WHEEL_BASE_M)
        if swept_poly:
            px_swept = [to_px(p[0], p[1]) for p in swept_poly]
            draw.polygon(px_swept, fill=(220, 248, 230))
            # Wheel tracks
            px_left = [to_px(p[0], p[1]) for p in left_t]
            px_right = [to_px(p[0], p[1]) for p in right_t]
            for i in range(len(px_left) - 1):
                draw.line([px_left[i], px_left[i + 1]], fill=(140, 200, 160), width=1)
                draw.line([px_right[i], px_right[i + 1]], fill=(140, 200, 160), width=1)

    # 3. Draw Planned Path
    if planned_path and len(planned_path) > 1:
        pts = [to_px(p[0], p[1]) for p in planned_path]
        for i in range(len(pts) - 1):
            draw.line([pts[i], pts[i + 1]], fill=(52, 152, 219), width=2)
        for pt in pts:
            draw.ellipse([pt[0] - 3, pt[1] - 3, pt[0] + 3, pt[1] + 3], fill=(41, 128, 185))

    # 4. Draw Actual Trajectory Centerline
    if trajectory and len(trajectory) > 1:
        pts = [to_px(p[0], p[1]) for p in trajectory]
        for i in range(len(pts) - 1):
            draw.line([pts[i], pts[i + 1]], fill=(30, 132, 73), width=3)

    # 5. Start Footprint & Marker
    start_theta = 0.0
    if trajectory and len(trajectory) > 1:
        start_theta = math.atan2(trajectory[1][1] - trajectory[0][1], trajectory[1][0] - trajectory[0][0])
    s_corners = [to_px(p[0], p[1]) for p in get_robot_box_corners(start_pos[0], start_pos[1], start_theta)]
    draw.polygon(s_corners, fill=(214, 234, 248), outline=(41, 128, 185))
    sp = to_px(start_pos[0], start_pos[1])
    draw.ellipse([sp[0] - 6, sp[1] - 6, sp[0] + 6, sp[1] + 6], fill=(41, 128, 185), outline=(0, 0, 0))
    draw.text((sp[0] + 12, sp[1] - 8), "Start", fill=(41, 128, 185))

    # 6. Goal Marker & Target Orientation
    gp = to_px(goal_pos[0], goal_pos[1])
    if target_theta is not None:
        g_corners = [to_px(p[0], p[1]) for p in get_robot_box_corners(goal_pos[0], goal_pos[1], target_theta)]
        draw.polygon(g_corners, fill=(254, 245, 231), outline=(211, 84, 0))
        # Arrow
        ax_end = to_px(goal_pos[0] + 0.30 * math.cos(target_theta), goal_pos[1] + 0.30 * math.sin(target_theta))
        draw.line([gp, ax_end], fill=(211, 84, 0), width=3)

    draw.rectangle([gp[0] - 8, gp[1] - 8, gp[0] + 8, gp[1] + 8], fill=(243, 156, 18), outline=(0, 0, 0))
    draw.text((gp[0] + 18, gp[1] - 8), f"Goal ({goal_pos[0]:.1f}, {goal_pos[1]:.1f})", fill=(214, 137, 16))

    # 7. Final Robot Pose Footprint & Marker
    f_corners = [to_px(p[0], p[1]) for p in get_robot_box_corners(final_pose[0], final_pose[1], final_pose[2])]
    draw.polygon(f_corners, fill=(235, 222, 240), outline=(142, 68, 173))
    fp = to_px(final_pose[0], final_pose[1])
    draw.ellipse([fp[0] - 6, fp[1] - 6, fp[0] + 6, fp[1] + 6], fill=(142, 68, 173), outline=(0, 0, 0))
    fax_end = to_px(final_pose[0] + 0.30 * math.cos(final_pose[2]), final_pose[1] + 0.30 * math.sin(final_pose[2]))
    draw.line([fp, fax_end], fill=(142, 68, 173), width=3)

    # 8. Visual Legend Box
    leg_w, leg_h = 210, 142
    leg_x = w - margin - leg_w
    leg_y = h - margin - leg_h
    draw.rectangle([leg_x, leg_y, leg_x + leg_w, leg_y + leg_h], fill=(255, 255, 255), outline=(210, 210, 215))

    # Legend items
    draw.rectangle([leg_x + 10, leg_y + 10, leg_x + 28, leg_y + 22], fill=(220, 248, 230), outline=(140, 200, 160))
    draw.text((leg_x + 35, leg_y + 10), "Robot Swath (13.4 cm)", fill=(50, 50, 60))

    draw.line([(leg_x + 10, leg_y + 35), (leg_x + 28, leg_y + 35)], fill=(176, 58, 46), width=3)
    draw.ellipse([leg_x + 17, leg_y + 33, leg_x + 21, leg_y + 37], fill=(231, 76, 60))
    draw.text((leg_x + 35, leg_y + 30), "Continuous Obstacle", fill=(50, 50, 60))

    draw.rectangle([leg_x + 10, leg_y + 51, leg_x + 28, leg_y + 63], fill=(255, 225, 225), outline=(231, 76, 60))
    draw.text((leg_x + 35, leg_y + 51), f"Clearance ({ROBOT_CLEARANCE_RADIUS*100:.0f} cm)", fill=(50, 50, 60))

    draw.rectangle([leg_x + 10, leg_y + 72, leg_x + 26, leg_y + 84], fill=(235, 222, 240), outline=(142, 68, 173))
    draw.text((leg_x + 35, leg_y + 72), "Chassis (18x13.4 cm)", fill=(50, 50, 60))

    draw.line([(leg_x + 10, leg_y + 96), (leg_x + 28, leg_y + 96)], fill=(30, 132, 73), width=2)
    draw.text((leg_x + 35, leg_y + 91), "Navigated Path", fill=(50, 50, 60))

    draw.line([(leg_x + 10, leg_y + 116), (leg_x + 28, leg_y + 116)], fill=(52, 152, 219), width=2)
    draw.text((leg_x + 35, leg_y + 112), "Planned A* Path", fill=(50, 50, 60))

    # Header Title & Stats
    target_orient_str = f" @ {math.degrees(target_theta):+.1f}°" if target_theta is not None else ""
    draw.text((margin, 20), f"Autonomous Navigation Trajectory: Target ({goal_pos[0]:.2f}, {goal_pos[1]:.2f}) m{target_orient_str}", fill=(20, 20, 30))
    stats_str = f"Status: {status} | Dist: {total_dist:.2f}m | Time: {elapsed_s:.1f}s | Obstacles: {len(recorded_obstacles)} | Width: 13.4cm"
    draw.text((margin, 42), stats_str, fill=(70, 80, 90))

    img.save(output_path)
