#!/usr/bin/env python3
"""Mission visualization implementation used by visualize_field.py."""

import argparse
import bisect
import math
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection, PolyCollection
from matplotlib.colors import to_rgba
from matplotlib.patches import Polygon, Rectangle
from matplotlib.path import Path as MarkerPath
from matplotlib.transforms import Affine2D
from matplotlib.widgets import Button

from mission_data import load_mission


PROJECT_ROOT = Path(__file__).resolve().parent.parent


parser = argparse.ArgumentParser(
    description="Visualize and interact with the generated drone route."
)
footprint_group = parser.add_mutually_exclusive_group()
footprint_group.add_argument(
    "--show-footprint",
    dest="show_footprint",
    action="store_true",
    help="show the moving camera footprint (default)",
)
footprint_group.add_argument(
    "--hide-footprint",
    dest="show_footprint",
    action="store_false",
    help="hide the moving camera footprint",
)
parser.set_defaults(show_footprint=True)
arguments = parser.parse_args()

mission = load_mission(PROJECT_ROOT)
route_metadata = mission.route.metadata
waypoints = mission.route.points
waypoint_types = mission.route.waypoint_types
mission_ids = mission.route.mission_ids
mission_safe = mission.route.mission_safe
battery_used = mission.route.battery_used
polygon_vertices = mission.field
obstacles = [{"name": item.name, "type": item.obstacle_type,
              "boundary": item.boundary, "vertices": item.vertices}
             for item in mission.obstacles]
coverage_statistics = mission.coverage_statistics
coverage_cells = [vars(cell) for cell in mission.coverage_cells]


figure, axes = plt.subplots(figsize=(12, 7.5))

field = Polygon(
    polygon_vertices,
    closed=True,
    facecolor="#f4f4ef",
    edgecolor="darkgreen",
    linewidth=2,
    zorder=0,
)

axes.add_patch(field)

coverage_styles = {
    "covered": ("cornflowerblue", 0.42, "Covered once"),
    "missed": ("crimson", 0.85, "Missed"),
    "overlap": ("royalblue", 0.52, "Covered multiple times"),
}
coverage_layers = {}
coverage_cell_polygons = []

for cell in coverage_cells:
    half_size = cell["size"] / 2.0
    coverage_cell_polygons.append(
        [
            (cell["x"] - half_size, cell["y"] - half_size),
            (cell["x"] + half_size, cell["y"] - half_size),
            (cell["x"] + half_size, cell["y"] + half_size),
            (cell["x"] - half_size, cell["y"] + half_size),
        ]
    )

for status, (color, alpha, label) in coverage_styles.items():
    cell_polygons = [
        coverage_cell_polygons[index]
        for index, cell in enumerate(coverage_cells)
        if cell["status"] == status
    ]

    quality_layer = PolyCollection(
        cell_polygons,
        facecolors=color,
        edgecolors="none",
        alpha=alpha,
        label=label,
        zorder=1,
    )
    quality_layer.set_clip_path(field)
    axes.add_collection(quality_layer)
    coverage_layers[status] = quality_layer

if coverage_cells:
    cell_size = coverage_cells[0]["size"]
    min_x, max_x = min(c["x"] for c in coverage_cells), max(c["x"] for c in coverage_cells)
    min_y, max_y = min(c["y"] for c in coverage_cells), max(c["y"] for c in coverage_cells)
    columns = int(round((max_x - min_x) / cell_size)) + 1
    rows = int(round((max_y - min_y) / cell_size)) + 1
    dynamic_coverage_grid = np.zeros((rows, columns, 4), dtype=float)
    coverage_grid_positions = [
        (int(round((cell["y"] - min_y) / cell_size)),
         int(round((cell["x"] - min_x) / cell_size)))
        for cell in coverage_cells
    ]
    coverage_extent = (min_x-cell_size/2, max_x+cell_size/2,
                       min_y-cell_size/2, max_y+cell_size/2)
else:
    dynamic_coverage_grid = np.zeros((1, 1, 4), dtype=float)
    coverage_grid_positions, coverage_extent = [], (0, 1, 0, 1)

dynamic_coverage_image = axes.imshow(dynamic_coverage_grid, extent=coverage_extent,
                                     origin="lower", interpolation="nearest",
                                     visible=False, zorder=3)
dynamic_coverage_image.set_clip_path(field)
dynamic_coverage_colors = {
    1: to_rgba(coverage_styles["covered"][0], coverage_styles["covered"][1]),
    2: to_rgba(coverage_styles["overlap"][0], coverage_styles["overlap"][1]),
}

obstacle_colors = {
    "barn": "saddlebrown",
    "pond": "deepskyblue",
    "trees": "forestgreen",
    "restricted": "crimson",
}
shown_types = set()
safety_label_added = False

for obstacle in obstacles:
    if obstacle["boundary"] == "safety":
        safety_boundary = Polygon(
            obstacle["vertices"],
            closed=True,
            fill=False,
            edgecolor="darkorange",
            linestyle="--",
            linewidth=1.5,
            label="Safety clearance" if not safety_label_added else None,
            zorder=7,
        )
        axes.add_patch(safety_boundary)
        safety_label_added = True
        continue

    obstacle_type = obstacle["type"]
    obstacle_patch = Polygon(
        obstacle["vertices"],
        closed=True,
        facecolor=obstacle_colors.get(obstacle_type, "gray"),
        edgecolor="black",
        alpha=0.75,
        linewidth=1.5,
        label=obstacle_type.title() if obstacle_type not in shown_types else None,
        zorder=8,
    )
    axes.add_patch(obstacle_patch)
    shown_types.add(obstacle_type)

polygon_x = [point[0] for point in polygon_vertices]
polygon_y = [point[1] for point in polygon_vertices]
margin = max(max(polygon_x) - min(polygon_x), max(polygon_y) - min(polygon_y)) * 0.05

axes.set_xlim(min(polygon_x) - margin, max(polygon_x) + margin)
axes.set_ylim(min(polygon_y) - margin, max(polygon_y) + margin)
axes.set_aspect("equal")
axes.set_xlabel("Width (m)")
axes.set_ylabel("Height (m)")
axes.set_title(
    f"Agricultural Drone Coverage Route — "
    f"Selected angle: {route_metadata['angle']:g}° — "
    f"{route_metadata['mission_count']} mission"
    f"{'s' if route_metadata['mission_count'] != 1 else ''} — "
    f"{coverage_statistics['coverage_percent']:.1f}% covered"
)
axes.grid(True)
axes.text(
    0.99,
    0.01,
    (
        f"Required: {coverage_statistics['required_area']:.0f} m²\n"
        f"Covered: {coverage_statistics['covered_area']:.0f} m²\n"
        f"Missed: {coverage_statistics['missed_area']:.0f} m²\n"
        f"Overlap: {coverage_statistics['overlap_area']:.0f} m²"
    ),
    transform=axes.transAxes,
    horizontalalignment="right",
    verticalalignment="bottom",
    bbox={"facecolor": "white", "alpha": 0.82, "edgecolor": "gray"},
    zorder=20,
)

x_values = [point[0] for point in waypoints]
y_values = [point[1] for point in waypoints]


def classify_route_segment(start_type, end_type):
    if start_type == "coverage_start" and end_type == "coverage_end":
        return "coverage_pass"
    if start_type == "transit" or end_type == "transit":
        return "home_transit"
    if start_type == "detour" or end_type == "detour":
        return "obstacle_detour"
    return "normal_transition"


route_segment_styles = {
    "coverage_pass": {
        "color": "#1756d1",
        "linestyle": "solid",
        "linewidth": 2.8,
        "label": "Coverage pass",
    },
    "normal_transition": {
        "color": "#f39c12",
        "linestyle": "dashed",
        "linewidth": 2.4,
        "label": "Normal transition",
    },
    "obstacle_detour": {
        "color": "#d62728",
        "linestyle": "dotted",
        "linewidth": 3.0,
        "label": "Obstacle detour",
    },
    "home_transit": {
        "color": "#7b3294",
        "linestyle": "dashdot",
        "linewidth": 2.4,
        "label": "Home transit",
    },
}
route_segments_by_type = {
    segment_type: []
    for segment_type in route_segment_styles
}

for index in range(1, len(waypoints)):
    if mission_ids[index - 1] != mission_ids[index]:
        continue

    segment_type = classify_route_segment(
        waypoint_types[index - 1],
        waypoint_types[index],
    )
    route_segments_by_type[segment_type].append(
        [waypoints[index - 1], waypoints[index]]
    )

for segment_type, segments in route_segments_by_type.items():
    if not segments:
        continue

    style = route_segment_styles[segment_type]
    axes.add_collection(
        LineCollection(
            segments,
            colors=style["color"],
            linestyles=style["linestyle"],
            linewidths=style["linewidth"],
            label=style["label"],
            zorder=10,
        )
    )

ANIMATION_STEP_METERS = 2.5
PLAYBACK_SPEED_METERS_PER_SECOND = 85.0
ANIMATION_TIMER_INTERVAL_MS = 30


def heading_between(start, end, fallback):
    delta_x = end[0] - start[0]
    delta_y = end[1] - start[1]

    if delta_x == 0.0 and delta_y == 0.0:
        return fallback

    return math.degrees(math.atan2(delta_y, delta_x))


def drone_marker_path(heading):
    """Return an arrow-shaped marker pointing along a route heading."""
    angle = math.radians(heading)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    base_vertices = [
        (1.0, 0.0),
        (-0.65, 0.68),
        (-0.30, 0.0),
        (-0.65, -0.68),
        (1.0, 0.0),
    ]
    rotated_vertices = [
        (
            x * cosine - y * sine,
            x * sine + y * cosine,
        )
        for x, y in base_vertices
    ]
    return MarkerPath(
        rotated_vertices,
        [
            MarkerPath.MOVETO,
            MarkerPath.LINETO,
            MarkerPath.LINETO,
            MarkerPath.LINETO,
            MarkerPath.CLOSEPOLY,
        ],
    )


mission_route_distances = {
    mission_id: 0.0
    for mission_id in set(mission_ids)
}
mission_battery_totals = {}

for index, mission_id in enumerate(mission_ids):
    mission_battery_totals.setdefault(mission_id, battery_used[index])

    if index == 0 or mission_ids[index - 1] != mission_id:
        continue

    mission_route_distances[mission_id] += math.dist(
        waypoints[index - 1],
        waypoints[index],
    )

initial_heading = (
    heading_between(waypoints[0], waypoints[1], route_metadata["angle"])
    if len(waypoints) > 1
    else route_metadata["angle"]
)
route_frames = [
    {
        "x": waypoints[0][0],
        "y": waypoints[0][1],
        "heading": initial_heading,
        "mission_id": mission_ids[0],
        "mission_distance": 0.0,
        "segment_index": 0,
        "segment_fraction": 0.0,
        "survey_segment": None,
    }
]
waypoint_frame_indices = [0] * len(waypoints)
mission_distance = 0.0

for segment_index in range(len(waypoints) - 1):
    start = waypoints[segment_index]
    end = waypoints[segment_index + 1]
    start_mission = mission_ids[segment_index]
    end_mission = mission_ids[segment_index + 1]
    heading = heading_between(start, end, route_metadata["angle"])

    if start_mission != end_mission:
        mission_distance = 0.0
        route_frames.append(
            {
                "x": end[0],
                "y": end[1],
                "heading": (
                    heading_between(
                        end,
                        waypoints[segment_index + 2],
                        route_metadata["angle"],
                    )
                    if segment_index + 2 < len(waypoints)
                    else route_metadata["angle"]
                ),
                "mission_id": end_mission,
                "mission_distance": 0.0,
                "segment_index": segment_index + 1,
                "segment_fraction": 0.0,
                "survey_segment": None,
            }
        )
        waypoint_frame_indices[segment_index + 1] = len(route_frames) - 1
        continue

    segment_length = math.dist(start, end)
    survey_segment = (
        segment_index
        if waypoint_types[segment_index] == "coverage_start"
        and waypoint_types[segment_index + 1] == "coverage_end"
        else None
    )

    if survey_segment is not None:
        route_frames.append(
            {
                "x": start[0],
                "y": start[1],
                "heading": heading,
                "mission_id": start_mission,
                "mission_distance": mission_distance,
                "segment_index": segment_index,
                "segment_fraction": 0.0,
                "survey_segment": survey_segment,
            }
        )

    step_count = max(1, math.ceil(segment_length / ANIMATION_STEP_METERS))

    for step in range(1, step_count + 1):
        fraction = step / step_count
        route_frames.append(
            {
                "x": start[0] + (end[0] - start[0]) * fraction,
                "y": start[1] + (end[1] - start[1]) * fraction,
                "heading": heading,
                "mission_id": start_mission,
                "mission_distance": mission_distance + segment_length * fraction,
                "segment_index": segment_index,
                "segment_fraction": fraction,
                "survey_segment": survey_segment,
            }
        )

    mission_distance += segment_length
    waypoint_frame_indices[segment_index + 1] = len(route_frames) - 1

route_frame_distances = [0.0]
for previous, current in zip(route_frames, route_frames[1:]):
    distance = (math.dist((previous["x"], previous["y"]), (current["x"], current["y"]))
                if previous["mission_id"] == current["mission_id"] else 0.0)
    route_frame_distances.append(route_frame_distances[-1] + distance)

route_line, = axes.plot(
    x_values,
    y_values,
    color="#1756d1",
    marker="o",
    linestyle="None",
    label="Route waypoints",
    picker=7,
    zorder=11,
)

if route_metadata["mission_count"] > 1:
    mission_colors = plt.get_cmap("tab10")

    for mission_id in sorted(set(mission_ids)):
        indices = [
            index
            for index, point_mission_id in enumerate(mission_ids)
            if point_mission_id == mission_id
        ]
        color = (
            mission_colors((mission_id - 1) % 10)
            if mission_safe[indices[0]]
            else "crimson"
        )
        axes.plot(
            [x_values[index] for index in indices],
            [y_values[index] for index in indices],
            color=color,
            linewidth=1.3,
            alpha=0.25,
            label=(
                f"Mission {mission_id}: "
                f"{battery_used[indices[0]]:.1f}% battery"
                f"{'' if mission_safe[indices[0]] else ' (unsafe)'}"
            ),
            zorder=11,
        )

detour_points = [
    point
    for point, waypoint_type in zip(waypoints, waypoint_types)
    if waypoint_type == "detour"
]

if detour_points:
    axes.scatter(
        [point[0] for point in detour_points],
        [point[1] for point in detour_points],
        color="red",
        marker="D",
        s=45,
        label="Detour waypoint",
        zorder=12,
    )

transit_points = [
    point
    for point, waypoint_type in zip(waypoints, waypoint_types)
    if waypoint_type == "transit"
]

if transit_points:
    axes.scatter(
        [point[0] for point in transit_points],
        [point[1] for point in transit_points],
        color="purple",
        marker="s",
        s=35,
        label="Home transit",
        zorder=12,
    )

footprint_width, footprint_height = mission.footprint
first_x, first_y = waypoints[0]

camera_footprint = Rectangle(
    (
        first_x - footprint_height / 2.0,
        first_y - footprint_width / 2.0,
    ),
    footprint_height,
    footprint_width,
    facecolor="orange",
    edgecolor="darkorange",
    alpha=0.30,
    linewidth=2,
    label="Camera FOV footprint",
    visible=arguments.show_footprint,
    zorder=6,
)
axes.add_patch(camera_footprint)

drone_marker, = axes.plot(
    [first_x],
    [first_y],
    color="black",
    marker=drone_marker_path(initial_heading),
    markersize=13,
    linestyle="None",
    label="Drone",
    zorder=15,
)

completed_route, = axes.plot(
    [first_x],
    [first_y],
    color="#202020",
    linewidth=4,
    alpha=0.45,
    zorder=13,
)

selected_waypoint, = axes.plot(
    [first_x],
    [first_y],
    marker="o",
    markersize=13,
    markerfacecolor="none",
    markeredgecolor="black",
    markeredgewidth=2,
    linestyle="None",
    zorder=16,
)

battery_axes = figure.add_axes([0.55, 0.025, 0.18, 0.035])
battery_axes.set_xlim(0.0, 100.0)
battery_axes.set_ylim(0.0, 1.0)
battery_axes.set_xticks([])
battery_axes.set_yticks([])
battery_axes.set_facecolor("#e6e6e6")
battery_fill = Rectangle(
    (0.0, 0.0),
    100.0,
    1.0,
    facecolor="forestgreen",
    edgecolor="none",
)
battery_axes.add_patch(battery_fill)
battery_label = battery_axes.text(
    50.0,
    0.5,
    "Battery: 100%",
    horizontalalignment="center",
    verticalalignment="center",
    fontsize=9,
    fontweight="bold",
)


current_frame = 0
is_playing = False
stop_at_frame = None
playback_started = False
playback_distance_target = 0.0
last_animation_time = None
drone_legend_handle = None
dynamic_coverage_segments = [set() for _cell in coverage_cells]


def update_dynamic_coverage(changed):
    for index in set(changed):
        row, column = coverage_grid_positions[index]
        count = len(dynamic_coverage_segments[index])
        dynamic_coverage_grid[row, column] = ((0, 0, 0, 0) if count == 0
                                               else dynamic_coverage_colors[min(count, 2)])
    dynamic_coverage_image.set_data(dynamic_coverage_grid)


def clear_dynamic_coverage():
    for observed_segments in dynamic_coverage_segments:
        observed_segments.clear()

    dynamic_coverage_grid.fill(0)
    dynamic_coverage_image.set_data(dynamic_coverage_grid)


def observe_survey_coverage(frame):
    """Reveal the portion of one camera swath reached by this route frame."""
    survey_segment = frame["survey_segment"]

    if survey_segment is None:
        return []

    start = waypoints[survey_segment]
    end = waypoints[survey_segment + 1]
    direction_x = end[0] - start[0]
    direction_y = end[1] - start[1]
    segment_length = math.hypot(direction_x, direction_y)

    if segment_length == 0.0:
        return []

    unit_x = direction_x / segment_length
    unit_y = direction_y / segment_length
    reached_distance = segment_length * frame["segment_fraction"]
    half_forward_footprint = footprint_height / 2.0
    half_side_footprint = footprint_width / 2.0
    changed = []

    for index, cell in enumerate(coverage_cells):
        if (
            cell["count"] == 0
            or survey_segment in dynamic_coverage_segments[index]
        ):
            continue

        relative_x = cell["x"] - start[0]
        relative_y = cell["y"] - start[1]
        along_track = relative_x * unit_x + relative_y * unit_y
        cross_track = -relative_x * unit_y + relative_y * unit_x

        if (
            -half_forward_footprint <= along_track
            <= reached_distance + half_forward_footprint
            and abs(cross_track) <= half_side_footprint
        ):
            dynamic_coverage_segments[index].add(survey_segment)
            changed.append(index)

    return changed


def begin_playback():
    global playback_started

    if playback_started:
        return

    playback_started = True
    coverage_layers["covered"].set_visible(False)
    coverage_layers["overlap"].set_visible(False)
    dynamic_coverage_image.set_visible(True)
    clear_dynamic_coverage()


def battery_remaining_for_frame(frame):
    mission_id = frame["mission_id"]
    total_distance = mission_route_distances[mission_id]
    progress = (
        min(1.0, frame["mission_distance"] / total_distance)
        if total_distance > 0.0
        else 0.0
    )
    used = mission_battery_totals[mission_id] * progress
    return max(0.0, 100.0 - used), used


def update_battery_display(frame):
    remaining, _used = battery_remaining_for_frame(frame)
    mission_id = frame["mission_id"]
    battery_fill.set_width(remaining)

    if remaining > 30.0:
        battery_fill.set_facecolor("forestgreen")
    elif remaining > 20.0:
        battery_fill.set_facecolor("darkorange")
    else:
        battery_fill.set_facecolor("crimson")

    battery_label.set_text(
        f"Mission {mission_id} battery: {remaining:.1f}%"
    )


def update_drone_orientation(heading):
    marker = drone_marker_path(heading)
    drone_marker.set_marker(marker)

    if drone_legend_handle is not None:
        drone_legend_handle.set_marker(marker)


def move_drone_to_frame(frame_index, rebuild_coverage=False):
    """Move the drone to a continuous route frame and update mission state."""
    global current_frame

    previous_index = current_frame
    current_frame = max(0, min(frame_index, len(route_frames) - 1))
    frame = route_frames[current_frame]

    if playback_started:
        if rebuild_coverage:
            clear_dynamic_coverage()
            coverage_frames = route_frames[:current_frame + 1]
        elif current_frame >= previous_index:
            coverage_frames = route_frames[previous_index + 1:current_frame + 1]
        else:
            clear_dynamic_coverage()
            coverage_frames = route_frames[:current_frame + 1]
        changed = []
        for coverage_frame in coverage_frames:
            changed.extend(observe_survey_coverage(coverage_frame))
        update_dynamic_coverage(changed)

    x = frame["x"]
    y = frame["y"]
    heading = frame["heading"]
    drone_marker.set_data([x], [y])
    selected_waypoint.set_data([x], [y])
    update_drone_orientation(heading)
    completed_route.set_data(
        [route_frame["x"] for route_frame in route_frames[: current_frame + 1]],
        [route_frame["y"] for route_frame in route_frames[: current_frame + 1]],
    )

    camera_footprint.set_xy(
        (
            x - footprint_height / 2.0,
            y - footprint_width / 2.0,
        )
    )
    camera_footprint.set_transform(
        Affine2D().rotate_deg_around(x, y, heading) + axes.transData
    )
    camera_footprint.set_visible(
        arguments.show_footprint and frame["survey_segment"] is not None
    )
    update_battery_display(frame)

    segment_index = frame["segment_index"]
    target_waypoint = min(segment_index + 1, len(waypoints) - 1)
    remaining, used = battery_remaining_for_frame(frame)
    route_type = (
        "survey"
        if frame["survey_segment"] is not None
        else waypoint_types[target_waypoint].replace("_", " ")
    )
    waypoint_status.set_text(
        f"Approaching waypoint {target_waypoint + 1}/{len(waypoints)}  |  "
        f"Mission {frame['mission_id']}/{route_metadata['mission_count']}  |  "
        f"Type: {route_type}  |  Position: ({x:.1f}, {y:.1f}) m  |  "
        f"Heading: {heading:.0f}°  |  "
        f"Battery: {remaining:.1f}% remaining ({used:.1f}% used)"
    )
    figure.canvas.draw_idle()


def select_waypoint(event):
    if event.artist is not route_line or not len(event.ind):
        return

    stop_animation()
    begin_playback()
    move_drone_to_frame(
        waypoint_frame_indices[int(event.ind[0])],
        rebuild_coverage=True,
    )


def previous_waypoint(_event):
    stop_animation()
    begin_playback()
    earlier_frames = [
        frame_index
        for frame_index in waypoint_frame_indices
        if frame_index < current_frame
    ]
    target_frame = max(earlier_frames) if earlier_frames else 0
    move_drone_to_frame(target_frame, rebuild_coverage=True)


def next_waypoint(_event):
    begin_playback()
    later_frames = [
        frame_index
        for frame_index in waypoint_frame_indices
        if frame_index > current_frame
    ]

    if not later_frames:
        return

    start_animation(min(later_frames))


def start_animation(target=None):
    global is_playing, stop_at_frame, playback_distance_target, last_animation_time
    stop_at_frame = target
    playback_distance_target = route_frame_distances[current_frame]
    last_animation_time = time.monotonic()
    is_playing = True
    play_button.label.set_text("Pause")
    animation_timer.start()


def advance_animation():
    global playback_distance_target, last_animation_time
    if not is_playing:
        return

    if current_frame >= len(route_frames) - 1:
        stop_animation()
        return

    now = time.monotonic()
    playback_distance_target += PLAYBACK_SPEED_METERS_PER_SECOND * max(0, now - last_animation_time)
    last_animation_time = now
    target = max(current_frame, bisect.bisect_right(route_frame_distances, playback_distance_target) - 1)
    if stop_at_frame is not None:
        target = min(target, stop_at_frame)
    if target > current_frame:
        move_drone_to_frame(target)

    if (
        current_frame >= len(route_frames) - 1
        or (
            stop_at_frame is not None
            and current_frame >= stop_at_frame
        )
    ):
        stop_animation()


def stop_animation():
    global is_playing, stop_at_frame, last_animation_time

    is_playing = False
    stop_at_frame = None
    last_animation_time = None
    animation_timer.stop()
    play_button.label.set_text("Play")


def reset_flight(_event=None):
    stop_animation()

    if playback_started:
        clear_dynamic_coverage()

    move_drone_to_frame(0, rebuild_coverage=playback_started)


def toggle_animation(_event):
    if is_playing:
        stop_animation()
        return

    begin_playback()

    if current_frame >= len(route_frames) - 1:
        reset_flight()

    start_animation()


waypoint_status = figure.text(0.50, 0.105, "", ha="center")

previous_axes = figure.add_axes([0.10, 0.02, 0.09, 0.055])
play_axes = figure.add_axes([0.20, 0.02, 0.09, 0.055])
next_axes = figure.add_axes([0.30, 0.02, 0.09, 0.055])
reset_axes = figure.add_axes([0.40, 0.02, 0.09, 0.055])

previous_button = Button(previous_axes, "Previous")
play_button = Button(play_axes, "Play")
next_button = Button(next_axes, "Next")
reset_button = Button(reset_axes, "Reset")

previous_button.on_clicked(previous_waypoint)
play_button.on_clicked(toggle_animation)
next_button.on_clicked(next_waypoint)
reset_button.on_clicked(reset_flight)

animation_timer = figure.canvas.new_timer(interval=ANIMATION_TIMER_INTERVAL_MS)
animation_timer.add_callback(advance_animation)

figure.canvas.mpl_connect("pick_event", select_waypoint)

route_legend = axes.legend(loc="upper left", bbox_to_anchor=(1.02, 1.0))

for legend_text, legend_handle in zip(
    route_legend.get_texts(),
    route_legend.legend_handles,
):
    if legend_text.get_text() == "Drone":
        drone_legend_handle = legend_handle
        break

figure.subplots_adjust(left=0.08, right=0.78, bottom=0.18, top=0.92)
move_drone_to_frame(0)

plt.show()
