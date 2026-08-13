#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import Polygon, Rectangle
from matplotlib.transforms import Affine2D
from matplotlib.widgets import Button


PROJECT_ROOT = Path(__file__).resolve().parent.parent
WAYPOINT_FILE = PROJECT_ROOT / "waypoints.csv"
POLYGON_FILE = PROJECT_ROOT / "field_polygon.csv"
FOOTPRINT_FILE = PROJECT_ROOT / "camera_footprint.csv"
OBSTACLE_FILE = PROJECT_ROOT / "obstacles.csv"
COVERAGE_STATISTICS_FILE = PROJECT_ROOT / "coverage_statistics.csv"
COVERAGE_GRID_FILE = PROJECT_ROOT / "coverage_grid.csv"


def read_points(file_path):
    if not file_path.exists():
        raise SystemExit(f"Run ./build/drone_survey first to generate {file_path.name}")

    points = []

    with file_path.open() as file:
        next(file)  # Skip the x,y header.

        for line in file:
            x, y = line.strip().split(",")
            points.append((float(x), float(y)))

    return points


def read_optimized_route(file_path):
    if not file_path.exists():
        raise SystemExit(f"Run ./build/drone_survey first to generate {file_path.name}")

    with file_path.open(newline="") as file:
        rows = list(csv.DictReader(file))

    if not rows:
        raise SystemExit(f"No optimized route found in {file_path.name}")

    metadata = {
        "angle": float(rows[0]["angle_degrees"]),
        "score": float(rows[0]["score"]),
        "distance": float(rows[0]["total_distance"]),
        "turns": int(rows[0]["turns"]),
        "mission_count": int(rows[0].get("mission_count", 1)),
        "all_missions_safe": (
            rows[0].get("all_missions_safe", "true").lower() == "true"
        ),
    }
    points = [(float(row["x"]), float(row["y"])) for row in rows]
    waypoint_types = [row["waypoint_type"] for row in rows]
    mission_ids = [int(row.get("mission_id", 1)) for row in rows]
    mission_safe = [
        row.get("mission_safe", "true").lower() == "true"
        for row in rows
    ]
    battery_used = [
        float(row.get("battery_used_percent", 0.0))
        for row in rows
    ]

    return (
        metadata,
        points,
        waypoint_types,
        mission_ids,
        mission_safe,
        battery_used,
    )


def read_obstacles(file_path):
    if not file_path.exists():
        raise SystemExit(f"Run ./build/drone_survey first to generate {file_path.name}")

    grouped = {}

    with file_path.open(newline="") as file:
        for row in csv.DictReader(file):
            key = (row["name"], row["boundary"])
            grouped.setdefault(
                key,
                {
                    "name": row["name"],
                    "type": row["type"],
                    "clearance": float(row["clearance"]),
                    "boundary": row["boundary"],
                    "vertices": [],
                },
            )
            grouped[key]["vertices"].append(
                (float(row["x"]), float(row["y"]))
            )

    return list(grouped.values())


def read_coverage_quality(statistics_path, grid_path):
    if not statistics_path.exists() or not grid_path.exists():
        raise SystemExit(
            "Run ./build/drone_survey first to generate coverage analysis data"
        )

    with statistics_path.open(newline="") as file:
        row = next(csv.DictReader(file), None)

    if row is None:
        raise SystemExit(f"No coverage statistics found in {statistics_path.name}")

    statistics = {
        name: float(value)
        for name, value in row.items()
    }
    cells = []

    with grid_path.open(newline="") as file:
        for cell in csv.DictReader(file):
            cells.append(
                {
                    "x": float(cell["x"]),
                    "y": float(cell["y"]),
                    "size": float(cell["cell_size"]),
                    "count": int(cell["coverage_count"]),
                    "status": cell["status"],
                }
            )

    return statistics, cells


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

(
    route_metadata,
    waypoints,
    waypoint_types,
    mission_ids,
    mission_safe,
    battery_used,
) = read_optimized_route(WAYPOINT_FILE)
polygon_vertices = read_points(POLYGON_FILE)
obstacles = read_obstacles(OBSTACLE_FILE)
coverage_statistics, coverage_cells = read_coverage_quality(
    COVERAGE_STATISTICS_FILE,
    COVERAGE_GRID_FILE,
)


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
    "overlap": ("mediumpurple", 0.62, "Covered multiple times"),
}

for status, (color, alpha, label) in coverage_styles.items():
    cell_polygons = []

    for cell in coverage_cells:
        if cell["status"] != status:
            continue

        half_size = cell["size"] / 2.0
        cell_polygons.append(
            [
                (cell["x"] - half_size, cell["y"] - half_size),
                (cell["x"] + half_size, cell["y"] - half_size),
                (cell["x"] + half_size, cell["y"] + half_size),
                (cell["x"] - half_size, cell["y"] + half_size),
            ]
        )

    if not cell_polygons:
        continue

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

route_line, = axes.plot(
    x_values,
    y_values,
    color="blue" if route_metadata["mission_count"] == 1 else "lightgray",
    marker="o",
    linewidth=2 if route_metadata["mission_count"] == 1 else 1,
    label=(
        f"Optimized route ({route_metadata['angle']:g}°)"
        if route_metadata["mission_count"] == 1
        else "All mission waypoints"
    ),
    picker=7,
    zorder=10,
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
            linewidth=2.5,
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

footprint_width, footprint_height = read_points(FOOTPRINT_FILE)[0]
first_x, first_y = waypoints[0]

coverage_trail = PolyCollection(
    [],
    facecolors="gold",
    edgecolors="goldenrod",
    linewidths=0.4,
    alpha=0.22,
    label="Camera coverage trail",
    zorder=3,
)
axes.add_collection(coverage_trail)

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
    marker="^",
    markersize=11,
    linestyle="None",
    label="Drone",
    zorder=15,
)

completed_route, = axes.plot(
    [first_x],
    [first_y],
    color="orange",
    linewidth=3,
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

current_waypoint = 0
furthest_waypoint = 0
is_playing = False


def route_heading(waypoint_index):
    """Return the heading from this waypoint toward the next route point."""
    if len(waypoints) == 1:
        return route_metadata["angle"]

    neighbor_index = (
        waypoint_index + 1
        if waypoint_index < len(waypoints) - 1
        else waypoint_index - 1
    )
    x, y = waypoints[waypoint_index]
    neighbor_x, neighbor_y = waypoints[neighbor_index]

    if waypoint_index == len(waypoints) - 1:
        delta_x = x - neighbor_x
        delta_y = y - neighbor_y
    else:
        delta_x = neighbor_x - x
        delta_y = neighbor_y - y

    if delta_x == 0.0 and delta_y == 0.0:
        return route_metadata["angle"]

    return math.degrees(math.atan2(delta_y, delta_x))


def coverage_rectangle(center, forward_length, cross_width, heading):
    """Build the corners of one rotated camera-coverage rectangle."""
    angle = math.radians(heading)
    forward_x = math.cos(angle)
    forward_y = math.sin(angle)
    cross_x = -forward_y
    cross_y = forward_x
    half_forward = forward_length / 2.0
    half_cross = cross_width / 2.0
    center_x, center_y = center

    return [
        (
            center_x - forward_x * half_forward - cross_x * half_cross,
            center_y - forward_y * half_forward - cross_y * half_cross,
        ),
        (
            center_x + forward_x * half_forward - cross_x * half_cross,
            center_y + forward_y * half_forward - cross_y * half_cross,
        ),
        (
            center_x + forward_x * half_forward + cross_x * half_cross,
            center_y + forward_y * half_forward + cross_y * half_cross,
        ),
        (
            center_x - forward_x * half_forward + cross_x * half_cross,
            center_y - forward_y * half_forward + cross_y * half_cross,
        ),
    ]


def build_coverage_trail(last_waypoint):
    """Create camera footprints and swept swaths through the flown route."""
    coverage_polygons = []

    for index in range(last_waypoint + 1):
        if waypoint_types[index] == "transit":
            continue

        coverage_polygons.append(
            coverage_rectangle(
                waypoints[index],
                footprint_height,
                footprint_width,
                route_heading(index),
            )
        )

    for index in range(1, last_waypoint + 1):
        if (
            waypoint_types[index - 1] == "transit"
            or waypoint_types[index] == "transit"
        ):
            continue

        start_x, start_y = waypoints[index - 1]
        end_x, end_y = waypoints[index]
        delta_x = end_x - start_x
        delta_y = end_y - start_y
        segment_length = math.hypot(delta_x, delta_y)

        if segment_length == 0.0:
            continue

        coverage_polygons.append(
            coverage_rectangle(
                ((start_x + end_x) / 2.0, (start_y + end_y) / 2.0),
                segment_length,
                footprint_width,
                math.degrees(math.atan2(delta_y, delta_x)),
            )
        )

    return coverage_polygons


def move_drone(waypoint_index):
    """Move the drone and its ground footprint to one route waypoint."""
    global current_waypoint, furthest_waypoint

    current_waypoint = max(0, min(waypoint_index, len(waypoints) - 1))
    furthest_waypoint = max(furthest_waypoint, current_waypoint)
    x, y = waypoints[current_waypoint]
    heading = route_heading(current_waypoint)

    drone_marker.set_data([x], [y])
    selected_waypoint.set_data([x], [y])
    completed_route.set_data(
        x_values[: furthest_waypoint + 1],
        y_values[: furthest_waypoint + 1],
    )
    coverage_trail.set_verts(build_coverage_trail(furthest_waypoint))

    camera_footprint.set_xy(
        (
            x - footprint_height / 2.0,
            y - footprint_width / 2.0,
        )
    )
    camera_footprint.set_transform(
        Affine2D().rotate_deg_around(
            x,
            y,
            heading,
        ) + axes.transData
    )
    camera_footprint.set_visible(
        arguments.show_footprint
        and waypoint_types[current_waypoint] != "transit"
    )

    waypoint_status.set_text(
        f"Waypoint {current_waypoint + 1}/{len(waypoints)}  |  "
        f"Mission {mission_ids[current_waypoint]}/"
        f"{route_metadata['mission_count']}  |  "
        f"Type: {waypoint_types[current_waypoint].replace('_', ' ')}  |  "
        f"Position: ({x:.1f}, {y:.1f}) m  |  Heading: {heading:.0f}°  |  "
        f"Battery: {battery_used[current_waypoint]:.1f}%"
    )
    figure.canvas.draw_idle()


def select_waypoint(event):
    if event.artist is not route_line or not len(event.ind):
        return

    move_drone(int(event.ind[0]))


def previous_waypoint(_event):
    move_drone(current_waypoint - 1)


def next_waypoint(_event):
    move_drone(current_waypoint + 1)


def advance_animation():
    if not is_playing:
        return

    if current_waypoint >= len(waypoints) - 1:
        stop_animation()
        return

    move_drone(current_waypoint + 1)

    if current_waypoint >= len(waypoints) - 1:
        stop_animation()


def stop_animation():
    global is_playing

    is_playing = False
    animation_timer.stop()
    play_button.label.set_text("Play")


def reset_flight(_event=None):
    global furthest_waypoint

    stop_animation()
    furthest_waypoint = 0
    move_drone(0)


def toggle_animation(_event):
    global is_playing

    if is_playing:
        stop_animation()
        return

    if current_waypoint >= len(waypoints) - 1:
        reset_flight()

    is_playing = True
    play_button.label.set_text("Pause")
    animation_timer.start()


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

animation_timer = figure.canvas.new_timer(interval=350)
animation_timer.add_callback(advance_animation)

figure.canvas.mpl_connect("pick_event", select_waypoint)

axes.legend(loc="upper left", bbox_to_anchor=(1.02, 1.0))
figure.subplots_adjust(left=0.08, right=0.78, bottom=0.18, top=0.92)
move_drone(0)

plt.show()
