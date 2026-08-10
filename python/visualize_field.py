#!/usr/bin/env python3

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle


PROJECT_ROOT = Path(__file__).resolve().parent.parent
WAYPOINT_FILE = PROJECT_ROOT / "waypoints.csv"
POLYGON_FILE = PROJECT_ROOT / "field_polygon.csv"
FOOTPRINT_FILE = PROJECT_ROOT / "camera_footprint.csv"


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


parser = argparse.ArgumentParser(description="Visualize the generated drone route.")
parser.add_argument(
    "--show-footprint",
    action="store_true",
    help="show the camera footprint centered on the first waypoint",
)
arguments = parser.parse_args()

waypoints = read_points(WAYPOINT_FILE)
polygon_vertices = read_points(POLYGON_FILE)


figure, axes = plt.subplots()

field = Polygon(
    polygon_vertices,
    closed=True,
    facecolor="lightgreen",
    edgecolor="darkgreen",
    linewidth=2,
)

axes.add_patch(field)

polygon_x = [point[0] for point in polygon_vertices]
polygon_y = [point[1] for point in polygon_vertices]
margin = max(max(polygon_x) - min(polygon_x), max(polygon_y) - min(polygon_y)) * 0.05

axes.set_xlim(min(polygon_x) - margin, max(polygon_x) + margin)
axes.set_ylim(min(polygon_y) - margin, max(polygon_y) + margin)
axes.set_aspect("equal")
axes.set_xlabel("Width (m)")
axes.set_ylabel("Height (m)")
axes.set_title("Agricultural Drone Coverage Route")
axes.grid(True)

x_values = [point[0] for point in waypoints]
y_values = [point[1] for point in waypoints]

axes.plot(
    x_values,
    y_values,
    color="blue",
    marker="o",
    linewidth=2,
    label="Drone route",
)

if arguments.show_footprint:
    footprint_width, footprint_height = read_points(FOOTPRINT_FILE)[0]
    first_x, first_y = waypoints[0]

    camera_footprint = Rectangle(
        (
            first_x - footprint_width / 2.0,
            first_y - footprint_height / 2.0,
        ),
        footprint_width,
        footprint_height,
        facecolor="orange",
        edgecolor="darkorange",
        alpha=0.35,
        linewidth=2,
        label="Camera footprint at waypoint 1",
    )
    axes.add_patch(camera_footprint)
    axes.set_xlim(
        min(polygon_x + [first_x - footprint_width / 2.0]) - margin,
        max(polygon_x + [first_x + footprint_width / 2.0]) + margin,
    )
    axes.set_ylim(
        min(polygon_y + [first_y - footprint_height / 2.0]) - margin,
        max(polygon_y + [first_y + footprint_height / 2.0]) + margin,
    )

for number, (x, y) in enumerate(waypoints, start=1):
    axes.annotate(str(number), (x, y), xytext=(5, 5), textcoords="offset points")

axes.legend()

plt.show()
