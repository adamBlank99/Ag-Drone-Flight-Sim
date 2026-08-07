#!/usr/bin/env python3

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


# These values match the field in the C++ program.
FIELD_WIDTH = 100.0
FIELD_HEIGHT = 60.0
WAYPOINT_FILE = Path(__file__).resolve().parent.parent / "waypoints.csv"


if not WAYPOINT_FILE.exists():
    raise SystemExit("Run ./build/drone_survey first to generate waypoints.csv")

waypoints = []

with WAYPOINT_FILE.open() as file:
    next(file)  # Skip the x,y header.

    for line in file:
        x, y = line.strip().split(",")
        waypoints.append((float(x), float(y)))


figure, axes = plt.subplots()

field = Rectangle(
    (0, 0),
    FIELD_WIDTH,
    FIELD_HEIGHT,
    facecolor="lightgreen",
    edgecolor="darkgreen",
    linewidth=2,
)

axes.add_patch(field)
axes.set_xlim(-5, FIELD_WIDTH + 5)
axes.set_ylim(-5, FIELD_HEIGHT + 5)
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

for number, (x, y) in enumerate(waypoints, start=1):
    axes.annotate(str(number), (x, y), xytext=(5, 5), textcoords="offset points")

axes.legend()

plt.show()
