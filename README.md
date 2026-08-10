# Ag Drone Flight Sim

A C++17 agricultural drone flight simulator.

## Build

```sh
cmake -S . -B build
cmake --build build
```

## Run

```sh
./build/drone_survey
```

## Test

```sh
ctest --test-dir build --output-on-failure
```

## Visualize the field

Install the Python dependency:

```sh
python3 -m pip install -r python/requirements.txt
```

Generate the route with C++, then display it over the field:

```sh
./build/drone_survey
python3 python/visualize_field.py
```

The C++ program writes `waypoints.csv` and `field_polygon.csv`, which the
Python visualizer reads. It also writes `camera_footprint.csv` with the
calculated ground footprint.

The visualization shows a movable drone and its calculated camera footprint.
Click a route point to move the drone directly to that waypoint, or use the
Previous, Play/Pause, and Next buttons to follow the complete route. The orange
line shows the portion of the route already flown. A translucent gold swath is
left behind by the camera to show everything it has seen. This coverage remains
visible when stepping backward; use Reset to clear it and return to waypoint 1.

The footprint is visible by default. To hide it:

```sh
python3 python/visualize_field.py --hide-footprint
```

The camera model uses separate overlap values:

- Side overlap controls spacing between adjacent flight lanes.
- Forward overlap controls spacing and capture time between photos along a lane.

The planner compares flight angles from 0 to 90 degrees in 15-degree steps.
It selects the lowest score using:

```text
score = total distance + 10 meters per turn
```

`waypoints.csv` stores the selected angle, score, distance, turn count, and
optimized waypoints for the Python visualization.

Obstacle-aware planning clips passes around barns, ponds, tree clusters, and
restricted zones. The sample mission includes all four obstacle types.
`obstacles.csv` stores original obstacle polygons and conservative safety
boundaries. Detour waypoints are identified in `waypoints.csv`.
