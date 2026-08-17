# Agricultural Drone Coverage Planner

A C++17 route-planning and mission-analysis project for camera-based
agricultural surveys. It plans a complete lawnmower route inside an irregular
field, avoids buffered obstacles and no-fly zones, selects a flight direction,
checks battery feasibility, splits long work across batteries, and measures the
quality of the resulting camera coverage.

The included desktop launcher generates safe random scenarios, builds and tests
the project, and opens a Matplotlib application showing the field, obstacles,
safety buffers, optimized route, battery missions, camera footprint, missed
regions, and overlapping coverage.

## Features

- Irregular and concave polygon fields
- Shoelace area, bounding boxes, point-in-polygon, and segment intersections
- Camera footprint from altitude and horizontal/vertical field of view
- Independent side and forward image overlap
- Scanline clipping for polygon coverage passes
- Multiple polygon obstacles and no-fly zones with clearance buffers
- Full segment-level route safety validation
- Visibility-graph detours with Dijkstra shortest paths
- Dynamic-programming pass direction and adjacent-lane ordering
- Flight-angle comparison from 0° through 90°
- Route distance, time, pass, turn, and waypoint statistics
- Battery capacity, reserve, turn, launch, and recovery modeling
- Safe return-to-home paths and automatic multi-battery mission splitting
- Covered, missed, overlapping, redundant, and efficient coverage analysis
- Interactive launcher with validated random fields and obstacles
- Live CMake/CTest output with dynamically parsed pass counts
- Interactive Python playback with clickable waypoints and route progress
- Automated CTest regression suite

## Architecture

The reusable C++ code is built as the drone_core library:

- Geometry: polygon area, containment, intersection, rotation, and segment
  safety primitives.
- CameraConfig: ground footprint and overlap spacing calculations.
- CoveragePlanner: horizontal scanlines clipped to the field and exclusions.
- ObstacleRouter: segment validation and shortest collision-free transitions.
- RouteOptimizer: candidate-angle generation, globally selected lane
  directions, final obstacle-aware route scoring, and best-angle selection.
- MissionModel: battery estimates, safe home transit, and pass-boundary mission
  splitting.
- CoverageAnalysis: grid-based integration of unique, missed, and repeated
  camera coverage.
- SurveyScenario: validated CSV input for launcher-generated fields and
  obstacles, plus the original built-in example.
- main.cpp: scenario selection, reporting, and CSV export.

The Python launcher is a demo layer only. It writes scenario CSV files and
invokes the same C++ executable used from the command line. The Python
visualizer reads the planner's generated CSV files and remains separate from
the route-planning logic.

## Build and run

Requirements:

- CMake 3.16 or newer
- A C++17 compiler
- Python 3.10 or newer
- Matplotlib for the launcher preview, controls, and mission visualization

From the project root:

~~~sh
cmake -S . -B build
cmake --build build
./build/drone_survey
~~~

Run all automated tests:

~~~sh
ctest --test-dir build --output-on-failure
~~~

Set up the Python environment:

~~~sh
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r python/requirements.txt
~~~

Launch the interactive demo:

~~~sh
python3 python/launcher.py
~~~

The launcher starts with a generated field and obstacle preview. Use
**Randomize Field** or **Randomize Obstacles**, then click **Build & Test**.
**Visualize Mission** becomes available only after CMake succeeds and the
actual CTest summary reports that every test passed. It runs the C++ planner on
the previewed scenario and opens the full visualization.

To run the original built-in scenario without the launcher:

~~~sh
./build/drone_survey
python3 python/visualize_field.py
~~~

Hide the moving camera footprint if desired:

~~~sh
python3 python/visualize_field.py --hide-footprint
~~~

In the visualization, click any waypoint or use Previous, Play/Pause, Next,
and Reset. Play and Next move the drone through interpolated positions instead
of jumping between waypoints. Once playback starts, the covered and overlap
layers reset and are revealed by survey-camera passes as the drone moves;
transit and detour segments do not add coverage. The oriented drone marker and
its legend symbol follow the current heading, and the battery gauge decreases
through each mission using the exported battery estimate.

Coverage passes, normal transitions, obstacle detours, and home-transit edges
use separate colors and line styles. The console also prints every transition's
coordinates, classification, and distance for route auditing.

For reproducible scenario generation without opening a window:

~~~sh
python3 python/launcher.py --generate-only --seed 42
./build/drone_survey \
  --field generated_scenario/field.csv \
  --obstacles generated_scenario/obstacles.csv
~~~

## Planning pipeline

1. Calculate the camera footprint and lane spacing.
2. Rotate the field and obstacle buffers for each candidate angle.
3. Generate horizontal scanlines and clip them to the usable field.
4. Group pass fragments by scanline and use dynamic programming to select each
   lane direction, considering both bottom-to-top and top-to-bottom traversal.
5. Build collision-free transitions with a visibility graph and Dijkstra.
6. Score each completed obstacle-aware route by distance and turn penalty.
7. Rotate the winning route back to the original field coordinates.
8. Add safe outbound and return-to-home paths.
9. Estimate time and energy, then split at complete pass boundaries if needed.
10. Integrate camera observations over the serviceable field grid.

## Coverage metrics

Obstacle safety buffers are removed from the required survey area. Only direct
CoverageStart-to-CoverageEnd segments activate the camera model; detours and
home transit are excluded.

- Coverage percentage = unique covered area / required area
- Overlap percentage = area seen at least twice / required area
- Redundant coverage = every extra observation beyond the first
- Coverage efficiency = unique covered area / accumulated camera exposure
- Area per survey meter = unique covered area / survey-only route distance

The default sample uses a 1 m analysis grid. Total polygon area uses the exact
shoelace formula. Coverage-region measurements are numerical grid estimates,
with cell-area weights normalized so required and excluded areas sum to that
exact field area.

## Battery model

Each sortie includes cruise time, an allowance for every actual heading change,
and fixed takeoff/landing time:

~~~text
mission time = distance / speed + turns × turn time + launch/recovery time
~~~

Energy is derived from mission time as a fraction of the configured full-pack
capacity and under-load flight time. A mission is safe only if it lands with at
least the configured reserve. Every split mission begins and ends at home using
the same obstacle-safe shortest-path router.

## Example result

The included irregular field and four obstacles currently produce:

~~~text
Selected best angle: 30 degrees
Coverage passes: 14
Total optimized distance: 1014.6 m

Single-battery estimate: 306.3 seconds, 34.0% battery
Safe on one battery: yes

Required survey area: 6072 m²
Covered area: 6071 m²
Missed area: 1 m²
Overlapping area: 2251.5 m²
Coverage: 100.0%
Unique coverage efficiency: 72.0%
~~~

Values can change when the sample field, obstacles, camera, battery, or analysis
resolution is edited.

## Generated data

Running drone_survey writes:

- waypoints.csv: optimized missions, waypoint roles, incoming segment classes,
  battery estimates, and route metadata.
- field_polygon.csv: original field vertices.
- obstacles.csv: obstacle polygons and calculated safety boundaries.
- camera_footprint.csv: footprint width and height.
- coverage_statistics.csv: field, coverage, overlap, and efficiency metrics.
- coverage_grid.csv: serviceable analysis cells, observation counts, and
  covered/missed/overlap status.

These generated files and the build directory are ignored by Git.
The launcher also writes `generated_scenario/field.csv` and
`generated_scenario/obstacles.csv`; that directory is ignored as runtime data.

## Tests

The suite covers geometry, camera footprint calculations, polygon clipping,
concave fields, obstacle intersections, full segment safety, shortest detours,
angle selection, route statistics, battery feasibility, mission splitting, and
coverage analysis for full coverage, gaps, overlap, irregular fields,
obstacles, and multiple missions. Targeted routing tests also cover unsafe
connecting segments, shortest detour selection, pass reversal, global lane
direction selection, long-transition avoidance, and segment classification.

## Assumptions and limitations

- Coordinates are planar meters. The project does not yet convert GPS/GIS
  coordinates or model terrain elevation.
- The camera is nadir-facing with a fixed rectangular footprint. Aircraft
  attitude, lens distortion, and terrain relief are not modeled.
- Coverage areas are grid estimates. Smaller cells improve boundary accuracy at
  increased runtime and export size.
- Clearance currently expands an obstacle to a conservative axis-aligned
  bounding box rather than computing an exact polygon offset.
- Polygons are expected to be simple and non-self-intersecting. Overlapping
  exclusion polygons are not explicitly unioned.
- Launcher fields use 5–8 angularly ordered vertices. Random obstacles use
  conservative, separated safety boxes that remain fully inside the field.
- The energy model is linear in modeled flight time. Wind, temperature,
  payload changes, battery aging, and detailed climb power are future work.
- Home is configured as the first sample-field vertex.
- Pass ordering is optimized within the lawnmower constraint: adjacent
  scanline lanes may reverse and the vertical traversal may start at either
  end, but the planner does not solve an unconstrained all-pass traveling
  salesman problem.
- Candidate-angle scoring includes completed survey transitions and obstacle
  detours. Battery-specific outbound and return-home transit is added and
  validated afterward, so it is not part of the angle score.
- Mission splitting occurs only between complete coverage passes; an individual
  pass that exceeds the battery budget is reported infeasible.
