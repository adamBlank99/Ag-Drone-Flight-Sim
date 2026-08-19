# Agricultural Drone Coverage Planner

A **C++17 / Python drone-survey planning system** that generates optimized flight paths for irregular agricultural fields while accounting for camera coverage, obstacles, no-fly zones, battery limits, and image overlap.

The planner evaluates multiple flight directions, generates collision-free survey routes, estimates mission feasibility, and analyzes how effectively the drone photographs the field.

---

## Demo

### Scenario Generator

Use the desktop launcher to generate a random irregular field and obstacles, then build and test the planner before running the mission.

<img width="1253" height="758" alt="Screenshot 2026-08-17 at 4 25 26 AM" src="https://github.com/user-attachments/assets/4c2513b5-4c56-438e-9499-9bca0033eba9" />

### Optimized Flight Simulation

The visualization plays the drone through the optimized route while displaying obstacles, safety buffers, camera coverage, missed regions, overlapping coverage, and battery usage.


https://github.com/user-attachments/assets/e217c281-da6d-4bd1-887b-6bcb77f22d72

---

## What It Does

Given an irregular field boundary, the system:

1. Models the drone camera footprint from **altitude and field of view**
2. Calculates survey-lane spacing from the required **image overlap**
3. Generates coverage passes inside the field
4. Avoids obstacles and no-fly zones using collision-safe routing
5. Tests multiple flight angles and selects the most efficient route
6. Estimates battery usage and automatically splits oversized missions
7. Measures covered, missed, and overlapping survey area
8. Visualizes the complete mission in Python

---

## Key Algorithms

* **Computational geometry** for polygon area, point containment, line intersections, clipping, and rotation
* **Scanline coverage planning** for irregular and concave fields
* **Visibility graphs + Dijkstra's algorithm** for shortest collision-free obstacle detours
* **Dynamic programming** for survey-lane direction and ordering
* **Multi-angle route optimization** using distance and turn penalties
* **Trigonometric camera modeling** for ground footprint and overlap spacing
* **Grid-based coverage analysis** for missed and redundant imagery
* **Battery-aware mission planning** with safe return-to-home routing

---

## Current Limitations

* Coordinates currently use planar meters rather than GPS/GIS coordinates
* Terrain elevation is not modeled
* Camera orientation is assumed to be nadir-facing
* Coverage measurements use a configurable spatial grid
* Battery consumption uses a simplified linear model
* Route ordering is optimized within a structured lawnmower coverage pattern rather than an unrestricted traveling-salesman solution

---

## Technologies

**C++17 · Python · CMake · Matplotlib · Computational Geometry · Dijkstra · Dynamic Programming · Route Optimization**
