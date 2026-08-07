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
Python visualizer reads.
