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
