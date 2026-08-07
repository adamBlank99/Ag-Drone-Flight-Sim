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

Open the default 100 m by 60 m field visualization:

```sh
python3 python/visualize_field.py
```

Use custom dimensions or save the plot to a file:

```sh
python3 python/visualize_field.py --width 150 --height 80
python3 python/visualize_field.py --save field.png
```
