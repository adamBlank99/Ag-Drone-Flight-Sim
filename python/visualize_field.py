#!/usr/bin/env python3
"""Open the interactive visualization for the latest planned mission."""

import runpy
from pathlib import Path

if __name__ == "__main__":
    runpy.run_path(Path(__file__).with_name("mission_visualizer.py"), run_name="__main__")
