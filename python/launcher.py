#!/usr/bin/env python3
"""Interactive launcher for the agricultural drone survey demo."""

import argparse, queue, random, re, subprocess, sys, threading
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.widgets import Button
from scenario_generator import (generate_random_field, generate_random_obstacles,
                                polygon_area, safety_corners, write_scenario)

ROOT = Path(__file__).resolve().parent.parent
SCENARIO = ROOT / "generated_scenario"
FIELD_FILE, OBSTACLE_FILE = SCENARIO / "field.csv", SCENARIO / "obstacles.csv"
VISUALIZER = ROOT / "python/visualize_field.py"
TITLE = "Drone Field Survey & Route Optimization Simulator"
COLORS = {"barn": "saddlebrown", "pond": "deepskyblue",
          "trees": "forestgreen", "restricted": "crimson"}


def parse_ctest_summary(output):
    match = re.search(r"\d+% tests passed,\s*(\d+) tests failed out of (\d+)", output)
    if match:
        return int(match.group(2)) - int(match.group(1)), int(match.group(2))
    match = re.search(r"100% tests passed out of (\d+)", output)
    return (int(match.group(1)), int(match.group(1))) if match else None


def planner_executable():
    choices = (ROOT / "build/drone_survey", ROOT / "build/drone_survey.exe",
               ROOT / "build/Debug/drone_survey.exe", ROOT / "build/Release/drone_survey.exe")
    found = next((path for path in choices if path.exists()), None)
    if not found:
        raise FileNotFoundError("The drone_survey executable was not found in build/")
    return found


class Launcher:
    def __init__(self, seed=None):
        self.rng, self.events, self.log_lines = random.Random(seed), queue.Queue(), []
        self.figure = plt.figure(figsize=(12.5, 7.6))
        self.figure.suptitle(TITLE, fontsize=18, fontweight="bold", x=.04, ha="left")
        self.figure.canvas.manager.set_window_title(TITLE)
        self.preview = self.figure.add_axes([.05, .12, .58, .78])
        panel = self.figure.add_axes([.67, .34, .30, .56]); panel.set(xticks=[], yticks=[], facecolor="#f6f7f8")
        self.status = panel.text(.04, .95, "Scenario ready", transform=panel.transAxes,
                                 va="top", fontweight="bold", fontsize=11, wrap=True)
        self.details = panel.text(.04, .82, "", transform=panel.transAxes, va="top")
        self.log = panel.text(.04, .69, "Build output will appear here.", transform=panel.transAxes,
                              va="top", family="monospace", fontsize=7.5, clip_on=True)
        self.field_button = self._button([.67, .245, .145, .055], "Randomize Field", self.randomize_field)
        self.obstacle_button = self._button([.825, .245, .145, .055], "Randomize Obstacles", self.randomize_obstacles)
        self.build_button = self._button([.67, .165, .30, .060], "Build & Test", self.build_and_test, "#d9ead3")
        self.visualize_button = self._button([.67, .085, .30, .060], "Visualize Mission", self.visualize, "#d9e7f7")
        self._enable(self.visualize_button, False)
        self.timer = self.figure.canvas.new_timer(interval=100); self.timer.add_callback(self._events); self.timer.start()
        self.randomize_field()

    def _button(self, bounds, label, callback, color="#e9edf2"):
        button = Button(self.figure.add_axes(bounds), label, color=color, hovercolor="#c7dcf3")
        button.on_clicked(callback); return button

    @staticmethod
    def _enable(button, enabled):
        button.set_active(enabled); button.ax.set_facecolor(button.color if enabled else "#d7d7d7")
        button.label.set_color("black" if enabled else "#777777")

    def _busy(self, busy, visualize=False):
        for button in (self.field_button, self.obstacle_button, self.build_button): self._enable(button, not busy)
        self._enable(self.visualize_button, not busy and visualize)

    def _status(self, text, color="black"):
        self.status.set(text=text, color=color); self.figure.canvas.draw_idle()

    def _save(self, message):
        write_scenario(self.field, self.obstacles, FIELD_FILE, OBSTACLE_FILE)
        self._draw(); self._busy(False); self._status(message)

    def randomize_field(self, _event=None):
        self.field = generate_random_field(self.rng); self.obstacles = generate_random_obstacles(self.field, self.rng)
        self._save("New field and obstacles ready — build and test")

    def randomize_obstacles(self, _event=None):
        self.obstacles = generate_random_obstacles(self.field, self.rng)
        self._save("New obstacles ready — build and test")

    def _draw(self):
        self.preview.clear()
        self.preview.add_patch(Polygon(self.field, facecolor="#eef4e8", edgecolor="#236b36", linewidth=2.2, label="Survey field"))
        for index, obstacle in enumerate(self.obstacles):
            self.preview.add_patch(Polygon(obstacle.vertices, facecolor=COLORS[obstacle.obstacle_type],
                                           edgecolor="black", alpha=.75, label=obstacle.obstacle_type.title()))
            self.preview.add_patch(Polygon(safety_corners(obstacle), fill=False, edgecolor="darkorange",
                                           linestyle="--", linewidth=1.2, label="Safety clearance" if index == 0 else None))
        xs, ys = zip(*self.field)
        self.preview.set(xlim=(min(xs)-8, max(xs)+8), ylim=(min(ys)-8, max(ys)+8), aspect="equal",
                         xlabel="X (m)", ylabel="Y (m)", title="Generated field and obstacle safety buffers")
        self.preview.grid(True, alpha=.25)
        handles, labels = self.preview.get_legend_handles_labels(); unique = dict(zip(labels, handles))
        self.preview.legend(unique.values(), unique.keys(), loc="upper right")
        self.details.set_text(f"Field: {len(self.field)} vertices, {polygon_area(self.field):,.0f} m²\nObstacles/no-fly zones: {len(self.obstacles)}")
        self.figure.canvas.draw_idle()

    def _post(self, name, payload=None): self.events.put((name, payload))

    def _events(self):
        while not self.events.empty():
            name, payload = self.events.get()
            if name == "log":
                self.log_lines.extend(str(payload).rstrip().splitlines()); self.log_lines = self.log_lines[-18:]
                self.log.set_text("\n".join(self.log_lines))
            else:
                message, color, visualize = payload; self._busy(False, visualize); self._status(message, color)

    def _command(self, command):
        heading = f"\n$ {' '.join(command)}\n"; self._post("log", heading); print(heading, end="", flush=True)
        process = subprocess.Popen(command, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        lines = []
        for line in process.stdout:
            lines.append(line); self._post("log", line); print(line, end="", flush=True)
        return process.wait(), "".join(lines)

    def _start(self, message, worker):
        self._busy(True); self._status(message); threading.Thread(target=worker, daemon=True).start()

    def build_and_test(self, _event=None):
        self.log_lines.clear(); self.log.set_text(""); self._start("Building project…", self._build_worker)

    def _build_worker(self):
        try:
            for command in (["cmake", "-S", ".", "-B", "build"], ["cmake", "--build", "build"]):
                if self._command(command)[0]: self._post("ready", ("Build failed — see output", "#b3261e", False)); return
            _, output = self._command(["ctest", "--test-dir", "build", "--output-on-failure"])
            summary = parse_ctest_summary(output)
            if not summary: result = ("Could not read the CTest summary", "#b3261e", False)
            else: result = (f"{summary[0]} / {summary[1]} tests passed", "#137333" if summary[0] == summary[1] else "#b3261e", summary[0] == summary[1])
            self._post("ready", result)
        except OSError as error: self._post("ready", (str(error), "#b3261e", False))

    def visualize(self, _event=None): self._start("Planning optimized mission…", self._visualize_worker)

    def _visualize_worker(self):
        try:
            result, _ = self._command([str(planner_executable()), "--field", str(FIELD_FILE), "--obstacles", str(OBSTACLE_FILE)])
            if result: self._post("ready", ("Planner failed — see output", "#b3261e", True)); return
            subprocess.Popen([sys.executable, str(VISUALIZER)], cwd=ROOT)
            self._post("ready", ("Mission planned — visualization opened", "#137333", True))
        except OSError as error: self._post("ready", (str(error), "#b3261e", True))

    def run(self): plt.show()


def main():
    parser = argparse.ArgumentParser(description=f"Launch the {TITLE}.")
    parser.add_argument("--generate-only", action="store_true"); parser.add_argument("--seed", type=int)
    args = parser.parse_args()
    if not args.generate_only: Launcher(args.seed).run(); return
    rng = random.Random(args.seed); field = generate_random_field(rng); obstacles = generate_random_obstacles(field, rng)
    write_scenario(field, obstacles, FIELD_FILE, OBSTACLE_FILE)
    print(f"Generated {len(field)}-vertex field with {len(obstacles)} obstacles\nField: {FIELD_FILE}\nObstacles: {OBSTACLE_FILE}")


if __name__ == "__main__": main()
