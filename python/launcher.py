#!/usr/bin/env python3

"""Desktop launcher for the agricultural drone survey demonstration."""

import argparse
import csv
import math
import queue
import random
import re
import subprocess
import sys
import threading
from dataclasses import dataclass
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent.parent
GENERATED_SCENARIO_DIR = PROJECT_ROOT / "generated_scenario"
GENERATED_FIELD_FILE = GENERATED_SCENARIO_DIR / "field.csv"
GENERATED_OBSTACLE_FILE = GENERATED_SCENARIO_DIR / "obstacles.csv"
VISUALIZER = PROJECT_ROOT / "python" / "visualize_field.py"
BUILD_DIR = PROJECT_ROOT / "build"

Point = tuple[float, float]


@dataclass(frozen=True)
class GeneratedObstacle:
    name: str
    obstacle_type: str
    clearance: float
    vertices: tuple[Point, ...]


def polygon_area(vertices: list[Point] | tuple[Point, ...]) -> float:
    twice_area = 0.0
    for index, current in enumerate(vertices):
        following = vertices[(index + 1) % len(vertices)]
        twice_area += current[0] * following[1] - following[0] * current[1]
    return abs(twice_area) / 2.0


def _orientation(first: Point, second: Point, third: Point) -> float:
    return (
        (second[0] - first[0]) * (third[1] - first[1])
        - (second[1] - first[1]) * (third[0] - first[0])
    )


def _point_on_segment(point: Point, start: Point, end: Point) -> bool:
    epsilon = 1e-9
    return (
        abs(_orientation(start, end, point)) <= epsilon
        and min(start[0], end[0]) - epsilon
        <= point[0]
        <= max(start[0], end[0]) + epsilon
        and min(start[1], end[1]) - epsilon
        <= point[1]
        <= max(start[1], end[1]) + epsilon
    )


def _segments_intersect(
    first_start: Point,
    first_end: Point,
    second_start: Point,
    second_end: Point,
) -> bool:
    first_a = _orientation(first_start, first_end, second_start)
    first_b = _orientation(first_start, first_end, second_end)
    second_a = _orientation(second_start, second_end, first_start)
    second_b = _orientation(second_start, second_end, first_end)
    epsilon = 1e-9

    if (
        ((first_a > epsilon and first_b < -epsilon)
         or (first_a < -epsilon and first_b > epsilon))
        and ((second_a > epsilon and second_b < -epsilon)
             or (second_a < -epsilon and second_b > epsilon))
    ):
        return True

    return (
        (abs(first_a) <= epsilon
         and _point_on_segment(second_start, first_start, first_end))
        or (abs(first_b) <= epsilon
            and _point_on_segment(second_end, first_start, first_end))
        or (abs(second_a) <= epsilon
            and _point_on_segment(first_start, second_start, second_end))
        or (abs(second_b) <= epsilon
            and _point_on_segment(first_end, second_start, second_end))
    )


def is_simple_polygon(vertices: list[Point] | tuple[Point, ...]) -> bool:
    if len(vertices) < 3:
        return False

    for first_index in range(len(vertices)):
        first_next = (first_index + 1) % len(vertices)
        if vertices[first_index] == vertices[first_next]:
            return False

        for second_index in range(first_index + 1, len(vertices)):
            second_next = (second_index + 1) % len(vertices)
            if first_index == second_next or first_next == second_index:
                continue
            if _segments_intersect(
                vertices[first_index],
                vertices[first_next],
                vertices[second_index],
                vertices[second_next],
            ):
                return False

    return polygon_area(vertices) > 1e-6


def point_in_polygon(point: Point, vertices: list[Point]) -> bool:
    inside = False
    for index, current in enumerate(vertices):
        following = vertices[(index + 1) % len(vertices)]
        if _point_on_segment(point, current, following):
            return True

        crosses_height = (current[1] > point[1]) != (following[1] > point[1])
        if crosses_height:
            intersection_x = (
                current[0]
                + (point[1] - current[1])
                * (following[0] - current[0])
                / (following[1] - current[1])
            )
            if point[0] < intersection_x:
                inside = not inside
    return inside


def generate_random_field(rng: random.Random) -> list[Point]:
    """Create a roomy, simple irregular polygon with five to eight vertices."""
    for _attempt in range(200):
        vertex_count = rng.randint(5, 8)
        vertices = []
        for index in range(vertex_count):
            sector = 2.0 * math.pi / vertex_count
            angle = index * sector + rng.uniform(-0.20, 0.20) * sector
            radius = rng.uniform(42.0, 58.0)
            vertices.append(
                (
                    72.0 + 1.35 * radius * math.cos(angle),
                    52.0 + radius * math.sin(angle),
                )
            )

        minimum_x = min(point[0] for point in vertices)
        minimum_y = min(point[1] for point in vertices)
        normalized = [
            (point[0] - minimum_x + 5.0, point[1] - minimum_y + 5.0)
            for point in vertices
        ]
        if is_simple_polygon(normalized) and polygon_area(normalized) >= 3500.0:
            return normalized

    raise RuntimeError("Unable to generate a valid field polygon")


def _regular_polygon(
    center: Point,
    radius_x: float,
    radius_y: float,
    vertex_count: int,
    rotation: float,
    rng: random.Random,
) -> tuple[Point, ...]:
    vertices = []
    for index in range(vertex_count):
        angle = rotation + index * 2.0 * math.pi / vertex_count
        radius_scale = rng.uniform(0.88, 1.12) if vertex_count > 4 else 1.0
        vertices.append(
            (
                center[0] + radius_x * radius_scale * math.cos(angle),
                center[1] + radius_y * radius_scale * math.sin(angle),
            )
        )
    return tuple(vertices)


def _safety_box(obstacle: GeneratedObstacle) -> tuple[float, float, float, float]:
    x_values = [point[0] for point in obstacle.vertices]
    y_values = [point[1] for point in obstacle.vertices]
    return (
        min(x_values) - obstacle.clearance,
        max(x_values) + obstacle.clearance,
        min(y_values) - obstacle.clearance,
        max(y_values) + obstacle.clearance,
    )


def _box_corners(box: tuple[float, float, float, float]) -> list[Point]:
    minimum_x, maximum_x, minimum_y, maximum_y = box
    return [
        (minimum_x, minimum_y),
        (maximum_x, minimum_y),
        (maximum_x, maximum_y),
        (minimum_x, maximum_y),
    ]


def _boxes_separated(
    first: tuple[float, float, float, float],
    second: tuple[float, float, float, float],
    gap: float = 4.0,
) -> bool:
    return (
        first[1] + gap < second[0]
        or second[1] + gap < first[0]
        or first[3] + gap < second[2]
        or second[3] + gap < first[2]
    )


def _box_strictly_inside_field(
    box: tuple[float, float, float, float],
    field: list[Point],
) -> bool:
    corners = _box_corners(box)

    if not all(point_in_polygon(corner, field) for corner in corners):
        return False

    for box_index, box_start in enumerate(corners):
        box_end = corners[(box_index + 1) % len(corners)]

        for field_index, field_start in enumerate(field):
            field_end = field[(field_index + 1) % len(field)]

            if _segments_intersect(box_start, box_end, field_start, field_end):
                return False

    return True


def generate_random_obstacles(
    field: list[Point],
    rng: random.Random,
) -> list[GeneratedObstacle]:
    """Generate separated obstacles whose conservative buffers stay in-field."""
    obstacle_types = ["barn", "pond", "trees", "restricted"]
    rng.shuffle(obstacle_types)
    desired_count = rng.randint(3, 4)
    minimum_x = min(point[0] for point in field)
    maximum_x = max(point[0] for point in field)
    minimum_y = min(point[1] for point in field)
    maximum_y = max(point[1] for point in field)
    obstacles: list[GeneratedObstacle] = []

    for obstacle_index in range(desired_count):
        obstacle_type = obstacle_types[obstacle_index]
        placed = False
        for _attempt in range(1500):
            center = (
                rng.uniform(minimum_x + 14.0, maximum_x - 14.0),
                rng.uniform(minimum_y + 14.0, maximum_y - 14.0),
            )
            clearance = rng.choice((1.0, 1.5, 2.0))
            if obstacle_type in ("barn", "restricted"):
                vertex_count = 4
                radius_x = rng.uniform(4.0, 7.0)
                radius_y = rng.uniform(3.5, 6.0)
            else:
                vertex_count = rng.randint(5, 7)
                radius_x = rng.uniform(4.5, 7.5)
                radius_y = rng.uniform(4.0, 7.0)

            candidate = GeneratedObstacle(
                name=f"{obstacle_type}_{obstacle_index + 1}",
                obstacle_type=obstacle_type,
                clearance=clearance,
                vertices=_regular_polygon(
                    center,
                    radius_x,
                    radius_y,
                    vertex_count,
                    rng.uniform(0.0, 2.0 * math.pi),
                    rng,
                ),
            )
            safety_box = _safety_box(candidate)
            if not _box_strictly_inside_field(safety_box, field):
                continue
            if not all(
                _boxes_separated(safety_box, _safety_box(existing))
                for existing in obstacles
            ):
                continue

            obstacles.append(candidate)
            placed = True
            break

        if not placed:
            raise RuntimeError("Unable to place separated obstacles inside the field")

    return obstacles


def write_scenario(field: list[Point], obstacles: list[GeneratedObstacle]) -> None:
    GENERATED_SCENARIO_DIR.mkdir(parents=True, exist_ok=True)
    with GENERATED_FIELD_FILE.open("w", newline="") as field_file:
        writer = csv.writer(field_file)
        writer.writerow(("x", "y"))
        writer.writerows(field)

    with GENERATED_OBSTACLE_FILE.open("w", newline="") as obstacle_file:
        writer = csv.writer(obstacle_file)
        writer.writerow(("name", "type", "clearance", "vertex_index", "x", "y"))
        for obstacle in obstacles:
            for vertex_index, point in enumerate(obstacle.vertices):
                writer.writerow(
                    (
                        obstacle.name,
                        obstacle.obstacle_type,
                        obstacle.clearance,
                        vertex_index,
                        point[0],
                        point[1],
                    )
                )


def parse_ctest_summary(output: str) -> tuple[int, int] | None:
    failure_summary = re.search(
        r"\d+% tests passed,\s*(\d+) tests failed out of (\d+)",
        output,
    )
    if failure_summary:
        failed = int(failure_summary.group(1))
        total = int(failure_summary.group(2))
        return total - failed, total

    success_summary = re.search(r"100% tests passed out of (\d+)", output)
    if success_summary:
        total = int(success_summary.group(1))
        return total, total

    return None


def find_planner_executable() -> Path:
    candidates = [
        BUILD_DIR / "drone_survey",
        BUILD_DIR / "drone_survey.exe",
        BUILD_DIR / "Debug" / "drone_survey.exe",
        BUILD_DIR / "Release" / "drone_survey.exe",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise FileNotFoundError("The drone_survey executable was not found in build/")


class Launcher:
    """Matplotlib-based launcher that reuses the project's existing UI stack."""

    def __init__(self, seed: int | None = None) -> None:
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Button

        self.plt = plt
        self.rng = random.Random(seed)
        self.field = generate_random_field(self.rng)
        self.obstacles = generate_random_obstacles(self.field, self.rng)
        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.log_lines: list[str] = []
        write_scenario(self.field, self.obstacles)

        self.figure = plt.figure(figsize=(12.5, 7.6))
        try:
            self.figure.canvas.manager.set_window_title(
                "Drone Field Survey & Route Optimization Simulator"
            )
        except AttributeError:
            pass
        self.figure.suptitle(
            "Drone Field Survey & Route Optimization Simulator",
            fontsize=18,
            fontweight="bold",
            x=0.04,
            ha="left",
        )

        self.preview_axes = self.figure.add_axes([0.05, 0.12, 0.58, 0.78])
        self.panel_axes = self.figure.add_axes([0.67, 0.34, 0.30, 0.56])
        self.panel_axes.set_facecolor("#f6f7f8")
        self.panel_axes.set_xticks([])
        self.panel_axes.set_yticks([])
        for spine in self.panel_axes.spines.values():
            spine.set_color("#c5c8cc")

        self.status_artist = self.panel_axes.text(
            0.04,
            0.95,
            "Scenario ready — build and test to continue",
            transform=self.panel_axes.transAxes,
            va="top",
            fontweight="bold",
            fontsize=11,
            wrap=True,
        )
        self.details_artist = self.panel_axes.text(
            0.04,
            0.82,
            "",
            transform=self.panel_axes.transAxes,
            va="top",
            fontsize=10,
        )
        self.log_artist = self.panel_axes.text(
            0.04,
            0.69,
            "Build output will appear here.",
            transform=self.panel_axes.transAxes,
            va="top",
            family="monospace",
            fontsize=7.5,
            clip_on=True,
        )

        self.randomize_field_button = Button(
            self.figure.add_axes([0.67, 0.245, 0.145, 0.055]),
            "Randomize Field",
            color="#e9edf2",
            hovercolor="#dce5ef",
        )
        self.randomize_obstacle_button = Button(
            self.figure.add_axes([0.825, 0.245, 0.145, 0.055]),
            "Randomize Obstacles",
            color="#e9edf2",
            hovercolor="#dce5ef",
        )
        self.build_button = Button(
            self.figure.add_axes([0.67, 0.165, 0.30, 0.060]),
            "Build & Test",
            color="#d9ead3",
            hovercolor="#c7e1bd",
        )
        self.visualize_button = Button(
            self.figure.add_axes([0.67, 0.085, 0.30, 0.060]),
            "Visualize Mission",
            color="#d9e7f7",
            hovercolor="#c7dcf3",
        )
        self.randomize_field_button.on_clicked(self.randomize_field)
        self.randomize_obstacle_button.on_clicked(self.randomize_obstacles)
        self.build_button.on_clicked(self.build_and_test)
        self.visualize_button.on_clicked(self.visualize_mission)
        self._set_button(self.visualize_button, False)

        self.timer = self.figure.canvas.new_timer(interval=100)
        self.timer.add_callback(self._drain_events)
        self.timer.start()
        self.draw_preview()

    @staticmethod
    def _set_button(button, enabled: bool) -> None:
        button.set_active(enabled)
        button.ax.set_facecolor(button.color if enabled else "#d7d7d7")
        button.label.set_color("black" if enabled else "#777777")

    def _set_scenario_buttons(self, enabled: bool) -> None:
        self._set_button(self.randomize_field_button, enabled)
        self._set_button(self.randomize_obstacle_button, enabled)

    def _set_status(self, message: str, color: str = "black") -> None:
        self.status_artist.set_text(message)
        self.status_artist.set_color(color)
        self.figure.canvas.draw_idle()

    def _append_log(self, text: str) -> None:
        self.log_lines.extend(text.rstrip().splitlines())
        self.log_lines = self.log_lines[-18:]
        self.log_artist.set_text("\n".join(self.log_lines))
        self.figure.canvas.draw_idle()

    def _invalidate_build(self, message: str) -> None:
        self._set_button(self.visualize_button, False)
        self._set_status(message)

    def randomize_field(self, _event=None) -> None:
        self.field = generate_random_field(self.rng)
        self.obstacles = generate_random_obstacles(self.field, self.rng)
        write_scenario(self.field, self.obstacles)
        self.draw_preview()
        self._invalidate_build("New field and obstacles ready — build and test again")

    def randomize_obstacles(self, _event=None) -> None:
        self.obstacles = generate_random_obstacles(self.field, self.rng)
        write_scenario(self.field, self.obstacles)
        self.draw_preview()
        self._invalidate_build("New obstacles ready — build and test again")

    def draw_preview(self) -> None:
        from matplotlib.patches import Polygon as PolygonPatch

        self.preview_axes.clear()
        self.preview_axes.add_patch(
            PolygonPatch(
                self.field,
                closed=True,
                facecolor="#eef4e8",
                edgecolor="#236b36",
                linewidth=2.2,
                label="Survey field",
            )
        )
        colors = {
            "barn": "saddlebrown",
            "pond": "deepskyblue",
            "trees": "forestgreen",
            "restricted": "crimson",
        }
        for obstacle_index, obstacle in enumerate(self.obstacles):
            self.preview_axes.add_patch(
                PolygonPatch(
                    obstacle.vertices,
                    closed=True,
                    facecolor=colors[obstacle.obstacle_type],
                    edgecolor="black",
                    alpha=0.75,
                    label=obstacle.obstacle_type.title(),
                )
            )
            self.preview_axes.add_patch(
                PolygonPatch(
                    _box_corners(_safety_box(obstacle)),
                    closed=True,
                    fill=False,
                    edgecolor="darkorange",
                    linestyle="--",
                    linewidth=1.2,
                    label="Safety clearance" if obstacle_index == 0 else None,
                )
            )

        x_values = [point[0] for point in self.field]
        y_values = [point[1] for point in self.field]
        self.preview_axes.set_xlim(min(x_values) - 8.0, max(x_values) + 8.0)
        self.preview_axes.set_ylim(min(y_values) - 8.0, max(y_values) + 8.0)
        self.preview_axes.set_aspect("equal")
        self.preview_axes.set_xlabel("X (m)")
        self.preview_axes.set_ylabel("Y (m)")
        self.preview_axes.grid(True, alpha=0.25)
        self.preview_axes.set_title("Generated field and obstacle safety buffers")
        handles, labels = self.preview_axes.get_legend_handles_labels()
        unique = dict(zip(labels, handles))
        self.preview_axes.legend(unique.values(), unique.keys(), loc="upper right")
        self.details_artist.set_text(
            f"Field: {len(self.field)} vertices, {polygon_area(self.field):,.0f} m²\n"
            f"Obstacles/no-fly zones: {len(self.obstacles)}"
        )
        self.figure.canvas.draw_idle()

    def _post(self, event_name: str, payload: object = None) -> None:
        self.events.put((event_name, payload))

    def _drain_events(self) -> None:
        while True:
            try:
                event_name, payload = self.events.get_nowait()
            except queue.Empty:
                break

            if event_name == "log":
                self._append_log(str(payload))
            elif event_name == "build_success":
                passed, total = payload
                self._set_status(f"{passed} / {total} tests passed", "#137333")
                self._set_scenario_buttons(True)
                self._set_button(self.build_button, True)
                self._set_button(self.visualize_button, True)
            elif event_name == "failure":
                self._set_status(str(payload), "#b3261e")
                self._set_scenario_buttons(True)
                self._set_button(self.build_button, True)
                self._set_button(self.visualize_button, False)
            elif event_name == "visualization_failure":
                self._set_status("Planner failed — see the output log", "#b3261e")
                self._set_scenario_buttons(True)
                self._set_button(self.build_button, True)
                self._set_button(self.visualize_button, True)
            elif event_name == "visualization_started":
                self._set_status("Mission planned — visualization opened", "#137333")
                self._set_scenario_buttons(True)
                self._set_button(self.build_button, True)
                self._set_button(self.visualize_button, True)

    def _run_command(self, command: list[str]) -> tuple[int, str]:
        command_text = f"\n$ {' '.join(command)}\n"
        self._post("log", command_text)
        print(command_text, end="", flush=True)
        process = subprocess.Popen(
            command,
            cwd=PROJECT_ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        output_lines = []
        assert process.stdout is not None
        for line in process.stdout:
            output_lines.append(line)
            self._post("log", line)
            print(line, end="", flush=True)
        return process.wait(), "".join(output_lines)

    def build_and_test(self, _event=None) -> None:
        self._set_scenario_buttons(False)
        self._set_button(self.build_button, False)
        self._set_button(self.visualize_button, False)
        self.log_lines.clear()
        self.log_artist.set_text("")
        self._set_status("Building project…")
        threading.Thread(target=self._build_worker, daemon=True).start()

    def _build_worker(self) -> None:
        commands = [
            ["cmake", "-S", ".", "-B", "build"],
            ["cmake", "--build", "build"],
            ["ctest", "--test-dir", "build", "--output-on-failure"],
        ]
        test_output = ""
        try:
            for command in commands:
                return_code, output = self._run_command(command)
                if return_code != 0 and command[0] != "ctest":
                    self._post("failure", "Build failed — see the output log")
                    return
                if command[0] == "ctest":
                    test_output = output
        except OSError as error:
            self._post("failure", str(error))
            return

        summary = parse_ctest_summary(test_output)
        if summary is None:
            self._post(
                "failure",
                "Tests finished, but the CTest summary could not be read",
            )
            return
        passed, total = summary
        if passed != total:
            self._post("failure", f"{passed} / {total} tests passed")
            return
        self._post("build_success", summary)

    def visualize_mission(self, _event=None) -> None:
        self._set_scenario_buttons(False)
        self._set_button(self.build_button, False)
        self._set_button(self.visualize_button, False)
        self._set_status("Planning optimized mission…")
        threading.Thread(target=self._visualize_worker, daemon=True).start()

    def _visualize_worker(self) -> None:
        try:
            executable = find_planner_executable()
            return_code, _output = self._run_command(
                [
                    str(executable),
                    "--field",
                    str(GENERATED_FIELD_FILE),
                    "--obstacles",
                    str(GENERATED_OBSTACLE_FILE),
                ]
            )
            if return_code != 0:
                self._post("visualization_failure")
                return
            subprocess.Popen([sys.executable, str(VISUALIZER)], cwd=PROJECT_ROOT)
        except OSError as error:
            self._post("failure", str(error))
            return
        self._post("visualization_started")

    def run(self) -> None:
        self.plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Launch the Drone Field Survey & Route Optimization Simulator."
    )
    parser.add_argument(
        "--generate-only",
        action="store_true",
        help="generate and validate one scenario without opening the GUI",
    )
    parser.add_argument("--seed", type=int, help="use a repeatable random seed")
    arguments = parser.parse_args()

    if arguments.generate_only:
        rng = random.Random(arguments.seed)
        field = generate_random_field(rng)
        obstacles = generate_random_obstacles(field, rng)
        write_scenario(field, obstacles)
        print(
            f"Generated {len(field)}-vertex field with {len(obstacles)} obstacles\n"
            f"Field: {GENERATED_FIELD_FILE}\n"
            f"Obstacles: {GENERATED_OBSTACLE_FILE}"
        )
        return

    Launcher(arguments.seed).run()


if __name__ == "__main__":
    main()
