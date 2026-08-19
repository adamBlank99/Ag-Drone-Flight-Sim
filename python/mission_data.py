"""Load planner CSV output for the mission visualizer."""

import csv
from dataclasses import dataclass
from pathlib import Path

Point = tuple[float, float]


@dataclass
class Route:
    metadata: dict
    points: list[Point]
    waypoint_types: list[str]
    mission_ids: list[int]
    mission_safe: list[bool]
    battery_used: list[float]


@dataclass
class Obstacle:
    name: str
    obstacle_type: str
    boundary: str
    vertices: list[Point]


@dataclass
class CoverageCell:
    x: float
    y: float
    size: float
    count: int
    status: str


@dataclass
class MissionData:
    route: Route
    field: list[Point]
    obstacles: list[Obstacle]
    footprint: Point
    coverage_statistics: dict[str, float]
    coverage_cells: list[CoverageCell]


def _require(path: Path) -> None:
    if not path.exists():
        raise SystemExit(f"Run ./build/drone_survey first to generate {path.name}")


def _points(path: Path) -> list[Point]:
    _require(path)
    with path.open(newline="") as source:
        return [tuple(float(value) for value in list(row.values())[:2])
                for row in csv.DictReader(source)]


def _route(path: Path) -> Route:
    _require(path)
    with path.open(newline="") as source:
        rows = list(csv.DictReader(source))
    if not rows:
        raise SystemExit(f"No optimized route found in {path.name}")
    first = rows[0]
    return Route(
        {"angle": float(first["angle_degrees"]), "score": float(first["score"]),
         "distance": float(first["total_distance"]), "turns": int(first["turns"]),
         "mission_count": int(first.get("mission_count", 1)),
         "all_missions_safe": first.get("all_missions_safe", "true").lower() == "true"},
        [(float(row["x"]), float(row["y"])) for row in rows],
        [row["waypoint_type"] for row in rows],
        [int(row.get("mission_id", 1)) for row in rows],
        [row.get("mission_safe", "true").lower() == "true" for row in rows],
        [float(row.get("battery_used_percent", 0)) for row in rows],
    )


def _obstacles(path: Path) -> list[Obstacle]:
    _require(path)
    grouped = {}
    with path.open(newline="") as source:
        for row in csv.DictReader(source):
            key = row["name"], row["boundary"]
            grouped.setdefault(key, Obstacle(row["name"], row["type"], row["boundary"], [])).vertices.append(
                (float(row["x"]), float(row["y"])))
    return list(grouped.values())


def _coverage(stats_path: Path, grid_path: Path):
    _require(stats_path); _require(grid_path)
    with stats_path.open(newline="") as source:
        stats = next(csv.DictReader(source), None)
    if stats is None:
        raise SystemExit(f"No coverage statistics found in {stats_path.name}")
    with grid_path.open(newline="") as source:
        cells = [CoverageCell(float(row["x"]), float(row["y"]), float(row["cell_size"]),
                              int(row["coverage_count"]), row["status"])
                 for row in csv.DictReader(source)]
    return {name: float(value) for name, value in stats.items()}, cells


def load_mission(root: Path) -> MissionData:
    statistics, cells = _coverage(root / "coverage_statistics.csv", root / "coverage_grid.csv")
    return MissionData(
        _route(root / "waypoints.csv"), _points(root / "field_polygon.csv"),
        _obstacles(root / "obstacles.csv"), _points(root / "camera_footprint.csv")[0],
        statistics, cells,
    )
