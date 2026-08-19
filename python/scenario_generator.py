"""Generate valid random fields and obstacles for the demo launcher."""

import csv
import math
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

Point = tuple[float, float]
Box = tuple[float, float, float, float]


@dataclass(frozen=True)
class GeneratedObstacle:
    name: str
    obstacle_type: str
    clearance: float
    vertices: tuple[Point, ...]


def polygon_area(vertices: Sequence[Point]) -> float:
    pairs = zip(vertices, vertices[1:] + vertices[:1])
    return abs(sum(x1 * y2 - x2 * y1 for (x1, y1), (x2, y2) in pairs)) / 2


def _cross(a: Point, b: Point, c: Point) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _on_segment(point: Point, start: Point, end: Point) -> bool:
    e = 1e-9
    return (abs(_cross(start, end, point)) <= e
            and min(start[0], end[0]) - e <= point[0] <= max(start[0], end[0]) + e
            and min(start[1], end[1]) - e <= point[1] <= max(start[1], end[1]) + e)


def _intersects(a: Point, b: Point, c: Point, d: Point) -> bool:
    values = _cross(a, b, c), _cross(a, b, d), _cross(c, d, a), _cross(c, d, b)
    e = 1e-9
    if values[0] * values[1] < -e and values[2] * values[3] < -e:
        return True
    return any((abs(values[0]) <= e and _on_segment(c, a, b),
                abs(values[1]) <= e and _on_segment(d, a, b),
                abs(values[2]) <= e and _on_segment(a, c, d),
                abs(values[3]) <= e and _on_segment(b, c, d)))


def _edges(vertices: Sequence[Point]):
    return zip(vertices, vertices[1:] + vertices[:1])


def is_simple_polygon(vertices: Sequence[Point]) -> bool:
    if len(vertices) < 3 or polygon_area(vertices) <= 1e-6:
        return False
    edges = list(_edges(vertices))
    for first, (a, b) in enumerate(edges):
        if a == b:
            return False
        for second in range(first + 1, len(edges)):
            if second in (first + 1, (first - 1) % len(edges)):
                continue
            if _intersects(a, b, *edges[second]):
                return False
    return True


def point_in_polygon(point: Point, vertices: Sequence[Point]) -> bool:
    inside = False
    for start, end in _edges(vertices):
        if _on_segment(point, start, end):
            return True
        if (start[1] > point[1]) != (end[1] > point[1]):
            crossing = start[0] + (point[1] - start[1]) * (end[0] - start[0]) / (end[1] - start[1])
            inside ^= point[0] < crossing
    return inside


def generate_random_field(rng: random.Random) -> list[Point]:
    for _ in range(200):
        count = rng.randint(5, 8)
        sector = 2 * math.pi / count
        vertices = []
        for index in range(count):
            angle = index * sector + rng.uniform(-0.20, 0.20) * sector
            radius = rng.uniform(42, 58)
            vertices.append((72 + 1.35 * radius * math.cos(angle),
                             52 + radius * math.sin(angle)))
        min_x, min_y = min(x for x, _ in vertices), min(y for _, y in vertices)
        vertices = [(x - min_x + 5, y - min_y + 5) for x, y in vertices]
        if is_simple_polygon(vertices) and polygon_area(vertices) >= 3500:
            return vertices
    raise RuntimeError("Unable to generate a valid field polygon")


def _polygon(center: Point, radii: Point, count: int, rotation: float,
             rng: random.Random) -> tuple[Point, ...]:
    points = []
    for index in range(count):
        angle = rotation + index * 2 * math.pi / count
        scale = rng.uniform(0.88, 1.12) if count > 4 else 1
        points.append((center[0] + radii[0] * scale * math.cos(angle),
                       center[1] + radii[1] * scale * math.sin(angle)))
    return tuple(points)


def _safety_box(obstacle: GeneratedObstacle) -> Box:
    xs, ys = zip(*obstacle.vertices)
    c = obstacle.clearance
    return min(xs) - c, max(xs) + c, min(ys) - c, max(ys) + c


def safety_corners(obstacle: GeneratedObstacle) -> list[Point]:
    min_x, max_x, min_y, max_y = _safety_box(obstacle)
    return [(min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y)]


def _box_inside(box: Box, field: Sequence[Point]) -> bool:
    corners = [(box[0], box[2]), (box[1], box[2]), (box[1], box[3]), (box[0], box[3])]
    return all(point_in_polygon(p, field) for p in corners) and not any(
        _intersects(a, b, c, d) for a, b in _edges(corners) for c, d in _edges(field))


def _separated(a: Box, b: Box, gap=4.0) -> bool:
    return (a[1] + gap < b[0] or b[1] + gap < a[0]
            or a[3] + gap < b[2] or b[3] + gap < a[2])


def generate_random_obstacles(field: list[Point], rng: random.Random) -> list[GeneratedObstacle]:
    types = ["barn", "pond", "trees", "restricted"]
    rng.shuffle(types)
    xs, ys = zip(*field)
    obstacles = []
    for index, kind in enumerate(types[:rng.randint(3, 4)], 1):
        for _ in range(1500):
            center = rng.uniform(min(xs) + 14, max(xs) - 14), rng.uniform(min(ys) + 14, max(ys) - 14)
            rectangular = kind in ("barn", "restricted")
            count = 4 if rectangular else rng.randint(5, 7)
            radii = (rng.uniform(4, 7) if rectangular else rng.uniform(4.5, 7.5),
                     rng.uniform(3.5, 6) if rectangular else rng.uniform(4, 7))
            candidate = GeneratedObstacle(f"{kind}_{index}", kind, rng.choice((1, 1.5, 2)),
                                          _polygon(center, radii, count, rng.uniform(0, 2 * math.pi), rng))
            box = _safety_box(candidate)
            if _box_inside(box, field) and all(_separated(box, _safety_box(item)) for item in obstacles):
                obstacles.append(candidate)
                break
        else:
            raise RuntimeError("Unable to place separated obstacles inside the field")
    return obstacles


def write_scenario(field, obstacles, field_file: Path, obstacle_file: Path) -> None:
    field_file.parent.mkdir(parents=True, exist_ok=True)
    with field_file.open("w", newline="") as output:
        writer = csv.writer(output); writer.writerow(("x", "y")); writer.writerows(field)
    with obstacle_file.open("w", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(("name", "type", "clearance", "vertex_index", "x", "y"))
        for obstacle in obstacles:
            for index, (x, y) in enumerate(obstacle.vertices):
                writer.writerow((obstacle.name, obstacle.obstacle_type, obstacle.clearance, index, x, y))
