#pragma once

#include <optional>

#include "Polygon.h"

struct BoundingBox {
    double minX;
    double maxX;
    double minY;
    double maxY;
};

enum class PointLocation {
    Outside,
    Inside,
    Boundary
};

struct LineSegment {
    Point start;
    Point end;
};

double calculatePolygonArea(const Polygon& polygon);
BoundingBox calculateBoundingBox(const Polygon& polygon);
PointLocation pointInPolygon(const Point& point, const Polygon& polygon);
std::optional<Point> calculateLineSegmentIntersection(
    const LineSegment& first,
    const LineSegment& second
);
