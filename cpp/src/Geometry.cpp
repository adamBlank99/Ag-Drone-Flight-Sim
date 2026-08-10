#include "Geometry.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

using namespace std;

namespace {

constexpr double EPSILON = 1e-9;
constexpr double PI = 3.14159265358979323846;

bool pointOnSegment(
    const Point& point,
    const Point& start,
    const Point& end
) {
    double crossProduct =
        (point.y - start.y) * (end.x - start.x) -
        (point.x - start.x) * (end.y - start.y);

    if (abs(crossProduct) > EPSILON) {
        return false;
    }

    return
        point.x >= min(start.x, end.x) - EPSILON &&
        point.x <= max(start.x, end.x) + EPSILON &&
        point.y >= min(start.y, end.y) - EPSILON &&
        point.y <= max(start.y, end.y) + EPSILON;
}

double crossProduct(
    double firstX,
    double firstY,
    double secondX,
    double secondY
) {
    return firstX * secondY - firstY * secondX;
}

} // namespace

double calculatePolygonArea(const Polygon& polygon) {
    if (polygon.vertices.size() < 3) {
        return 0.0;
    }

    double twiceArea = 0.0;

    for (size_t i = 0; i < polygon.vertices.size(); ++i) {
        const Point& current = polygon.vertices[i];
        const Point& next =
            polygon.vertices[(i + 1) % polygon.vertices.size()];

        twiceArea +=
            current.x * next.y -
            next.x * current.y;
    }

    return abs(twiceArea) / 2.0;
}

BoundingBox calculateBoundingBox(const Polygon& polygon) {
    if (polygon.vertices.empty()) {
        throw invalid_argument("Cannot calculate the bounding box of an empty polygon");
    }

    const Point& firstVertex = polygon.vertices.front();

    BoundingBox boundingBox{
        firstVertex.x,
        firstVertex.x,
        firstVertex.y,
        firstVertex.y
    };

    for (const Point& vertex : polygon.vertices) {
        boundingBox.minX = min(boundingBox.minX, vertex.x);
        boundingBox.maxX = max(boundingBox.maxX, vertex.x);
        boundingBox.minY = min(boundingBox.minY, vertex.y);
        boundingBox.maxY = max(boundingBox.maxY, vertex.y);
    }

    return boundingBox;
}

Point rotatePoint(
    const Point& point,
    const Point& center,
    double angleDegrees
) {
    double angleRadians = angleDegrees * PI / 180.0;
    double cosine = cos(angleRadians);
    double sine = sin(angleRadians);
    double translatedX = point.x - center.x;
    double translatedY = point.y - center.y;

    return {
        center.x + translatedX * cosine - translatedY * sine,
        center.y + translatedX * sine + translatedY * cosine
    };
}

Polygon rotatePolygon(
    const Polygon& polygon,
    const Point& center,
    double angleDegrees
) {
    Polygon rotated;
    rotated.vertices.reserve(polygon.vertices.size());

    for (const Point& vertex : polygon.vertices) {
        rotated.vertices.push_back(
            rotatePoint(vertex, center, angleDegrees)
        );
    }

    return rotated;
}

PointLocation pointInPolygon(const Point& point, const Polygon& polygon) {
    if (polygon.vertices.size() < 3) {
        return PointLocation::Outside;
    }

    bool inside = false;

    for (size_t i = 0; i < polygon.vertices.size(); ++i) {
        const Point& current = polygon.vertices[i];
        const Point& next =
            polygon.vertices[(i + 1) % polygon.vertices.size()];

        if (pointOnSegment(point, current, next)) {
            return PointLocation::Boundary;
        }

        bool crossesPointHeight =
            (current.y > point.y) != (next.y > point.y);

        if (crossesPointHeight) {
            double intersectionX =
                current.x +
                (point.y - current.y) *
                (next.x - current.x) /
                (next.y - current.y);

            if (point.x < intersectionX) {
                inside = !inside;
            }
        }
    }

    return inside ? PointLocation::Inside : PointLocation::Outside;
}

optional<Point> calculateLineSegmentIntersection(
    const LineSegment& first,
    const LineSegment& second
) {
    double firstDirectionX = first.end.x - first.start.x;
    double firstDirectionY = first.end.y - first.start.y;
    double secondDirectionX = second.end.x - second.start.x;
    double secondDirectionY = second.end.y - second.start.y;

    double startDifferenceX = second.start.x - first.start.x;
    double startDifferenceY = second.start.y - first.start.y;

    double denominator = crossProduct(
        firstDirectionX,
        firstDirectionY,
        secondDirectionX,
        secondDirectionY
    );

    if (abs(denominator) <= EPSILON) {
        bool collinear = abs(crossProduct(
            startDifferenceX,
            startDifferenceY,
            firstDirectionX,
            firstDirectionY
        )) <= EPSILON;

        if (!collinear) {
            return nullopt;
        }

        if (pointOnSegment(second.start, first.start, first.end)) {
            return second.start;
        }

        if (pointOnSegment(second.end, first.start, first.end)) {
            return second.end;
        }

        if (pointOnSegment(first.start, second.start, second.end)) {
            return first.start;
        }

        if (pointOnSegment(first.end, second.start, second.end)) {
            return first.end;
        }

        return nullopt;
    }

    double firstAmount = crossProduct(
        startDifferenceX,
        startDifferenceY,
        secondDirectionX,
        secondDirectionY
    ) / denominator;

    double secondAmount = crossProduct(
        startDifferenceX,
        startDifferenceY,
        firstDirectionX,
        firstDirectionY
    ) / denominator;

    if (
        firstAmount < -EPSILON || firstAmount > 1.0 + EPSILON ||
        secondAmount < -EPSILON || secondAmount > 1.0 + EPSILON
    ) {
        return nullopt;
    }

    firstAmount = clamp(firstAmount, 0.0, 1.0);

    return Point{
        first.start.x + firstAmount * firstDirectionX,
        first.start.y + firstAmount * firstDirectionY
    };
}

namespace {

vector<double> segmentPolygonParameters(
    const LineSegment& segment,
    const Polygon& polygon
) {
    vector<double> parameters{0.0, 1.0};
    double changeInX = segment.end.x - segment.start.x;
    double changeInY = segment.end.y - segment.start.y;

    for (size_t i = 0; i < polygon.vertices.size(); ++i) {
        const Point& edgeStart = polygon.vertices[i];
        const Point& edgeEnd =
            polygon.vertices[(i + 1) % polygon.vertices.size()];
        auto intersection = calculateLineSegmentIntersection(
            segment,
            LineSegment{edgeStart, edgeEnd}
        );

        if (!intersection) {
            continue;
        }

        double parameter = 0.0;

        if (abs(changeInX) >= abs(changeInY) && abs(changeInX) > EPSILON) {
            parameter = (intersection->x - segment.start.x) / changeInX;
        }
        else if (abs(changeInY) > EPSILON) {
            parameter = (intersection->y - segment.start.y) / changeInY;
        }

        parameters.push_back(clamp(parameter, 0.0, 1.0));
    }

    sort(parameters.begin(), parameters.end());
    parameters.erase(
        unique(
            parameters.begin(),
            parameters.end(),
            [](double first, double second) {
                return abs(first - second) <= EPSILON;
            }
        ),
        parameters.end()
    );

    return parameters;
}

Point pointAlongSegment(const LineSegment& segment, double parameter) {
    return {
        segment.start.x + parameter * (segment.end.x - segment.start.x),
        segment.start.y + parameter * (segment.end.y - segment.start.y)
    };
}

} // namespace

bool segmentIntersectsPolygonInterior(
    const LineSegment& segment,
    const Polygon& polygon
) {
    if (
        pointInPolygon(segment.start, polygon) == PointLocation::Inside ||
        pointInPolygon(segment.end, polygon) == PointLocation::Inside
    ) {
        return true;
    }

    vector<double> parameters = segmentPolygonParameters(segment, polygon);

    for (size_t i = 1; i < parameters.size(); ++i) {
        double midpoint = (parameters[i - 1] + parameters[i]) / 2.0;

        if (
            pointInPolygon(pointAlongSegment(segment, midpoint), polygon) ==
            PointLocation::Inside
        ) {
            return true;
        }
    }

    return false;
}

bool segmentStaysInsidePolygon(
    const LineSegment& segment,
    const Polygon& polygon
) {
    if (
        pointInPolygon(segment.start, polygon) == PointLocation::Outside ||
        pointInPolygon(segment.end, polygon) == PointLocation::Outside
    ) {
        return false;
    }

    vector<double> parameters = segmentPolygonParameters(segment, polygon);

    for (size_t i = 1; i < parameters.size(); ++i) {
        double midpoint = (parameters[i - 1] + parameters[i]) / 2.0;

        if (
            pointInPolygon(pointAlongSegment(segment, midpoint), polygon) ==
            PointLocation::Outside
        ) {
            return false;
        }
    }

    return true;
}
