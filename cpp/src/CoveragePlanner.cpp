#include "CoveragePlanner.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "Geometry.h"

using namespace std;

namespace {

constexpr double EPSILON = 1e-9;

vector<double> generateLaneLevels(
    double minY,
    double maxY,
    const DroneConfig& drone
) {
    if (drone.footprintWidth <= 0.0) {
        throw invalid_argument("Camera footprint width must be positive");
    }

    if (drone.overlap < 0.0 || drone.overlap >= 1.0) {
        throw invalid_argument("Camera overlap must be between 0 and 1");
    }

    double fieldHeight = maxY - minY;

    if (fieldHeight <= 0.0) {
        throw invalid_argument("Field height must be positive");
    }

    double laneSpacing =
        drone.footprintWidth * (1.0 - drone.overlap);

    int numberOfLevels = 1;

    if (fieldHeight > drone.footprintWidth) {
        numberOfLevels = 1 + static_cast<int>(ceil(
            (fieldHeight - drone.footprintWidth) / laneSpacing
        ));
    }

    double routeHeight = (numberOfLevels - 1) * laneSpacing;
    double firstY = minY + (fieldHeight - routeHeight) / 2.0;

    vector<double> levels;
    levels.reserve(numberOfLevels);

    for (int i = 0; i < numberOfLevels; ++i) {
        levels.push_back(firstY + i * laneSpacing);
    }

    return levels;
}

vector<Point> findHorizontalIntersections(
    const Polygon& field,
    const BoundingBox& bounds,
    double y
) {
    LineSegment candidateLine{
        {bounds.minX, y},
        {bounds.maxX, y}
    };

    vector<Point> intersections;

    for (size_t i = 0; i < field.vertices.size(); ++i) {
        const Point& start = field.vertices[i];
        const Point& end =
            field.vertices[(i + 1) % field.vertices.size()];

        bool crossesLevel =
            (start.y <= y && end.y > y) ||
            (end.y <= y && start.y > y);

        if (!crossesLevel) {
            continue;
        }

        auto intersection = calculateLineSegmentIntersection(
            candidateLine,
            LineSegment{start, end}
        );

        if (intersection) {
            intersections.push_back(*intersection);
        }
    }

    sort(
        intersections.begin(),
        intersections.end(),
        [](const Point& first, const Point& second) {
            return first.x < second.x;
        }
    );

    if (intersections.size() % 2 != 0) {
        throw runtime_error("Polygon produced an odd number of line intersections");
    }

    return intersections;
}

} // namespace

vector<Point> CoveragePlanner::generatePath(
    const Field& field,
    const DroneConfig& drone
) const {

    if (field.width <= 0.0) {
        throw invalid_argument("Field width must be positive");
    }

    vector<Point> path;
    vector<double> levels = generateLaneLevels(0.0, field.height, drone);

    for (size_t i = 0; i < levels.size(); ++i) {
        if (i % 2 == 0) {
            path.push_back({0.0, levels[i]});
            path.push_back({field.width, levels[i]});
        }
        else {
            path.push_back({field.width, levels[i]});
            path.push_back({0.0, levels[i]});
        }
    }

    return path;
}

vector<Point> CoveragePlanner::generatePath(
    const Polygon& field,
    const DroneConfig& drone
) const {
    vector<LineSegment> segments = generateCoverageSegments(field, drone);
    vector<Point> path;
    path.reserve(segments.size() * 2);

    for (size_t i = 0; i < segments.size(); ++i) {
        if (i % 2 == 0) {
            path.push_back(segments[i].start);
            path.push_back(segments[i].end);
        }
        else {
            path.push_back(segments[i].end);
            path.push_back(segments[i].start);
        }
    }

    return path;
}

vector<LineSegment> CoveragePlanner::generateCoverageSegments(
    const Polygon& field,
    const DroneConfig& drone
) const {
    if (field.vertices.size() < 3) {
        throw invalid_argument("A polygon field needs at least three vertices");
    }

    BoundingBox bounds = calculateBoundingBox(field);
    vector<double> levels = generateLaneLevels(bounds.minY, bounds.maxY, drone);
    vector<LineSegment> segments;

    for (double y : levels) {
        vector<Point> intersections =
            findHorizontalIntersections(field, bounds, y);

        for (size_t i = 0; i < intersections.size(); i += 2) {
            const Point& start = intersections[i];
            const Point& end = intersections[i + 1];

            if (abs(end.x - start.x) > EPSILON) {
                segments.push_back({start, end});
            }
        }
    }

    return segments;
}
