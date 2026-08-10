#include "ObstacleRouter.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

using namespace std;

namespace {

constexpr double EPSILON = 1e-9;

double pointDistance(const Point& first, const Point& second) {
    return hypot(second.x - first.x, second.y - first.y);
}

bool samePoint(const Point& first, const Point& second) {
    return
        abs(first.x - second.x) <= EPSILON &&
        abs(first.y - second.y) <= EPSILON;
}

bool pointIsUsable(
    const Point& point,
    const Polygon& field,
    const vector<Polygon>& exclusionZones
) {
    if (pointInPolygon(point, field) == PointLocation::Outside) {
        return false;
    }

    for (const Polygon& exclusion : exclusionZones) {
        if (pointInPolygon(point, exclusion) == PointLocation::Inside) {
            return false;
        }
    }

    return true;
}

bool segmentIsSafe(
    const LineSegment& segment,
    const Polygon& field,
    const vector<Polygon>& exclusionZones
) {
    if (!segmentStaysInsidePolygon(segment, field)) {
        return false;
    }

    for (const Polygon& exclusion : exclusionZones) {
        if (segmentIntersectsPolygonInterior(segment, exclusion)) {
            return false;
        }
    }

    return true;
}

void appendUniquePoint(vector<Point>& points, const Point& candidate) {
    for (const Point& point : points) {
        if (samePoint(point, candidate)) {
            return;
        }
    }

    points.push_back(candidate);
}

vector<Point> findSafeTransition(
    const Point& start,
    const Point& end,
    const Polygon& field,
    const vector<Polygon>& exclusionZones
) {
    if (samePoint(start, end)) {
        return {start};
    }

    if (segmentIsSafe({start, end}, field, exclusionZones)) {
        return {start, end};
    }

    vector<Point> nodes{start, end};

    for (const Polygon& exclusion : exclusionZones) {
        for (const Point& vertex : exclusion.vertices) {
            if (pointIsUsable(vertex, field, exclusionZones)) {
                appendUniquePoint(nodes, vertex);
            }
        }
    }

    for (const Point& vertex : field.vertices) {
        if (pointIsUsable(vertex, field, exclusionZones)) {
            appendUniquePoint(nodes, vertex);
        }
    }

    const double infinity = numeric_limits<double>::infinity();
    vector<double> distances(nodes.size(), infinity);
    vector<size_t> previous(nodes.size(), nodes.size());
    vector<bool> visited(nodes.size(), false);
    distances[0] = 0.0;

    for (size_t iteration = 0; iteration < nodes.size(); ++iteration) {
        size_t current = nodes.size();

        for (size_t i = 0; i < nodes.size(); ++i) {
            if (
                !visited[i] &&
                (current == nodes.size() || distances[i] < distances[current])
            ) {
                current = i;
            }
        }

        if (current == nodes.size() || distances[current] == infinity) {
            break;
        }

        if (current == 1) {
            break;
        }

        visited[current] = true;

        for (size_t next = 0; next < nodes.size(); ++next) {
            if (visited[next] || current == next) {
                continue;
            }

            if (!segmentIsSafe(
                {nodes[current], nodes[next]},
                field,
                exclusionZones
            )) {
                continue;
            }

            double candidateDistance =
                distances[current] + pointDistance(nodes[current], nodes[next]);

            if (candidateDistance < distances[next]) {
                distances[next] = candidateDistance;
                previous[next] = current;
            }
        }
    }

    if (distances[1] == infinity) {
        throw runtime_error("No collision-free transition route was found");
    }

    vector<Point> reversedPath;

    for (size_t current = 1; current != nodes.size(); current = previous[current]) {
        reversedPath.push_back(nodes[current]);

        if (current == 0) {
            break;
        }
    }

    reverse(reversedPath.begin(), reversedPath.end());
    return reversedPath;
}

} // namespace

MissionRoute buildSafeMissionRoute(
    const Polygon& field,
    const vector<LineSegment>& coverageSegments,
    const vector<Polygon>& exclusionZones
) {
    MissionRoute route{{}, {}, coverageSegments.size(), 0};

    for (size_t i = 0; i < coverageSegments.size(); ++i) {
        Point passStart =
            i % 2 == 0 ? coverageSegments[i].start : coverageSegments[i].end;
        Point passEnd =
            i % 2 == 0 ? coverageSegments[i].end : coverageSegments[i].start;

        if (!route.waypoints.empty()) {
            vector<Point> transition = findSafeTransition(
                route.waypoints.back(),
                passStart,
                field,
                exclusionZones
            );

            if (transition.size() > 1) {
                route.transitionSegments += transition.size() - 1;
            }

            for (size_t j = 1; j + 1 < transition.size(); ++j) {
                route.waypoints.push_back(transition[j]);
                route.waypointTypes.push_back(WaypointType::Detour);
            }
        }

        route.waypoints.push_back(passStart);
        route.waypointTypes.push_back(WaypointType::CoverageStart);
        route.waypoints.push_back(passEnd);
        route.waypointTypes.push_back(WaypointType::CoverageEnd);
    }

    return route;
}

bool routeAvoidsExclusionZones(
    const MissionRoute& route,
    const vector<Polygon>& exclusionZones
) {
    for (size_t i = 1; i < route.waypoints.size(); ++i) {
        LineSegment routeSegment{
            route.waypoints[i - 1],
            route.waypoints[i]
        };

        for (const Polygon& exclusion : exclusionZones) {
            if (segmentIntersectsPolygonInterior(routeSegment, exclusion)) {
                return false;
            }
        }
    }

    return true;
}
