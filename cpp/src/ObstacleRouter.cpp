#include "ObstacleRouter.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

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

void validateRoutingPolygons(
    const Polygon& field,
    const vector<Polygon>& exclusionZones
) {
    if (field.vertices.size() < 3) {
        throw invalid_argument("A route field needs at least three vertices");
    }

    for (const Polygon& exclusion : exclusionZones) {
        if (exclusion.vertices.size() < 3) {
            throw invalid_argument(
                "A route exclusion zone needs at least three vertices"
            );
        }
    }
}

void appendUniquePoint(vector<Point>& points, const Point& candidate) {
    for (const Point& point : points) {
        if (samePoint(point, candidate)) {
            return;
        }
    }

    points.push_back(candidate);
}

vector<vector<pair<size_t, double>>> buildVisibilityGraph(
    const vector<Point>& nodes,
    const Polygon& field,
    const vector<Polygon>& exclusionZones
) {
    vector<vector<pair<size_t, double>>> graph(nodes.size());

    for (size_t first = 0; first < nodes.size(); ++first) {
        for (size_t second = first + 1; second < nodes.size(); ++second) {
            if (!isRouteSegmentSafe(
                {nodes[first], nodes[second]},
                field,
                exclusionZones
            )) {
                continue;
            }

            double distance = pointDistance(nodes[first], nodes[second]);
            graph[first].push_back({second, distance});
            graph[second].push_back({first, distance});
        }
    }

    return graph;
}

vector<Point> runDijkstra(
    const vector<Point>& nodes,
    const vector<vector<pair<size_t, double>>>& graph
) {
    const double infinity = numeric_limits<double>::infinity();
    vector<double> distances(nodes.size(), infinity);
    vector<size_t> previous(nodes.size(), nodes.size());
    priority_queue<
        pair<double, size_t>,
        vector<pair<double, size_t>>,
        greater<pair<double, size_t>>
    > frontier;

    distances[0] = 0.0;
    frontier.push({0.0, 0});

    while (!frontier.empty()) {
        auto [distanceToCurrent, current] = frontier.top();
        frontier.pop();

        if (distanceToCurrent > distances[current] + EPSILON) {
            continue;
        }

        if (current == 1) {
            break;
        }

        for (const auto& [next, edgeLength] : graph[current]) {
            double candidateDistance = distanceToCurrent + edgeLength;

            if (candidateDistance + EPSILON >= distances[next]) {
                continue;
            }

            distances[next] = candidateDistance;
            previous[next] = current;
            frontier.push({candidateDistance, next});
        }
    }

    if (!isfinite(distances[1])) {
        throw runtime_error("No collision-free transition route was found");
    }

    vector<Point> reversedPath;

    for (size_t current = 1; ; current = previous[current]) {
        reversedPath.push_back(nodes[current]);

        if (current == 0) {
            break;
        }

        if (previous[current] == nodes.size()) {
            throw logic_error("Shortest-path reconstruction failed");
        }
    }

    reverse(reversedPath.begin(), reversedPath.end());
    return reversedPath;
}

} // namespace

bool isRouteSegmentSafe(
    const LineSegment& segment,
    const Polygon& field,
    const vector<Polygon>& exclusionZones
) {
    if (field.vertices.size() < 3) {
        return false;
    }

    if (!segmentStaysInsidePolygon(segment, field)) {
        return false;
    }

    for (const Polygon& exclusion : exclusionZones) {
        if (
            exclusion.vertices.size() < 3 ||
            segmentIntersectsPolygonInterior(segment, exclusion)
        ) {
            return false;
        }
    }

    return true;
}

vector<Point> findShortestSafePath(
    const Point& start,
    const Point& end,
    const Polygon& field,
    const vector<Polygon>& exclusionZones
) {
    validateRoutingPolygons(field, exclusionZones);

    if (
        !pointIsUsable(start, field, exclusionZones) ||
        !pointIsUsable(end, field, exclusionZones)
    ) {
        throw invalid_argument(
            "Transition endpoints must be inside the field and outside exclusions"
        );
    }

    if (samePoint(start, end)) {
        return {start};
    }

    if (isRouteSegmentSafe({start, end}, field, exclusionZones)) {
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

    return runDijkstra(
        nodes,
        buildVisibilityGraph(nodes, field, exclusionZones)
    );
}

MissionRoute buildSafeMissionRoute(
    const Polygon& field,
    const vector<LineSegment>& coverageSegments,
    const vector<Polygon>& exclusionZones
) {
    validateRoutingPolygons(field, exclusionZones);
    MissionRoute route{{}, {}, coverageSegments.size(), 0};

    for (size_t i = 0; i < coverageSegments.size(); ++i) {
        Point passStart =
            i % 2 == 0 ? coverageSegments[i].start : coverageSegments[i].end;
        Point passEnd =
            i % 2 == 0 ? coverageSegments[i].end : coverageSegments[i].start;

        if (!isRouteSegmentSafe(
            {passStart, passEnd},
            field,
            exclusionZones
        )) {
            throw invalid_argument(
                "Coverage segment crosses a field boundary or exclusion zone"
            );
        }

        if (!route.waypoints.empty()) {
            vector<Point> transition = findShortestSafePath(
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

    if (!isMissionRouteSafe(route, field, exclusionZones)) {
        throw logic_error("Generated mission route failed final safety validation");
    }

    return route;
}

bool isMissionRouteSafe(
    const MissionRoute& route,
    const Polygon& field,
    const vector<Polygon>& exclusionZones
) {
    if (
        field.vertices.size() < 3 ||
        route.waypoints.size() != route.waypointTypes.size()
    ) {
        return false;
    }

    for (const Polygon& exclusion : exclusionZones) {
        if (exclusion.vertices.size() < 3) {
            return false;
        }
    }

    for (const Point& waypoint : route.waypoints) {
        if (!pointIsUsable(waypoint, field, exclusionZones)) {
            return false;
        }
    }

    for (size_t i = 1; i < route.waypoints.size(); ++i) {
        if (!isRouteSegmentSafe(
            {route.waypoints[i - 1], route.waypoints[i]},
            field,
            exclusionZones
        )) {
            return false;
        }
    }

    return true;
}

bool routeAvoidsExclusionZones(
    const MissionRoute& route,
    const vector<Polygon>& exclusionZones
) {
    for (const Polygon& exclusion : exclusionZones) {
        if (exclusion.vertices.size() < 3) {
            return false;
        }

        for (const Point& waypoint : route.waypoints) {
            if (pointInPolygon(waypoint, exclusion) == PointLocation::Inside) {
                return false;
            }
        }
    }

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
