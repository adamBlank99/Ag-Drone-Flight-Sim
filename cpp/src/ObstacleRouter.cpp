#include "ObstacleRouter.h"

#include <algorithm>
#include <array>
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

double pathDistance(const vector<Point>& path) {
    double distance = 0.0;

    for (size_t i = 1; i < path.size(); ++i) {
        distance += pointDistance(path[i - 1], path[i]);
    }

    return distance;
}

double shortestTransitionDistance(
    const Point& start,
    const Point& end,
    const Polygon& field,
    const vector<Polygon>& exclusionZones
) {
    return pathDistance(
        findShortestSafePath(start, end, field, exclusionZones)
    );
}

struct LaneOption {
    vector<LineSegment> passes;
    Point start;
    Point end;
    double internalTransitionDistance;
};

struct OrderedCoveragePlan {
    vector<LineSegment> passes;
    double transitionDistance;
};

vector<vector<LineSegment>> groupCoverageLanes(
    const vector<LineSegment>& coverageSegments
) {
    vector<LineSegment> sortedSegments = coverageSegments;

    sort(
        sortedSegments.begin(),
        sortedSegments.end(),
        [](const LineSegment& first, const LineSegment& second) {
            double firstY = (first.start.y + first.end.y) / 2.0;
            double secondY = (second.start.y + second.end.y) / 2.0;

            if (abs(firstY - secondY) > EPSILON) {
                return firstY < secondY;
            }

            return min(first.start.x, first.end.x) <
                min(second.start.x, second.end.x);
        }
    );

    vector<vector<LineSegment>> lanes;

    for (const LineSegment& segment : sortedSegments) {
        double segmentY = (segment.start.y + segment.end.y) / 2.0;

        if (lanes.empty()) {
            lanes.push_back({segment});
            continue;
        }

        const LineSegment& previousLaneSegment = lanes.back().front();
        double previousY =
            (previousLaneSegment.start.y + previousLaneSegment.end.y) / 2.0;

        if (abs(segmentY - previousY) > EPSILON) {
            lanes.push_back({segment});
        }
        else {
            lanes.back().push_back(segment);
        }
    }

    return lanes;
}

LineSegment orientLeftToRight(const LineSegment& segment) {
    if (
        segment.start.x < segment.end.x ||
        (
            abs(segment.start.x - segment.end.x) <= EPSILON &&
            segment.start.y <= segment.end.y
        )
    ) {
        return segment;
    }

    return {segment.end, segment.start};
}

LaneOption buildLaneOption(
    const vector<LineSegment>& lane,
    bool leftToRight,
    const Polygon& field,
    const vector<Polygon>& exclusionZones
) {
    vector<LineSegment> passes;
    passes.reserve(lane.size());

    if (leftToRight) {
        for (const LineSegment& segment : lane) {
            passes.push_back(orientLeftToRight(segment));
        }
    }
    else {
        for (auto iterator = lane.rbegin(); iterator != lane.rend(); ++iterator) {
            LineSegment oriented = orientLeftToRight(*iterator);
            passes.push_back({oriented.end, oriented.start});
        }
    }

    double internalTransitionDistance = 0.0;

    for (size_t i = 1; i < passes.size(); ++i) {
        internalTransitionDistance += shortestTransitionDistance(
            passes[i - 1].end,
            passes[i].start,
            field,
            exclusionZones
        );
    }

    return {
        passes,
        passes.front().start,
        passes.back().end,
        internalTransitionDistance
    };
}

OrderedCoveragePlan optimizeLaneOrder(
    const vector<array<LaneOption, 2>>& laneOptions,
    const vector<size_t>& laneOrder,
    const Polygon& field,
    const vector<Polygon>& exclusionZones
) {
    struct State {
        double distance;
        size_t previousDirection;
    };

    const double infinity = numeric_limits<double>::infinity();
    vector<array<State, 2>> states(laneOrder.size());

    for (size_t direction = 0; direction < 2; ++direction) {
        const LaneOption& option = laneOptions[laneOrder.front()][direction];
        states[0][direction] = {
            option.internalTransitionDistance,
            2
        };
    }

    for (size_t lanePosition = 1;
         lanePosition < laneOrder.size();
         ++lanePosition) {
        for (size_t direction = 0; direction < 2; ++direction) {
            states[lanePosition][direction] = {infinity, 2};
            const LaneOption& current =
                laneOptions[laneOrder[lanePosition]][direction];

            for (size_t previousDirection = 0;
                 previousDirection < 2;
                 ++previousDirection) {
                const LaneOption& previous =
                    laneOptions[
                        laneOrder[lanePosition - 1]
                    ][previousDirection];
                double candidate =
                    states[lanePosition - 1][previousDirection].distance +
                    shortestTransitionDistance(
                        previous.end,
                        current.start,
                        field,
                        exclusionZones
                    ) +
                    current.internalTransitionDistance;

                if (candidate + EPSILON < states[lanePosition][direction].distance) {
                    states[lanePosition][direction] = {
                        candidate,
                        previousDirection
                    };
                }
            }
        }
    }

    size_t finalPosition = laneOrder.size() - 1;
    size_t direction =
        states[finalPosition][0].distance <=
            states[finalPosition][1].distance
        ? 0
        : 1;
    vector<size_t> selectedDirections(laneOrder.size());

    for (size_t lanePosition = laneOrder.size(); lanePosition-- > 0;) {
        selectedDirections[lanePosition] = direction;

        if (lanePosition > 0) {
            direction = states[lanePosition][direction].previousDirection;
        }
    }

    vector<LineSegment> orderedPasses;

    for (size_t lanePosition = 0;
         lanePosition < laneOrder.size();
         ++lanePosition) {
        const LaneOption& option = laneOptions[
            laneOrder[lanePosition]
        ][selectedDirections[lanePosition]];
        orderedPasses.insert(
            orderedPasses.end(),
            option.passes.begin(),
            option.passes.end()
        );
    }

    double bestDistance = min(
        states[finalPosition][0].distance,
        states[finalPosition][1].distance
    );
    return {orderedPasses, bestDistance};
}

vector<LineSegment> orderCoverageSegments(
    const vector<LineSegment>& coverageSegments,
    const Polygon& field,
    const vector<Polygon>& exclusionZones
) {
    if (coverageSegments.empty()) {
        return {};
    }

    vector<vector<LineSegment>> lanes = groupCoverageLanes(coverageSegments);
    vector<array<LaneOption, 2>> laneOptions;
    laneOptions.reserve(lanes.size());

    for (const vector<LineSegment>& lane : lanes) {
        laneOptions.push_back({
            buildLaneOption(lane, true, field, exclusionZones),
            buildLaneOption(lane, false, field, exclusionZones)
        });
    }

    vector<size_t> ascendingOrder(lanes.size());

    for (size_t i = 0; i < ascendingOrder.size(); ++i) {
        ascendingOrder[i] = i;
    }

    vector<size_t> descendingOrder = ascendingOrder;
    reverse(descendingOrder.begin(), descendingOrder.end());
    OrderedCoveragePlan ascending = optimizeLaneOrder(
        laneOptions,
        ascendingOrder,
        field,
        exclusionZones
    );
    OrderedCoveragePlan descending = optimizeLaneOrder(
        laneOptions,
        descendingOrder,
        field,
        exclusionZones
    );

    return
        ascending.transitionDistance <= descending.transitionDistance
        ? ascending.passes
        : descending.passes;
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

    for (const LineSegment& segment : coverageSegments) {
        if (!isRouteSegmentSafe(
            segment,
            field,
            exclusionZones
        )) {
            throw invalid_argument(
                "Coverage segment crosses a field boundary or exclusion zone"
            );
        }
    }

    vector<LineSegment> orderedSegments = orderCoverageSegments(
        coverageSegments,
        field,
        exclusionZones
    );

    for (const LineSegment& pass : orderedSegments) {
        Point passStart = pass.start;
        Point passEnd = pass.end;

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
