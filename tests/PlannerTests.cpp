#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "CameraConfig.h"
#include "CoverageAnalysis.h"
#include "CoveragePlanner.h"
#include "DroneConfig.h"
#include "Geometry.h"
#include "MissionModel.h"
#include "MissionRoute.h"
#include "Obstacle.h"
#include "ObstacleRouter.h"
#include "Polygon.h"
#include "RouteOptimizer.h"
#include "RouteStatistics.h"

using namespace std;

namespace {

constexpr double TOLERANCE = 1e-6;

void require(bool condition, const string& message) {
    if (!condition) {
        throw runtime_error(message);
    }
}

void requireNear(
    double actual,
    double expected,
    const string& message,
    double tolerance = TOLERANCE
) {
    if (abs(actual - expected) > tolerance) {
        throw runtime_error(
            message + ": expected " + to_string(expected) +
            ", got " + to_string(actual)
        );
    }
}

void requireSamePoint(
    const Point& actual,
    const Point& expected,
    const string& message
) {
    requireNear(actual.x, expected.x, message + " x");
    requireNear(actual.y, expected.y, message + " y");
}

Polygon sampleField() {
    return {{
        {0.0, 0.0},
        {100.0, 0.0},
        {120.0, 30.0},
        {90.0, 70.0},
        {40.0, 80.0},
        {0.0, 50.0}
    }};
}

Polygon concaveField() {
    return {{
        {0.0, 0.0},
        {10.0, 0.0},
        {10.0, 10.0},
        {7.0, 10.0},
        {7.0, 3.0},
        {3.0, 3.0},
        {3.0, 10.0},
        {0.0, 10.0}
    }};
}

Polygon obstacleTestField() {
    return {{
        {0.0, 0.0},
        {100.0, 0.0},
        {100.0, 50.0},
        {0.0, 50.0}
    }};
}

DroneConfig obstacleTestDrone() {
    return {{5.0, 90.0, 60.0, 0.0, 0.70}, 5.0};
}

vector<Polygon> safetyBoundaries(const vector<Obstacle>& obstacles) {
    vector<Polygon> boundaries;

    for (const Obstacle& obstacle : obstacles) {
        boundaries.push_back(calculateSafetyBoundary(obstacle));
    }

    return boundaries;
}

void requireSegmentsAvoid(
    const vector<LineSegment>& segments,
    const vector<Polygon>& exclusions
) {
    for (const LineSegment& segment : segments) {
        for (const Polygon& exclusion : exclusions) {
            require(
                !segmentIntersectsPolygonInterior(segment, exclusion),
                "Coverage segment enters an exclusion zone"
            );
        }
    }
}

void requireWaypointsInside(
    const vector<Point>& path,
    const Polygon& field
) {
    for (const Point& waypoint : path) {
        require(
            pointInPolygon(waypoint, field) != PointLocation::Outside,
            "Generated waypoint lies outside the polygon"
        );
    }
}

double routeDistanceByType(
    const MissionRoute& route,
    RouteSegmentType requestedType
) {
    double distance = 0.0;

    for (size_t i = 1; i < route.waypoints.size(); ++i) {
        RouteSegmentType type = classifyRouteSegment(
            route.waypointTypes[i - 1],
            route.waypointTypes[i]
        );

        if (type == requestedType) {
            distance += hypot(
                route.waypoints[i].x - route.waypoints[i - 1].x,
                route.waypoints[i].y - route.waypoints[i - 1].y
            );
        }
    }

    return distance;
}

double totalTransitionDistance(const MissionRoute& route) {
    double distance = 0.0;

    for (RouteSegmentType type : {
        RouteSegmentType::NormalTransition,
        RouteSegmentType::ObstacleDetour,
        RouteSegmentType::HomeTransit
    }) {
        distance += routeDistanceByType(route, type);
    }

    return distance;
}

vector<LineSegment> extractCoveragePasses(const MissionRoute& route) {
    vector<LineSegment> passes;

    for (size_t i = 1; i < route.waypoints.size(); ++i) {
        if (
            classifyRouteSegment(
                route.waypointTypes[i - 1],
                route.waypointTypes[i]
            ) == RouteSegmentType::CoveragePass
        ) {
            passes.push_back({
                route.waypoints[i - 1],
                route.waypoints[i]
            });
        }
    }

    return passes;
}

void testCameraFootprint() {
    CameraFootprint lowAltitude =
        calculateFootprint({10.0, 90.0, 60.0, 0.30, 0.70});
    requireNear(lowAltitude.width, 20.0, "10 m / 90 degree footprint width");
    requireNear(
        lowAltitude.height,
        11.5470053838,
        "10 m / 60 degree footprint height"
    );
    requireNear(
        calculateLaneSpacing(lowAltitude, 0.30),
        14.0,
        "Calculated lane spacing"
    );
    requireNear(
        calculatePhotoSpacing(lowAltitude, 0.70),
        3.4641016151,
        "Calculated photo spacing"
    );

    CameraFootprint surveyAltitude =
        calculateFootprint({50.0, 60.0, 45.0, 0.40, 0.75});
    requireNear(
        surveyAltitude.width,
        57.735026919,
        "50 m / 60 degree footprint width"
    );
    requireNear(
        surveyAltitude.height,
        41.421356237,
        "50 m / 45 degree footprint height"
    );

    CameraFootprint square =
        calculateFootprint({20.0, 90.0, 90.0, 0.20, 0.60});
    requireNear(square.width, 40.0, "Square footprint width");
    requireNear(square.height, 40.0, "Square footprint height");

    bool rejectedAltitude = false;
    bool rejectedHorizontalFov = false;
    bool rejectedVerticalFov = false;
    bool rejectedForwardOverlap = false;

    try {
        calculateFootprint({0.0, 90.0, 60.0, 0.30, 0.70});
    }
    catch (const invalid_argument&) {
        rejectedAltitude = true;
    }

    try {
        calculateFootprint({10.0, 0.0, 60.0, 0.30, 0.70});
    }
    catch (const invalid_argument&) {
        rejectedHorizontalFov = true;
    }

    try {
        calculateFootprint({10.0, 90.0, 180.0, 0.30, 0.70});
    }
    catch (const invalid_argument&) {
        rejectedVerticalFov = true;
    }

    try {
        calculatePhotoSpacing(lowAltitude, 1.0);
    }
    catch (const invalid_argument&) {
        rejectedForwardOverlap = true;
    }

    require(rejectedAltitude, "Zero altitude must be rejected");
    require(rejectedHorizontalFov, "Zero horizontal FOV must be rejected");
    require(rejectedVerticalFov, "180 degree vertical FOV must be rejected");
    require(rejectedForwardOverlap, "Full forward overlap must be rejected");
}

void testOverlapBehavior() {
    CoveragePlanner planner;
    Polygon field = sampleField();

    DroneConfig lowForwardOverlap{
        {10.0, 90.0, 60.0, 0.30, 0.10},
        6.0
    };
    DroneConfig highForwardOverlap{
        {10.0, 90.0, 60.0, 0.30, 0.90},
        6.0
    };
    DroneConfig highSideOverlap{
        {10.0, 90.0, 60.0, 0.50, 0.70},
        6.0
    };

    vector<Point> lowForwardPath =
        planner.generatePath(field, lowForwardOverlap);
    vector<Point> highForwardPath =
        planner.generatePath(field, highForwardOverlap);
    vector<Point> highSidePath =
        planner.generatePath(field, highSideOverlap);

    require(
        lowForwardPath.size() == highForwardPath.size(),
        "Forward overlap must not change route waypoint count"
    );

    for (size_t i = 0; i < lowForwardPath.size(); ++i) {
        requireSamePoint(
            lowForwardPath[i],
            highForwardPath[i],
            "Forward overlap must not change route geometry"
        );
    }

    require(
        highSidePath.size() > lowForwardPath.size(),
        "Higher side overlap must create more coverage passes"
    );

    CameraFootprint footprint = calculateFootprint(lowForwardOverlap.camera);
    double lowForwardSpacing = calculatePhotoSpacing(footprint, 0.10);
    double highForwardSpacing = calculatePhotoSpacing(footprint, 0.90);

    require(
        highForwardSpacing < lowForwardSpacing,
        "Higher forward overlap must shorten photo spacing"
    );
    requireNear(
        calculatePhotoSpacing(footprint, 0.70) / lowForwardOverlap.speed,
        0.5773502692,
        "Photo capture interval"
    );
}

void testRotationGeometry() {
    Point center{1.0, 1.0};
    Point rotatedPoint = rotatePoint({2.0, 1.0}, center, 90.0);
    requireSamePoint(rotatedPoint, {1.0, 2.0}, "90-degree point rotation");

    Polygon triangle{{
        {1.0, 1.0},
        {3.0, 1.0},
        {1.0, 3.0}
    }};
    Polygon rotatedTriangle = rotatePolygon(triangle, center, 90.0);

    requireSamePoint(
        rotatedTriangle.vertices[1],
        {1.0, 3.0},
        "Polygon vertex rotation"
    );
    requireNear(
        calculatePolygonArea(rotatedTriangle),
        calculatePolygonArea(triangle),
        "Rotation must preserve polygon area"
    );

    Polygon restored = rotatePolygon(rotatedTriangle, center, -90.0);

    for (size_t i = 0; i < triangle.vertices.size(); ++i) {
        requireSamePoint(
            restored.vertices[i],
            triangle.vertices[i],
            "Forward and reverse polygon rotation"
        );
    }
}

void testCandidateRoutes() {
    vector<double> angles = generateCandidateAngles();
    require(angles.size() == 7, "Candidate angle count");

    for (size_t i = 0; i < angles.size(); ++i) {
        requireNear(
            angles[i],
            static_cast<double>(i * 15),
            "Candidate angle sequence"
        );
    }

    Polygon field = sampleField();
    DroneConfig drone{{10.0, 90.0, 60.0, 0.30, 0.70}, 6.0};
    RouteOptimizationResult result = optimizeRoute(field, drone, angles, 10.0);

    require(result.candidates.size() == angles.size(), "Candidate route count");

    for (const RouteCandidate& candidate : result.candidates) {
        require(!candidate.waypoints.empty(), "Candidate route cannot be empty");
        require(
            candidate.statistics.coveragePasses == candidate.waypoints.size() / 2,
            "Candidate pass count"
        );
        requireNear(
            candidate.score,
            candidate.statistics.totalDistance +
                10.0 * candidate.statistics.transitionSegments,
            "Candidate route score"
        );
        requireWaypointsInside(candidate.waypoints, field);
    }

    for (const RouteCandidate& candidate : result.candidates) {
        require(
            result.bestRoute.score <= candidate.score + TOLERANCE,
            "Selected route must have the lowest score"
        );
    }
}

void testBestAngle() {
    Point center{0.0, 0.0};
    Polygon horizontalRectangle{{
        {-50.0, -10.0},
        {50.0, -10.0},
        {50.0, 10.0},
        {-50.0, 10.0}
    }};
    Polygon rotatedRectangle =
        rotatePolygon(horizontalRectangle, center, 30.0);
    DroneConfig drone{{10.0, 90.0, 60.0, 0.30, 0.70}, 6.0};

    RouteOptimizationResult result = optimizeRoute(
        rotatedRectangle,
        drone,
        generateCandidateAngles(),
        10.0
    );

    requireNear(result.bestRoute.angleDegrees, 30.0, "Rotated field best angle");
    require(
        result.bestRoute.angleDegrees != 0.0,
        "Rotated field best angle must not be zero"
    );
    requireWaypointsInside(result.bestRoute.waypoints, rotatedRectangle);
}

void testObstacleIntersections() {
    Polygon field = obstacleTestField();
    DroneConfig drone = obstacleTestDrone();
    vector<Obstacle> obstacles{
        {
            "barn",
            ObstacleType::Barn,
            {{{40.0, 10.0}, {60.0, 10.0}, {60.0, 20.0}, {40.0, 20.0}}},
            0.0
        }
    };
    vector<Polygon> exclusions = safetyBoundaries(obstacles);
    CoveragePlanner planner;
    vector<LineSegment> segments =
        planner.generateCoverageSegments(field, drone, exclusions);

    require(segments.size() == 6, "Blocked lane must split into two passes");
    requireSegmentsAvoid(segments, exclusions);

    MissionRoute route = buildSafeMissionRoute(field, segments, exclusions);
    require(route.coveragePasses == segments.size(), "Obstacle pass count");
    require(
        routeAvoidsExclusionZones(route, exclusions),
        "Detoured route must avoid the direct obstacle"
    );
    require(
        find(
            route.waypointTypes.begin(),
            route.waypointTypes.end(),
            WaypointType::Detour
        ) != route.waypointTypes.end(),
        "A split pass must create a detour waypoint"
    );
}

void testFullRouteSegmentSafety() {
    Polygon field{{
        {0.0, 0.0},
        {100.0, 0.0},
        {100.0, 100.0},
        {0.0, 100.0}
    }};
    Obstacle obstacle{
        "buffered_barn",
        ObstacleType::Barn,
        {{{42.0, 32.0}, {58.0, 32.0}, {58.0, 68.0}, {42.0, 68.0}}},
        2.0
    };
    vector<Polygon> exclusions{calculateSafetyBoundary(obstacle)};
    LineSegment crossingSegment{{10.0, 40.0}, {90.0, 40.0}};

    require(
        pointInPolygon(crossingSegment.start, exclusions[0]) ==
            PointLocation::Outside,
        "Unsafe segment start must be outside the safety buffer"
    );
    require(
        pointInPolygon(crossingSegment.end, exclusions[0]) ==
            PointLocation::Outside,
        "Unsafe segment end must be outside the safety buffer"
    );
    require(
        !isRouteSegmentSafe(crossingSegment, field, exclusions),
        "A full segment crossing a buffer must be unsafe"
    );

    MissionRoute unsafeRoute{
        {crossingSegment.start, crossingSegment.end},
        {WaypointType::CoverageStart, WaypointType::CoverageEnd},
        1,
        0
    };

    require(
        !isMissionRouteSafe(unsafeRoute, field, exclusions),
        "Route validation must inspect the space between waypoints"
    );
    require(
        !routeAvoidsExclusionZones(unsafeRoute, exclusions),
        "Compatibility route validation must reject a crossing segment"
    );

    bool rejectedUnsafeCoverageSegment = false;

    try {
        buildSafeMissionRoute(field, {crossingSegment}, exclusions);
    }
    catch (const invalid_argument&) {
        rejectedUnsafeCoverageSegment = true;
    }

    require(
        rejectedUnsafeCoverageSegment,
        "Mission construction must reject an unclipped unsafe coverage pass"
    );
}

void testShortestObstacleDetour() {
    Polygon field{{
        {0.0, 0.0},
        {100.0, 0.0},
        {100.0, 100.0},
        {0.0, 100.0}
    }};
    Obstacle obstacle{
        "buffered_barn",
        ObstacleType::Barn,
        {{{42.0, 32.0}, {58.0, 32.0}, {58.0, 68.0}, {42.0, 68.0}}},
        2.0
    };
    vector<Polygon> exclusions{calculateSafetyBoundary(obstacle)};
    Point start{10.0, 40.0};
    Point end{90.0, 40.0};

    vector<Point> path = findShortestSafePath(
        start,
        end,
        field,
        exclusions
    );

    require(path.size() == 4, "Shortest rectangle detour needs two vertices");
    requireSamePoint(path.front(), start, "Shortest detour start");
    requireSamePoint(path[1], {40.0, 30.0}, "Shortest detour first corner");
    requireSamePoint(path[2], {60.0, 30.0}, "Shortest detour second corner");
    requireSamePoint(path.back(), end, "Shortest detour end");

    double distance = 0.0;

    for (size_t i = 1; i < path.size(); ++i) {
        LineSegment segment{path[i - 1], path[i]};
        require(
            isRouteSegmentSafe(segment, field, exclusions),
            "Every shortest-detour edge must be collision-free"
        );
        distance += hypot(
            segment.end.x - segment.start.x,
            segment.end.y - segment.start.y
        );
    }

    requireNear(
        distance,
        2.0 * hypot(30.0, 10.0) + 20.0,
        "Dijkstra must select the shorter side of the obstacle"
    );

    vector<LineSegment> coverageSegments{
        {{0.0, 40.0}, start},
        {{80.0, 40.0}, end}
    };
    MissionRoute mission = buildSafeMissionRoute(
        field,
        coverageSegments,
        exclusions
    );

    require(
        mission.transitionSegments == 3,
        "Mission transition must use all three shortest-path edges"
    );
    require(
        mission.waypointTypes[2] == WaypointType::Detour &&
            mission.waypointTypes[3] == WaypointType::Detour,
        "Shortest-path corners must be stored as detour waypoints"
    );
    requireSamePoint(
        mission.waypoints[2],
        {40.0, 30.0},
        "Mission first detour corner"
    );
    requireSamePoint(
        mission.waypoints[3],
        {60.0, 30.0},
        "Mission second detour corner"
    );
    require(
        isMissionRouteSafe(mission, field, exclusions),
        "Mission using the shortest detour must pass final validation"
    );
}

void testRouteSegmentClassification() {
    require(
        classifyRouteSegment(
            WaypointType::CoverageStart,
            WaypointType::CoverageEnd
        ) == RouteSegmentType::CoveragePass,
        "Coverage endpoints must classify as a coverage pass"
    );
    require(
        classifyRouteSegment(
            WaypointType::CoverageEnd,
            WaypointType::CoverageStart
        ) == RouteSegmentType::NormalTransition,
        "Adjacent passes must classify as a normal transition"
    );
    require(
        classifyRouteSegment(
            WaypointType::CoverageEnd,
            WaypointType::Detour
        ) == RouteSegmentType::ObstacleDetour,
        "A segment entering a detour must classify as obstacle detour"
    );
    require(
        classifyRouteSegment(
            WaypointType::Transit,
            WaypointType::CoverageStart
        ) == RouteSegmentType::HomeTransit,
        "A segment from home must classify as home transit"
    );
}

void testCoveragePassReversal() {
    Polygon field{{
        {0.0, 0.0},
        {100.0, 0.0},
        {100.0, 30.0},
        {0.0, 30.0}
    }};
    vector<LineSegment> passes{
        {{0.0, 5.0}, {100.0, 5.0}},
        {{0.0, 15.0}, {100.0, 15.0}}
    };
    MissionRoute route = buildSafeMissionRoute(field, passes, {});
    vector<LineSegment> orderedPasses = extractCoveragePasses(route);

    require(orderedPasses.size() == 2, "Reversal test pass count");
    requireNear(
        totalTransitionDistance(route),
        10.0,
        "The second pass must start on the side where the first pass ends"
    );
    requireNear(
        orderedPasses[0].end.x,
        orderedPasses[1].start.x,
        "Consecutive coverage directions must reverse when shorter"
    );
}

void testGlobalPassOrdering() {
    Polygon field{{
        {0.0, 0.0},
        {100.0, 0.0},
        {100.0, 30.0},
        {0.0, 30.0}
    }};
    vector<LineSegment> passes{
        {{0.0, 5.0}, {90.0, 5.0}},
        {{0.0, 15.0}, {70.0, 15.0}},
        {{30.0, 25.0}, {70.0, 25.0}}
    };
    MissionRoute route = buildSafeMissionRoute(field, passes, {});
    RouteStatistics statistics = calculateRouteStatistics(route, 1.0);
    double locallyGreedyDistance =
        200.0 +
        hypot(20.0, 10.0) +
        hypot(30.0, 10.0);

    requireNear(
        totalTransitionDistance(route),
        20.0,
        "Global lane directions must use two short vertical transitions"
    );
    requireNear(statistics.totalDistance, 220.0, "Globally ordered route length");
    require(
        statistics.totalDistance + TOLERANCE < locallyGreedyDistance,
        "Global direction selection must beat a locally greedy route"
    );
}

void testLongTransitionAvoidance() {
    Polygon field{{
        {0.0, 0.0},
        {100.0, 0.0},
        {100.0, 40.0},
        {0.0, 40.0}
    }};
    Polygon exclusion{{
        {40.0, 5.0},
        {60.0, 5.0},
        {60.0, 15.0},
        {40.0, 15.0}
    }};
    vector<LineSegment> passes{
        {{0.0, 10.0}, {40.0, 10.0}},
        {{60.0, 10.0}, {100.0, 10.0}},
        {{0.0, 20.0}, {100.0, 20.0}}
    };
    MissionRoute route = buildSafeMissionRoute(field, passes, {exclusion});
    double longestTransitionEdge = 0.0;

    for (size_t i = 1; i < route.waypoints.size(); ++i) {
        LineSegment segment{route.waypoints[i - 1], route.waypoints[i]};
        require(
            isRouteSegmentSafe(segment, field, {exclusion}),
            "Every long-transition regression segment must be safe"
        );

        if (
            classifyRouteSegment(
                route.waypointTypes[i - 1],
                route.waypointTypes[i]
            ) != RouteSegmentType::CoveragePass
        ) {
            longestTransitionEdge = max(
                longestTransitionEdge,
                hypot(
                    segment.end.x - segment.start.x,
                    segment.end.y - segment.start.y
                )
            );
        }
    }

    requireNear(
        totalTransitionDistance(route),
        40.0,
        "Split lane must detour locally before entering the adjacent lane"
    );
    require(
        longestTransitionEdge <= 20.0 + TOLERANCE,
        "Route must not contain the former cross-field transition"
    );
}

void testMultipleObstacles() {
    Polygon field = obstacleTestField();
    DroneConfig drone = obstacleTestDrone();
    vector<Obstacle> obstacles{
        {
            "barn",
            ObstacleType::Barn,
            {{{30.0, 10.0}, {45.0, 10.0}, {45.0, 22.0}, {30.0, 22.0}}},
            1.0
        },
        {
            "restricted",
            ObstacleType::Restricted,
            {{{65.0, 28.0}, {82.0, 28.0}, {82.0, 40.0}, {65.0, 40.0}}},
            1.0
        }
    };
    vector<Polygon> exclusions = safetyBoundaries(obstacles);
    CoveragePlanner planner;
    vector<LineSegment> segments =
        planner.generateCoverageSegments(field, drone, exclusions);
    MissionRoute route = buildSafeMissionRoute(field, segments, exclusions);

    require(segments.size() > 5, "Multiple obstacles must split coverage passes");
    requireSegmentsAvoid(segments, exclusions);
    require(
        routeAvoidsExclusionZones(route, exclusions),
        "Route must avoid every obstacle and no-fly zone"
    );

    RouteStatistics statistics = calculateRouteStatistics(route, drone.speed);
    require(statistics.totalDistance > 0.0, "Obstacle route distance");
    require(
        statistics.transitionSegments >= statistics.coveragePasses - 1,
        "Detours must be included in transition metrics"
    );
}

void testObstacleNearEdge() {
    Polygon field = obstacleTestField();
    DroneConfig drone = obstacleTestDrone();
    Polygon edgeObstacle{{
        {0.0, 20.0},
        {15.0, 20.0},
        {15.0, 30.0},
        {0.0, 30.0}
    }};
    CoveragePlanner planner;
    vector<LineSegment> segments =
        planner.generateCoverageSegments(field, drone, {edgeObstacle});

    bool foundClippedEdgePass = false;

    for (const LineSegment& segment : segments) {
        if (abs(segment.start.y - 25.0) <= TOLERANCE) {
            requireNear(segment.start.x, 15.0, "Edge obstacle clipped start");
            foundClippedEdgePass = true;
        }
    }

    require(foundClippedEdgePass, "Obstacle near field edge must clip cleanly");
    requireSegmentsAvoid(segments, {edgeObstacle});
}

void testNarrowGap() {
    Polygon field = obstacleTestField();
    DroneConfig drone = obstacleTestDrone();
    vector<Polygon> exclusions{
        {{{40.0, 10.0}, {60.0, 10.0}, {60.0, 22.0}, {40.0, 22.0}}},
        {{{40.0, 28.0}, {60.0, 28.0}, {60.0, 40.0}, {40.0, 40.0}}}
    };
    CoveragePlanner planner;
    vector<LineSegment> segments =
        planner.generateCoverageSegments(field, drone, exclusions);
    bool fullPassThroughGap = false;

    for (const LineSegment& segment : segments) {
        if (
            abs(segment.start.y - 25.0) <= TOLERANCE &&
            abs(segment.start.x) <= TOLERANCE &&
            abs(segment.end.x - 100.0) <= TOLERANCE
        ) {
            fullPassThroughGap = true;
        }
    }

    require(fullPassThroughGap, "Valid narrow gap must remain available");
    requireSegmentsAvoid(segments, exclusions);
}

void testConcaveObstacle() {
    Polygon field = obstacleTestField();
    DroneConfig drone = obstacleTestDrone();
    Polygon concaveObstacle{{
        {30.0, 10.0},
        {60.0, 10.0},
        {60.0, 20.0},
        {40.0, 20.0},
        {40.0, 40.0},
        {30.0, 40.0}
    }};
    CoveragePlanner planner;
    vector<LineSegment> segments =
        planner.generateCoverageSegments(field, drone, {concaveObstacle});

    requireSegmentsAvoid(segments, {concaveObstacle});

    bool foundNarrowConcaveCut = false;

    for (const LineSegment& segment : segments) {
        if (
            abs(segment.start.y - 25.0) <= TOLERANCE &&
            abs(segment.start.x - 40.0) <= TOLERANCE
        ) {
            foundNarrowConcaveCut = true;
        }
    }

    require(foundNarrowConcaveCut, "Concave obstacle shape must clip exactly");
}

void testBlockedPass() {
    Polygon field = obstacleTestField();
    DroneConfig drone = obstacleTestDrone();
    Polygon blockingZone{{
        {0.0, 20.0},
        {100.0, 20.0},
        {100.0, 30.0},
        {0.0, 30.0}
    }};
    CoveragePlanner planner;
    vector<LineSegment> segments =
        planner.generateCoverageSegments(field, drone, {blockingZone});

    for (const LineSegment& segment : segments) {
        require(
            abs(segment.start.y - 25.0) > TOLERANCE,
            "Completely blocked pass must be removed"
        );
    }

    require(segments.size() == 4, "Exactly one blocked lane must be removed");
}

void testObstacleOptimization() {
    Polygon field = sampleField();
    DroneConfig drone{{10.0, 90.0, 60.0, 0.30, 0.70}, 6.0};
    vector<Obstacle> obstacles{
        {
            "barn",
            ObstacleType::Barn,
            {{{45.0, 25.0}, {65.0, 25.0}, {65.0, 40.0}, {45.0, 40.0}}},
            2.0
        },
        {
            "pond",
            ObstacleType::Pond,
            {{
                {74.0, 48.0},
                {84.0, 46.0},
                {90.0, 52.0},
                {87.0, 59.0},
                {77.0, 60.0},
                {71.0, 54.0}
            }},
            2.0
        }
    };
    RouteOptimizationResult result = optimizeRoute(
        field,
        drone,
        obstacles,
        generateCandidateAngles(),
        10.0
    );
    vector<Polygon> exclusions = safetyBoundaries(obstacles);

    for (const RouteCandidate& candidate : result.candidates) {
        MissionRoute route{
            candidate.waypoints,
            candidate.waypointTypes,
            candidate.statistics.coveragePasses,
            candidate.statistics.transitionSegments
        };
        require(
            isMissionRouteSafe(route, field, exclusions),
            "Every optimized candidate segment must remain collision-free"
        );

        RouteStatistics recalculated = calculateRouteStatistics(route, drone.speed);
        requireNear(
            recalculated.totalDistance,
            candidate.statistics.totalDistance,
            "Obstacle candidate distance recomputation"
        );
        requireNear(
            candidate.score,
            recalculated.totalDistance +
                result.turnPenalty * recalculated.transitionSegments,
            "Obstacle candidate must be scored after detours"
        );
        require(
            result.bestRoute.score <= candidate.score + TOLERANCE,
            "Obstacle-aware optimization must select the final lowest score"
        );
    }

    require(
        find(
            result.bestRoute.waypointTypes.begin(),
            result.bestRoute.waypointTypes.end(),
            WaypointType::Detour
        ) != result.bestRoute.waypointTypes.end(),
        "Validated obstacle field must contain visible detours"
    );
}

void testPolygonArea() {
    Polygon field = sampleField();
    requireNear(calculatePolygonArea(field), 7550.0, "Sample polygon area");

    reverse(field.vertices.begin(), field.vertices.end());
    requireNear(calculatePolygonArea(field), 7550.0, "Clockwise polygon area");

    Polygon incomplete{{{0.0, 0.0}, {1.0, 1.0}}};
    requireNear(calculatePolygonArea(incomplete), 0.0, "Incomplete polygon area");
}

void testBoundingBox() {
    Polygon field{{
        {-5.0, 2.0},
        {4.0, -3.0},
        {8.0, 7.0},
        {-2.0, 9.0}
    }};

    BoundingBox bounds = calculateBoundingBox(field);
    requireNear(bounds.minX, -5.0, "Bounding-box minX");
    requireNear(bounds.maxX, 8.0, "Bounding-box maxX");
    requireNear(bounds.minY, -3.0, "Bounding-box minY");
    requireNear(bounds.maxY, 9.0, "Bounding-box maxY");

    bool threwForEmptyPolygon = false;

    try {
        calculateBoundingBox(Polygon{});
    }
    catch (const invalid_argument&) {
        threwForEmptyPolygon = true;
    }

    require(threwForEmptyPolygon, "Empty polygon must not have a bounding box");
}

void testPointInPolygon() {
    Polygon field = sampleField();

    require(
        pointInPolygon({50.0, 40.0}, field) == PointLocation::Inside,
        "Inside point classification"
    );
    require(
        pointInPolygon({130.0, 40.0}, field) == PointLocation::Outside,
        "Outside point classification"
    );
    require(
        pointInPolygon({60.0, 0.0}, field) == PointLocation::Boundary,
        "Edge point classification"
    );
    require(
        pointInPolygon({120.0, 30.0}, field) == PointLocation::Boundary,
        "Vertex point classification"
    );
}

void testLineIntersections() {
    LineSegment horizontal{{0.0, 5.0}, {10.0, 5.0}};
    LineSegment vertical{{4.0, 0.0}, {4.0, 10.0}};
    auto crossing = calculateLineSegmentIntersection(horizontal, vertical);
    require(crossing.has_value(), "Crossing segments must intersect");
    requireSamePoint(*crossing, {4.0, 5.0}, "Crossing intersection");

    LineSegment parallel{{0.0, 7.0}, {10.0, 7.0}};
    require(
        !calculateLineSegmentIntersection(horizontal, parallel),
        "Separate parallel segments must not intersect"
    );

    LineSegment endpointTouch{{10.0, 5.0}, {10.0, 9.0}};
    auto endpoint = calculateLineSegmentIntersection(horizontal, endpointTouch);
    require(endpoint.has_value(), "Segments touching at an endpoint must intersect");
    requireSamePoint(*endpoint, {10.0, 5.0}, "Endpoint intersection");

    LineSegment overlap{{5.0, 5.0}, {15.0, 5.0}};
    auto collinear = calculateLineSegmentIntersection(horizontal, overlap);
    require(collinear.has_value(), "Collinear overlapping segments must intersect");
    requireSamePoint(*collinear, {5.0, 5.0}, "Collinear overlap start");
}

void testClippedSegments() {
    CoveragePlanner planner;
    Polygon field = sampleField();
    DroneConfig drone{{10.0, 90.0, 60.0, 0.30, 0.70}, 6.0};

    vector<LineSegment> segments =
        planner.generateCoverageSegments(field, drone);

    require(segments.size() == 6, "Sample polygon coverage-pass count");

    const vector<double> expectedY{5.0, 19.0, 33.0, 47.0, 61.0, 75.0};

    for (size_t i = 0; i < segments.size(); ++i) {
        requireNear(segments[i].start.y, expectedY[i], "Segment start Y");
        requireNear(segments[i].end.y, expectedY[i], "Segment end Y");
        require(segments[i].start.x < segments[i].end.x, "Clipped segment ordering");

        Point midpoint{
            (segments[i].start.x + segments[i].end.x) / 2.0,
            expectedY[i]
        };

        require(
            pointInPolygon(midpoint, field) != PointLocation::Outside,
            "Clipped segment midpoint must be in the field"
        );
    }

    requireSamePoint(segments.front().start, {0.0, 5.0}, "First clipped start");
    requireNear(segments.front().end.x, 103.333333333, "First clipped end X");
}

void testConcaveField() {
    CoveragePlanner planner;
    Polygon field = concaveField();
    DroneConfig drone{{1.0, 90.0, 60.0, 0.0, 0.70}, 2.0};

    vector<LineSegment> segments =
        planner.generateCoverageSegments(field, drone);
    vector<Point> path = planner.generatePath(field, drone);

    require(segments.size() == 9, "Concave field must produce split passes");
    require(path.size() == 18, "Concave field waypoint count");

    for (size_t i = 0; i < segments.size(); ++i) {
        Point midpoint{
            (segments[i].start.x + segments[i].end.x) / 2.0,
            segments[i].start.y
        };

        require(
            pointInPolygon(midpoint, field) != PointLocation::Outside,
            "Concave clipped segment midpoint must be in the field"
        );

        if (i % 2 == 0) {
            requireSamePoint(path[i * 2], segments[i].start, "Left-to-right start");
            requireSamePoint(path[i * 2 + 1], segments[i].end, "Left-to-right end");
        }
        else {
            requireSamePoint(path[i * 2], segments[i].end, "Right-to-left start");
            requireSamePoint(path[i * 2 + 1], segments[i].start, "Right-to-left end");
        }
    }

    requireWaypointsInside(path, field);
}

void testSmallField() {
    CoveragePlanner planner;
    Polygon field{{
        {0.0, 0.0},
        {5.0, 0.0},
        {5.0, 4.0},
        {0.0, 4.0}
    }};
    DroneConfig drone{{10.0, 90.0, 60.0, 0.30, 0.70}, 2.0};

    vector<Point> path = planner.generatePath(field, drone);
    require(path.size() == 2, "Small field needs one coverage pass");
    requireSamePoint(path[0], {0.0, 2.0}, "Small-field route start");
    requireSamePoint(path[1], {5.0, 2.0}, "Small-field route end");

    RouteStatistics statistics = calculateRouteStatistics(path, drone.speed);
    requireNear(statistics.totalDistance, 5.0, "Small-field distance");
    requireNear(statistics.estimatedFlightTime, 2.5, "Small-field flight time");

    bool rejectedInvalidFootprint = false;

    try {
        planner.generatePath(
            field,
            DroneConfig{{0.0, 90.0, 60.0, 0.30, 0.70}, 2.0}
        );
    }
    catch (const invalid_argument&) {
        rejectedInvalidFootprint = true;
    }

    require(rejectedInvalidFootprint, "Zero camera footprint must be rejected");

    bool rejectedInvalidOverlap = false;

    try {
        planner.generatePath(
            field,
            DroneConfig{{10.0, 90.0, 60.0, 1.0, 0.70}, 2.0}
        );
    }
    catch (const invalid_argument&) {
        rejectedInvalidOverlap = true;
    }

    require(rejectedInvalidOverlap, "Full camera overlap must be rejected");
}

void testVertexAndEdgeTouch() {
    CoveragePlanner planner;
    DroneConfig drone{{1.0, 90.0, 60.0, 0.0, 0.70}, 1.0};

    Polygon vertexTouchField{{
        {0.0, 0.0},
        {10.0, 0.0},
        {10.0, 10.0},
        {6.0, 10.0},
        {5.0, 5.0},
        {4.0, 10.0},
        {0.0, 10.0}
    }};

    vector<LineSegment> vertexSegments =
        planner.generateCoverageSegments(vertexTouchField, drone);

    size_t segmentsAtVertexLevel = 0;

    for (const LineSegment& segment : vertexSegments) {
        if (abs(segment.start.y - 5.0) <= TOLERANCE) {
            ++segmentsAtVertexLevel;
        }
    }

    require(
        segmentsAtVertexLevel == 2,
        "A scanline touching a notch vertex must retain even intersections"
    );

    Polygon edgeTouchField = concaveField();
    vector<LineSegment> edgeSegments =
        planner.generateCoverageSegments(edgeTouchField, drone);
    size_t segmentsOnHorizontalEdge = 0;

    for (const LineSegment& segment : edgeSegments) {
        if (abs(segment.start.y - 3.0) <= TOLERANCE) {
            ++segmentsOnHorizontalEdge;
        }
    }

    require(
        segmentsOnHorizontalEdge == 2,
        "A scanline touching a horizontal polygon edge must clip cleanly"
    );
}

void testWaypointContainment() {
    CoveragePlanner planner;
    DroneConfig sampleDrone{{10.0, 90.0, 60.0, 0.30, 0.70}, 6.0};
    DroneConfig concaveDrone{{1.0, 90.0, 60.0, 0.0, 0.70}, 2.0};

    Polygon sample = sampleField();
    Polygon concave = concaveField();

    requireWaypointsInside(planner.generatePath(sample, sampleDrone), sample);
    requireWaypointsInside(planner.generatePath(concave, concaveDrone), concave);
}

void testRouteStatistics() {
    vector<Point> knownPath{
        {0.0, 0.0},
        {3.0, 4.0},
        {6.0, 8.0},
        {6.0, 11.0}
    };

    RouteStatistics known = calculateRouteStatistics(knownPath, 2.0);
    require(known.coveragePasses == 2, "Known route coverage-pass count");
    require(known.transitionSegments == 1, "Known route transition count");
    require(known.waypointCount == 4, "Known route waypoint count");
    requireNear(known.totalDistance, 13.0, "Known route distance");
    requireNear(known.estimatedFlightTime, 6.5, "Known route flight time");

    CoveragePlanner planner;
    DroneConfig drone{{10.0, 90.0, 60.0, 0.30, 0.70}, 6.0};
    RouteStatistics generated = calculateRouteStatistics(
        planner.generatePath(sampleField(), drone),
        drone.speed
    );

    requireNear(generated.totalDistance, 658.051, "Generated route distance", 0.001);
    requireNear(
        generated.estimatedFlightTime,
        generated.totalDistance / drone.speed,
        "Generated route flight time"
    );

    bool rejectedZeroSpeed = false;

    try {
        calculateRouteStatistics(knownPath, 0.0);
    }
    catch (const invalid_argument&) {
        rejectedZeroSpeed = true;
    }

    require(rejectedZeroSpeed, "Zero speed must be rejected");

    bool rejectedOddWaypointCount = false;

    try {
        calculateRouteStatistics({{0.0, 0.0}}, 2.0);
    }
    catch (const invalid_argument&) {
        rejectedOddWaypointCount = true;
    }

    require(rejectedOddWaypointCount, "Odd waypoint count must be rejected");
}

void testFeasibleBatteryMission() {
    Polygon field{{
        {0.0, 0.0},
        {600.0, 0.0},
        {600.0, 100.0},
        {0.0, 100.0}
    }};
    MissionRoute route{
        {{0.0, 0.0}, {600.0, 0.0}},
        {WaypointType::CoverageStart, WaypointType::CoverageEnd},
        1,
        0
    };
    BatteryConfig battery{
        200.0,
        600.0,
        0.20,
        2.0,
        20.0
    };
    BatteryMissionPlan plan = planBatteryMissions(
        route,
        {0.0, 0.0},
        field,
        {},
        10.0,
        battery
    );

    require(plan.singleMissionFeasible, "Short mission must fit one battery");
    require(plan.allMissionsFeasible, "Short mission plan must be feasible");
    require(plan.missions.size() == 1, "Short route needs one mission");
    requireNear(plan.safeFlightTimeSeconds, 480.0, "Reserved flight time");
    requireNear(
        plan.singleMissionEstimate.distance,
        1200.0,
        "Battery estimate must include return-to-home distance"
    );
    require(
        plan.singleMissionEstimate.turnCount == 1,
        "Return-to-home reversal must count as a turn"
    );
    requireNear(
        plan.singleMissionEstimate.totalTimeSeconds,
        142.0,
        "Cruise, turn, takeoff, and landing time"
    );
    requireNear(
        plan.singleMissionEstimate.energyUsedWh,
        47.3333333333,
        "Time-based battery energy estimate"
    );
    require(
        plan.singleMissionEstimate.batteryRemainingPercent >= 20.0,
        "Feasible mission must preserve its configured reserve"
    );
}

MissionRoute fourPassBatteryRoute() {
    return {
        {
            {0.0, 10.0}, {100.0, 10.0},
            {100.0, 20.0}, {0.0, 20.0},
            {0.0, 30.0}, {100.0, 30.0},
            {100.0, 40.0}, {0.0, 40.0}
        },
        {
            WaypointType::CoverageStart, WaypointType::CoverageEnd,
            WaypointType::CoverageStart, WaypointType::CoverageEnd,
            WaypointType::CoverageStart, WaypointType::CoverageEnd,
            WaypointType::CoverageStart, WaypointType::CoverageEnd
        },
        4,
        3
    };
}

void testBatteryMissionSplitting() {
    Polygon field = obstacleTestField();
    MissionRoute route = fourPassBatteryRoute();
    BatteryConfig battery{
        100.0,
        60.0,
        0.20,
        0.0,
        10.0
    };
    BatteryMissionPlan plan = planBatteryMissions(
        route,
        {0.0, 0.0},
        field,
        {},
        10.0,
        battery
    );

    require(
        !plan.singleMissionFeasible,
        "Full route must exceed the reserved one-battery budget"
    );
    require(plan.allMissionsFeasible, "Split route must be feasible");
    require(plan.missions.size() == 2, "Route should split into two missions");
    require(
        plan.missions[0].coveragePasses == 2 &&
            plan.missions[1].coveragePasses == 2,
        "Splitter must keep complete passes and maximize each battery"
    );
    require(
        plan.missions[0].battery.safeWithReserve &&
            plan.missions[1].battery.safeWithReserve,
        "Every split mission must preserve the battery reserve"
    );

    for (const BatteryMission& mission : plan.missions) {
        MissionRoute safetyCheck{
            mission.waypoints,
            mission.waypointTypes,
            mission.coveragePasses,
            0
        };
        require(
            isMissionRouteSafe(safetyCheck, field, {}),
            "Every battery mission and home transit must remain route-safe"
        );
    }

    requireSamePoint(
        plan.missions[0].waypoints.front(),
        {0.0, 0.0},
        "First split mission starts at home"
    );
    requireSamePoint(
        plan.missions[0].waypoints.back(),
        {0.0, 0.0},
        "First split mission returns home"
    );
    requireSamePoint(
        plan.missions[1].waypoints.front(),
        {0.0, 0.0},
        "Second split mission starts at home"
    );
    requireSamePoint(
        plan.missions[1].waypoints.back(),
        {0.0, 0.0},
        "Second split mission returns home"
    );
    requireNear(
        plan.totalCampaignDistance,
        520.0,
        "Split campaign includes both home transits"
    );
}

void testInfeasibleBatteryMission() {
    Polygon field = obstacleTestField();
    MissionRoute route = fourPassBatteryRoute();
    BatteryConfig battery{
        100.0,
        30.0,
        0.20,
        0.0,
        10.0
    };
    BatteryMissionPlan plan = planBatteryMissions(
        route,
        {0.0, 0.0},
        field,
        {},
        10.0,
        battery
    );

    require(!plan.singleMissionFeasible, "Tiny battery cannot fly full route");
    require(
        !plan.allMissionsFeasible,
        "A pass that exceeds one battery must be reported as infeasible"
    );
    require(
        any_of(
            plan.missions.begin(),
            plan.missions.end(),
            [](const BatteryMission& mission) {
                return !mission.battery.safeWithReserve;
            }
        ),
        "Infeasible plan must identify at least one unsafe mission"
    );
}

Polygon coverageTestField() {
    return {{
        {0.0, 0.0},
        {10.0, 0.0},
        {10.0, 10.0},
        {0.0, 10.0}
    }};
}

MissionRoute coverageRoute(const vector<LineSegment>& segments) {
    MissionRoute route{{}, {}, segments.size(), 0};

    for (const LineSegment& segment : segments) {
        route.waypoints.push_back(segment.start);
        route.waypointTypes.push_back(WaypointType::CoverageStart);
        route.waypoints.push_back(segment.end);
        route.waypointTypes.push_back(WaypointType::CoverageEnd);
    }

    return route;
}

void testFullCoverageAnalysis() {
    CoverageAnalysis analysis = analyzeCoverage(
        {coverageRoute({{{0.0, 5.0}, {10.0, 5.0}}})},
        coverageTestField(),
        {},
        {10.0, 2.0},
        1.0
    );
    const CoverageStatistics& statistics = analysis.statistics;

    requireNear(statistics.totalFieldArea, 100.0, "Full coverage field area");
    requireNear(statistics.requiredArea, 100.0, "Full required area");
    requireNear(statistics.coveredArea, 100.0, "Fully covered area");
    requireNear(statistics.missedArea, 0.0, "Full coverage missed area");
    requireNear(statistics.coveragePercent, 100.0, "Full coverage percentage");
    require(statistics.surveySegments == 1, "Full coverage segment count");
}

void testCoverageGapAnalysis() {
    CoverageAnalysis analysis = analyzeCoverage(
        {coverageRoute({{{0.0, 2.0}, {10.0, 2.0}}})},
        coverageTestField(),
        {},
        {2.0, 2.0},
        1.0
    );
    const CoverageStatistics& statistics = analysis.statistics;

    requireNear(statistics.coveredArea, 20.0, "Intentional gap covered area");
    requireNear(statistics.missedArea, 80.0, "Intentional gap missed area");
    requireNear(statistics.coveragePercent, 20.0, "Intentional gap percentage");
    requireNear(statistics.overlapArea, 0.0, "Intentional gap overlap");
}

void testCoverageOverlapAnalysis() {
    MissionRoute route = coverageRoute({
        {{0.0, 4.0}, {10.0, 4.0}},
        {{10.0, 6.0}, {0.0, 6.0}}
    });
    CoverageAnalysis analysis = analyzeCoverage(
        {route},
        coverageTestField(),
        {},
        {4.0, 2.0},
        1.0
    );
    const CoverageStatistics& statistics = analysis.statistics;

    requireNear(statistics.coveredArea, 60.0, "Overlap unique covered area");
    requireNear(statistics.overlapArea, 20.0, "Multiply covered area");
    requireNear(
        statistics.redundantCoverageArea,
        20.0,
        "Redundant camera exposure"
    );
    requireNear(statistics.totalCameraExposureArea, 80.0, "Total exposure area");
    requireNear(statistics.coveragePercent, 60.0, "Overlap coverage percentage");
    requireNear(statistics.overlapPercent, 20.0, "Overlap percentage");
    requireNear(
        statistics.coverageEfficiencyPercent,
        75.0,
        "Unique-to-total exposure efficiency"
    );
}

void testIrregularCoverageAnalysis() {
    Polygon triangle{{
        {0.0, 0.0},
        {10.0, 0.0},
        {0.0, 10.0}
    }};
    CoverageAnalysis analysis = analyzeCoverage(
        {coverageRoute({{{-1.0, 5.0}, {11.0, 5.0}}})},
        triangle,
        {},
        {20.0, 4.0},
        0.25
    );

    requireNear(
        analysis.statistics.totalFieldArea,
        50.0,
        "Irregular field exact shoelace area"
    );
    requireNear(
        analysis.statistics.coveragePercent,
        100.0,
        "Irregular field full coverage"
    );
    requireNear(
        analysis.statistics.missedArea,
        0.0,
        "Irregular field missed area"
    );
}

void testObstacleCoverageAnalysis() {
    Polygon obstacle{{
        {4.0, 0.0},
        {6.0, 0.0},
        {6.0, 10.0},
        {4.0, 10.0}
    }};
    MissionRoute route = coverageRoute({
        {{0.0, 5.0}, {4.0, 5.0}},
        {{6.0, 5.0}, {10.0, 5.0}}
    });
    CoverageAnalysis analysis = analyzeCoverage(
        {route},
        coverageTestField(),
        {obstacle},
        {10.0, 2.0},
        1.0
    );
    const CoverageStatistics& statistics = analysis.statistics;

    requireNear(statistics.excludedArea, 20.0, "Obstacle excluded area");
    requireNear(statistics.requiredArea, 80.0, "Obstacle-adjusted required area");
    requireNear(statistics.coveredArea, 80.0, "Obstacle-adjusted covered area");
    requireNear(
        statistics.coveragePercent,
        100.0,
        "Obstacle area must not count as missed"
    );
}

void testMultiMissionCoverageAnalysis() {
    MissionRoute firstMission{
        {{0.0, 0.0}, {0.0, 2.0}, {10.0, 2.0}, {0.0, 0.0}},
        {
            WaypointType::Transit,
            WaypointType::CoverageStart,
            WaypointType::CoverageEnd,
            WaypointType::Transit
        },
        1,
        0
    };
    MissionRoute secondMission{
        {{0.0, 0.0}, {0.0, 8.0}, {10.0, 8.0}, {0.0, 0.0}},
        {
            WaypointType::Transit,
            WaypointType::CoverageStart,
            WaypointType::CoverageEnd,
            WaypointType::Transit
        },
        1,
        0
    };
    CoverageAnalysis analysis = analyzeCoverage(
        {firstMission, secondMission},
        coverageTestField(),
        {},
        {2.0, 2.0},
        1.0
    );
    const CoverageStatistics& statistics = analysis.statistics;

    requireNear(statistics.coveredArea, 40.0, "Multi-mission covered area");
    requireNear(statistics.missedArea, 60.0, "Multi-mission missed area");
    requireNear(statistics.overlapArea, 0.0, "Multi-mission overlap area");
    requireNear(
        statistics.surveyDistance,
        20.0,
        "Home transit must not count as survey distance"
    );
    require(
        statistics.surveySegments == 2,
        "Only survey segments count across battery missions"
    );
}

struct TestCase {
    const char* name;
    void (*run)();
};

const vector<TestCase> TEST_CASES{
    {"camera_footprint", testCameraFootprint},
    {"overlap_behavior", testOverlapBehavior},
    {"rotation_geometry", testRotationGeometry},
    {"candidate_routes", testCandidateRoutes},
    {"best_angle", testBestAngle},
    {"obstacle_intersections", testObstacleIntersections},
    {"full_route_segment_safety", testFullRouteSegmentSafety},
    {"shortest_obstacle_detour", testShortestObstacleDetour},
    {"route_segment_classification", testRouteSegmentClassification},
    {"coverage_pass_reversal", testCoveragePassReversal},
    {"global_pass_ordering", testGlobalPassOrdering},
    {"long_transition_avoidance", testLongTransitionAvoidance},
    {"multiple_obstacles", testMultipleObstacles},
    {"obstacle_near_edge", testObstacleNearEdge},
    {"narrow_gap", testNarrowGap},
    {"concave_obstacle", testConcaveObstacle},
    {"blocked_pass", testBlockedPass},
    {"obstacle_optimization", testObstacleOptimization},
    {"polygon_area", testPolygonArea},
    {"bounding_box", testBoundingBox},
    {"point_in_polygon", testPointInPolygon},
    {"line_intersections", testLineIntersections},
    {"clipped_segments", testClippedSegments},
    {"concave_field", testConcaveField},
    {"small_field", testSmallField},
    {"vertex_edge_touch", testVertexAndEdgeTouch},
    {"waypoint_containment", testWaypointContainment},
    {"route_statistics", testRouteStatistics},
    {"battery_feasible", testFeasibleBatteryMission},
    {"battery_split", testBatteryMissionSplitting},
    {"battery_infeasible", testInfeasibleBatteryMission},
    {"coverage_full", testFullCoverageAnalysis},
    {"coverage_gap", testCoverageGapAnalysis},
    {"coverage_overlap", testCoverageOverlapAnalysis},
    {"coverage_irregular", testIrregularCoverageAnalysis},
    {"coverage_obstacle", testObstacleCoverageAnalysis},
    {"coverage_multi_mission", testMultiMissionCoverageAnalysis}
};

} // namespace

int main(int argumentCount, char* arguments[]) {
    if (argumentCount != 2) {
        cerr << "Usage: drone_tests TEST_NAME\n";
        return 2;
    }

    string requestedTest = arguments[1];

    for (const TestCase& test : TEST_CASES) {
        if (requestedTest != test.name) {
            continue;
        }

        try {
            test.run();
            cout << "PASS: " << test.name << "\n";
            return 0;
        }
        catch (const exception& error) {
            cerr << "FAIL: " << test.name << ": " << error.what() << "\n";
            return 1;
        }
    }

    cerr << "Unknown test: " << requestedTest << "\n";
    return 2;
}
