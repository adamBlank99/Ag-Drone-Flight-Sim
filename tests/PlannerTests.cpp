#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "CoveragePlanner.h"
#include "DroneConfig.h"
#include "Geometry.h"
#include "Polygon.h"
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
    DroneConfig drone{20.0, 0.30, 6.0};

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
    DroneConfig drone{2.0, 0.0, 2.0};

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
    DroneConfig drone{20.0, 0.30, 2.0};

    vector<Point> path = planner.generatePath(field, drone);
    require(path.size() == 2, "Small field needs one coverage pass");
    requireSamePoint(path[0], {0.0, 2.0}, "Small-field route start");
    requireSamePoint(path[1], {5.0, 2.0}, "Small-field route end");

    RouteStatistics statistics = calculateRouteStatistics(path, drone.speed);
    requireNear(statistics.totalDistance, 5.0, "Small-field distance");
    requireNear(statistics.estimatedFlightTime, 2.5, "Small-field flight time");

    bool rejectedInvalidFootprint = false;

    try {
        planner.generatePath(field, DroneConfig{0.0, 0.30, 2.0});
    }
    catch (const invalid_argument&) {
        rejectedInvalidFootprint = true;
    }

    require(rejectedInvalidFootprint, "Zero camera footprint must be rejected");

    bool rejectedInvalidOverlap = false;

    try {
        planner.generatePath(field, DroneConfig{20.0, 1.0, 2.0});
    }
    catch (const invalid_argument&) {
        rejectedInvalidOverlap = true;
    }

    require(rejectedInvalidOverlap, "Full camera overlap must be rejected");
}

void testVertexAndEdgeTouch() {
    CoveragePlanner planner;
    DroneConfig drone{2.0, 0.0, 1.0};

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
    DroneConfig sampleDrone{20.0, 0.30, 6.0};
    DroneConfig concaveDrone{2.0, 0.0, 2.0};

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
    DroneConfig drone{20.0, 0.30, 6.0};
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

struct TestCase {
    const char* name;
    void (*run)();
};

const vector<TestCase> TEST_CASES{
    {"polygon_area", testPolygonArea},
    {"bounding_box", testBoundingBox},
    {"point_in_polygon", testPointInPolygon},
    {"line_intersections", testLineIntersections},
    {"clipped_segments", testClippedSegments},
    {"concave_field", testConcaveField},
    {"small_field", testSmallField},
    {"vertex_edge_touch", testVertexAndEdgeTouch},
    {"waypoint_containment", testWaypointContainment},
    {"route_statistics", testRouteStatistics}
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
