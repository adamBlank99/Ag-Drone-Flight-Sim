#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "CameraConfig.h"
#include "CoveragePlanner.h"
#include "DroneConfig.h"
#include "Geometry.h"
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
