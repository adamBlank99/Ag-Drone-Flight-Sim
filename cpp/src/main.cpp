#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include "CameraConfig.h"
#include "DroneConfig.h"
#include "CoveragePlanner.h"
#include "Geometry.h"
#include "Polygon.h"
#include "RouteStatistics.h"

using namespace std;

const char* pointLocationName(PointLocation location) {
    switch (location) {
        case PointLocation::Inside:
            return "inside";
        case PointLocation::Outside:
            return "outside";
        case PointLocation::Boundary:
            return "boundary";
    }

    return "unknown";
}

int main() {
    DroneConfig drone{
        {
            10.0,
            90.0,
            60.0,
            0.30,
            0.70
        },
        6.0
    };

    Polygon irregularField{{
        {0.0, 0.0},
        {100.0, 0.0},
        {120.0, 30.0},
        {90.0, 70.0},
        {40.0, 80.0},
        {0.0, 50.0}
    }};

    CoveragePlanner planner;

    vector<Point> path =
        planner.generatePath(irregularField, drone);

    ofstream waypointFile("waypoints.csv");
    ofstream polygonFile("field_polygon.csv");
    ofstream footprintFile("camera_footprint.csv");

    if (!waypointFile || !polygonFile || !footprintFile) {
        cerr << "Could not create visualization data files\n";
        return 1;
    }

    waypointFile << "x,y\n";
    polygonFile << "x,y\n";
    footprintFile << "width,height\n";

    CameraFootprint footprint = calculateFootprint(drone.camera);
    double laneSpacing = calculateLaneSpacing(
        footprint,
        drone.camera.sideOverlap
    );
    double photoSpacing = calculatePhotoSpacing(
        footprint,
        drone.camera.forwardOverlap
    );
    double photoCaptureInterval = photoSpacing / drone.speed;

    footprintFile << setprecision(15)
                  << footprint.width << ","
                  << footprint.height << "\n";

    RouteStatistics routeStatistics =
        calculateRouteStatistics(path, drone.speed);
    double irregularFieldArea = calculatePolygonArea(irregularField);
    BoundingBox boundingBox = calculateBoundingBox(irregularField);

    Point insidePoint{50.0, 40.0};
    Point outsidePoint{130.0, 40.0};
    Point boundaryPoint{60.0, 0.0};

    LineSegment horizontalSegment{{0.0, 30.0}, {100.0, 30.0}};
    LineSegment verticalSegment{{50.0, 0.0}, {50.0, 60.0}};
    LineSegment separateSegment{{0.0, 70.0}, {100.0, 70.0}};

    auto crossingIntersection = calculateLineSegmentIntersection(
        horizontalSegment,
        verticalSegment
    );

    auto separateIntersection = calculateLineSegmentIntersection(
        horizontalSegment,
        separateSegment
    );

    for (const Point& waypoint : path) {
        waypointFile
            << waypoint.x << ","
            << waypoint.y << "\n";
    }

    for (const Point& vertex : irregularField.vertices) {
        polygonFile
            << vertex.x << ","
            << vertex.y << "\n";
    }

    cout << "AGRICULTURAL DRONE SURVEY SYSTEM\n\n";

    cout << "Field:\n";
    cout << "Type: irregular polygon\n";
    cout << "Vertices: " << irregularField.vertices.size() << "\n";
    cout << "Area: " << irregularFieldArea << " square meters\n";
    cout << "Bounding box:\n";
    cout << "minX: " << boundingBox.minX << " m\n";
    cout << "maxX: " << boundingBox.maxX << " m\n";
    cout << "minY: " << boundingBox.minY << " m\n";
    cout << "maxY: " << boundingBox.maxY << " m\n\n";

    cout << "Point-in-polygon tests:\n";
    cout << "Inside point (50, 40): "
         << pointLocationName(pointInPolygon(insidePoint, irregularField))
         << "\n";
    cout << "Outside point (130, 40): "
         << pointLocationName(pointInPolygon(outsidePoint, irregularField))
         << "\n";
    cout << "Boundary point (60, 0): "
         << pointLocationName(pointInPolygon(boundaryPoint, irregularField))
         << "\n\n";

    cout << "Line-segment intersection tests:\n";

    if (crossingIntersection) {
        cout << "Crossing segments: intersection at ("
             << crossingIntersection->x << ", "
             << crossingIntersection->y << ")\n";
    }

    cout << "Separate segments: "
         << (separateIntersection ? "intersection" : "no intersection")
         << "\n\n";

    cout << "Camera:\n";
    cout << "Altitude: " << drone.camera.altitude << " m\n";
    cout << "Horizontal FOV: "
         << drone.camera.horizontalFovDegrees << " degrees\n";
    cout << "Vertical FOV: "
         << drone.camera.verticalFovDegrees << " degrees\n";
    cout << "Footprint width: " << footprint.width << " m\n";
    cout << "Footprint height: " << footprint.height << " m\n";
    cout << "Side overlap: "
         << drone.camera.sideOverlap * 100.0 << "%\n";
    cout << "Forward overlap: "
         << drone.camera.forwardOverlap * 100.0 << "%\n";
    cout << "Lane spacing: " << laneSpacing << " m\n";
    cout << "Photo spacing: " << photoSpacing << " m\n";
    cout << "Photo capture interval: "
         << photoCaptureInterval << " seconds\n\n";

    cout << "Route:\n";
    cout << "Coverage passes: " << routeStatistics.coveragePasses << "\n";
    cout << "Transition segments: "
         << routeStatistics.transitionSegments << "\n";
    cout << "Waypoints: " << routeStatistics.waypointCount << "\n";
    cout << "Total distance: " << routeStatistics.totalDistance << " m\n";
    cout << fixed << setprecision(1);
    cout << "Estimated flight time: "
         << routeStatistics.estimatedFlightTime << " seconds\n";

    return 0;
}
