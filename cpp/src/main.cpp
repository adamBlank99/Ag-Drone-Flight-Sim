#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include "CameraConfig.h"
#include "DroneConfig.h"
#include "Geometry.h"
#include "MissionModel.h"
#include "MissionRoute.h"
#include "Obstacle.h"
#include "Polygon.h"
#include "RouteOptimizer.h"
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
        6.0,
        {
            220.0,  // watt-hours
            900.0,  // 15 minutes under expected flight load
            0.20,   // land with at least 20% remaining
            1.5,    // additional seconds per heading change
            45.0    // combined takeoff and landing time
        }
    };

    Polygon irregularField{{
        {0.0, 0.0},
        {100.0, 0.0},
        {120.0, 30.0},
        {90.0, 70.0},
        {40.0, 80.0},
        {0.0, 50.0}
    }};

    vector<Obstacle> obstacles{
        {
            "barn_1",
            ObstacleType::Barn,
            {{{45.0, 25.0}, {65.0, 25.0}, {65.0, 40.0}, {45.0, 40.0}}},
            2.0
        },
        {
            "pond_1",
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
        },
        {
            "restricted_1",
            ObstacleType::Restricted,
            {{{20.0, 52.0}, {32.0, 52.0}, {32.0, 62.0}, {20.0, 62.0}}},
            1.0
        },
        {
            "tree_cluster_1",
            ObstacleType::Trees,
            {{
                {12.0, 18.0},
                {20.0, 16.0},
                {27.0, 21.0},
                {25.0, 30.0},
                {17.0, 33.0},
                {10.0, 27.0}
            }},
            1.5
        }
    };

    vector<double> candidateAngles = generateCandidateAngles();
    RouteOptimizationResult optimization = optimizeRoute(
        irregularField,
        drone,
        obstacles,
        candidateAngles,
        10.0
    );
    const RouteCandidate& bestRoute = optimization.bestRoute;
    vector<Polygon> safetyBoundaries;

    for (const Obstacle& obstacle : obstacles) {
        safetyBoundaries.push_back(calculateSafetyBoundary(obstacle));
    }

    MissionRoute optimizedMission{
        bestRoute.waypoints,
        bestRoute.waypointTypes,
        bestRoute.statistics.coveragePasses,
        bestRoute.statistics.transitionSegments
    };
    Point home = irregularField.vertices.front();
    BatteryMissionPlan batteryPlan = planBatteryMissions(
        optimizedMission,
        home,
        irregularField,
        safetyBoundaries,
        drone.speed,
        drone.battery
    );

    ofstream waypointFile("waypoints.csv");
    ofstream polygonFile("field_polygon.csv");
    ofstream footprintFile("camera_footprint.csv");
    ofstream obstacleFile("obstacles.csv");

    if (!waypointFile || !polygonFile || !footprintFile || !obstacleFile) {
        cerr << "Could not create visualization data files\n";
        return 1;
    }

    waypointFile
        << "angle_degrees,score,total_distance,turns,"
        << "mission_id,mission_count,mission_safe,all_missions_safe,"
        << "first_coverage_pass,coverage_passes,"
        << "mission_distance,mission_duration_seconds,energy_used_wh,"
        << "battery_used_percent,battery_remaining_percent,"
        << "waypoint_type,x,y\n";
    polygonFile << "x,y\n";
    footprintFile << "width,height\n";
    obstacleFile
        << "name,type,clearance,boundary,vertex_index,x,y\n";

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

    const RouteStatistics& routeStatistics = bestRoute.statistics;
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

    for (const BatteryMission& mission : batteryPlan.missions) {
        for (size_t i = 0; i < mission.waypoints.size(); ++i) {
            waypointFile << setprecision(15)
                         << bestRoute.angleDegrees << ","
                         << bestRoute.score << ","
                         << routeStatistics.totalDistance << ","
                         << routeStatistics.transitionSegments << ","
                         << mission.missionNumber << ","
                         << batteryPlan.missions.size() << ","
                         << (mission.battery.safeWithReserve ? "true" : "false")
                         << ","
                         << (batteryPlan.allMissionsFeasible ? "true" : "false")
                         << ","
                         << mission.firstCoveragePass << ","
                         << mission.coveragePasses << ","
                         << mission.battery.distance << ","
                         << mission.battery.totalTimeSeconds << ","
                         << mission.battery.energyUsedWh << ","
                         << mission.battery.batteryUsedPercent << ","
                         << mission.battery.batteryRemainingPercent << ","
                         << waypointTypeName(mission.waypointTypes[i]) << ","
                         << mission.waypoints[i].x << ","
                         << mission.waypoints[i].y << "\n";
        }
    }

    for (const Point& vertex : irregularField.vertices) {
        polygonFile
            << vertex.x << ","
            << vertex.y << "\n";
    }

    for (size_t obstacleIndex = 0;
         obstacleIndex < obstacles.size();
         ++obstacleIndex) {
        const Obstacle& obstacle = obstacles[obstacleIndex];
        const Polygon& safetyBoundary = safetyBoundaries[obstacleIndex];

        for (size_t i = 0; i < obstacle.boundary.vertices.size(); ++i) {
            const Point& vertex = obstacle.boundary.vertices[i];
            obstacleFile
                << obstacle.name << ","
                << obstacleTypeName(obstacle.type) << ","
                << obstacle.clearance << ",original,"
                << i << "," << vertex.x << "," << vertex.y << "\n";
        }

        for (size_t i = 0; i < safetyBoundary.vertices.size(); ++i) {
            const Point& vertex = safetyBoundary.vertices[i];
            obstacleFile
                << obstacle.name << ","
                << obstacleTypeName(obstacle.type) << ","
                << obstacle.clearance << ",safety,"
                << i << "," << vertex.x << "," << vertex.y << "\n";
        }
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

    cout << "Obstacles and no-fly zones: " << obstacles.size() << "\n";
    for (const Obstacle& obstacle : obstacles) {
        cout << obstacle.name << ": " << obstacleTypeName(obstacle.type)
             << ", clearance " << obstacle.clearance << " m\n";
    }
    cout << "\n";

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

    cout << "Route optimization:\n";
    cout << "Score = distance + " << optimization.turnPenalty
         << " m per turn\n";

    for (const RouteCandidate& candidate : optimization.candidates) {
        cout << candidate.angleDegrees << " degrees -> "
             << candidate.statistics.totalDistance << " m, "
             << candidate.statistics.coveragePasses << " passes, "
             << candidate.statistics.transitionSegments << " turns, "
             << candidate.statistics.estimatedFlightTime << " seconds, "
             << "score " << candidate.score << "\n";
    }

    cout << "Selected best angle: "
         << bestRoute.angleDegrees << " degrees\n\n";

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
    cout << "Selected angle: " << bestRoute.angleDegrees << " degrees\n";
    cout << "Optimization score: " << bestRoute.score << "\n";
    cout << "Coverage passes: " << routeStatistics.coveragePasses << "\n";
    cout << "Transition segments: "
         << routeStatistics.transitionSegments << "\n";
    cout << "Waypoints: " << routeStatistics.waypointCount << "\n";
    cout << "Detour waypoints: "
         << count(
                bestRoute.waypointTypes.begin(),
                bestRoute.waypointTypes.end(),
                WaypointType::Detour
            )
         << "\n";
    cout << "Total distance: " << routeStatistics.totalDistance << " m\n";
    cout << fixed << setprecision(1);
    cout << "Estimated flight time: "
         << routeStatistics.estimatedFlightTime << " seconds\n";

    cout << "\nBattery and mission model:\n";
    cout << "Capacity: " << drone.battery.capacityWh << " Wh\n";
    cout << "Modeled full-battery flight time: "
         << drone.battery.usableFlightTimeSeconds / 60.0
         << " minutes\n";
    cout << "Reserve margin: "
         << drone.battery.reserveFraction * 100.0 << "%\n";
    cout << "Safe time per battery: "
         << batteryPlan.safeFlightTimeSeconds / 60.0
         << " minutes\n";
    cout << "Turn allowance: "
         << drone.battery.secondsPerTurn << " seconds per turn\n";
    cout << "Takeoff/landing allowance: "
         << drone.battery.takeoffLandingTimeSeconds << " seconds\n";
    cout << "Home: (" << batteryPlan.home.x << ", "
         << batteryPlan.home.y << ")\n";
    cout << "Single-battery estimate: "
         << batteryPlan.singleMissionEstimate.totalTimeSeconds
         << " seconds, "
         << batteryPlan.singleMissionEstimate.batteryUsedPercent
         << "% battery\n";
    cout << "Safe on one battery: "
         << (batteryPlan.singleMissionFeasible ? "yes" : "no") << "\n";
    cout << "Planned missions: " << batteryPlan.missions.size() << "\n";

    for (const BatteryMission& mission : batteryPlan.missions) {
        cout << "Mission " << mission.missionNumber << ": "
             << mission.coveragePasses << " passes, "
             << mission.battery.distance << " m, "
             << mission.battery.turnCount << " turns, "
             << mission.battery.totalTimeSeconds << " seconds, "
             << mission.battery.energyUsedWh << " Wh, "
             << mission.battery.batteryUsedPercent << "% battery, "
             << mission.battery.batteryRemainingPercent << "% remaining, "
             << (mission.battery.safeWithReserve ? "safe" : "unsafe")
             << "\n";
    }

    cout << "All missions feasible: "
         << (batteryPlan.allMissionsFeasible ? "yes" : "no") << "\n";
    cout << "Campaign distance: "
         << batteryPlan.totalCampaignDistance << " m\n";
    cout << "Campaign flight time: "
         << batteryPlan.totalCampaignTimeSeconds << " seconds\n";
    cout << "Campaign energy across fresh batteries: "
         << batteryPlan.totalEnergyUsedWh << " Wh\n";

    return 0;
}
