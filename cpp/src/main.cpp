#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include "CameraConfig.h"
#include "CoverageAnalysis.h"
#include "DroneConfig.h"
#include "Geometry.h"
#include "MissionModel.h"
#include "MissionRoute.h"
#include "Obstacle.h"
#include "ObstacleRouter.h"
#include "Polygon.h"
#include "RouteOptimizer.h"
#include "RouteStatistics.h"
#include "SurveyScenario.h"

using namespace std;

int main(int argc, char* argv[]) {
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

    SurveyScenario scenario;

    if (argc == 1) {
        scenario = createDefaultSurveyScenario();
    }
    else if (
        argc == 5 &&
        string(argv[1]) == "--field" &&
        string(argv[3]) == "--obstacles"
    ) {
        try {
            scenario = loadSurveyScenario(argv[2], argv[4]);
        }
        catch (const exception& error) {
            cerr << "Invalid survey scenario: " << error.what() << "\n";
            return 1;
        }
    }
    else {
        cerr
            << "Usage: " << argv[0]
            << " [--field field.csv --obstacles obstacles.csv]\n";
        return 1;
    }

    const Polygon& irregularField = scenario.field;
    const vector<Obstacle>& obstacles = scenario.obstacles;

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
    vector<MissionRoute> flightMissions;

    for (const BatteryMission& mission : batteryPlan.missions) {
        flightMissions.push_back({
            mission.waypoints,
            mission.waypointTypes,
            mission.coveragePasses,
            0
        });
    }

    CoverageAnalysis coverage = analyzeCoverage(
        flightMissions,
        irregularField,
        safetyBoundaries,
        footprint,
        1.0
    );
    const CoverageStatistics& coverageStatistics = coverage.statistics;

    ofstream waypointFile("waypoints.csv");
    ofstream polygonFile("field_polygon.csv");
    ofstream footprintFile("camera_footprint.csv");
    ofstream obstacleFile("obstacles.csv");
    ofstream coverageStatisticsFile("coverage_statistics.csv");
    ofstream coverageGridFile("coverage_grid.csv");

    if (
        !waypointFile ||
        !polygonFile ||
        !footprintFile ||
        !obstacleFile ||
        !coverageStatisticsFile ||
        !coverageGridFile
    ) {
        cerr << "Could not create visualization data files\n";
        return 1;
    }

    waypointFile
        << "angle_degrees,score,total_distance,turns,"
        << "mission_id,mission_count,mission_safe,all_missions_safe,"
        << "first_coverage_pass,coverage_passes,"
        << "mission_distance,mission_duration_seconds,energy_used_wh,"
        << "battery_used_percent,battery_remaining_percent,"
        << "waypoint_type,incoming_segment_type,x,y\n";
    polygonFile << "x,y\n";
    footprintFile << "width,height\n";
    obstacleFile
        << "name,type,clearance,boundary,vertex_index,x,y\n";
    coverageStatisticsFile
        << "total_field_area,excluded_area,required_area,covered_area,"
        << "missed_area,overlap_area,redundant_coverage_area,"
        << "total_camera_exposure_area,coverage_percent,overlap_percent,"
        << "coverage_efficiency_percent,survey_distance,"
        << "covered_area_per_survey_meter,survey_segments,cell_size\n";
    coverageGridFile << "x,y,cell_size,coverage_count,status\n";

    footprintFile << setprecision(15)
                  << footprint.width << ","
                  << footprint.height << "\n";

    const RouteStatistics& routeStatistics = bestRoute.statistics;
    BoundingBox boundingBox = calculateBoundingBox(irregularField);

    coverageStatisticsFile
        << setprecision(15)
        << coverageStatistics.totalFieldArea << ","
        << coverageStatistics.excludedArea << ","
        << coverageStatistics.requiredArea << ","
        << coverageStatistics.coveredArea << ","
        << coverageStatistics.missedArea << ","
        << coverageStatistics.overlapArea << ","
        << coverageStatistics.redundantCoverageArea << ","
        << coverageStatistics.totalCameraExposureArea << ","
        << coverageStatistics.coveragePercent << ","
        << coverageStatistics.overlapPercent << ","
        << coverageStatistics.coverageEfficiencyPercent << ","
        << coverageStatistics.surveyDistance << ","
        << coverageStatistics.coveredAreaPerSurveyMeter << ","
        << coverageStatistics.surveySegments << ","
        << coverageStatistics.cellSize << "\n";

    for (const CoverageCell& cell : coverage.cells) {
        coverageGridFile
            << setprecision(15)
            << cell.center.x << ","
            << cell.center.y << ","
            << cell.size << ","
            << cell.coverageCount << ","
            << coverageCellStatusName(cell.status) << "\n";
    }

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
                         << waypointTypeName(mission.waypointTypes[i]) << ",";

            if (i == 0) {
                waypointFile << "start,";
            }
            else {
                waypointFile
                    << routeSegmentTypeName(classifyRouteSegment(
                        mission.waypointTypes[i - 1],
                        mission.waypointTypes[i]
                    ))
                    << ",";
            }

            waypointFile
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
    cout << "Area: " << coverageStatistics.totalFieldArea
         << " square meters\n";
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
    bool allRouteSegmentsSafe = all_of(
        flightMissions.begin(),
        flightMissions.end(),
        [&](const MissionRoute& mission) {
            return isMissionRouteSafe(
                mission,
                irregularField,
                safetyBoundaries
            );
        }
    );
    cout << "All route segments collision-free: "
         << (allRouteSegmentsSafe ? "yes" : "no") << "\n";
    cout << "Total distance: " << routeStatistics.totalDistance << " m\n";
    cout << fixed << setprecision(1);
    cout << "Estimated flight time: "
         << routeStatistics.estimatedFlightTime << " seconds\n";

    cout << "\nTransition diagnostics:\n";

    for (const BatteryMission& mission : batteryPlan.missions) {
        for (size_t i = 1; i < mission.waypoints.size(); ++i) {
            RouteSegmentType segmentType = classifyRouteSegment(
                mission.waypointTypes[i - 1],
                mission.waypointTypes[i]
            );

            if (segmentType == RouteSegmentType::CoveragePass) {
                continue;
            }

            const Point& start = mission.waypoints[i - 1];
            const Point& end = mission.waypoints[i];
            double distance = hypot(
                end.x - start.x,
                end.y - start.y
            );

            cout << "Mission " << mission.missionNumber
                 << ", segment " << i << ": "
                 << "(" << start.x << ", " << start.y << ") -> "
                 << "(" << end.x << ", " << end.y << "), "
                 << routeSegmentTypeName(segmentType) << ", "
                 << distance << " m\n";
        }
    }

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

    cout << "\nCoverage quality ("
         << coverageStatistics.cellSize << " m grid):\n";
    cout << "Total field area: "
         << coverageStatistics.totalFieldArea << " m^2\n";
    cout << "Excluded obstacle/no-fly area: "
         << coverageStatistics.excludedArea << " m^2\n";
    cout << "Required survey area: "
         << coverageStatistics.requiredArea << " m^2\n";
    cout << "Covered area: "
         << coverageStatistics.coveredArea << " m^2\n";
    cout << "Missed area: "
         << coverageStatistics.missedArea << " m^2\n";
    cout << "Overlapping area: "
         << coverageStatistics.overlapArea << " m^2\n";
    cout << "Redundant coverage: "
         << coverageStatistics.redundantCoverageArea << " m^2\n";
    cout << "Coverage: "
         << coverageStatistics.coveragePercent << "%\n";
    cout << "Overlap: "
         << coverageStatistics.overlapPercent << "%\n";
    cout << "Unique coverage efficiency: "
         << coverageStatistics.coverageEfficiencyPercent << "%\n";
    cout << "Survey-only distance: "
         << coverageStatistics.surveyDistance << " m\n";
    cout << "Covered area per survey meter: "
         << coverageStatistics.coveredAreaPerSurveyMeter << " m^2/m\n";

    return 0;
}
