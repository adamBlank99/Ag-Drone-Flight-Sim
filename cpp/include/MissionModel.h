#pragma once

#include <cstddef>
#include <vector>

#include "BatteryConfig.h"
#include "MissionRoute.h"
#include "Point.h"
#include "Polygon.h"

struct BatteryEstimate {
    std::size_t turnCount;
    double distance;
    double cruiseTimeSeconds;
    double turnTimeSeconds;
    double fixedTimeSeconds;
    double totalTimeSeconds;
    double energyUsedWh;
    double batteryUsedPercent;
    double batteryRemainingPercent;
    bool safeWithReserve;
};

struct BatteryMission {
    std::size_t missionNumber;
    std::size_t firstCoveragePass;
    std::size_t coveragePasses;
    std::vector<Point> waypoints;
    std::vector<WaypointType> waypointTypes;
    BatteryEstimate battery;
};

struct BatteryMissionPlan {
    Point home;
    double safeFlightTimeSeconds;
    BatteryEstimate singleMissionEstimate;
    bool singleMissionFeasible;
    bool allMissionsFeasible;
    std::vector<BatteryMission> missions;
    double totalCampaignDistance;
    double totalCampaignTimeSeconds;
    double totalEnergyUsedWh;
};

BatteryMissionPlan planBatteryMissions(
    const MissionRoute& optimizedRoute,
    const Point& home,
    const Polygon& field,
    const std::vector<Polygon>& exclusionZones,
    double speed,
    const BatteryConfig& battery
);
