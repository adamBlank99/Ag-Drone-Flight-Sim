#include "MissionModel.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <utility>

#include "ObstacleRouter.h"

using namespace std;

namespace {

constexpr double EPSILON = 1e-9;
constexpr double TURN_THRESHOLD_RADIANS =
    1.0 * 3.14159265358979323846 / 180.0;

struct CoveragePassRange {
    size_t startIndex;
    size_t endIndex;
};

bool samePoint(const Point& first, const Point& second) {
    return
        abs(first.x - second.x) <= EPSILON &&
        abs(first.y - second.y) <= EPSILON;
}

void validateBatteryConfiguration(
    double speed,
    const BatteryConfig& battery
) {
    if (speed <= 0.0) {
        throw invalid_argument("Drone speed must be positive");
    }

    if (battery.capacityWh <= 0.0) {
        throw invalid_argument("Battery capacity must be positive");
    }

    if (battery.usableFlightTimeSeconds <= 0.0) {
        throw invalid_argument("Usable battery flight time must be positive");
    }

    if (
        battery.reserveFraction < 0.0 ||
        battery.reserveFraction >= 1.0
    ) {
        throw invalid_argument(
            "Battery reserve fraction must be at least zero and less than one"
        );
    }

    if (battery.secondsPerTurn < 0.0) {
        throw invalid_argument("Turn time cannot be negative");
    }

    if (battery.takeoffLandingTimeSeconds < 0.0) {
        throw invalid_argument("Takeoff and landing time cannot be negative");
    }
}

vector<CoveragePassRange> findCoveragePasses(const MissionRoute& route) {
    if (
        route.waypoints.empty() ||
        route.waypoints.size() != route.waypointTypes.size()
    ) {
        throw invalid_argument(
            "Battery planning requires a non-empty route with waypoint types"
        );
    }

    vector<CoveragePassRange> passes;
    optional<size_t> passStart;

    for (size_t i = 0; i < route.waypointTypes.size(); ++i) {
        switch (route.waypointTypes[i]) {
            case WaypointType::CoverageStart:
                if (passStart) {
                    throw invalid_argument(
                        "Coverage pass starts before the previous pass ends"
                    );
                }
                passStart = i;
                break;

            case WaypointType::CoverageEnd:
                if (!passStart) {
                    throw invalid_argument(
                        "Coverage pass ends without a matching start"
                    );
                }
                passes.push_back({*passStart, i});
                passStart.reset();
                break;

            case WaypointType::Detour:
            case WaypointType::Transit:
                break;
        }
    }

    if (passStart) {
        throw invalid_argument("Coverage route has an unfinished pass");
    }

    if (
        passes.empty() ||
        passes.size() != route.coveragePasses ||
        passes.front().startIndex != 0 ||
        passes.back().endIndex + 1 != route.waypoints.size()
    ) {
        throw invalid_argument(
            "Coverage pass metadata does not match the optimized route"
        );
    }

    return passes;
}

double calculatePathDistance(const vector<Point>& path) {
    double distance = 0.0;

    for (size_t i = 1; i < path.size(); ++i) {
        distance += hypot(
            path[i].x - path[i - 1].x,
            path[i].y - path[i - 1].y
        );
    }

    return distance;
}

size_t countHeadingChanges(const vector<Point>& path) {
    size_t turns = 0;

    for (size_t i = 1; i + 1 < path.size(); ++i) {
        double firstX = path[i].x - path[i - 1].x;
        double firstY = path[i].y - path[i - 1].y;
        double secondX = path[i + 1].x - path[i].x;
        double secondY = path[i + 1].y - path[i].y;
        double firstLength = hypot(firstX, firstY);
        double secondLength = hypot(secondX, secondY);

        if (firstLength <= EPSILON || secondLength <= EPSILON) {
            continue;
        }

        double cosine = clamp(
            (firstX * secondX + firstY * secondY) /
                (firstLength * secondLength),
            -1.0,
            1.0
        );

        if (acos(cosine) > TURN_THRESHOLD_RADIANS) {
            ++turns;
        }
    }

    return turns;
}

BatteryEstimate estimateBatteryUse(
    const vector<Point>& path,
    double speed,
    const BatteryConfig& battery
) {
    double distance = calculatePathDistance(path);
    size_t turnCount = countHeadingChanges(path);
    double cruiseTime = distance / speed;
    double turnTime = turnCount * battery.secondsPerTurn;
    double totalTime =
        cruiseTime +
        turnTime +
        battery.takeoffLandingTimeSeconds;
    double usedFraction = totalTime / battery.usableFlightTimeSeconds;
    double safeTime =
        battery.usableFlightTimeSeconds * (1.0 - battery.reserveFraction);

    return {
        turnCount,
        distance,
        cruiseTime,
        turnTime,
        battery.takeoffLandingTimeSeconds,
        totalTime,
        usedFraction * battery.capacityWh,
        usedFraction * 100.0,
        (1.0 - usedFraction) * 100.0,
        totalTime <= safeTime + EPSILON
    };
}

void appendWaypoint(
    vector<Point>& waypoints,
    vector<WaypointType>& waypointTypes,
    const Point& point,
    WaypointType type
) {
    if (!waypoints.empty() && samePoint(waypoints.back(), point)) {
        if (type != WaypointType::Transit) {
            waypointTypes.back() = type;
        }
        return;
    }

    waypoints.push_back(point);
    waypointTypes.push_back(type);
}

BatteryMission buildMission(
    size_t missionNumber,
    size_t firstPass,
    size_t lastPass,
    const vector<CoveragePassRange>& passes,
    const MissionRoute& route,
    const Point& home,
    const Polygon& field,
    const vector<Polygon>& exclusionZones,
    double speed,
    const BatteryConfig& battery
) {
    size_t firstWaypoint = passes[firstPass].startIndex;
    size_t lastWaypoint = passes[lastPass].endIndex;
    vector<Point> outbound = findShortestSafePath(
        home,
        route.waypoints[firstWaypoint],
        field,
        exclusionZones
    );
    vector<Point> inbound = findShortestSafePath(
        route.waypoints[lastWaypoint],
        home,
        field,
        exclusionZones
    );
    vector<Point> waypoints;
    vector<WaypointType> waypointTypes;

    for (size_t i = 0; i + 1 < outbound.size(); ++i) {
        appendWaypoint(
            waypoints,
            waypointTypes,
            outbound[i],
            WaypointType::Transit
        );
    }

    for (size_t i = firstWaypoint; i <= lastWaypoint; ++i) {
        appendWaypoint(
            waypoints,
            waypointTypes,
            route.waypoints[i],
            route.waypointTypes[i]
        );
    }

    for (size_t i = 1; i < inbound.size(); ++i) {
        appendWaypoint(
            waypoints,
            waypointTypes,
            inbound[i],
            WaypointType::Transit
        );
    }

    MissionRoute safetyCheck{
        waypoints,
        waypointTypes,
        lastPass - firstPass + 1,
        0
    };

    if (!isMissionRouteSafe(safetyCheck, field, exclusionZones)) {
        throw logic_error(
            "Battery mission contains an unsafe transit or route segment"
        );
    }

    return {
        missionNumber,
        firstPass + 1,
        lastPass - firstPass + 1,
        std::move(waypoints),
        std::move(waypointTypes),
        estimateBatteryUse(safetyCheck.waypoints, speed, battery)
    };
}

} // namespace

BatteryMissionPlan planBatteryMissions(
    const MissionRoute& optimizedRoute,
    const Point& home,
    const Polygon& field,
    const vector<Polygon>& exclusionZones,
    double speed,
    const BatteryConfig& battery
) {
    validateBatteryConfiguration(speed, battery);
    vector<CoveragePassRange> passes = findCoveragePasses(optimizedRoute);
    BatteryMission singleMission = buildMission(
        1,
        0,
        passes.size() - 1,
        passes,
        optimizedRoute,
        home,
        field,
        exclusionZones,
        speed,
        battery
    );
    double safeFlightTime =
        battery.usableFlightTimeSeconds * (1.0 - battery.reserveFraction);
    BatteryMissionPlan plan{
        home,
        safeFlightTime,
        singleMission.battery,
        singleMission.battery.safeWithReserve,
        true,
        {},
        0.0,
        0.0,
        0.0
    };

    if (plan.singleMissionFeasible) {
        plan.missions.push_back(std::move(singleMission));
    }
    else {
        size_t firstPass = 0;

        while (firstPass < passes.size()) {
            optional<BatteryMission> largestSafeMission;

            for (size_t lastPass = firstPass;
                 lastPass < passes.size();
                 ++lastPass) {
                BatteryMission candidate = buildMission(
                    plan.missions.size() + 1,
                    firstPass,
                    lastPass,
                    passes,
                    optimizedRoute,
                    home,
                    field,
                    exclusionZones,
                    speed,
                    battery
                );

                if (!candidate.battery.safeWithReserve) {
                    break;
                }

                largestSafeMission = std::move(candidate);
            }

            if (!largestSafeMission) {
                BatteryMission infeasibleMission = buildMission(
                    plan.missions.size() + 1,
                    firstPass,
                    firstPass,
                    passes,
                    optimizedRoute,
                    home,
                    field,
                    exclusionZones,
                    speed,
                    battery
                );
                plan.allMissionsFeasible = false;
                plan.missions.push_back(std::move(infeasibleMission));
                ++firstPass;
                continue;
            }

            firstPass += largestSafeMission->coveragePasses;
            plan.missions.push_back(std::move(*largestSafeMission));
        }
    }

    for (const BatteryMission& mission : plan.missions) {
        plan.totalCampaignDistance += mission.battery.distance;
        plan.totalCampaignTimeSeconds += mission.battery.totalTimeSeconds;
        plan.totalEnergyUsedWh += mission.battery.energyUsedWh;
    }

    return plan;
}
