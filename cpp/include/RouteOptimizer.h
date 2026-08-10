#pragma once

#include <vector>

#include "DroneConfig.h"
#include "MissionRoute.h"
#include "Obstacle.h"
#include "Point.h"
#include "Polygon.h"
#include "RouteStatistics.h"

struct RouteCandidate {
    double angleDegrees;
    RouteStatistics statistics;
    double score;
    std::vector<Point> waypoints;
    std::vector<WaypointType> waypointTypes;
};

struct RouteOptimizationResult {
    std::vector<RouteCandidate> candidates;
    RouteCandidate bestRoute;
    double turnPenalty;
};

std::vector<double> generateCandidateAngles();
double calculateRouteScore(
    const RouteStatistics& statistics,
    double turnPenalty
);
RouteOptimizationResult optimizeRoute(
    const Polygon& field,
    const DroneConfig& drone,
    const std::vector<double>& candidateAngles,
    double turnPenalty = 10.0
);
RouteOptimizationResult optimizeRoute(
    const Polygon& field,
    const DroneConfig& drone,
    const std::vector<Obstacle>& obstacles,
    const std::vector<double>& candidateAngles,
    double turnPenalty = 10.0
);
