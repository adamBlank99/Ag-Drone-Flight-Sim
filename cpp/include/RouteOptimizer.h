#pragma once

#include <vector>

#include "DroneConfig.h"
#include "Point.h"
#include "Polygon.h"
#include "RouteStatistics.h"

struct RouteCandidate {
    double angleDegrees;
    RouteStatistics statistics;
    double score;
    std::vector<Point> waypoints;
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
