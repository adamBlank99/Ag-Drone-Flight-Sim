#pragma once

#include <cstddef>
#include <vector>

#include "Point.h"

struct RouteStatistics {
    std::size_t coveragePasses;
    std::size_t transitionSegments;
    std::size_t waypointCount;
    double totalDistance;
    double estimatedFlightTime;
};

RouteStatistics calculateRouteStatistics(
    const std::vector<Point>& path,
    double speed
);
