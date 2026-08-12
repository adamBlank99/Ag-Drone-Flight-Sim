#pragma once

#include <cstddef>
#include <vector>

#include "Point.h"

enum class WaypointType {
    CoverageStart,
    CoverageEnd,
    Detour,
    Transit
};

struct MissionRoute {
    std::vector<Point> waypoints;
    std::vector<WaypointType> waypointTypes;
    std::size_t coveragePasses;
    std::size_t transitionSegments;
};

const char* waypointTypeName(WaypointType type);
