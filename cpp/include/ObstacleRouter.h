#pragma once

#include <vector>

#include "Geometry.h"
#include "MissionRoute.h"
#include "Polygon.h"

MissionRoute buildSafeMissionRoute(
    const Polygon& field,
    const std::vector<LineSegment>& coverageSegments,
    const std::vector<Polygon>& exclusionZones
);

bool routeAvoidsExclusionZones(
    const MissionRoute& route,
    const std::vector<Polygon>& exclusionZones
);
