#pragma once

#include <vector>

#include "Geometry.h"
#include "MissionRoute.h"
#include "Polygon.h"

bool isRouteSegmentSafe(
    const LineSegment& segment,
    const Polygon& field,
    const std::vector<Polygon>& exclusionZones
);

std::vector<Point> findShortestSafePath(
    const Point& start,
    const Point& end,
    const Polygon& field,
    const std::vector<Polygon>& exclusionZones
);

MissionRoute buildSafeMissionRoute(
    const Polygon& field,
    const std::vector<LineSegment>& coverageSegments,
    const std::vector<Polygon>& exclusionZones
);

bool isMissionRouteSafe(
    const MissionRoute& route,
    const Polygon& field,
    const std::vector<Polygon>& exclusionZones
);

bool routeAvoidsExclusionZones(
    const MissionRoute& route,
    const std::vector<Polygon>& exclusionZones
);
