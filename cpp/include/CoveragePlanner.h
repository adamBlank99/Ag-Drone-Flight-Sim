#pragma once

#include <vector>

#include "Point.h"
#include "Field.h"
#include "DroneConfig.h"
#include "Geometry.h"
#include "Polygon.h"

class CoveragePlanner {
public:
    std::vector<Point> generatePath(
        const Field& field,
        const DroneConfig& drone
    ) const;

    std::vector<Point> generatePath(
        const Polygon& field,
        const DroneConfig& drone
    ) const;

    std::vector<LineSegment> generateCoverageSegments(
        const Polygon& field,
        const DroneConfig& drone
    ) const;

    std::vector<LineSegment> generateCoverageSegments(
        const Polygon& field,
        const DroneConfig& drone,
        const std::vector<Polygon>& exclusionZones
    ) const;
};
