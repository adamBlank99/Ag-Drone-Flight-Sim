#pragma once

#include <vector>

#include "Point.h"
#include "Field.h"
#include "DroneConfig.h"

class CoveragePlanner {
public:
    std::vector<Point> generatePath(
        const Field& field,
        const DroneConfig& drone
    ) const;
};
