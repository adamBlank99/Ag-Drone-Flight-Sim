#pragma once

#include <string>

#include "Polygon.h"

enum class ObstacleType {
    Barn,
    Pond,
    Trees,
    Restricted
};

struct Obstacle {
    std::string name;
    ObstacleType type;
    Polygon boundary;
    double clearance;
};

Polygon calculateSafetyBoundary(const Obstacle& obstacle);
const char* obstacleTypeName(ObstacleType type);
