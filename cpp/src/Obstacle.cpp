#include "Obstacle.h"

#include <stdexcept>

#include "Geometry.h"

using namespace std;

Polygon calculateSafetyBoundary(const Obstacle& obstacle) {
    if (obstacle.boundary.vertices.size() < 3) {
        throw invalid_argument("An obstacle needs at least three vertices");
    }

    if (obstacle.clearance < 0.0) {
        throw invalid_argument("Obstacle clearance cannot be negative");
    }

    if (obstacle.clearance == 0.0) {
        return obstacle.boundary;
    }

    BoundingBox bounds = calculateBoundingBox(obstacle.boundary);

    return {{
        {bounds.minX - obstacle.clearance, bounds.minY - obstacle.clearance},
        {bounds.maxX + obstacle.clearance, bounds.minY - obstacle.clearance},
        {bounds.maxX + obstacle.clearance, bounds.maxY + obstacle.clearance},
        {bounds.minX - obstacle.clearance, bounds.maxY + obstacle.clearance}
    }};
}

const char* obstacleTypeName(ObstacleType type) {
    switch (type) {
        case ObstacleType::Barn:
            return "barn";
        case ObstacleType::Pond:
            return "pond";
        case ObstacleType::Trees:
            return "trees";
        case ObstacleType::Restricted:
            return "restricted";
    }

    return "unknown";
}
