#include "RouteOptimizer.h"

#include <stdexcept>
#include <utility>

#include "CoveragePlanner.h"
#include "Geometry.h"

using namespace std;

vector<double> generateCandidateAngles() {
    return {0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0};
}

double calculateRouteScore(
    const RouteStatistics& statistics,
    double turnPenalty
) {
    if (turnPenalty < 0.0) {
        throw invalid_argument("Turn penalty cannot be negative");
    }

    return
        statistics.totalDistance +
        turnPenalty * statistics.transitionSegments;
}

RouteOptimizationResult optimizeRoute(
    const Polygon& field,
    const DroneConfig& drone,
    const vector<double>& candidateAngles,
    double turnPenalty
) {
    if (candidateAngles.empty()) {
        throw invalid_argument("At least one candidate angle is required");
    }

    if (turnPenalty < 0.0) {
        throw invalid_argument("Turn penalty cannot be negative");
    }

    BoundingBox bounds = calculateBoundingBox(field);
    Point center{
        (bounds.minX + bounds.maxX) / 2.0,
        (bounds.minY + bounds.maxY) / 2.0
    };

    CoveragePlanner planner;
    vector<RouteCandidate> candidates;
    candidates.reserve(candidateAngles.size());
    size_t bestIndex = 0;

    for (double angle : candidateAngles) {
        Polygon rotatedField = rotatePolygon(field, center, -angle);
        vector<Point> rotatedPath =
            planner.generatePath(rotatedField, drone);
        vector<Point> originalPath;
        originalPath.reserve(rotatedPath.size());

        for (const Point& waypoint : rotatedPath) {
            originalPath.push_back(rotatePoint(waypoint, center, angle));
        }

        RouteStatistics statistics =
            calculateRouteStatistics(originalPath, drone.speed);
        double score = calculateRouteScore(statistics, turnPenalty);

        candidates.push_back({
            angle,
            statistics,
            score,
            std::move(originalPath)
        });

        if (
            candidates.size() == 1 ||
            candidates.back().score < candidates[bestIndex].score
        ) {
            bestIndex = candidates.size() - 1;
        }
    }

    return {
        candidates,
        candidates[bestIndex],
        turnPenalty
    };
}
