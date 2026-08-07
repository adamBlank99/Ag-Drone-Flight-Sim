#include "RouteStatistics.h"

#include <cmath>
#include <stdexcept>

using namespace std;

RouteStatistics calculateRouteStatistics(
    const vector<Point>& path,
    double speed
) {
    if (speed <= 0.0) {
        throw invalid_argument("Drone speed must be positive");
    }

    if (path.size() % 2 != 0) {
        throw invalid_argument("A coverage route must contain waypoint pairs");
    }

    double totalDistance = 0.0;

    for (size_t i = 1; i < path.size(); ++i) {
        double changeInX = path[i].x - path[i - 1].x;
        double changeInY = path[i].y - path[i - 1].y;

        totalDistance += hypot(changeInX, changeInY);
    }

    size_t coveragePasses = path.size() / 2;
    size_t transitionSegments =
        coveragePasses > 0 ? coveragePasses - 1 : 0;

    return {
        coveragePasses,
        transitionSegments,
        path.size(),
        totalDistance,
        totalDistance / speed
    };
}
