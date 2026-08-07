#include "CoveragePlanner.h"

#include <cmath>

using namespace std;

vector<Point> CoveragePlanner::generatePath(
    const Field& field,
    const DroneConfig& drone
) const {

    vector<Point> path;

    double laneSpacing =
        drone.footprintWidth * (1.0 - drone.overlap);

    int numberOfPasses = 1;

    if (field.height > drone.footprintWidth) {
        numberOfPasses = 1 + static_cast<int>(ceil(
            (field.height - drone.footprintWidth) / laneSpacing
        ));
    }

    double routeHeight =
        (numberOfPasses - 1) * laneSpacing;

    double firstLaneY =
        (field.height - routeHeight) / 2.0;

    for (int i = 0; i < numberOfPasses; ++i) {

        double y = firstLaneY + i * laneSpacing;

        if (i % 2 == 0) {
            path.push_back({0.0, y});
            path.push_back({field.width, y});
        }
        else {
            path.push_back({field.width, y});
            path.push_back({0.0, y});
        }
    }

    return path;
}
