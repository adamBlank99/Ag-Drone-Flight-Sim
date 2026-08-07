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

    int numberOfPasses =
        static_cast<int>(ceil(field.height / laneSpacing));

    double actualSpacing =
        field.height / numberOfPasses;

    for (int i = 0; i < numberOfPasses; ++i) {

        double y =
            actualSpacing / 2.0 +
            i * actualSpacing;

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
