#include <fstream>
#include <iostream>
#include <vector>

#include "Field.h"
#include "DroneConfig.h"
#include "CoveragePlanner.h"

int main() {

    Field field{100.0, 60.0};

    DroneConfig drone{
        20.0,
        0.30,
        6.0
    };

    CoveragePlanner planner;

    std::vector<Point> path =
        planner.generatePath(field, drone);

    std::ofstream waypointFile("waypoints.csv");

    if (!waypointFile) {
        std::cerr << "Could not create waypoints.csv\n";
        return 1;
    }

    waypointFile << "x,y\n";

    double laneSpacing =
        drone.footprintWidth *
        (1.0 - drone.overlap);

    std::cout << "Agricultural Drone Survey System\n\n";

    std::cout << "Field Width: "
              << field.width << " m\n";

    std::cout << "Field Height: "
              << field.height << " m\n";

    std::cout << "Lane Spacing: "
              << laneSpacing << " m\n\n";

    std::cout << "Generated Route:\n";

    for (std::size_t i = 0; i < path.size(); ++i) {
        waypointFile
            << path[i].x << ","
            << path[i].y << "\n";

        std::cout
            << "Waypoint " << i + 1
            << ": ("
            << path[i].x
            << ", "
            << path[i].y
            << ")\n";
    }

    std::cout << "\nSaved waypoints to waypoints.csv\n";

    return 0;
}
