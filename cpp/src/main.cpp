#include <fstream>
#include <iostream>
#include <vector>

#include "Field.h"
#include "DroneConfig.h"
#include "CoveragePlanner.h"

using namespace std;

int main() {

    Field field{100.0, 60.0};

    DroneConfig drone{
        20.0,
        0.30,
        6.0
    };

    CoveragePlanner planner;

    vector<Point> path =
        planner.generatePath(field, drone);

    ofstream waypointFile("waypoints.csv");

    if (!waypointFile) {
        cerr << "Could not create waypoints.csv\n";
        return 1;
    }

    waypointFile << "x,y\n";

    double laneSpacing =
        drone.footprintWidth *
        (1.0 - drone.overlap);

    cout << "Agricultural Drone Survey System\n\n";

    cout << "Field Width: "
         << field.width << " m\n";

    cout << "Field Height: "
         << field.height << " m\n";

    cout << "Lane Spacing: "
         << laneSpacing << " m\n\n";

    cout << "Generated Route:\n";

    for (size_t i = 0; i < path.size(); ++i) {
        waypointFile
            << path[i].x << ","
            << path[i].y << "\n";

        cout
            << "Waypoint " << i + 1
            << ": ("
            << path[i].x
            << ", "
            << path[i].y
            << ")\n";
    }

    cout << "\nSaved waypoints to waypoints.csv\n";

    return 0;
}
