#include <iostream>

#include "Point.h"
#include "Field.h"
#include "DroneConfig.h"

int main() {
    Field field{100.0, 60.0};

    DroneConfig drone{
        20.0,   // camera footprint width
        0.30,   // 30% overlap
        6.0     // meters per second
    };

    double laneSpacing =
        drone.footprintWidth * (1.0 - drone.overlap);

    std::cout << "Agricultural Drone Survey System\n\n";

    std::cout << "Field Width: " << field.width << " m\n";
    std::cout << "Field Height: " << field.height << " m\n";

    std::cout << "Camera Footprint: "
              << drone.footprintWidth << " m\n";

    std::cout << "Overlap: "
              << drone.overlap * 100 << "%\n";

    std::cout << "Lane Spacing: "
              << laneSpacing << " m\n";

    return 0;
}
