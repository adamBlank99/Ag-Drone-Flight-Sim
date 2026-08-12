#pragma once

#include "BatteryConfig.h"
#include "CameraConfig.h"

struct DroneConfig {
    CameraConfig camera;
    double speed;
    BatteryConfig battery;

    DroneConfig(
        CameraConfig cameraConfiguration,
        double flightSpeed,
        BatteryConfig batteryConfiguration = {}
    )
        : camera(cameraConfiguration),
          speed(flightSpeed),
          battery(batteryConfiguration) {
    }
};
