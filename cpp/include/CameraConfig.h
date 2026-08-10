#pragma once

struct CameraConfig {
    double altitude;
    double horizontalFovDegrees;
    double verticalFovDegrees;
    double sideOverlap;
    double forwardOverlap;
};

struct CameraFootprint {
    double width;
    double height;
};

CameraFootprint calculateFootprint(const CameraConfig& camera);
double calculateLaneSpacing(
    const CameraFootprint& footprint,
    double sideOverlap
);
double calculatePhotoSpacing(
    const CameraFootprint& footprint,
    double forwardOverlap
);
