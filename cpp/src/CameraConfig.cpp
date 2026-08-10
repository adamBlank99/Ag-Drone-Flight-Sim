#include "CameraConfig.h"

#include <cmath>
#include <stdexcept>
#include <string>

using namespace std;

namespace {

constexpr double PI = 3.14159265358979323846;

double degreesToRadians(double degrees) {
    return degrees * PI / 180.0;
}

void validateFov(double fovDegrees, const char* name) {
    if (fovDegrees <= 0.0 || fovDegrees >= 180.0) {
        throw invalid_argument(string(name) + " must be between 0 and 180 degrees");
    }
}

void validateOverlap(double overlap, const char* name) {
    if (overlap < 0.0 || overlap >= 1.0) {
        throw invalid_argument(string(name) + " must be between 0 and 1");
    }
}

} // namespace

CameraFootprint calculateFootprint(const CameraConfig& camera) {
    if (camera.altitude <= 0.0) {
        throw invalid_argument("Camera altitude must be positive");
    }

    validateFov(camera.horizontalFovDegrees, "Horizontal FOV");
    validateFov(camera.verticalFovDegrees, "Vertical FOV");

    double halfHorizontalFov =
        degreesToRadians(camera.horizontalFovDegrees) / 2.0;
    double halfVerticalFov =
        degreesToRadians(camera.verticalFovDegrees) / 2.0;

    return {
        2.0 * camera.altitude * tan(halfHorizontalFov),
        2.0 * camera.altitude * tan(halfVerticalFov)
    };
}

double calculateLaneSpacing(
    const CameraFootprint& footprint,
    double sideOverlap
) {
    if (footprint.width <= 0.0) {
        throw invalid_argument("Camera footprint width must be positive");
    }

    validateOverlap(sideOverlap, "Camera side overlap");

    return footprint.width * (1.0 - sideOverlap);
}

double calculatePhotoSpacing(
    const CameraFootprint& footprint,
    double forwardOverlap
) {
    if (footprint.height <= 0.0) {
        throw invalid_argument("Camera footprint height must be positive");
    }

    validateOverlap(forwardOverlap, "Camera forward overlap");

    return footprint.height * (1.0 - forwardOverlap);
}
