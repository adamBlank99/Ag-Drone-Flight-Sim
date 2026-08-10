#include "MissionRoute.h"

const char* waypointTypeName(WaypointType type) {
    switch (type) {
        case WaypointType::CoverageStart:
            return "coverage_start";
        case WaypointType::CoverageEnd:
            return "coverage_end";
        case WaypointType::Detour:
            return "detour";
    }

    return "unknown";
}
