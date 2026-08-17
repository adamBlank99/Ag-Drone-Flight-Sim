#include "MissionRoute.h"

const char* waypointTypeName(WaypointType type) {
    switch (type) {
        case WaypointType::CoverageStart:
            return "coverage_start";
        case WaypointType::CoverageEnd:
            return "coverage_end";
        case WaypointType::Detour:
            return "detour";
        case WaypointType::Transit:
            return "transit";
    }

    return "unknown";
}

RouteSegmentType classifyRouteSegment(
    WaypointType startType,
    WaypointType endType
) {
    if (
        startType == WaypointType::CoverageStart &&
        endType == WaypointType::CoverageEnd
    ) {
        return RouteSegmentType::CoveragePass;
    }

    if (
        startType == WaypointType::Transit ||
        endType == WaypointType::Transit
    ) {
        return RouteSegmentType::HomeTransit;
    }

    if (
        startType == WaypointType::Detour ||
        endType == WaypointType::Detour
    ) {
        return RouteSegmentType::ObstacleDetour;
    }

    return RouteSegmentType::NormalTransition;
}

const char* routeSegmentTypeName(RouteSegmentType type) {
    switch (type) {
        case RouteSegmentType::CoveragePass:
            return "coverage_pass";
        case RouteSegmentType::NormalTransition:
            return "normal_transition";
        case RouteSegmentType::ObstacleDetour:
            return "obstacle_detour";
        case RouteSegmentType::HomeTransit:
            return "home_transit";
    }

    return "unknown";
}
