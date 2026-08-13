#pragma once

#include <cstddef>
#include <vector>

#include "CameraConfig.h"
#include "MissionRoute.h"
#include "Point.h"
#include "Polygon.h"

enum class CoverageCellStatus {
    Missed,
    Covered,
    Overlap
};

struct CoverageCell {
    Point center;
    double size;
    std::size_t coverageCount;
    CoverageCellStatus status;
};

struct CoverageStatistics {
    double totalFieldArea;
    double excludedArea;
    double requiredArea;
    double coveredArea;
    double missedArea;
    double overlapArea;
    double redundantCoverageArea;
    double totalCameraExposureArea;
    double coveragePercent;
    double overlapPercent;
    double coverageEfficiencyPercent;
    double surveyDistance;
    double coveredAreaPerSurveyMeter;
    std::size_t surveySegments;
    double cellSize;
};

struct CoverageAnalysis {
    CoverageStatistics statistics;
    std::vector<CoverageCell> cells;
};

CoverageAnalysis analyzeCoverage(
    const std::vector<MissionRoute>& missions,
    const Polygon& field,
    const std::vector<Polygon>& exclusionZones,
    const CameraFootprint& footprint,
    double cellSize = 1.0
);

const char* coverageCellStatusName(CoverageCellStatus status);
