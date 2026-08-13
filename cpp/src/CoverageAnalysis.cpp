#include "CoverageAnalysis.h"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "Geometry.h"

using namespace std;

namespace {

constexpr double EPSILON = 1e-9;

vector<LineSegment> collectSurveySegments(
    const vector<MissionRoute>& missions
) {
    vector<LineSegment> segments;

    for (const MissionRoute& mission : missions) {
        if (mission.waypoints.size() != mission.waypointTypes.size()) {
            throw invalid_argument(
                "Coverage analysis requires matching waypoint types"
            );
        }

        for (size_t i = 1; i < mission.waypoints.size(); ++i) {
            if (
                mission.waypointTypes[i - 1] ==
                    WaypointType::CoverageStart &&
                mission.waypointTypes[i] ==
                    WaypointType::CoverageEnd
            ) {
                segments.push_back({
                    mission.waypoints[i - 1],
                    mission.waypoints[i]
                });
            }
        }
    }

    return segments;
}

bool pointIsExcluded(
    const Point& point,
    const vector<Polygon>& exclusionZones
) {
    for (const Polygon& exclusion : exclusionZones) {
        if (pointInPolygon(point, exclusion) != PointLocation::Outside) {
            return true;
        }
    }

    return false;
}

bool pointIsCoveredBySwath(
    const Point& point,
    const LineSegment& segment,
    const CameraFootprint& footprint
) {
    double directionX = segment.end.x - segment.start.x;
    double directionY = segment.end.y - segment.start.y;
    double segmentLength = hypot(directionX, directionY);

    if (segmentLength <= EPSILON) {
        return false;
    }

    double unitX = directionX / segmentLength;
    double unitY = directionY / segmentLength;
    double relativeX = point.x - segment.start.x;
    double relativeY = point.y - segment.start.y;
    double alongTrack = relativeX * unitX + relativeY * unitY;
    double crossTrack = -relativeX * unitY + relativeY * unitX;
    double halfForwardFootprint = footprint.height / 2.0;
    double halfSideFootprint = footprint.width / 2.0;

    return
        alongTrack >= -halfForwardFootprint - EPSILON &&
        alongTrack <= segmentLength + halfForwardFootprint + EPSILON &&
        abs(crossTrack) <= halfSideFootprint + EPSILON;
}

double segmentLength(const LineSegment& segment) {
    return hypot(
        segment.end.x - segment.start.x,
        segment.end.y - segment.start.y
    );
}

} // namespace

CoverageAnalysis analyzeCoverage(
    const vector<MissionRoute>& missions,
    const Polygon& field,
    const vector<Polygon>& exclusionZones,
    const CameraFootprint& footprint,
    double cellSize
) {
    if (field.vertices.size() < 3) {
        throw invalid_argument(
            "Coverage analysis requires a field with at least three vertices"
        );
    }

    for (const Polygon& exclusion : exclusionZones) {
        if (exclusion.vertices.size() < 3) {
            throw invalid_argument(
                "Coverage exclusion zones need at least three vertices"
            );
        }
    }

    if (footprint.width <= 0.0 || footprint.height <= 0.0) {
        throw invalid_argument("Camera footprint dimensions must be positive");
    }

    if (cellSize <= 0.0) {
        throw invalid_argument("Coverage analysis cell size must be positive");
    }

    vector<LineSegment> surveySegments = collectSurveySegments(missions);
    BoundingBox bounds = calculateBoundingBox(field);
    size_t fieldCellCount = 0;
    size_t excludedCellCount = 0;
    size_t requiredCellCount = 0;
    size_t coveredCellCount = 0;
    size_t missedCellCount = 0;
    size_t overlapCellCount = 0;
    size_t redundantObservations = 0;
    size_t totalObservations = 0;
    vector<CoverageCell> cells;

    for (double y = bounds.minY + cellSize / 2.0;
         y < bounds.maxY;
         y += cellSize) {
        for (double x = bounds.minX + cellSize / 2.0;
             x < bounds.maxX;
             x += cellSize) {
            Point center{x, y};

            if (pointInPolygon(center, field) == PointLocation::Outside) {
                continue;
            }

            ++fieldCellCount;

            if (pointIsExcluded(center, exclusionZones)) {
                ++excludedCellCount;
                continue;
            }

            ++requiredCellCount;
            size_t coverageCount = 0;

            for (const LineSegment& segment : surveySegments) {
                if (pointIsCoveredBySwath(center, segment, footprint)) {
                    ++coverageCount;
                }
            }

            CoverageCellStatus status = CoverageCellStatus::Missed;

            if (coverageCount == 0) {
                ++missedCellCount;
            }
            else {
                ++coveredCellCount;
                status = CoverageCellStatus::Covered;

                if (coverageCount > 1) {
                    ++overlapCellCount;
                    redundantObservations += coverageCount - 1;
                    status = CoverageCellStatus::Overlap;
                }
            }

            totalObservations += coverageCount;
            cells.push_back({
                center,
                cellSize,
                coverageCount,
                status
            });
        }
    }

    if (fieldCellCount == 0) {
        throw invalid_argument(
            "Coverage grid is too coarse to sample the field"
        );
    }

    double totalFieldArea = calculatePolygonArea(field);
    double normalizedCellArea =
        totalFieldArea / static_cast<double>(fieldCellCount);
    double requiredArea = requiredCellCount * normalizedCellArea;
    double coveredArea = coveredCellCount * normalizedCellArea;
    double overlapArea = overlapCellCount * normalizedCellArea;
    double totalExposureArea = totalObservations * normalizedCellArea;
    double surveyDistance = 0.0;

    for (const LineSegment& segment : surveySegments) {
        surveyDistance += segmentLength(segment);
    }

    CoverageStatistics statistics{
        totalFieldArea,
        excludedCellCount * normalizedCellArea,
        requiredArea,
        coveredArea,
        missedCellCount * normalizedCellArea,
        overlapArea,
        redundantObservations * normalizedCellArea,
        totalExposureArea,
        requiredArea > 0.0 ? coveredArea / requiredArea * 100.0 : 0.0,
        requiredArea > 0.0 ? overlapArea / requiredArea * 100.0 : 0.0,
        totalExposureArea > 0.0
            ? coveredArea / totalExposureArea * 100.0
            : 0.0,
        surveyDistance,
        surveyDistance > 0.0 ? coveredArea / surveyDistance : 0.0,
        surveySegments.size(),
        cellSize
    };

    return {statistics, std::move(cells)};
}

const char* coverageCellStatusName(CoverageCellStatus status) {
    switch (status) {
        case CoverageCellStatus::Missed:
            return "missed";
        case CoverageCellStatus::Covered:
            return "covered";
        case CoverageCellStatus::Overlap:
            return "overlap";
    }

    return "unknown";
}
