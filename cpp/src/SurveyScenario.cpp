#include "SurveyScenario.h"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "Geometry.h"

using namespace std;

namespace {

constexpr double EPSILON = 1e-9;

vector<string> splitCsvRow(const string& row) {
    vector<string> values;
    string value;
    stringstream stream(row);

    while (getline(stream, value, ',')) {
        const size_t firstCharacter = value.find_first_not_of(" \t\r");
        const size_t lastCharacter = value.find_last_not_of(" \t\r");

        if (firstCharacter == string::npos) {
            value.clear();
        }
        else {
            value = value.substr(
                firstCharacter,
                lastCharacter - firstCharacter + 1
            );
        }

        values.push_back(value);
    }

    return values;
}

double parseNumber(const string& value, const string& description) {
    size_t parsedCharacters = 0;

    try {
        double number = stod(value, &parsedCharacters);

        if (parsedCharacters != value.size() || !isfinite(number)) {
            throw invalid_argument("not a finite number");
        }

        return number;
    }
    catch (const exception&) {
        throw runtime_error("Invalid " + description + ": '" + value + "'");
    }
}

ObstacleType parseObstacleType(const string& value) {
    if (value == "barn") {
        return ObstacleType::Barn;
    }
    if (value == "pond") {
        return ObstacleType::Pond;
    }
    if (value == "trees") {
        return ObstacleType::Trees;
    }
    if (value == "restricted") {
        return ObstacleType::Restricted;
    }

    throw runtime_error("Unknown obstacle type: '" + value + "'");
}

bool polygonsHaveSamePoint(const Point& first, const Point& second) {
    return
        abs(first.x - second.x) <= EPSILON &&
        abs(first.y - second.y) <= EPSILON;
}

bool isSimplePolygon(const Polygon& polygon) {
    const size_t vertexCount = polygon.vertices.size();

    for (size_t firstIndex = 0; firstIndex < vertexCount; ++firstIndex) {
        size_t firstNext = (firstIndex + 1) % vertexCount;
        LineSegment first{
            polygon.vertices[firstIndex],
            polygon.vertices[firstNext]
        };

        if (polygonsHaveSamePoint(first.start, first.end)) {
            return false;
        }

        for (size_t secondIndex = firstIndex + 1;
             secondIndex < vertexCount;
             ++secondIndex) {
            size_t secondNext = (secondIndex + 1) % vertexCount;
            bool adjacent =
                firstIndex == secondNext ||
                firstNext == secondIndex;

            if (adjacent) {
                continue;
            }

            LineSegment second{
                polygon.vertices[secondIndex],
                polygon.vertices[secondNext]
            };

            if (calculateLineSegmentIntersection(first, second)) {
                return false;
            }
        }
    }

    return true;
}

void validatePolygon(
    const Polygon& polygon,
    const string& description,
    size_t maximumVertices
) {
    if (polygon.vertices.size() < 3) {
        throw runtime_error(description + " needs at least three vertices");
    }

    if (polygon.vertices.size() > maximumVertices) {
        throw runtime_error(
            description + " exceeds the " +
            to_string(maximumVertices) + " vertex limit"
        );
    }

    if (calculatePolygonArea(polygon) <= EPSILON) {
        throw runtime_error(description + " has zero area");
    }

    if (!isSimplePolygon(polygon)) {
        throw runtime_error(description + " is self-intersecting");
    }
}

void validateScenario(const SurveyScenario& scenario) {
    validatePolygon(scenario.field, "Field polygon", 8);

    for (const Obstacle& obstacle : scenario.obstacles) {
        validatePolygon(obstacle.boundary, "Obstacle '" + obstacle.name + "'", 16);

        if (obstacle.clearance < 0.0) {
            throw runtime_error(
                "Obstacle '" + obstacle.name + "' has negative clearance"
            );
        }

        Polygon safetyBoundary = calculateSafetyBoundary(obstacle);

        for (const Point& vertex : safetyBoundary.vertices) {
            if (pointInPolygon(vertex, scenario.field) == PointLocation::Outside) {
                throw runtime_error(
                    "Obstacle '" + obstacle.name +
                    "' safety buffer extends outside the field"
                );
            }
        }

        for (size_t index = 0;
             index < safetyBoundary.vertices.size();
             ++index) {
            LineSegment safetyEdge{
                safetyBoundary.vertices[index],
                safetyBoundary.vertices[
                    (index + 1) % safetyBoundary.vertices.size()
                ]
            };

            if (!segmentStaysInsidePolygon(safetyEdge, scenario.field)) {
                throw runtime_error(
                    "Obstacle '" + obstacle.name +
                    "' safety buffer crosses the field boundary"
                );
            }
        }
    }
}

} // namespace

SurveyScenario createDefaultSurveyScenario() {
    return {
        {{
            {0.0, 0.0},
            {100.0, 0.0},
            {120.0, 30.0},
            {90.0, 70.0},
            {40.0, 80.0},
            {0.0, 50.0}
        }},
        {
            {
                "barn_1",
                ObstacleType::Barn,
                {{{45.0, 25.0}, {65.0, 25.0}, {65.0, 40.0}, {45.0, 40.0}}},
                2.0
            },
            {
                "pond_1",
                ObstacleType::Pond,
                {{
                    {74.0, 48.0},
                    {84.0, 46.0},
                    {90.0, 52.0},
                    {87.0, 59.0},
                    {77.0, 60.0},
                    {71.0, 54.0}
                }},
                2.0
            },
            {
                "restricted_1",
                ObstacleType::Restricted,
                {{{20.0, 52.0}, {32.0, 52.0}, {32.0, 62.0}, {20.0, 62.0}}},
                1.0
            },
            {
                "tree_cluster_1",
                ObstacleType::Trees,
                {{
                    {12.0, 18.0},
                    {20.0, 16.0},
                    {27.0, 21.0},
                    {25.0, 30.0},
                    {17.0, 33.0},
                    {10.0, 27.0}
                }},
                1.5
            }
        }
    };
}

SurveyScenario loadSurveyScenario(
    const string& fieldPath,
    const string& obstaclePath
) {
    ifstream fieldFile(fieldPath);

    if (!fieldFile) {
        throw runtime_error("Could not open field scenario file: " + fieldPath);
    }

    SurveyScenario scenario;
    string row;
    size_t lineNumber = 0;

    while (getline(fieldFile, row)) {
        ++lineNumber;

        if (row.empty() || row.front() == '#') {
            continue;
        }

        vector<string> values = splitCsvRow(row);

        if (lineNumber == 1 && values.size() >= 2 && values[0] == "x") {
            continue;
        }

        if (values.size() != 2) {
            throw runtime_error(
                "Field CSV line " + to_string(lineNumber) +
                " must contain x,y"
            );
        }

        scenario.field.vertices.push_back({
            parseNumber(values[0], "field x coordinate"),
            parseNumber(values[1], "field y coordinate")
        });
    }

    ifstream obstacleFile(obstaclePath);

    if (!obstacleFile) {
        throw runtime_error("Could not open obstacle scenario file: " + obstaclePath);
    }

    lineNumber = 0;

    while (getline(obstacleFile, row)) {
        ++lineNumber;

        if (row.empty() || row.front() == '#') {
            continue;
        }

        vector<string> values = splitCsvRow(row);

        if (lineNumber == 1 && !values.empty() && values[0] == "name") {
            continue;
        }

        if (values.size() != 6) {
            throw runtime_error(
                "Obstacle CSV line " + to_string(lineNumber) +
                " must contain name,type,clearance,vertex_index,x,y"
            );
        }

        const string& name = values[0];
        ObstacleType type = parseObstacleType(values[1]);
        double clearance = parseNumber(values[2], "obstacle clearance");
        double parsedVertexIndex = parseNumber(
            values[3],
            "obstacle vertex index"
        );

        if (
            parsedVertexIndex < 0.0 ||
            floor(parsedVertexIndex) != parsedVertexIndex
        ) {
            throw runtime_error("Obstacle vertex index must be a whole number");
        }

        size_t vertexIndex = static_cast<size_t>(parsedVertexIndex);

        if (
            scenario.obstacles.empty() ||
            scenario.obstacles.back().name != name
        ) {
            scenario.obstacles.push_back({name, type, {}, clearance});
        }

        Obstacle& obstacle = scenario.obstacles.back();

        if (
            obstacle.type != type ||
            abs(obstacle.clearance - clearance) > EPSILON
        ) {
            throw runtime_error(
                "Obstacle '" + name + "' has inconsistent metadata"
            );
        }

        if (vertexIndex != obstacle.boundary.vertices.size()) {
            throw runtime_error(
                "Obstacle '" + name + "' vertices are out of order"
            );
        }

        obstacle.boundary.vertices.push_back({
            parseNumber(values[4], "obstacle x coordinate"),
            parseNumber(values[5], "obstacle y coordinate")
        });
    }

    validateScenario(scenario);
    return scenario;
}
