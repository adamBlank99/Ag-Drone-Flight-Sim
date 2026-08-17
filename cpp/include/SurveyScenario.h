#pragma once

#include <string>
#include <vector>

#include "Obstacle.h"
#include "Polygon.h"

struct SurveyScenario {
    Polygon field;
    std::vector<Obstacle> obstacles;
};

SurveyScenario createDefaultSurveyScenario();

SurveyScenario loadSurveyScenario(
    const std::string& fieldPath,
    const std::string& obstaclePath
);
