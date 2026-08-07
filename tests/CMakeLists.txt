add_test(
    NAME drone_survey.starts
    COMMAND drone_survey
)

set_tests_properties(
    drone_survey.starts
    PROPERTIES
        PASS_REGULAR_EXPRESSION "AGRICULTURAL DRONE SURVEY SYSTEM"
)

add_test(
    NAME drone_survey.irregular_route
    COMMAND drone_survey
)

set_tests_properties(
    drone_survey.irregular_route
    PROPERTIES
        PASS_REGULAR_EXPRESSION "Coverage passes: 6"
)

add_test(
    NAME drone_survey.irregular_waypoints
    COMMAND drone_survey
)

set_tests_properties(
    drone_survey.irregular_waypoints
    PROPERTIES
        PASS_REGULAR_EXPRESSION "Waypoints: 12"
)

add_test(
    NAME drone_survey.route_transitions
    COMMAND drone_survey
)

set_tests_properties(
    drone_survey.route_transitions
    PROPERTIES
        PASS_REGULAR_EXPRESSION "Transition segments: 5"
)

add_test(
    NAME drone_survey.polygon_area
    COMMAND drone_survey
)

set_tests_properties(
    drone_survey.polygon_area
    PROPERTIES
        PASS_REGULAR_EXPRESSION "Area: 7550 square meters"
)

add_test(
    NAME drone_survey.bounding_box
    COMMAND drone_survey
)

set_tests_properties(
    drone_survey.bounding_box
    PROPERTIES
        PASS_REGULAR_EXPRESSION "maxX: 120 m"
)

add_test(
    NAME drone_survey.point_in_polygon
    COMMAND drone_survey
)

set_tests_properties(
    drone_survey.point_in_polygon
    PROPERTIES
        PASS_REGULAR_EXPRESSION "Boundary point .*: boundary"
)

add_test(
    NAME drone_survey.segment_crossing
    COMMAND drone_survey
)

set_tests_properties(
    drone_survey.segment_crossing
    PROPERTIES
        PASS_REGULAR_EXPRESSION "Crossing segments: intersection at .*50, 30"
)

add_test(
    NAME drone_survey.segment_separate
    COMMAND drone_survey
)

set_tests_properties(
    drone_survey.segment_separate
    PROPERTIES
        PASS_REGULAR_EXPRESSION "Separate segments: no intersection"
)
