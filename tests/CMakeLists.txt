add_test(
    NAME drone_survey.starts
    COMMAND drone_survey
)

set_tests_properties(
    drone_survey.starts
    PROPERTIES
        PASS_REGULAR_EXPRESSION "Agricultural Drone Survey System"
)
