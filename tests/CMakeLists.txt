add_executable(drone_tests PlannerTests.cpp)
target_link_libraries(drone_tests PRIVATE drone_core)
set_target_properties(drone_tests PROPERTIES CXX_EXTENSIONS OFF)
enable_project_warnings(drone_tests)

set(DRONE_TEST_CASES
    camera_footprint
    overlap_behavior
    rotation_geometry
    candidate_routes
    best_angle
    obstacle_intersections
    full_route_segment_safety
    shortest_obstacle_detour
    multiple_obstacles
    obstacle_near_edge
    narrow_gap
    concave_obstacle
    blocked_pass
    obstacle_optimization
    polygon_area
    bounding_box
    point_in_polygon
    line_intersections
    clipped_segments
    concave_field
    small_field
    vertex_edge_touch
    waypoint_containment
    route_statistics
    battery_feasible
    battery_split
    battery_infeasible
    coverage_full
    coverage_gap
    coverage_overlap
    coverage_irregular
    coverage_obstacle
    coverage_multi_mission
)

foreach(test_case IN LISTS DRONE_TEST_CASES)
    add_test(
        NAME drone.${test_case}
        COMMAND drone_tests ${test_case}
    )
endforeach()

add_test(NAME drone_survey.smoke COMMAND drone_survey)
set_tests_properties(
    drone_survey.smoke
    PROPERTIES
        PASS_REGULAR_EXPRESSION "AGRICULTURAL DRONE SURVEY SYSTEM"
)
