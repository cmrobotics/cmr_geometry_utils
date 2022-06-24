set(TESTS_UTILS_PROJECT "cmr_tests_utils")

set(dependencies
  ${dependencies}
  ${TESTS_UTILS_PROJECT}
)

find_package(${TESTS_UTILS_PROJECT} REQUIRED)


ament_add_gtest(basic_geometry_test basic_geometry_test.cpp)
target_link_libraries(basic_geometry_test
  cmr_geometry_utils_lib
)
ament_target_dependencies(basic_geometry_test ${dependencies})
target_include_directories(basic_geometry_test PRIVATE "../include" "include" ${${TESTS_UTILS_PROJECT}_INCLUDE_DIRS})


# ament_add_gtest(transforms_test transforms_test.cpp)
# target_link_libraries(transforms_test
#   cmr_geometry_utils_lib
# )
# ament_target_dependencies(transforms_test ${dependencies})
# target_include_directories(transforms_test PRIVATE "../include" "include" "include/${PROJECT_NAME}_test"  ${${TESTS_UTILS_PROJECT}_INCLUDE_DIRS})
