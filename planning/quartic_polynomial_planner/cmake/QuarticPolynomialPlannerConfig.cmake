include(CMakeFindDependencyMacro)

find_dependency(Eigen3 REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/cubic_spline_planner/QuarticPolynomialPlannerTargets.cmake")