include(CMakeFindDependencyMacro)

find_dependency(Eigen3 REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/quartic_polynomial_planner/QuarticPolynomialPlannerTargets.cmake")