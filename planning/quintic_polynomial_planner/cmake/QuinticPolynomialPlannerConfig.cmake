include(CMakeFindDependencyMacro)

find_dependency(Eigen3 REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/quintic_polynomial_planner/QuinticPolynomialPlannerTargets.cmake")