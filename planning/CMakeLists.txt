option(CUBIC_SPLINE "Enable compilation of Cubic Spline Planner" ON)
option(QUARTIC_POLYNOMIAL_PLANNER "Enable compilation of Quartic Polynomial Planner" ON)
option(QUINTIC_POLYNOMIAL_PLANNER "Enable compilation of Quintic Polynomial Planner" ON)

add_subdirectory(speed_profile)

# Cubic Spline Planner
if(CUBIC_SPLINE)
  message("Compiling Cubic Spline Planner")
  add_subdirectory(cubic_spline_planner)
endif()
# Quartic Polynomial Planner
if(QUARTIC_POLYNOMIAL_PLANNER)
  message("Compiling Quartic Polynomial Planner")
  add_subdirectory(quartic_polynomial_planner)
endif()
# Quintic Polynomial Planner
if(QUINTIC_POLYNOMIAL_PLANNER)
  message("Compiling Quintic Polynomial Planner")
  add_subdirectory(quintic_polynomial_planner)
endif()

add_subdirectory(frenet)
