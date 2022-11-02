option(CUBIC_SPLINE "Enable compilation of Cubic Spline Planner" ON)

# Cubic Spline Planner
if(CUBIC_SPLINE)
  message("Compiling Cubic Spline Planner")
  add_subdirectory(cubic_spline_planner)
  add_subdirectory(speed_profile)
endif()
