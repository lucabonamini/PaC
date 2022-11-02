cmake_minimum_required(VERSION 3.15)

project(PaC)
include(cmake/StandardProjectSettings.cmake)
include(cmake/PreventInSourceBuilds.cmake)

add_library(project_options INTERFACE)
target_compile_features(project_options INTERFACE cxx_std_17)

add_library(project_warnings INTERFACE)

include(cmake/Cache.cmake)

include(cmake/CompilerWarnings.cmake)
set_project_warnings(project_warnings)

include(cmake/Sanitizers.cmake)
enable_sanitizers(project_options)

include(cmake/Doxygen.cmake)
enable_doxygen()

include(cmake/StaticAnalyzers.cmake)

option(USE_UTILITIES "Enable compilation of Utilities library" ON)
option(USE_MODELS "Enable compilation of Models library" ON)
option(USE_PLANNING "Enable compilation of Planning algorithms" ON)
option(USE_CONTROL "Enable compilation of Control algorithms" ON)

option(USE_TESTS "Enable tests" ON)

# Utilities
if(USE_UTILITIES)
  message("Compiling Utilities")
  add_subdirectory(utilities)
endif()

# Models
if(USE_MODELS)
  message("Compiling Models")
  add_subdirectory(models)
endif()

# Planning
if(USE_PLANNING)
  message("Compiling Planners")
  add_subdirectory(planning)
endif()

# Control
if(USE_CONTROL)
  message("Compiling Controllers")
  add_subdirectory(control)
endif()
