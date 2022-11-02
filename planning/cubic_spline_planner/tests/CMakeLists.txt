set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(PythonLibs 3.0 REQUIRED)
include_directories(${PYTHON3_INCLUDE_DIRS} ${NumPy_INCLUDE_DIRS})

include(FetchContent)
FetchContent_Declare(
    matplotlib
    GIT_REPOSITORY https://github.com/lava/matplotlib-cpp.git
    GIT_TAG        f23347fca25219d1c42cbb91608b5556814bf572
)
FetchContent_GetProperties(matplotlib)
if(NOT matplotlib_POPULATED)
    FetchContent_Populate(matplotlib)
endif()
include_directories(SYSTEM ${matplotlib_SOURCE_DIR})

add_executable(example_cubic_spline_planner example_cubic_spline_planner.cpp)
target_link_libraries(example_cubic_spline_planner
  Utilities::Utilities
  Planning::SpeedProfile
  Planning::CubicSplinePlanner
  ${PYTHON_LIBRARIES}
  Python3::NumPy
)
target_compile_options(example_cubic_spline_planner PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>)