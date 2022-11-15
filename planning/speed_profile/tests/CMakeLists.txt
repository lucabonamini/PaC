set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

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

add_executable(example_speed_profile example_speed_profile.cpp)
target_link_libraries(example_speed_profile
  Utilities::Utilities
  Planning::CubicSplinePlanner
  Planning::SpeedProfile
  Python3::Python
  Python3::Module
)
target_compile_options(example_speed_profile PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>)
