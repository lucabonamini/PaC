set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(example_pure_pursuit example_pure_pursuit.cpp)
target_link_libraries(example_pure_pursuit
  ${OpenCV_LIBS} 
  Utilities::Utilities
  Models::Unicycle
  Planning::CubicSplinePlanner
  Planning::SpeedProfile
  Control::PurePursuit
)
target_compile_options(example_pure_pursuit PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>)
