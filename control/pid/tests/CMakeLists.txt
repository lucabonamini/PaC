set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(example_pid example_pid.cpp)
target_link_libraries(example_pid
  ${OpenCV_LIBS}
  Utilities::Utilities
  Models::DynamicUnicycle
  Planning::CubicSplinePlanner
  Planning::SpeedProfile
  Control::Pid
)

target_compile_options(example_pid PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>)
