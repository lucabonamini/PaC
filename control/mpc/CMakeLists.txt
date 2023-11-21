cmake_minimum_required(VERSION 3.15)

project(mpc)

set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

find_package(Eigen3 REQUIRED)

add_library(Mpc SHARED src/mpc.cpp)
add_library(Control::Mpc ALIAS Mpc)
target_include_directories(Mpc PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(Mpc
    PRIVATE
      Eigen3::Eigen
      ipopt
)
target_compile_options(Mpc PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>)

if(USE_TESTS)
    set(CMAKE_CXX_CLANG_TIDY "")
    message("Compiling Mpc tests")
    add_subdirectory(tests)
endif()

install(TARGETS Mpc
    EXPORT Mpc-export
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(EXPORT Mpc-export
    FILE MpcTargets.cmake
    NAMESPACE Control::
    DESTINATION lib/cmake/Mpc
)
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
    DESTINATION "${CMAKE_INSTALL_INCLUDE_DIR}/mpc"
)
install(
    FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ModelsConfig.cmake
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/Mpc"
)