cmake_minimum_required(VERSION 3.15)

project(speed_profile)

set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

add_library(SpeedProfile SHARED src/speed_profile.cpp)
add_library(Planning::SpeedProfile ALIAS SpeedProfile)
target_include_directories(SpeedProfile PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(SpeedProfile
  PRIVATE
    Utilities::Utilities
)
target_compile_options(SpeedProfile PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>)

# if(USE_TESTS)
#     set(CMAKE_CXX_CLANG_TIDY "")
#     message("Compiling SpeedProfile tests")
#     add_subdirectory(tests)
# endif()

install(TARGETS SpeedProfile
    EXPORT SpeedProfile-export
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(EXPORT SpeedProfile-export
    FILE SpeedProfileTargets.cmake
    NAMESPACE Planning::
    DESTINATION lib/cmake/SpeedProfile
)
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
    DESTINATION "${CMAKE_INSTALL_INCLUDE_DIR}/speed_profile"
)
install(
    FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/SpeedProfileConfig.cmake
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/SpeedProfile"
)