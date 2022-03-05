cmake_minimum_required(VERSION 3.15)

project(pure_pursuit)

set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

add_library(PurePursuit SHARED src/pure_pursuit.cpp)
add_library(Control::PurePursuit ALIAS PurePursuit)
target_include_directories(PurePursuit PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>    
)
target_link_libraries(PurePursuit
    PRIVATE
        Utilities::Utilities
)
target_compile_options(PurePursuit PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>)

if (USE_TESTS)
    set(CMAKE_CXX_CLANG_TIDY "")
    message("Compiling PurePursuit tests")
    add_subdirectory(tests)
endif()

install(TARGETS PurePursuit
    EXPORT PurePursuit-export
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(EXPORT PurePursuit-export
    FILE PurePursuitTargets.cmake
    NAMESPACE Planning::
    DESTINATION lib/cmake/PurePursuit
)
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
    DESTINATION "${CMAKE_INSTALL_INCLUDE_DIR}/pure_pursuit"
)
install(
    FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ModelsConfig.cmake
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/PurePursuit"
)