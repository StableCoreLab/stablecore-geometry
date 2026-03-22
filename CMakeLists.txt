cmake_minimum_required(VERSION 3.16)

project(stablecore_geometry LANGUAGES CXX)

option(STABLECORE_GEOMETRY_BUILD_TESTS "Build stablecore geometry tests" ON)

add_library(stablecore_geometry
    src/serialize/GeometryText.cpp
    src/sdk/ArcSegment2d.cpp
    src/sdk/GeometryAlgorithms.cpp
    src/sdk/LineSegment2d.cpp
    src/sdk/Polyline2d.cpp
    src/sdk/Polygon2d.cpp)
add_library(stablecore::geometry ALIAS stablecore_geometry)

target_include_directories(stablecore_geometry
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_compile_features(stablecore_geometry PUBLIC cxx_std_20)

if (BUILD_SHARED_LIBS)
    target_compile_definitions(stablecore_geometry PRIVATE GEOMETRY_BUILD_DLL)
endif()

if (STABLECORE_GEOMETRY_BUILD_TESTS)
    include(CTest)
    enable_testing()
    add_subdirectory(tests)
endif()
