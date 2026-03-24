cmake_minimum_required(VERSION 3.16)

project(stablecore_geometry LANGUAGES CXX)

option(STABLECORE_GEOMETRY_BUILD_TESTS "Build stablecore geometry tests" ON)

add_library(stablecore_geometry
    src/serialize/GeometryText.cpp
    src/sdk/ArcSegment2d.cpp
    src/sdk/Circle2d.cpp
    src/sdk/Ellipse2d.cpp
    src/sdk/GeometryBoolean.cpp
    src/sdk/GeometryEditing.cpp
    src/sdk/GeometryAxisOps.cpp
    src/sdk/GeometryBoxTree.cpp
    src/sdk/GeometryIntersection.cpp
    src/sdk/GeometryMetrics.cpp
    src/sdk/GeometryKDTree.cpp
    src/sdk/GeometryOffset.cpp
    src/sdk/GeometryPathOps.cpp
    src/sdk/GeometryProjection.cpp
    src/sdk/GeometryRelation.cpp
    src/sdk/GeometrySampling.cpp
    src/sdk/GeometrySegmentSearch.cpp
    src/sdk/GeometryShapeOps.cpp
    src/sdk/GeometryTopology.cpp
    src/sdk/GeometryValidation.cpp
    src/sdk/GeometryTransform.cpp
    src/sdk/LineSegment2d.cpp
    src/sdk/MultiPolyline2d.cpp
    src/sdk/MultiPolygon2d.cpp
    src/sdk/Polyline2d.cpp
    src/sdk/Polygon2d.cpp
    src/sdk/Rectangle2d.cpp)
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
