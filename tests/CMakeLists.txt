include(FetchContent)
include(GoogleTest)

find_package(GTest CONFIG QUIET)
if (NOT GTest_FOUND)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
endif()

set(STABLECORE_GEOMETRY_CAPABILITY_TEST_SOURCES
    capabilities/test_box.cpp
    capabilities/test_offset.cpp
    capabilities/test_point_vector.cpp
    capabilities/test_polygon.cpp
    capabilities/test_polyline.cpp
    capabilities/test_relation_boolean.cpp
    capabilities/test_sdk.cpp
    capabilities/test_sdk_algorithms.cpp
    capabilities/test_sdk_multigeometry.cpp
    capabilities/test_segment.cpp
    capabilities/test_serialize.cpp
    capabilities/test_shapes_pathops.cpp
    capabilities/test_topology_indexing.cpp
    capabilities/test_transform_sampling.cpp)

set(STABLECORE_GEOMETRY_GAP_TEST_SOURCES
    gaps/test_offset_gaps.cpp
    gaps/test_searchpoly_gaps.cpp
    gaps/test_relation_gaps.cpp)

add_executable(stablecore_geometry_capabilities_gtest ${STABLECORE_GEOMETRY_CAPABILITY_TEST_SOURCES})
target_link_libraries(stablecore_geometry_capabilities_gtest PRIVATE stablecore_geometry GTest::gtest GTest::gtest_main)
target_include_directories(stablecore_geometry_capabilities_gtest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
gtest_discover_tests(stablecore_geometry_capabilities_gtest)

add_executable(stablecore_geometry_gap_gtest ${STABLECORE_GEOMETRY_GAP_TEST_SOURCES})
target_link_libraries(stablecore_geometry_gap_gtest PRIVATE stablecore_geometry GTest::gtest GTest::gtest_main)
target_include_directories(stablecore_geometry_gap_gtest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
gtest_discover_tests(stablecore_geometry_gap_gtest)
