set(STABLECORE_GEOMETRY_TEST_SOURCES
    test_box.cpp
    test_shapes_pathops.cpp
    test_relation_boolean.cpp
    test_sdk_multigeometry.cpp
    test_topology_indexing.cpp
    test_transform_sampling.cpp
    test_point_vector.cpp
    test_sdk_algorithms.cpp
    test_offset.cpp
    test_serialize.cpp
    test_polygon.cpp
    test_polyline.cpp
    test_sdk.cpp
    test_segment.cpp)

foreach(test_source IN LISTS STABLECORE_GEOMETRY_TEST_SOURCES)
    get_filename_component(test_name "${test_source}" NAME_WE)
    add_executable(${test_name} "${test_source}")
    target_link_libraries(${test_name} PRIVATE stablecore_geometry)
    target_include_directories(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
