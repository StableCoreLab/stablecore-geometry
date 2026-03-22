set(STABLECORE_GEOMETRY_TEST_SOURCES
    test_box.cpp
    test_point_vector.cpp
    test_polygon.cpp
    test_polyline.cpp
    test_sdk.cpp
    test_segment.cpp)

foreach(test_source IN LISTS STABLECORE_GEOMETRY_TEST_SOURCES)
    get_filename_component(test_name "${test_source}" NAME_WE)
    add_executable(${test_name} "${test_source}")
    target_link_libraries(${test_name} PRIVATE stablecore_geometry)
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
