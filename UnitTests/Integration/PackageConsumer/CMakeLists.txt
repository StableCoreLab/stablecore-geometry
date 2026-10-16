cmake_minimum_required(VERSION 3.16)

project(SCGeometryPackageConsumer LANGUAGES CXX)

find_package(SCGeometry CONFIG REQUIRED)

add_executable(SCGeometryPackageConsumer main.cpp)
target_link_libraries(SCGeometryPackageConsumer PRIVATE SCGeometry::Geometry)
get_target_property(SCGEOMETRY_PACKAGE_TYPE SCGeometry::Geometry TYPE)
if(SCGEOMETRY_PACKAGE_TYPE STREQUAL "SHARED_LIBRARY")
    target_compile_definitions(SCGeometryPackageConsumer PRIVATE SCGEOMETRY_EXPECT_SHARED)
endif()
target_compile_features(SCGeometryPackageConsumer PRIVATE cxx_std_20)
