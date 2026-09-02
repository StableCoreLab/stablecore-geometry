cmake_minimum_required(VERSION 3.16)

project(SCGeometryPackageConsumer LANGUAGES CXX)

find_package(SCGeometry CONFIG REQUIRED)

add_executable(SCGeometryPackageConsumer main.cpp)
target_link_libraries(SCGeometryPackageConsumer PRIVATE SCGeometry::Geometry)
target_compile_features(SCGeometryPackageConsumer PRIVATE cxx_std_20)
