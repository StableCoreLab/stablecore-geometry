include(GoogleTest)

set(SCGEOMETRY_CAPABILITY_TEST_SOURCES
    Capabilities/Core/TestCore.cpp
    Capabilities/Core/TestCoreAlgorithms.cpp
    Capabilities/Core/TestCoreMultigeometry.cpp
    Capabilities/Core/TestDllExport.cpp
    Capabilities/Core/TestFailureSentinels.cpp
    Capabilities/Core/TestGeometryTestSupport.cpp
    Capabilities/Core/TestMetricsPolylineBox.cpp
    Capabilities/Core/TestMissingAlgorithms.cpp
    Capabilities/Core/TestPathGeometry.cpp
    Capabilities/Core/TestPlaneOps.cpp
    Capabilities/Core/TestPointOps.cpp
    Capabilities/Core/TestPolygonPositiveAreaIntersection.cpp
    Capabilities/Core/TestPolygonTopology.cpp
    Capabilities/Core/TestRelationClassification.cpp
    Capabilities/Core/TestSegmentCodec.cpp
    Capabilities/Core/TestSerialize.cpp
    Capabilities/Core/TestTriangulation.cpp
    Capabilities/Core/TestUmbrella.cpp
    Capabilities/Geometry2d/TestBox.cpp
    Capabilities/Geometry2d/TestMatrixTransform2d.cpp
    Capabilities/Geometry2d/TestOffset.cpp
    Capabilities/Geometry2d/TestPointVector.cpp
    Capabilities/Geometry2d/TestPolygon.cpp
    Capabilities/Geometry2d/TestPolyline.cpp
    Capabilities/Geometry2d/TestRelationBoolean.cpp
    Capabilities/Geometry2d/TestRingTraversalRegression.cpp
    Capabilities/Geometry2d/TestSearchPoly.cpp
    Capabilities/Geometry2d/TestSegment.cpp
    Capabilities/Geometry2d/TestShapesPathOps.cpp
    Capabilities/Geometry2d/TestSpatialIndexOracle.cpp
    Capabilities/Geometry2d/TestTopologyIndexing.cpp
    Capabilities/Geometry2d/TestTransformSampling.cpp
    Capabilities/Geometry3d/Test3dBodyBoolean.cpp
    Capabilities/Geometry3d/Test3dBrep.cpp
    Capabilities/Geometry3d/Test3dConversion.cpp
    Capabilities/Geometry3d/Test3dHealing.cpp
    Capabilities/Geometry3d/Test3dSection.cpp
    Capabilities/Geometry3d/TestGeometryEvidence.cpp
    Capabilities/Geometry3d/TestNurbsNumerics.cpp
)
list(TRANSFORM SCGEOMETRY_CAPABILITY_TEST_SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")

set(SCGEOMETRY_GAP_TEST_SOURCES
    Gaps/Core/TestMissingAlgorithmsGaps.cpp
    Gaps/Geometry2d/TestOffsetGaps.cpp
    Gaps/Geometry2d/TestRelationGaps.cpp
    Gaps/Geometry2d/TestSearchPolyGaps.cpp
    Gaps/Geometry3d/Test3dBodyBooleanGaps.cpp
    Gaps/Geometry3d/Test3dBrepGaps.cpp
    Gaps/Geometry3d/Test3dConversionGaps.cpp
    Gaps/Geometry3d/Test3dHealingGaps.cpp
    Gaps/Geometry3d/Test3dSectionGaps.cpp
    Gaps/Geometry3d/TestMissingAlgorithmsGaps.cpp
)
list(TRANSFORM SCGEOMETRY_GAP_TEST_SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")

set(SCGEOMETRY_CAPABILITY_TESTS_TARGET SCGeometryCapabilitiesTests)
set(SCGEOMETRY_GAP_TESTS_TARGET SCGeometryGapTests)
set(SCGEOMETRY_PUBLIC_HEADERS_COMPILE_TARGET SCGeometryPublicHeadersCompile)

add_library(${SCGEOMETRY_PUBLIC_HEADERS_COMPILE_TARGET} OBJECT
    ${CMAKE_CURRENT_SOURCE_DIR}/Support/PublicHeadersCompile.cpp
)
target_link_libraries(${SCGEOMETRY_PUBLIC_HEADERS_COMPILE_TARGET} PRIVATE ${SCGEOMETRY_TARGET})
SCGeometryConfigureTarget(${SCGEOMETRY_PUBLIC_HEADERS_COMPILE_TARGET})

# 每个状态 API 头独立成为翻译单元，避免伞头掩盖缺失依赖。
foreach(SCGEOMETRY_STATUS_HEADER GeometryEvidence Projection Intersection Measure Relation)
    set(SCGEOMETRY_STATUS_HEADER_TARGET "SCGeometryHeader${SCGEOMETRY_STATUS_HEADER}Compile")
    add_library(${SCGEOMETRY_STATUS_HEADER_TARGET} OBJECT
        ${CMAKE_CURRENT_SOURCE_DIR}/Support/EvidencePublicHeaderCompile.cpp)
    target_compile_definitions(${SCGEOMETRY_STATUS_HEADER_TARGET} PRIVATE
        "SCGEOMETRY_HEADER_UNDER_TEST=\"Core/${SCGEOMETRY_STATUS_HEADER}.h\"")
    target_link_libraries(${SCGEOMETRY_STATUS_HEADER_TARGET} PRIVATE ${SCGEOMETRY_TARGET})
    SCGeometryConfigureTarget(${SCGEOMETRY_STATUS_HEADER_TARGET})
endforeach()

add_executable(${SCGEOMETRY_CAPABILITY_TESTS_TARGET}
    ${SCGEOMETRY_CAPABILITY_TEST_SOURCES}
    ${SCGEOMETRY_ROOT_DIR}/Source/Detail/TriangulateContours2d.cpp
)
target_link_libraries(${SCGEOMETRY_CAPABILITY_TESTS_TARGET} PRIVATE ${SCGEOMETRY_TARGET} GTest::gtest GTest::gtest_main)
if(BUILD_SHARED_LIBS)
    target_compile_definitions(${SCGEOMETRY_CAPABILITY_TESTS_TARGET} PRIVATE SCGEOMETRY_EXPECT_SHARED)
endif()
target_include_directories(${SCGEOMETRY_CAPABILITY_TESTS_TARGET} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
SCGeometryConfigureTarget(${SCGEOMETRY_CAPABILITY_TESTS_TARGET})
SCGeometryConfigurePerformance(${SCGEOMETRY_CAPABILITY_TESTS_TARGET})
SCCopyRuntimeDependenciesToTarget(${SCGEOMETRY_CAPABILITY_TESTS_TARGET} ${SCGEOMETRY_TARGET})
SCCopyRuntimeDependenciesToTarget(${SCGEOMETRY_CAPABILITY_TESTS_TARGET} GTest::gtest)
SCCopyRuntimeDependenciesToTarget(${SCGEOMETRY_CAPABILITY_TESTS_TARGET} GTest::gtest_main)
# 多配置工程必须在测试前按 CTEST_CONFIGURATION_TYPE 发现，避免最后一次构建
# 覆盖共享测试清单而令 -C Debug 实际执行 RelWithDebInfo 程序。
gtest_discover_tests(${SCGEOMETRY_CAPABILITY_TESTS_TARGET} DISCOVERY_MODE PRE_TEST DISCOVERY_TIMEOUT 60)
set_property(DIRECTORY APPEND PROPERTY TEST_INCLUDE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/Support/PerformanceTestProperties.cmake")

add_executable(${SCGEOMETRY_GAP_TESTS_TARGET} ${SCGEOMETRY_GAP_TEST_SOURCES})
target_link_libraries(${SCGEOMETRY_GAP_TESTS_TARGET} PRIVATE ${SCGEOMETRY_TARGET} GTest::gtest GTest::gtest_main)
target_include_directories(${SCGEOMETRY_GAP_TESTS_TARGET} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
SCGeometryConfigureTarget(${SCGEOMETRY_GAP_TESTS_TARGET})
SCGeometryConfigurePerformance(${SCGEOMETRY_GAP_TESTS_TARGET})
SCCopyRuntimeDependenciesToTarget(${SCGEOMETRY_GAP_TESTS_TARGET} ${SCGEOMETRY_TARGET})
SCCopyRuntimeDependenciesToTarget(${SCGEOMETRY_GAP_TESTS_TARGET} GTest::gtest)
SCCopyRuntimeDependenciesToTarget(${SCGEOMETRY_GAP_TESTS_TARGET} GTest::gtest_main)
gtest_discover_tests(${SCGEOMETRY_GAP_TESTS_TARGET} DISCOVERY_MODE PRE_TEST DISCOVERY_TIMEOUT 60)

# This validates the installed CMake package, rather than only the build-tree
# target used by the GTest executables above.
add_test(
    NAME SCGeometry.PackageConsumer
    COMMAND ${CMAKE_COMMAND}
        "-DSCGEOMETRY_BINARY_DIR=${CMAKE_BINARY_DIR}"
        "-DSCGEOMETRY_CONSUMER_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/Integration/PackageConsumer"
        "-DSCGEOMETRY_VERIFY_ROOT=${CMAKE_BINARY_DIR}/PackageConsumerVerification"
        "-DSCGEOMETRY_CONFIG=$<CONFIG>"
        "-DSCGEOMETRY_GENERATOR=${CMAKE_GENERATOR}"
        "-DSCGEOMETRY_GENERATOR_PLATFORM=${CMAKE_GENERATOR_PLATFORM}"
        "-DSCGEOMETRY_INSTALL_BINDIR=${CMAKE_INSTALL_BINDIR}"
        "-DSCGEOMETRY_INSTALL_LIBDIR=${CMAKE_INSTALL_LIBDIR}"
        "-DSCGEOMETRY_PLATFORM=${SCGEOMETRY_PLATFORM}"
        "-DSCGEOMETRY_EXPECT_SHARED=${BUILD_SHARED_LIBS}"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/Integration/VerifyPackageConsumer.cmake"
)
