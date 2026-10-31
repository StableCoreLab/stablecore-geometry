include(GoogleTest)

set(SCGEOMETRY_CAPABILITY_TEST_SOURCES
    Capabilities/Core/TestCoreAlgorithms.cpp
    Capabilities/Core/TestCoreMultigeometry.cpp
    Capabilities/Core/TestDllExport.cpp
    Capabilities/Core/TestFailureSentinels.cpp
    Capabilities/Core/TestGeometryTestSupport.cpp
    Capabilities/Core/TestMetricsPolylineBox.cpp
    Capabilities/Core/TestAlgorithmCapabilities.cpp
    Capabilities/Core/TestPathGeometry.cpp
    Capabilities/Core/TestPathGeometryMapping.cpp
    Capabilities/Core/TestPathGeometryOffset.cpp
    Capabilities/Core/TestPathGeometryCorner.cpp
    Capabilities/Core/TestPlaneOps.cpp
    Capabilities/Core/TestPointOps.cpp
    Capabilities/Core/TestPolygonPositiveAreaIntersection.cpp
    Capabilities/Core/TestPolygonTopology.cpp
    Capabilities/Core/TestRelationClassification.cpp
    Capabilities/Core/TestSegmentCodec.cpp
    Capabilities/Core/TestSerialize.cpp
    Capabilities/Core/TestTriangulation.cpp
    Capabilities/Core/TestValidationTolerance.cpp
    Capabilities/Core/TestUmbrella.cpp
    Capabilities/Geometry2d/TestBox.cpp
    Capabilities/Geometry2d/TestCorePrimitives2d.cpp
    Capabilities/Geometry2d/TestMatrixTransform2d.cpp
    Capabilities/Geometry2d/TestMeasure2d.cpp
    Capabilities/Geometry2d/TestOffset.cpp
    Capabilities/Geometry2d/TestPointVector.cpp
    Capabilities/Geometry2d/TestPolygon.cpp
    Capabilities/Geometry2d/TestPolyline.cpp
    Capabilities/Geometry2d/TestRelationBoolean.cpp
    Capabilities/Geometry2d/TestRelationBooleanEdgeCases.cpp
    Capabilities/Geometry2d/TestRelationBooleanDegeneracies.cpp
    Capabilities/Geometry2d/TestRingTraversalRegression.cpp
    Capabilities/Geometry2d/TestSearchPoly.cpp
    Capabilities/Geometry2d/TestSearchPolyRepairRanking.cpp
    Capabilities/Geometry2d/TestSearchPolyExplanations.cpp
    Capabilities/Geometry2d/TestSegment.cpp
    Capabilities/Geometry2d/TestShapesPathOps.cpp
    Capabilities/Geometry2d/TestShapesPathOpsPolygon.cpp
    Capabilities/Geometry2d/TestSpatialIndexOracle.cpp
    Capabilities/Geometry2d/TestTopologyIndexing.cpp
    Capabilities/Geometry2d/TestTransformSampling.cpp
    Capabilities/Geometry3d/TestBodyBoolean3d.cpp
    Capabilities/Geometry3d/TestBodyBooleanPolyhedron3d.cpp
    Capabilities/Geometry3d/TestBodyBooleanRotated3d.cpp
    Capabilities/Geometry3d/TestBodyBooleanBrep3d.cpp
    Capabilities/Geometry3d/TestBrep3d.cpp
    Capabilities/Geometry3d/TestConversionBasic3d.cpp
    Capabilities/Geometry3d/TestConversion3d.cpp
    Capabilities/Geometry3d/TestConversionTinyScale3d.cpp
    Capabilities/Geometry3d/TestConversionCuboid3d.cpp
    Capabilities/Geometry3d/TestConversionRepresentative3d.cpp
    Capabilities/Geometry3d/TestConversionClosedRepresentative3d.cpp
    Capabilities/Geometry3d/TestHealing3d.cpp
    Capabilities/Geometry3d/TestHealing3dBoundaryCaps.cpp
    Capabilities/Geometry3d/TestHealing3dBoundaryArbitration.cpp
    Capabilities/Geometry3d/TestHealing3dPartialOverlapShells.cpp
    Capabilities/Geometry3d/TestHealing3dVertexTouchArbitration.cpp
    Capabilities/Geometry3d/TestHealing3dShellCombinations.cpp
    Capabilities/Geometry3d/TestHealing3dMixedShellCombinations.cpp
    Capabilities/Geometry3d/TestHealing3dMultiFaceShellCombinations.cpp
    Capabilities/Geometry3d/TestHealing3dSupportMismatch.cpp
    Capabilities/Geometry3d/TestHealing3dSupportMismatchBackfill.cpp
    Capabilities/Geometry3d/TestHealing3dTrimPlanes.cpp
    Capabilities/Geometry3d/TestSection3d.cpp
    Capabilities/Geometry3d/TestSection3dCoplanarMerges.cpp
    Capabilities/Geometry3d/TestSection3dMixedOpenContours.cpp
    Capabilities/Geometry3d/TestSection3dMixedAreas.cpp
    Capabilities/Geometry3d/TestSection3dMixedStrip.cpp
    Capabilities/Geometry3d/TestSection3dMixedContours.cpp
    Capabilities/Geometry3d/TestSection3dMixedContourHoles.cpp
    Capabilities/Geometry3d/TestSection3dMixedContourOrdering.cpp
    Capabilities/Geometry3d/TestSection3dRebar.cpp
    Capabilities/Geometry3d/TestSection3dRebarMidplane.cpp
    Capabilities/Geometry3d/TestSection3dRebarAxisCuts.cpp
    Capabilities/Geometry3d/TestValueTypes3d.cpp
    Capabilities/Geometry3d/TestBrepCore.cpp
    Capabilities/Geometry3d/TestBrepFaceRegion.cpp
    Capabilities/Geometry3d/TestBrepLoopGeometry.cpp
    Capabilities/Geometry3d/TestBrepMeshConversion3d.cpp
    Capabilities/Geometry3d/TestCorePrimitives3d.cpp
    Capabilities/Geometry3d/TestCurveOnSurfaceInterface.cpp
    Capabilities/Geometry3d/TestCurveOnSurfaceNumerics.cpp
    Capabilities/Geometry3d/TestCurveOnSurfaceHighDegree.cpp
    Capabilities/Geometry3d/TestGeometryEvidence.cpp
    Capabilities/Geometry3d/TestGeometryEvidenceIntersection.cpp
    Capabilities/Geometry3d/TestGeometryEvidenceTopology.cpp
    Capabilities/Geometry3d/TestMeshHealingContract.cpp
    Capabilities/Geometry3d/TestNurbsNumerics.cpp
    Capabilities/Geometry3d/TestNurbsEvaluationNumerics.cpp
    Capabilities/Geometry3d/TestNurbsWeightExtremes.cpp
    Capabilities/Geometry3d/TestNurbsProjectionIntersection.cpp
    Capabilities/Geometry3d/TestNurbsIntersectionIsolation.cpp
    Capabilities/Geometry3d/TestNurbsSurfaceSearch.cpp
    Capabilities/Geometry3d/TestNurbsSurfaceBudgets.cpp
    Capabilities/Geometry3d/TestOffsetSurfaceEval.cpp
    Capabilities/Geometry3d/TestSectionRebuildCore.cpp
    Capabilities/Geometry3d/TestTriangleMeshCore.cpp
)
list(TRANSFORM SCGEOMETRY_CAPABILITY_TEST_SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")

set(SCGEOMETRY_GAP_TEST_SOURCES
    Gaps/Core/TestMissingAlgorithmsGaps.cpp
    Gaps/Geometry2d/TestSearchPolyGaps.cpp
    Gaps/Geometry3d/TestBodyBoolean3dGaps.cpp
    Gaps/Geometry3d/TestBrep3dGaps.cpp
    Gaps/Geometry3d/TestConversion3dGaps.cpp
    Gaps/Geometry3d/TestHealing3dGaps.cpp
    Gaps/Geometry3d/TestSection3dGaps.cpp
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

# 聚合头和 NURBS 公开头分别独立编译，检查传递包含依赖。
foreach(SCGEOMETRY_STANDALONE_HEADER IN ITEMS
    Geometry.h
    Core/GeometryApi.h
    Core/Algorithms.h
    Core/IntersectionResults3d.h
    Core/GeometryTypes.h
    Core/PolygonRelation2d.h
    Core/BodyBoolean.h
    Core/Healing.h
    Core/MeshConversion.h
    Core/MeshRepair.h
    Brep/Topology.h
    Brep/BodyBoolean.h
    Brep/Healing.h
    Brep/MeshConversion.h
    Brep/MeshRepair.h
    Geometry3d/SCCurveOnSurface.h
    Geometry3d/SCNurbsCurve3d.h
    Geometry3d/SCNurbsSurface.h
    Geometry3d/SCOffsetSurface.h)
    string(REPLACE "/" "_" SCGEOMETRY_STANDALONE_TARGET "${SCGEOMETRY_STANDALONE_HEADER}")
    string(REPLACE "." "_" SCGEOMETRY_STANDALONE_TARGET "${SCGEOMETRY_STANDALONE_TARGET}")
    add_library(SCGeometryStandalone${SCGEOMETRY_STANDALONE_TARGET}Compile OBJECT
        ${CMAKE_CURRENT_SOURCE_DIR}/Support/StandalonePublicHeaderCompile.cpp)
    target_compile_definitions(SCGeometryStandalone${SCGEOMETRY_STANDALONE_TARGET}Compile PRIVATE
        "SCGEOMETRY_HEADER_UNDER_TEST=\"${SCGEOMETRY_STANDALONE_HEADER}\"")
    target_link_libraries(SCGeometryStandalone${SCGEOMETRY_STANDALONE_TARGET}Compile PRIVATE ${SCGEOMETRY_TARGET})
    SCGeometryConfigureTarget(SCGeometryStandalone${SCGEOMETRY_STANDALONE_TARGET}Compile)
endforeach()

add_executable(${SCGEOMETRY_CAPABILITY_TESTS_TARGET}
    ${SCGEOMETRY_CAPABILITY_TEST_SOURCES}
    ${SCGEOMETRY_ROOT_DIR}/Source/Core/Detail/TriangulateContours2d.cpp
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
