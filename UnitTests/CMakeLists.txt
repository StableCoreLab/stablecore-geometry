include(GoogleTest)

set(SCGEOMETRY_CAPABILITY_TEST_SOURCES
    Capabilities/Core/TestCoreAlgorithms.cpp
    Capabilities/Core/TestArcConstruction2d.cpp
    Capabilities/Core/TestIntersection2d.cpp
    Capabilities/Core/TestProjection2d.cpp
    Capabilities/Core/TestAxisOps2d.cpp
    Capabilities/Core/TestBoxTreeBalance.cpp
    Capabilities/Core/TestCoreMultigeometry.cpp
    Capabilities/Core/TestDllExport.cpp
    Capabilities/Core/TestFailureSentinels.cpp
    Capabilities/Core/TestIndexHitContracts.cpp
    Capabilities/Core/TestGeometryTestSupport.cpp
    Capabilities/Core/TestMetricsPolylineBox.cpp
    Capabilities/Core/TestAlgorithmCapabilities.cpp
    Capabilities/Core/TestAlgorithmCapabilityIndexes.cpp
    Capabilities/Core/TestAlgorithmCapabilityIntersections.cpp
    Capabilities/Core/TestAlgorithmCapabilityQueries.cpp
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
    Capabilities/Geometry2d/TestOffsetTolerance.cpp
    Capabilities/Geometry2d/TestPointVector.cpp
    Capabilities/Geometry2d/TestPredicate2d.cpp
    Capabilities/Geometry2d/TestPolygon.cpp
    Capabilities/Geometry2d/TestPolyline.cpp
    Capabilities/Geometry2d/TestPolylineCurve2d.cpp
    Capabilities/Geometry2d/TestRelationBoolean.cpp
    Capabilities/Geometry2d/TestRelationBooleanEdgeCases.cpp
    Capabilities/Geometry2d/TestRelationBooleanDegeneracies.cpp
    Capabilities/Geometry2d/TestRingTraversalRegression.cpp
    Capabilities/Geometry2d/TestSearchPoly.cpp
    Capabilities/Geometry2d/TestSearchPolyRepairRanking.cpp
    Capabilities/Geometry2d/TestSearchPolyCandidateRanking.cpp
    Capabilities/Geometry2d/TestSearchPolyExplanations.cpp
    Capabilities/Geometry2d/TestSegmentNumerics.cpp
    Capabilities/Geometry2d/TestSegment.cpp
    Capabilities/Geometry2d/TestSegmentIntersectionAll.cpp
    Capabilities/Geometry2d/TestSegmentSubdivision2d.cpp
    Capabilities/Geometry2d/TestCoincidentArcArrangement.cpp
    Capabilities/Geometry2d/TestShapesPathOps.cpp
    Capabilities/Geometry2d/TestShapesPathOpsPolygon.cpp
    Capabilities/Geometry2d/TestSpatialIndexOracle.cpp
    Capabilities/Geometry2d/TestSpatialIndexMutations.cpp
    Capabilities/Geometry2d/TestSpatialIndexPerformance.cpp
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
    Capabilities/Geometry3d/TestHealingBoundaryCaps3d.cpp
    Capabilities/Geometry3d/TestHealingBoundaryArbitration3d.cpp
    Capabilities/Geometry3d/TestHealingPartialOverlapShells3d.cpp
    Capabilities/Geometry3d/TestHealingVertexTouchArbitration3d.cpp
    Capabilities/Geometry3d/TestHealingVertexTouchCombinations3d.cpp
    Capabilities/Geometry3d/TestHealingFourShellArbitration3d.cpp
    Capabilities/Geometry3d/TestHealingShellCombinations3d.cpp
    Capabilities/Geometry3d/TestHealingMixedShellCombinations3d.cpp
    Capabilities/Geometry3d/TestHealingMultiFaceShellCombinations3d.cpp
    Capabilities/Geometry3d/TestHealingSupportMismatch3d.cpp
    Capabilities/Geometry3d/TestHealingSupportMismatchBackfill3d.cpp
    Capabilities/Geometry3d/TestHealingTrimPlanes3d.cpp
    Capabilities/Geometry3d/TestSection3d.cpp
    Capabilities/Geometry3d/TestSectionCoplanarMerges3d.cpp
    Capabilities/Geometry3d/TestSectionMixedOpenContours3d.cpp
    Capabilities/Geometry3d/TestSectionMixedAreas3d.cpp
    Capabilities/Geometry3d/TestSectionMixedStrip3d.cpp
    Capabilities/Geometry3d/TestSectionMixedContours3d.cpp
    Capabilities/Geometry3d/TestSectionMixedContourHoles3d.cpp
    Capabilities/Geometry3d/TestSectionMixedContourOrdering3d.cpp
    Capabilities/Geometry3d/TestSectionRebar3d.cpp
    Capabilities/Geometry3d/TestSectionRebarMidplane3d.cpp
    Capabilities/Geometry3d/TestSectionRebarAxisCuts3d.cpp
    Capabilities/Geometry3d/TestValueTypes3d.cpp
    Capabilities/Geometry3d/TestResultTolerance3d.cpp
    Capabilities/Geometry3d/TestBrepTopologyValidity3d.cpp
    Capabilities/Geometry3d/TestPolyhedronCore3d.cpp
    Capabilities/Geometry3d/TestBrepCore3d.cpp
    Capabilities/Geometry3d/TestPolyhedronMeasure3d.cpp
    Capabilities/Geometry3d/TestBrepFaceRegion3d.cpp
    Capabilities/Geometry3d/TestBrepLoopGeometry3d.cpp
    Capabilities/Geometry3d/TestBrepMeshConversion3d.cpp
    Capabilities/Geometry3d/TestCorePrimitives3d.cpp
    Capabilities/Geometry3d/TestProjection3d.cpp
    Capabilities/Geometry3d/TestSegmentSearch3d.cpp
    Capabilities/Geometry3d/TestGenericUvCurve3d.cpp
    Capabilities/Geometry3d/TestCurveOnSurfaceInterface3d.cpp
    Capabilities/Geometry3d/TestCurveOnSurfaceNumerics3d.cpp
    Capabilities/Geometry3d/TestCurveOnSurfaceHighDegree3d.cpp
    Capabilities/Geometry3d/TestCurveOnSurfaceRestrictionEvidence3d.cpp
    Capabilities/Geometry3d/TestCurveOnSurfaceSeparation3d.cpp
    Capabilities/Core/TestStableNumerics.cpp
    Capabilities/Geometry3d/TestGeometryEvidence3d.cpp
    Capabilities/Geometry3d/TestGeometryEvidenceIntersection3d.cpp
    Capabilities/Geometry3d/TestGeometryEvidenceTopology3d.cpp
    Capabilities/Geometry3d/TestMeshHealingContract3d.cpp
    Capabilities/Geometry3d/TestNurbsNumerics3d.cpp
    Capabilities/Geometry3d/TestNurbsEvaluationNumerics3d.cpp
    Capabilities/Geometry3d/TestNurbsWeightExtremes3d.cpp
    Capabilities/Geometry3d/TestNurbsProjectionIntersection3d.cpp
    Capabilities/Geometry3d/TestNurbsRootIsolationLimits3d.cpp
    Capabilities/Geometry3d/TestNurbsIntersectionIsolation3d.cpp
    Capabilities/Geometry3d/TestNurbsSurfaceSearch3d.cpp
    Capabilities/Geometry3d/TestNurbsSurfaceBudgets3d.cpp
    Capabilities/Geometry3d/TestOffsetSurfaceEval3d.cpp
    Capabilities/Geometry3d/TestSectionRebuildCore3d.cpp
    Capabilities/Geometry3d/TestTriangleMeshCore3d.cpp
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
    Gaps/Geometry3d/TestMissingAlgorithms3dGaps.cpp
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

# 复用库的公开头清单，逐头独立编译，避免测试清单遗漏新增安装接口。
foreach(SCGEOMETRY_STANDALONE_HEADER_FILE IN LISTS SCGEOMETRY_HEADERS)
    file(RELATIVE_PATH SCGEOMETRY_STANDALONE_HEADER
        "${SCGEOMETRY_ROOT_DIR}/Include" "${SCGEOMETRY_STANDALONE_HEADER_FILE}")
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
    # 私有 arrangement 回归直接编译同一份实现，不增加 DLL 公共导出。
    ${SCGEOMETRY_ROOT_DIR}/Source/Core/Detail/CurveArrangement2d.cpp
    ${SCGEOMETRY_ROOT_DIR}/Source/Core/Detail/PositiveAreaArrangement2d.cpp
    ${SCGEOMETRY_ROOT_DIR}/Source/Core/Detail/PolygonPositiveAreaIntersection2d.cpp
    ${SCGEOMETRY_ROOT_DIR}/Source/Core/Detail/SegmentKernel2d.cpp
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

# 校验低层源码的完整传递包含链，兼容转发头只豁免自身作为检查根的情形。
add_test(NAME SCGeometry.ModuleDependencies
    COMMAND ${CMAKE_COMMAND}
        "-DSCGEOMETRY_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/Integration/VerifyModuleDependencies.cmake")
