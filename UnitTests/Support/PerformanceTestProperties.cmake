# 固定耗时验收独占运行，避免其他测试竞争 CPU 影响门槛。
if("NurbsNumericsTest.FixedLargeCoordinateWorkloadPerformance" IN_LIST SCGeometryCapabilitiesTests_TESTS)
    set_tests_properties(NurbsNumericsTest.FixedLargeCoordinateWorkloadPerformance
        PROPERTIES RUN_SERIAL TRUE)
endif()
if("NurbsNumericsTest.BodySurfaceBudgetsRemainIndependentAfterActualTruncation" IN_LIST SCGeometryCapabilitiesTests_TESTS)
    set_tests_properties(NurbsNumericsTest.BodySurfaceBudgetsRemainIndependentAfterActualTruncation
        PROPERTIES RUN_SERIAL TRUE)
endif()
if("NurbsNumericsTest.ActualSurfaceBudgetRetainsCandidateWhileLegacyContinues" IN_LIST SCGeometryCapabilitiesTests_TESTS)
    set_tests_properties(NurbsNumericsTest.ActualSurfaceBudgetRetainsCandidateWhileLegacyContinues
        PROPERTIES RUN_SERIAL TRUE)
endif()
if("NurbsNumericsTest.MultiCellWorkloadPerformanceMatrix" IN_LIST SCGeometryCapabilitiesTests_TESTS)
    set_tests_properties(NurbsNumericsTest.MultiCellWorkloadPerformanceMatrix
        PROPERTIES RUN_SERIAL TRUE)
endif()

# 索引性能记录独占 CTest 运行，不与其他测试争用 CPU；不设置速度阈值。
foreach(index_test IN LISTS SCGeometryCapabilitiesTests_TESTS)
    if(index_test MATCHES "^SpatialIndexPerformanceTest\\.")
        set_tests_properties("${index_test}" PROPERTIES RUN_SERIAL TRUE LABELS "performance")
    endif()
endforeach()
