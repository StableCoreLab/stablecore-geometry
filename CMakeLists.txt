cmake_minimum_required(VERSION 3.16)

project(SCGeometry VERSION 1.0.0 LANGUAGES CXX)

set(SCGEOMETRY_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

# 设置 CMP0135 政策，消除 FetchContent 下载时间戳警告
if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

if(WIN32)
    if(NOT DEFINED CMAKE_INSTALL_BINDIR)
        set(CMAKE_INSTALL_BINDIR Bin CACHE PATH "Executable install directory")
    endif()
    if(NOT DEFINED CMAKE_INSTALL_LIBDIR)
        set(CMAKE_INSTALL_LIBDIR Lib CACHE PATH "Library install directory")
    endif()
    if(NOT DEFINED CMAKE_INSTALL_INCLUDEDIR)
        set(CMAKE_INSTALL_INCLUDEDIR Include CACHE PATH "Header install directory")
    endif()
endif()
include(FetchContent)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/SCUtf8.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/SCTestHelpers.cmake)

if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
    set(SCGEOMETRY_BUILD_TESTS_DEFAULT ON)
else()
    set(SCGEOMETRY_BUILD_TESTS_DEFAULT OFF)
endif()
option(SCGEOMETRY_BUILD_TESTS "Build stablecore geometry tests" ${SCGEOMETRY_BUILD_TESTS_DEFAULT})
option(BUILD_SHARED_LIBS "Build SCGeometry as a shared library" ON)
option(SCGEOMETRY_ENABLE_PCH "Enable precompiled headers for SCGeometry targets" OFF)
option(SCGEOMETRY_ENABLE_UNITY_BUILD "Enable unity build for SCGeometry targets" OFF)
set(SCGEOMETRY_UNITY_BUILD_BATCH_SIZE 8 CACHE STRING "Number of source files per unity build batch")

set(SCGEOMETRY_TARGET SCGeometry)
set(SCGEOMETRY_BIN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../Bin" CACHE PATH
    "Root directory for SCGeometry build outputs")
set(SCGEOMETRY_INSTALL_INCLUDEDIR "${CMAKE_INSTALL_INCLUDEDIR}/SCGeometry")
set(SCGEOMETRY_INSTALL_CMAKE_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/SCGeometry")
if(CMAKE_GENERATOR MATCHES "Visual Studio")
    set(SCGEOMETRY_PLATFORM "${CMAKE_VS_PLATFORM_NAME}")
elseif(WIN32 AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(AMD64|[Xx]86_64|[Xx]86|i[3-6]86)$")
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(SCGEOMETRY_PLATFORM "x64")
    else()
        set(SCGEOMETRY_PLATFORM "Win32")
    endif()
else()
    set(SCGEOMETRY_PLATFORM "${CMAKE_SYSTEM_PROCESSOR}")
endif()
if(NOT SCGEOMETRY_PLATFORM)
    message(FATAL_ERROR "Cannot determine the SCGeometry platform directory")
endif()

set(SCGEOMETRY_INSTALL_BIN_DIR "${CMAKE_INSTALL_BINDIR}/${SCGEOMETRY_PLATFORM}/$<CONFIG>")
set(SCGEOMETRY_INSTALL_LIB_DIR "${CMAKE_INSTALL_LIBDIR}/${SCGEOMETRY_PLATFORM}/$<CONFIG>")
set(SCGEOMETRY_INSTALL_PDB_DIR "${SCGEOMETRY_INSTALL_BIN_DIR}")
set(SCGEOMETRY_INSTALL_DOC_DIR "${CMAKE_INSTALL_DATADIR}/SCGeometry/Docs")

function(SCGeometryConfigureOutputDirs target_name)
    if (MSVC)
        set_target_properties(${target_name} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY "${SCGEOMETRY_BIN_ROOT}/$<CONFIG>/${SCGEOMETRY_PLATFORM}"
            LIBRARY_OUTPUT_DIRECTORY "${SCGEOMETRY_BIN_ROOT}/$<CONFIG>/${SCGEOMETRY_PLATFORM}"
            ARCHIVE_OUTPUT_DIRECTORY "${SCGEOMETRY_BIN_ROOT}/$<CONFIG>/${SCGEOMETRY_PLATFORM}"
            PDB_OUTPUT_DIRECTORY "${SCGEOMETRY_BIN_ROOT}/$<CONFIG>/${SCGEOMETRY_PLATFORM}"
            COMPILE_PDB_OUTPUT_DIRECTORY "${SCGEOMETRY_BIN_ROOT}/$<CONFIG>/${SCGEOMETRY_PLATFORM}"
            PDB_NAME "${target_name}"
            COMPILE_PDB_NAME "${target_name}"
            MSVC_DEBUG_INFORMATION_FORMAT ProgramDatabase
        )
        target_compile_options(${target_name} PRIVATE
            $<$<CONFIG:Release>:/Zi>
            $<$<CONFIG:RelWithDebInfo>:/Zi>
        )
        target_link_options(${target_name} PRIVATE
            $<$<CONFIG:Release>:/DEBUG>
            $<$<CONFIG:RelWithDebInfo>:/DEBUG>
        )
    else()
        set_target_properties(${target_name} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY "${SCGEOMETRY_BIN_ROOT}/$<CONFIG>/${SCGEOMETRY_PLATFORM}"
            LIBRARY_OUTPUT_DIRECTORY "${SCGEOMETRY_BIN_ROOT}/$<CONFIG>/${SCGEOMETRY_PLATFORM}"
            ARCHIVE_OUTPUT_DIRECTORY "${SCGEOMETRY_BIN_ROOT}/$<CONFIG>/${SCGEOMETRY_PLATFORM}"
        )
    endif()
endfunction()

function(SCGeometryConfigureTarget target_name)
    SCGeometryConfigureOutputDirs(${target_name})
    SCEnableUtf8ForTargets(${target_name})
endfunction()

function(SCGeometryConfigurePerformance target_name)
    if (SCGEOMETRY_ENABLE_PCH)
        target_precompile_headers(${target_name} PRIVATE
            "$<$<COMPILE_LANGUAGE:CXX>:${SCGEOMETRY_ROOT_DIR}/Source/SCGeometryPch.h>"
        )
    endif()

    if (SCGEOMETRY_ENABLE_UNITY_BUILD)
        set_target_properties(${target_name} PROPERTIES
            UNITY_BUILD ON
            UNITY_BUILD_BATCH_SIZE ${SCGEOMETRY_UNITY_BUILD_BATCH_SIZE}
        )
    endif()
endfunction()

# -------------------
# SCGeometry 库
# -------------------
set(SCGEOMETRY_HEADERS
    Brep/BodyBoolean.h
    Brep/BrepConversion.h
    Brep/BrepEditing.h
    Brep/Healing.h
    Brep/MeshConversion.h
    Brep/MeshOps.h
    Brep/MeshRepair.h
    Brep/PolyhedronBody.h
    Brep/PolyhedronFace3d.h
    Brep/PolyhedronLoop3d.h
    Brep/SCBrepBody.h
    Brep/SCBrepCoedge.h
    Brep/SCBrepEdge.h
    Brep/SCBrepFace.h
    Brep/SCBrepLoop.h
    Brep/SCBrepShell.h
    Brep/SCBrepVertex.h
    Brep/Tessellation.h
    Brep/Topology.h
    Core/Algorithms.h
    Core/ArcOps3d.h
    Core/AxisOps.h
    Core/Boolean.h
    Core/Editing.h
    Core/Geometry2dResults.h
    Core/Geometry3dResults.h
    Core/GeometryApi.h
    Core/GeometryEvidence.h
    Core/GeometryTypes.h
    Core/IntersectionResults3d.h
    Core/GeometryTypesFwd.h
    Core/GeometryTypesPrimitives.h
    Core/Intersection.h
    Core/Measure.h
    Core/Metrics.h
    Core/Offset.h
    Core/PathGeometry.h
    Core/PlaneOps.h
    Core/PointOps.h
    Core/PolygonRelation2d.h
    Core/PolygonTopology.h
    Core/Projection.h
    Core/ProjectionResults.h
    Core/Relation.h
    Core/Results.h
    Core/Sampling.h
    Core/SearchPoly.h
    Core/Section.h
    Core/ShapeOps.h
    Core/Snap3d.h
    Core/Tangent.h
    Core/Transform.h
    Core/Triangulation.h
    Core/Validation.h
    Export/GeometryExport.h
    Geometry.h
    Geometry2d/ISCSegment2d.h
    Geometry2d/SCArcSegment2d.h
    Geometry2d/SCBoxTree2d.h
    Geometry2d/SCCircle2d.h
    Geometry2d/SCEllipse2d.h
    Geometry2d/SCKDTree2d.h
    Geometry2d/SCLine2d.h
    Geometry2d/SCLineSegment2d.h
    Geometry2d/SCMultiPolygon2d.h
    Geometry2d/SCMultiPolyline2d.h
    Geometry2d/SCPathOps.h
    Geometry2d/SCPolygon2d.h
    Geometry2d/SCPolyline2d.h
    Geometry2d/SCRectangle2d.h
    Geometry2d/SCSegmentSearch2d.h
    Geometry2d/SegmentTypes.h
    Geometry3d/Detail/SurfaceNumerics.h
    Geometry3d/ISCCurve3d.h
    Geometry3d/ISCSurface.h
    Geometry3d/SCBoxTree3d.h
    Geometry3d/SCCurveOnSurface.h
    Geometry3d/SCLineCurve3d.h
    Geometry3d/SCNurbsCurve3d.h
    Geometry3d/SCNurbsSurface.h
    Geometry3d/SCOffsetSurface.h
    Geometry3d/SCPlaneSurface.h
    Geometry3d/SCRuledSurface.h
    Geometry3d/SCSegmentSearch3d.h
    Geometry3d/TriangleMesh.h
    Serialize/GeometryText.h
    Serialize/SegmentCodec.h
    Support/Epsilon.h
    Support/Geometry2d/Normalize2.h
    Support/Geometry2d/Predicate2.h
    Support/StableNumerics.h
    Types/Geometry2d/SCBox2.h
    Types/Geometry2d/SCLine2d.h
    Types/Geometry2d/SCMatrix2d.h
    Types/Geometry2d/SCPoint2.h
    Types/Geometry2d/SCTransform2d.h
    Types/Geometry2d/SCVector2.h
    Types/Geometry3d/SCArcSegment3d.h
    Types/Geometry3d/SCBox3.h
    Types/Geometry3d/SCDirection3d.h
    Types/Geometry3d/SCGeometryTypes3d.h
    Types/Geometry3d/SCIntervald.h
    Types/Geometry3d/SCLine3d.h
    Types/Geometry3d/SCLineSegment3d.h
    Types/Geometry3d/SCMatrix3d.h
    Types/Geometry3d/SCPlane.h
    Types/Geometry3d/SCPlaneFrame3d.h
    Types/Geometry3d/SCPoint3.h
    Types/Geometry3d/SCRay3d.h
    Types/Geometry3d/SCTransform3d.h
    Types/Geometry3d/SCTriangle3d.h
    Types/Geometry3d/SCVector3.h
    Types/SCGeometryScalar.h
    Types/SCVecStorageOps.h
)
list(TRANSFORM SCGEOMETRY_HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/Include/")

set(SCGEOMETRY_INTERNAL_HEADERS
    Brep/LoopGeometry.h
    Core/RingIntegral2d.h
    Detail/AffinePlaneGeometry.h
    Detail/ArrangementVertices2d.h
    Detail/BoxTreeCore.h
    Detail/BrepFaceRegion.h
    Detail/CheckedSegmentIntersection2d.h
    Detail/CurveArrangement2d.h
    Detail/CurveOnSurfaceNumerics.h
    Detail/DirectedEdgeFans2d.h
    Detail/GeometryContainerOps.h
    Detail/LineFaceHits.h
    Detail/NurbsBasis.h
    Detail/NurbsRootLimits.h
    Detail/ParametricSeeds.h
    Detail/PathNumerics3d.h
    Detail/PolygonNesting2d.h
    Detail/PolygonPositiveAreaIntersection2d.h
    Detail/PolylineRing2d.h
    Detail/ProjectionCandidates.h
    Detail/RingVertices2d.h
    Detail/TolerancePolicy2d.h
    Detail/SegmentBoxRelation2d.h
    Detail/SegmentKernel2d.h
    Detail/SegmentParameters2d.h
    Detail/SegmentSubdivision2d.h
    Detail/SurfaceProjectionSolver.h
    Detail/TriangulateContours2d.h
    SCGeometryPch.h
)
list(TRANSFORM SCGEOMETRY_INTERNAL_HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/Source/")

set(SCGEOMETRY_SOURCES
    Brep/BodyBoolean.cpp
    Brep/BrepConversion.cpp
    Brep/BrepEditing.cpp
    Brep/Healing.cpp
    Brep/MeshConversion.cpp
    Brep/MeshOps.cpp
    Brep/MeshRepair.cpp
    Brep/Tessellation.cpp
    Brep/Topology.cpp
    Core/ArcOps3d.cpp
    Core/AxisOps.cpp
    Core/Boolean.cpp
    Core/Editing.cpp
    Core/Intersection.cpp
    Core/MappedOffset.cpp
    Core/Measure.cpp
    Core/Metrics.cpp
    Core/Offset.cpp
    Core/PathGeometry.cpp
    Core/PlaneOps.cpp
    Core/PointOps.cpp
    Core/PolygonTopology.cpp
    Core/Projection.cpp
    Core/Relation.cpp
    Core/Sampling.cpp
    Core/SearchPoly.cpp
    Core/Section.cpp
    Core/ShapeOps.cpp
    Core/Snap3d.cpp
    Core/Tangent.cpp
    Core/Transform.cpp
    Core/Triangulation.cpp
    Core/Validation.cpp
    Detail/CurveArrangement2d.cpp
    Detail/PolygonPositiveAreaIntersection2d.cpp
    Detail/PositiveAreaArrangement2d.cpp
    Detail/SegmentBoxRelation2d.cpp
    Detail/SegmentKernel2d.cpp
    Detail/TriangulateContours2d.cpp
    Geometry2d/ArcSegment2d.cpp
    Geometry2d/LineSegment2d.cpp
    Geometry2d/MultiPolygon2d.cpp
    Geometry2d/MultiPolyline2d.cpp
    Geometry2d/Polygon2d.cpp
    Geometry2d/Polyline2d.cpp
    Geometry2d/SCBoxTree2d.cpp
    Geometry2d/SCCircle2d.cpp
    Geometry2d/SCEllipse2d.cpp
    Geometry2d/SCKDTree2d.cpp
    Geometry2d/SCPathOps.cpp
    Geometry2d/SCRectangle2d.cpp
    Geometry2d/SCSegmentSearch2d.cpp
    Geometry3d/GeometryEvidence3d.cpp
    Geometry3d/Intersection3d.cpp
    Geometry3d/PlaneCurveRoots.cpp
    Geometry3d/PlaneSurface.cpp
    Geometry3d/Projection3d.cpp
    Geometry3d/Relation3d.cpp
    Geometry3d/SCBoxTree3d.cpp
    Geometry3d/SCCurveOnSurface.cpp
    Geometry3d/SCNurbsCurve3d.cpp
    Geometry3d/SCNurbsSurface.cpp
    Geometry3d/SCOffsetSurface.cpp
    Geometry3d/SCSegmentSearch3d.cpp
    Serialize/GeometryText.cpp
    Serialize/SegmentCodec.cpp
    Support/Epsilon.cpp
)
list(TRANSFORM SCGEOMETRY_SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/Source/")

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/Include"
    PREFIX "Header Files"
    FILES ${SCGEOMETRY_HEADERS}
)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/Source"
    PREFIX "Source Files"
    FILES ${SCGEOMETRY_INTERNAL_HEADERS} ${SCGEOMETRY_SOURCES}
)

add_library(${SCGEOMETRY_TARGET}
    ${SCGEOMETRY_SOURCES}
    ${SCGEOMETRY_INTERNAL_HEADERS}
    ${SCGEOMETRY_HEADERS}
)
SCGeometryConfigureTarget(${SCGEOMETRY_TARGET})
SCGeometryConfigurePerformance(${SCGEOMETRY_TARGET})

add_library(SCGeometry::Geometry ALIAS ${SCGEOMETRY_TARGET})

target_include_directories(${SCGEOMETRY_TARGET}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Include>
        $<INSTALL_INTERFACE:${SCGEOMETRY_INSTALL_INCLUDEDIR}>
)

target_compile_features(${SCGEOMETRY_TARGET} PUBLIC cxx_std_20)
if (MSVC)
    target_compile_options(${SCGEOMETRY_TARGET} PRIVATE /MP)
endif()
target_compile_options(${SCGEOMETRY_TARGET} INTERFACE
    $<$<CXX_COMPILER_ID:MSVC>:/utf-8>
)

# The library exports its own symbols while every consumer of the shared target
# imports them.  Keeping these definitions on their respective usage sides is
# essential for exported data declarations on MSVC.
if(BUILD_SHARED_LIBS)
    target_compile_definitions(${SCGEOMETRY_TARGET}
        PRIVATE GEOMETRY_BUILD_DLL
        INTERFACE GEOMETRY_USE_DLL
    )
endif()

set_target_properties(${SCGEOMETRY_TARGET} PROPERTIES
    EXPORT_NAME Geometry
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

# -------------------
# 测试配置
# -------------------
if (SCGEOMETRY_BUILD_TESTS)

    # 尝试查找系统或已安装 GTest
    find_package(GTest CONFIG QUIET)

    if (NOT GTest_FOUND)
        # 嵌入本地 GTest 时不安装它，并与本库使用相同的 CRT。
        set(gtest_force_shared_crt ON)
        set(INSTALL_GTEST OFF)

        set(SCGEOMETRY_GTEST_LOCAL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../ThirdParty/GoogleTest")
        if (EXISTS "${SCGEOMETRY_GTEST_LOCAL_PATH}")
            # 优先使用本地 ThirdParty 路径的 GTest（与其他依赖库共享的本地副本）
            FetchContent_Declare(
                googletest
                SOURCE_DIR "${SCGEOMETRY_GTEST_LOCAL_PATH}"
            )
        else()
            message(FATAL_ERROR "GTest was not found. Install GTest or provide ../ThirdParty/GoogleTest; configure with -DSCGEOMETRY_BUILD_TESTS=OFF to omit tests.")
        endif()

        FetchContent_MakeAvailable(googletest)
    endif()

    # 添加测试子目录
    enable_testing()
    add_subdirectory(UnitTests)
endif()

install(TARGETS ${SCGEOMETRY_TARGET}
    EXPORT SCGeometryTargets
    RUNTIME DESTINATION ${SCGEOMETRY_INSTALL_BIN_DIR}
    LIBRARY DESTINATION ${SCGEOMETRY_INSTALL_LIB_DIR}
    ARCHIVE DESTINATION ${SCGEOMETRY_INSTALL_LIB_DIR}
    INCLUDES DESTINATION ${SCGEOMETRY_INSTALL_INCLUDEDIR}
)

if(MSVC AND BUILD_SHARED_LIBS)
    install(FILES $<TARGET_PDB_FILE:${SCGEOMETRY_TARGET}>
        CONFIGURATIONS Debug RelWithDebInfo Release
        DESTINATION ${SCGEOMETRY_INSTALL_PDB_DIR}
        OPTIONAL
    )
endif()

foreach(SCGEOMETRY_HEADER IN LISTS SCGEOMETRY_HEADERS)
    file(RELATIVE_PATH SCGEOMETRY_HEADER_PATH
        "${CMAKE_CURRENT_SOURCE_DIR}/Include" "${SCGEOMETRY_HEADER}")
    if(SCGEOMETRY_HEADER_PATH MATCHES "^Types/(Compat|Detail)/")
        continue()
    endif()
    get_filename_component(SCGEOMETRY_HEADER_DIR "${SCGEOMETRY_HEADER_PATH}" DIRECTORY)
    if(SCGEOMETRY_HEADER_DIR STREQUAL "." OR SCGEOMETRY_HEADER_DIR STREQUAL "")
        install(FILES "${SCGEOMETRY_HEADER}" DESTINATION "${SCGEOMETRY_INSTALL_INCLUDEDIR}")
    else()
        install(FILES "${SCGEOMETRY_HEADER}"
            DESTINATION "${SCGEOMETRY_INSTALL_INCLUDEDIR}/${SCGEOMETRY_HEADER_DIR}")
    endif()
endforeach()

install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/使用/使用说明.md"
    DESTINATION ${SCGEOMETRY_INSTALL_DOC_DIR}
)

install(EXPORT SCGeometryTargets
    FILE SCGeometryTargets.cmake
    NAMESPACE SCGeometry::
    DESTINATION ${SCGEOMETRY_INSTALL_CMAKE_DIR}
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/SCGeometryConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/SCGeometryConfig.cmake
    INSTALL_DESTINATION ${SCGEOMETRY_INSTALL_CMAKE_DIR}
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/SCGeometryConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/SCGeometryConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/SCGeometryConfigVersion.cmake
    DESTINATION ${SCGEOMETRY_INSTALL_CMAKE_DIR}
)
