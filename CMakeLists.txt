cmake_minimum_required(VERSION 3.16)

project(SCGeometry LANGUAGES CXX)

set(SCGEOMETRY_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

# 设置 CMP0135 政策，消除 FetchContent 下载时间戳警告
if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

include(FetchContent)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/SCUtf8.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/SCTestHelpers.cmake)

option(SCGEOMETRY_BUILD_TESTS "Build stablecore geometry tests" ON)
option(SCGEOMETRY_ENABLE_PCH "Enable precompiled headers for SCGeometry targets" OFF)
option(SCGEOMETRY_ENABLE_UNITY_BUILD "Enable unity build for SCGeometry targets" OFF)
set(SCGEOMETRY_UNITY_BUILD_BATCH_SIZE 8 CACHE STRING "Number of source files per unity build batch")

set(SCGEOMETRY_TARGET SCGeometry)
set(SCGEOMETRY_BIN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../Bin")
set(CMAKE_INSTALL_BINDIR "Bin")
set(CMAKE_INSTALL_INCLUDEDIR "Include/SCGeometry")
set(CMAKE_INSTALL_LIBDIR "Lib")

set(SCGEOMETRY_INSTALL_CMAKE_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/SCGeometry")
if (MSVC AND CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(SCGEOMETRY_INSTALL_PLATFORM "x64")
else()
    set(SCGEOMETRY_INSTALL_PLATFORM "${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(SCGEOMETRY_INSTALL_BIN_DIR "${CMAKE_INSTALL_BINDIR}/${SCGEOMETRY_INSTALL_PLATFORM}/$<CONFIG>")
set(SCGEOMETRY_INSTALL_LIB_DIR "${CMAKE_INSTALL_LIBDIR}/${SCGEOMETRY_INSTALL_PLATFORM}/$<CONFIG>")
set(SCGEOMETRY_INSTALL_PDB_DIR "${SCGEOMETRY_INSTALL_BIN_DIR}")

function(SCGeometryConfigureOutputDirs target_name)
    if (MSVC AND CMAKE_GENERATOR MATCHES "Visual Studio")
        set(SCGEOMETRY_OUTPUT_PLATFORM "${CMAKE_VS_PLATFORM_NAME}")
    else()
        set(SCGEOMETRY_OUTPUT_PLATFORM "$<PLATFORM_ID>")
    endif()

    if (MSVC)
        set_target_properties(${target_name} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY "${SCGEOMETRY_BIN_ROOT}/$<CONFIG>/${SCGEOMETRY_OUTPUT_PLATFORM}"
            LIBRARY_OUTPUT_DIRECTORY "${SCGEOMETRY_BIN_ROOT}/$<CONFIG>/${SCGEOMETRY_OUTPUT_PLATFORM}"
            ARCHIVE_OUTPUT_DIRECTORY "${SCGEOMETRY_BIN_ROOT}/$<CONFIG>/${SCGEOMETRY_OUTPUT_PLATFORM}"
            PDB_OUTPUT_DIRECTORY "${SCGEOMETRY_BIN_ROOT}/$<CONFIG>/${SCGEOMETRY_OUTPUT_PLATFORM}"
            COMPILE_PDB_OUTPUT_DIRECTORY "${SCGEOMETRY_BIN_ROOT}/$<CONFIG>/${SCGEOMETRY_OUTPUT_PLATFORM}"
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
            RUNTIME_OUTPUT_DIRECTORY "${SCGEOMETRY_BIN_ROOT}/$<CONFIG>/${SCGEOMETRY_OUTPUT_PLATFORM}"
            LIBRARY_OUTPUT_DIRECTORY "${SCGEOMETRY_BIN_ROOT}/$<CONFIG>/${SCGEOMETRY_OUTPUT_PLATFORM}"
            ARCHIVE_OUTPUT_DIRECTORY "${SCGEOMETRY_BIN_ROOT}/$<CONFIG>/${SCGEOMETRY_OUTPUT_PLATFORM}"
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

function(SCGeometryShouldInstallPublicHeader header_path should_install_var)
    file(RELATIVE_PATH SCGEOMETRY_HEADER_RELATIVE_PATH
        "${CMAKE_CURRENT_SOURCE_DIR}/Include"
        "${header_path}"
    )

    if(SCGEOMETRY_HEADER_RELATIVE_PATH MATCHES "^Types/Detail/")
        set(${should_install_var} OFF PARENT_SCOPE)
        return()
    endif()

    set(${should_install_var} ON PARENT_SCOPE)
endfunction()

function(SCGeometryInstallPublicHeader header_path)
    file(RELATIVE_PATH SCGEOMETRY_HEADER_RELATIVE_PATH
        "${CMAKE_CURRENT_SOURCE_DIR}/Include"
        "${header_path}"
    )
    get_filename_component(SCGEOMETRY_HEADER_DIR "${SCGEOMETRY_HEADER_RELATIVE_PATH}" DIRECTORY)

    if(SCGEOMETRY_HEADER_DIR STREQUAL ".")
        set(SCGEOMETRY_HEADER_DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
    else()
        set(SCGEOMETRY_HEADER_DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${SCGEOMETRY_HEADER_DIR}")
    endif()

    install(FILES "${header_path}"
        DESTINATION "${SCGEOMETRY_HEADER_DESTINATION}"
    )
endfunction()

function(SCGeometryInstallPublicHeaders)
    foreach(SCGEOMETRY_HEADER IN LISTS SCGEOMETRY_HEADERS)
        SCGeometryShouldInstallPublicHeader("${SCGEOMETRY_HEADER}" SCGEOMETRY_HEADER_SHOULD_INSTALL)
        if(NOT SCGEOMETRY_HEADER_SHOULD_INSTALL)
            continue()
        endif()

        SCGeometryInstallPublicHeader("${SCGEOMETRY_HEADER}")
    endforeach()
endfunction()

# -------------------
# SCGeometry 库
# -------------------
file(GLOB_RECURSE SCGEOMETRY_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/Include/*.h"
)

file(GLOB_RECURSE SCGEOMETRY_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/Source/*.cpp"
)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/Include"
    PREFIX "Header Files"
    FILES ${SCGEOMETRY_HEADERS}
)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/Source"
    PREFIX "Source Files"
    FILES ${SCGEOMETRY_SOURCES}
)

add_library(${SCGEOMETRY_TARGET} SHARED
    ${SCGEOMETRY_SOURCES}
    ${SCGEOMETRY_HEADERS}
)
SCGeometryConfigureTarget(${SCGEOMETRY_TARGET})
SCGeometryConfigurePerformance(${SCGEOMETRY_TARGET})

add_library(SCGeometry::Geometry ALIAS ${SCGEOMETRY_TARGET})

target_include_directories(${SCGEOMETRY_TARGET}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Include>
        $<INSTALL_INTERFACE:Include/SCGeometry>
)

target_compile_features(${SCGEOMETRY_TARGET} PUBLIC cxx_std_20)
if (MSVC)
    target_compile_options(${SCGEOMETRY_TARGET} PRIVATE /MP)
endif()

target_compile_definitions(${SCGEOMETRY_TARGET} PRIVATE GEOMETRY_BUILD_DLL)

set_target_properties(${SCGEOMETRY_TARGET} PROPERTIES
    EXPORT_NAME Geometry
)

# -------------------
# 测试配置
# -------------------
if (SCGEOMETRY_BUILD_TESTS)

    # 尝试查找系统或已安装 GTest
    find_package(GTest CONFIG QUIET)

    if (NOT GTest_FOUND)
        # 强制使用共享 CRT
        set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
        set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
        set(INSTALL_GMOCK OFF CACHE BOOL "" FORCE)

        set(SCGEOMETRY_GTEST_LOCAL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../ThirdParty/GoogleTest")
        if (EXISTS "${SCGEOMETRY_GTEST_LOCAL_PATH}")
            # 优先使用本地 ThirdParty 路径的 GTest（与其他依赖库共享的本地副本）
            FetchContent_Declare(
                googletest
                SOURCE_DIR "${SCGEOMETRY_GTEST_LOCAL_PATH}"
            )
        else()
            # 本地路径不存在时，回退到从 GitHub 下载
            FetchContent_Declare(
                googletest
                URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
                DOWNLOAD_EXTRACT_TIMESTAMP TRUE
            )
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
    LIBRARY DESTINATION ${SCGEOMETRY_INSTALL_BIN_DIR}
    ARCHIVE DESTINATION ${SCGEOMETRY_INSTALL_LIB_DIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES $<TARGET_PDB_FILE:${SCGEOMETRY_TARGET}>
    CONFIGURATIONS Debug RelWithDebInfo Release
    DESTINATION ${SCGEOMETRY_INSTALL_PDB_DIR}
)

install(CODE [[
    file(REMOVE_RECURSE
        "${CMAKE_INSTALL_PREFIX}/Include/SCGeometry/Types/Compat"
        "${CMAKE_INSTALL_PREFIX}/Include/SCGeometry/Types/Detail"
    )
]])

SCGeometryInstallPublicHeaders()

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
    VERSION 1.0.0
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/SCGeometryConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/SCGeometryConfigVersion.cmake
    DESTINATION ${SCGEOMETRY_INSTALL_CMAKE_DIR}
)
