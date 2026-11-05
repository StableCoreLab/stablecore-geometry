#pragma once

#include <array>

#include "Export/GeometryExport.h"

namespace Geometry
{
    constexpr double kDefaultEpsilon = 1e-9;
    constexpr double kDefaultAngleEpsilon = kDefaultEpsilon;
    constexpr double kDefaultParameterEpsilon = kDefaultEpsilon;
    extern GEOMETRY_DATA_API const double kPi;
    extern GEOMETRY_DATA_API const double kTwoPi;
    extern GEOMETRY_DATA_API const std::array<double, 4> kArcBoundsCriticalAngles;

    // Shared topology matching for cross-face representative grouping.
    extern GEOMETRY_DATA_API const double kSharedTopologyMatchEpsilon;

    // Loop cleanup removes consecutive duplicate vertices and trims closed
    // loops.
    extern GEOMETRY_DATA_API const double kLoopCleanupEpsilon;

    // Representative-id aggregation uses a looser match than exact topology
    // comparison so near-equal inputs collapse to the same shared target.
    extern GEOMETRY_DATA_API const double kRepresentativeMatchEpsilon;

    // Explicit call-site name for the representative grouping pass.
    extern GEOMETRY_DATA_API const double kRepresentativeGroupingEpsilon;

    // BRep vertex deduplication uses representative-style matching when merging
    // converted vertices onto shared output topology.
    extern GEOMETRY_DATA_API const double kBrepVertexDedupEpsilon;

    // 以下模块级默认别名仅为源码和二进制兼容保留；新代码按容差量纲使用
    // kDefaultEpsilon、调用者传入的容差或 SCGeometryTolerance3d。
    [[deprecated("Use a dimension-appropriate tolerance")]] extern GEOMETRY_DATA_API const double kPlaneProjectionEpsilon;
    [[deprecated("Use a dimension-appropriate tolerance")]] extern GEOMETRY_DATA_API const double kBodyBooleanDefaultEpsilon;
    [[deprecated("Use a dimension-appropriate tolerance")]] extern GEOMETRY_DATA_API const double kIntersectionDefaultEpsilon;
    [[deprecated("Use a dimension-appropriate tolerance")]] extern GEOMETRY_DATA_API const double kProjectionDefaultEpsilon;
    [[deprecated("Use a dimension-appropriate tolerance")]] extern GEOMETRY_DATA_API const double kPathOpsDefaultEpsilon;
    [[deprecated("Use a dimension-appropriate tolerance")]] extern GEOMETRY_DATA_API const double kOffsetDefaultEpsilon;
    [[deprecated("Use a dimension-appropriate tolerance")]] extern GEOMETRY_DATA_API const double kAxisOpsDefaultEpsilon;
    [[deprecated("Use a dimension-appropriate tolerance")]] extern GEOMETRY_DATA_API const double kAlgorithmsDefaultEpsilon;
    [[deprecated("Use a dimension-appropriate tolerance")]] extern GEOMETRY_DATA_API const double kArcSegmentDefaultEpsilon;
    [[deprecated("Use a dimension-appropriate tolerance")]] extern GEOMETRY_DATA_API const double kHealingDefaultEpsilon;
    [[deprecated("Use a dimension-appropriate tolerance")]] extern GEOMETRY_DATA_API const double kMeshConversionDefaultEpsilon;
    [[deprecated("Use a dimension-appropriate tolerance")]] extern GEOMETRY_DATA_API const double kShapeOpsDefaultEpsilon;
    [[deprecated("Use a dimension-appropriate tolerance")]] extern GEOMETRY_DATA_API const double kTransformDefaultEpsilon;
    [[deprecated("Use a dimension-appropriate tolerance")]] extern GEOMETRY_DATA_API const double kSectionDefaultEpsilon;
    [[deprecated("Use a dimension-appropriate tolerance")]] extern GEOMETRY_DATA_API const double kSearchPolyDefaultEpsilon;

    // 兼容旧的模块专用常量名；内部二维算法的固定下限统一定义于
    // Source/Detail/TolerancePolicy2d.h，并保持这些导出符号的既有值。
    extern GEOMETRY_DATA_API const double kBooleanComparisonEpsilon;
    extern GEOMETRY_DATA_API const double kBooleanAreaEpsilon;
    extern GEOMETRY_DATA_API const double kBooleanRebuildFallbackEpsilon;

    // 旧 PathOps 导出常量保留兼容；当前内部路径不再依赖这组固定下限。
    extern GEOMETRY_DATA_API const double kPathOpsComparisonEpsilon;
    extern GEOMETRY_DATA_API const double kPathOpsAreaEpsilon;
    extern GEOMETRY_DATA_API const double kPathOpsRebuildFallbackEpsilon;

    // 旧 SearchPoly 比较常量保留兼容；当前候选排序使用严格次序，
    // 评分近等解释按调用者容差与候选尺度判断。
    extern GEOMETRY_DATA_API const double kSearchPolyComparisonEpsilon;

    // 旧 Offset 常量保留兼容；默认入口仍有扩大重建重试，显式 epsilon
    // 入口关闭该重试。内部使用 TolerancePolicy2d.h 的共同定义。
    extern GEOMETRY_DATA_API const double kOffsetRebuildFallbackEpsilon;

    // Hole distances are weighted slightly higher during support-plane scoring
    // so hole-dominated faces prefer the plane implied by their interior loop.
    extern GEOMETRY_DATA_API const double kSupportPlaneHoleDistanceWeight;

    // Support-plane scoring treats distances below this threshold as
    // effectively on-plane when comparing candidate refit planes.
    extern GEOMETRY_DATA_API const double kSupportPlaneOnPlaneEpsilon;
}  // namespace Geometry
