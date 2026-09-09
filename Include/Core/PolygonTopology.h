#pragma once

#include <cstdint>
#include <vector>

#include "Core/Triangulation.h"
#include "Export/GeometryExport.h"
#include "Geometry2d/SCPolygon2d.h"

namespace Geometry
{
    enum class SCPolygonTopologyFailure
    {
        None,
        InvalidValidationTolerance,
        InvalidDeterministicTolerance,
        InvalidOuterRing,
        InvalidHoleRing,
        UnsupportedSegmentType,
        UnsupportedSegmentOperation,
        DegenerateRing,
        SelfIntersection,
        BoundaryTouching,
        BoundaryOverlap,
        HoleOutsideOuterRing,
        HoleIntersection,
        HoleContainment,
        NonFiniteResult,
        TessellationFailure,
        TriangulationFailure,
        IndexOverflow,
        InvalidTriangulation,
        AmbiguousTopology
    };

    struct GEOMETRY_API SCPolygonNormalizeResult
    {
        bool success{false};
        SCPolygon2d polygon{};
        SCPolygonTopologyFailure failure{SCPolygonTopologyFailure::None};
        std::uint32_t loopIndex{0};
        std::uint32_t segmentIndex{0};
    };

    struct GEOMETRY_API SCTessellatedBoundaryVertex2d
    {
        std::uint64_t stableVertexId{0};
        std::uint32_t loopIndex{0};
        std::uint32_t sourceSegmentIndex{0};
        double sourceParameter{0.0};
        SCPoint2d point{};
    };

    struct GEOMETRY_API SCTessellatedPolygon2d
    {
        std::uint32_t algorithmVersion{1};
        SCPolygon2d normalizedPolygon{};
        double deterministicTolerance{0.0};
        std::vector<SCTessellatedBoundaryVertex2d> vertices{};
        std::vector<std::uint32_t> loopStartIndices{};
        std::vector<SCTriangleIndex3> triangles{};
    };

    struct GEOMETRY_API SCTessellatedPolygonResult
    {
        bool success{false};
        SCTessellatedPolygon2d polygon{};
        SCPolygonTopologyFailure failure{SCPolygonTopologyFailure::None};
        std::uint32_t loopIndex{0};
        std::uint32_t segmentIndex{0};
    };

    enum class SCPolygonContainment
    {
        Unknown,
        StrictInside,
        InsideHole,
        Touching,
        Intersecting,
        Disjoint
    };

    struct GEOMETRY_API SCPolygonContainmentResult
    {
        bool success{false};
        SCPolygonContainment containment{SCPolygonContainment::Unknown};
        SCPolygonTopologyFailure failure{SCPolygonTopologyFailure::None};
    };

    struct GEOMETRY_API SCPolygonAppendHoleResult
    {
        bool success{false};
        SCPolygon2d polygon{};
        SCPolygonTopologyFailure failure{SCPolygonTopologyFailure::None};
    };

    [[nodiscard]] GEOMETRY_API SCPolygonNormalizeResult NormalizePolygon(
        const SCPolygon2d& polygon, double validationTolerance);

    [[nodiscard]] GEOMETRY_API SCTessellatedPolygonResult TessellateAndTriangulatePolygon(
        const SCPolygon2d& polygon, double validationTolerance, double deterministicTolerance);

    [[nodiscard]] GEOMETRY_API SCPolygonContainmentResult ClassifyContainment(
        const SCPolygon2d& container, const SCPolygon2d& candidate, double validationTolerance);

    [[nodiscard]] GEOMETRY_API SCPolygonAppendHoleResult AppendHole(
        const SCPolygon2d& polygon, const SCPolyline2d& hole, double validationTolerance);

    // 多边形正面积交集查询的专用失败原因，仅服务于本查询，不修改既有
    // SCPolygonTopologyFailure 及 NormalizePolygon、ClassifyContainment 等返回契约。
    enum class SCPolygonPositiveAreaIntersectionFailure2d
    {
        None,
        InvalidInput,
        NormalizationFailure,
        ArrangementFailure,
        FaceClassificationFailure,
        NumericalIndeterminate,
        NonFiniteResult
    };

    struct GEOMETRY_API SCPolygonPositiveAreaIntersectionResult2d
    {
        bool success{false};
        bool hasPositiveAreaIntersection{false};
        SCPolygonPositiveAreaIntersectionFailure2d failure{
            SCPolygonPositiveAreaIntersectionFailure2d::InvalidInput};
    };

    // 查询两个填充集合是否存在可可靠判定的正面积公共区域。eps 仅用于输入规范化和
    // 普通浮点稳健控制，不是面积阈值。成功且 hasPositiveAreaIntersection == false 只能表示
    // 确认不相交或仅边界接触/重合；可能丢失正面积事实时返回 NumericalIndeterminate。
    [[nodiscard]] GEOMETRY_API SCPolygonPositiveAreaIntersectionResult2d
    QueryPolygonPositiveAreaIntersection(const SCPolygon2d& first,
                                         const SCPolygon2d& second,
                                         double eps = Geometry::kDefaultEpsilon);
}  // namespace Geometry
