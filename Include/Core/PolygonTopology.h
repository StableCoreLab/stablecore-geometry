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
}  // namespace Geometry
