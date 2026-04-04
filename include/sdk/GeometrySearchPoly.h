#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "export/GeometryExport.h"
#include "sdk/GeometryTypes.h"
#include "sdk/MultiPolygon2d.h"
#include "sdk/MultiPolyline2d.h"

namespace geometry::sdk
{
enum class SearchPolyIssue2d
{
    None,
    InvalidInput,
    NoClosedPolygonFound
};

struct GEOMETRY_API SearchPolyDiagnostics2d
{
    std::size_t inputPolylineCount{0};
    std::size_t inputSegmentCount{0};
    std::size_t uniqueVertexCount{0};
    std::size_t danglingEndpointCount{0};
    std::size_t branchVertexCount{0};
    std::size_t inferredSyntheticEdgeCount{0};

    [[nodiscard]] bool RequiresRepair() const
    {
        return danglingEndpointCount > 0 || branchVertexCount > 0;
    }
};

struct GEOMETRY_API SearchPolyOptions2d
{
    double epsilon{1e-9};
    bool autoClose{true};
    bool autoExtend{true};
    bool allowFakeEdges{true};
    bool removeBranches{true};
};

struct GEOMETRY_API SearchPolyCandidate2d
{
    Polygon2d polygon{};
    double absoluteArea{0.0};
    double branchScore{0.0};
    double inferredSyntheticPerimeter{0.0};
    std::size_t holeCount{0};
    std::size_t inferredSyntheticEdgeCount{0};
    std::size_t branchVertexCount{0};
    std::size_t syntheticBranchVertexCount{0};
    std::size_t rank{0};

    [[nodiscard]] bool IsValid() const
    {
        return polygon.IsValid() && absoluteArea >= 0.0 && inferredSyntheticPerimeter >= 0.0;
    }
};

struct GEOMETRY_API SearchPolyResult2d
{
    SearchPolyIssue2d issue{SearchPolyIssue2d::None};
    MultiPolygon2d polygons{};
    std::vector<SearchPolyCandidate2d> candidates{};
    SearchPolyDiagnostics2d diagnostics{};
    bool usedAutoClose{false};
    bool usedAutoExtend{false};
    bool usedSyntheticEdges{false};
    bool usedBranchScoring{false};

    [[nodiscard]] bool IsSuccess() const
    {
        return issue == SearchPolyIssue2d::None && polygons.Count() > 0;
    }
};

[[nodiscard]] GEOMETRY_API SearchPolyResult2d SearchPolygons(
    const MultiPolyline2d& lines,
    SearchPolyOptions2d options = {});

[[nodiscard]] GEOMETRY_API std::optional<SearchPolyCandidate2d> SearchPolygonContainingPoint(
    const MultiPolyline2d& lines,
    const Point2d& point,
    SearchPolyOptions2d options = {});
} // namespace geometry::sdk
