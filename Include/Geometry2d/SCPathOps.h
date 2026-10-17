#pragma once

#include <cstddef>
#include <optional>

#include "Export/GeometryExport.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Geometry2d/SCMultiPolygon2d.h"
#include "Geometry2d/SCMultiPolyline2d.h"
#include "Geometry2d/SCPolygon2d.h"
#include "Geometry2d/SCPolyline2d.h"
#include "Support/Epsilon.h"

namespace Geometry
{
    struct GEOMETRY_API SCPolygonBuildDiagnostics2d
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

    struct GEOMETRY_API SCPolygonBuildOptions2d
    {
        // allowFakeEdges 为总开关；关闭时 autoClose 和 autoExtend 均不生成边。
        bool autoClose{true};
        bool autoExtend{true};
        bool allowFakeEdges{true};
        bool removeBranches{true};
    };

    struct GEOMETRY_API SCPolygonBuildResult2d
    {
        SCMultiPolygon2d polygons{};
        SCPolygonBuildDiagnostics2d diagnostics{};
        bool usedAutoClose{false};
        bool usedAutoExtend{false};
        bool usedBranchCleanup{false};
    };

    struct GEOMETRY_API SCPolygonCutResult2d
    {
        bool success{false};
        SCMultiPolygon2d left{};
        SCMultiPolygon2d right{};
    };

    struct GEOMETRY_API SCPolygonCutParts2d
    {
        SCMultiPolygon2d left{};
        SCMultiPolygon2d right{};
    };

    [[nodiscard]] GEOMETRY_API SCPolyline2d SubPolyline(const SCPolyline2d& polyline, double startLength, double endLength);

    // cutter 所在的无限直线定义左右侧，方向为 startPoint 到 endPoint。
    // cutter 端点须有效且两端距离大于 eps；两侧均有有效面积时返回全部连通分量，
    // 否则返回 nullopt。切线长度由调用者的 eps 判断。
    // 有孔和无孔多边形使用相同的布尔裁剪及 eps 规则。
    [[nodiscard]] GEOMETRY_API std::optional<SCPolygonCutParts2d> CutPolygonParts(
        const SCPolygon2d& polygon,
        const SCLineSegment2d& cutter,
        double eps = Geometry::kDefaultEpsilon);
    // 兼容入口与 CutPolygonParts 共用实现；失败时 success=false，左右结果均为空。
    [[deprecated("Use CutPolygonParts() and its optional result")]]
    [[nodiscard]] GEOMETRY_API SCPolygonCutResult2d CutPolygon(const SCPolygon2d& polygon,
                                                               const SCLineSegment2d& cutter,
                                                               double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API SCMultiPolygon2d BuildMultiPolygonByLines(const SCMultiPolyline2d& polylines,
                                                                       double eps = Geometry::kDefaultEpsilon);

    // 返回输入线网诊断及实际执行的修复步骤；SearchPolygons 使用同一重建策略。
    [[nodiscard]] GEOMETRY_API SCPolygonBuildResult2d BuildMultiPolygonByLinesWithDiagnostics(
        const SCMultiPolyline2d& polylines,
        double eps = Geometry::kDefaultEpsilon,
        SCPolygonBuildOptions2d options = {});

    [[nodiscard]] GEOMETRY_API SCPolygon2d NormalizePolygonByLines(const SCPolygon2d& polygon,
                                                                 double eps = Geometry::kDefaultEpsilon);
}  // namespace Geometry
