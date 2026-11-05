#pragma once

#include <optional>

#include "Export/GeometryExport.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Support/Epsilon.h"

namespace Geometry
{
    // 直接持有正式圆弧类型，避免重复维护圆心、半径、方向和取点算法。
    struct SCArcConstructionResult2d
    {
        SCArcSegment2d arc;
        bool radiusClamped{false};
    };

    // eps 为距离容差，必须为有限正数；同时约束输入退化判断与成弧后的点残差。
    // 重合、共线、不可表示或不能在 eps 内满足输入点约束的圆弧返回空值。
    [[nodiscard]] GEOMETRY_API std::optional<SCArcConstructionResult2d> SolveArcThroughThreePoints(
        const SCPoint2d& start, const SCPoint2d& through, const SCPoint2d& end,
        double eps = kDefaultEpsilon);

    // 保留方向和大/小弧两个独立选择；半径不足时夹到半弦长并报告 radiusClamped。
    // 半圆属于小弧，半圆请求 largeArc=true 无解。
    // 最终圆弧的起终点须分别在 eps 内匹配输入端点，否则返回空值。
    [[nodiscard]] GEOMETRY_API std::optional<SCArcConstructionResult2d> SolveArcFromStartEndRadius(
        const SCPoint2d& start, const SCPoint2d& end, double radius,
        SCArcDirection2d direction, bool largeArc, double eps = kDefaultEpsilon);

    // 便捷入口使用默认容差；失败时保持 out 不变。
    [[nodiscard]] GEOMETRY_API bool CreateArcThroughThreePoints2d(
        const SCPoint2d& start, const SCPoint2d& through, const SCPoint2d& end,
        SCArcSegment2d& out);
    [[nodiscard]] GEOMETRY_API bool CreateArcFromStartEndRadius2d(
        const SCPoint2d& start, const SCPoint2d& end, double radius,
        SCArcDirection2d direction, bool largeArc, SCArcSegment2d& out);
}  // namespace Geometry
