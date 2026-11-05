#include "Core/ArcConstruction2d.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <numeric>

#include "Core/Projection.h"
#include "Types/Geometry2d/SCVector2.h"

namespace Geometry
{
    namespace
    {
        [[nodiscard]] bool WithinDistance(const SCPoint2d& actual, const SCPoint2d& expected, double eps)
        {
            const double distance = (actual - expected).Length();
            return std::isfinite(distance) && distance <= eps;
        }

        // 同时核验正式圆弧契约与输入约束，避免圆心舍入后仍报告成功。
        [[nodiscard]] std::optional<SCArcConstructionResult2d> CheckedArc(
            const SCPoint2d& center, double radius, double startAngle, double sweep,
            bool radiusClamped, const SCPoint2d& start, const SCPoint2d& end, double eps,
            const SCPoint2d* through = nullptr)
        {
            SCArcSegment2d arc(center, radius, startAngle, sweep);
            if (!arc.IsValid() || !WithinDistance(arc.StartPoint(), start, eps) ||
                !WithinDistance(arc.EndPoint(), end, eps)) return std::nullopt;
            // 复用有限弧投影，检查 through 位于所选弧段而非仅位于支撑圆上。
            // 使用点差的 hypot 距离，不形成可能上溢/下溢的距离平方。
            if (through && !WithinDistance(ProjectPointToArcSegment(*through, arc, true).point,
                                           *through, eps)) return std::nullopt;
            return SCArcConstructionResult2d{arc, radiusClamped};
        }
    }

    std::optional<SCArcConstructionResult2d> SolveArcThroughThreePoints(
        const SCPoint2d& start, const SCPoint2d& through, const SCPoint2d& end, double eps)
    {
        if (!start.IsValid() || !through.IsValid() || !end.IsValid() ||
            !std::isfinite(eps) || eps <= 0.0) return std::nullopt;

        const SCVector2d a = through - start;
        const SCVector2d b = end - start;
        const double aLength = a.Length();
        const double bLength = b.Length();
        const double cLength = (end - through).Length();
        const double scale = std::max({aLength, bLength, cLength});
        if (!std::isfinite(scale) || std::min({aLength, bLength, cLength}) <= eps)
            return std::nullopt;

        // 平移至 start 后共同缩放，避免绝对坐标平方及尺度相关的面积容差。
        const SCVector2d u = a / scale;
        const SCVector2d v = b / scale;
        const double cross = Cross(u, v);
        // 相对于最长边的高不大于距离容差时视为共线。
        if (std::abs(cross) <= eps / scale) return std::nullopt;
        const double u2 = Dot(u, u);
        const double v2 = Dot(v, v);
        const SCVector2d offset{
            ((u2 * v.y - v2 * u.y) / (2.0 * cross)) * scale,
            ((u.x * v2 - v.x * u2) / (2.0 * cross)) * scale};
        const SCPoint2d center = start + offset;
        const double radius = offset.Length();
        if (!center.IsValid() || !std::isfinite(radius) || radius <= 0.0)
            return std::nullopt;

        const double startAngle = std::atan2(-offset.y, -offset.x);
        const SCVector2d toEnd = end - center;
        double sweep = std::atan2(toEnd.y, toEnd.x) - startAngle;
        constexpr double fullTurn = 2.0 * std::numbers::pi_v<double>;
        if (cross > 0.0 && sweep <= 0.0) sweep += fullTurn;
        if (cross < 0.0 && sweep >= 0.0) sweep -= fullTurn;
        if (std::abs(sweep) >= fullTurn) return std::nullopt;
        return CheckedArc(center, radius, startAngle, sweep, false, start, end, eps, &through);
    }

    std::optional<SCArcConstructionResult2d> SolveArcFromStartEndRadius(
        const SCPoint2d& start, const SCPoint2d& end, double radius,
        SCArcDirection2d direction, bool largeArc, double eps)
    {
        if (!start.IsValid() || !end.IsValid() || !std::isfinite(radius) || radius <= 0.0 ||
            !std::isfinite(eps) || eps <= 0.0 ||
            (direction != SCArcDirection2d::CounterClockwise && direction != SCArcDirection2d::Clockwise))
            return std::nullopt;

        const SCVector2d chord = end - start;
        const double length = chord.Length();
        if (!std::isfinite(length) || length <= eps) return std::nullopt;
        const double half = length * 0.5;
        const bool clamped = radius < half;
        const double effectiveRadius = std::max(radius, half);
        if (largeArc && effectiveRadius == half) return std::nullopt;

        // 分解平方差，既不形成 r*r，也保留接近半圆时的半径差。
        const double height = effectiveRadius *
            std::sqrt((effectiveRadius - half) / effectiveRadius) *
            std::sqrt(1.0 + half / effectiveRadius);
        const SCVector2d normal{-chord.y / length, chord.x / length};
        const double directionSign = direction == SCArcDirection2d::CounterClockwise ? 1.0 : -1.0;
        const double centerSign = largeArc ? -directionSign : directionSign;
        const SCPoint2d mid{std::midpoint(start.x, end.x), std::midpoint(start.y, end.y)};
        const SCPoint2d center = mid + normal * (centerSign * height);
        const double smallSweep = 2.0 * std::atan2(half, height);
        const double magnitude = largeArc ? 2.0 * std::numbers::pi_v<double> - smallSweep : smallSweep;
        // 不能把数值上无法区分的半圆/整圆当作所请求的大弧。
        if (largeArc && (magnitude <= std::numbers::pi_v<double> ||
                         magnitude >= 2.0 * std::numbers::pi_v<double>)) return std::nullopt;
        const SCVector2d toStart = start - center;
        return CheckedArc(center, effectiveRadius, std::atan2(toStart.y, toStart.x),
                          directionSign * magnitude, clamped, start, end, eps);
    }

    bool CreateArcThroughThreePoints2d(const SCPoint2d& start, const SCPoint2d& through,
                                       const SCPoint2d& end, SCArcSegment2d& out)
    {
        const auto result = SolveArcThroughThreePoints(start, through, end);
        if (!result) return false;
        out = result->arc;
        return true;
    }

    bool CreateArcFromStartEndRadius2d(const SCPoint2d& start, const SCPoint2d& end,
                                       double radius, SCArcDirection2d direction,
                                       bool largeArc, SCArcSegment2d& out)
    {
        const auto result = SolveArcFromStartEndRadius(start, end, radius, direction, largeArc);
        if (!result) return false;
        out = result->arc;
        return true;
    }
}  // namespace Geometry
