#include "Geometry2d/SCPolylineCurve2d.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include "Geometry2d/SCArcSegment2d.h"
namespace Geometry
{
    bool SCPolylineCurve2d::IsValid(double epsilon) const
    {
        if (!path_.IsValid(epsilon) || path_.SegmentCount() == 0) return false;
        for (std::size_t i = 0; i < path_.SegmentCount(); ++i)
        {
            const auto segment = path_.SegmentAt(i);
            if (!segment || (segment->Kind() != SCSegmentKind2d::Line &&
                dynamic_cast<const SCArcSegment2d*>(segment.get()) == nullptr)) return false;
        }
        return true;
    }
    SCIntervald SCPolylineCurve2d::ParameterRange() const
    {
        return {0.0, static_cast<double>(path_.SegmentCount())};
    }
    SCIntervald SCPolylineCurve2d::PieceRange(std::size_t index) const
    {
        if (index >= PieceCount()) throw std::out_of_range("SCPolylineCurve2d::PieceRange");
        return {static_cast<double>(index), static_cast<double>(index + 1)};
    }
    bool SCPolylineCurve2d::IsLinearPiece(std::size_t index) const
    {
        if (index >= PieceCount()) return false;
        const auto segment = path_.SegmentAt(index);
        return segment && segment->Kind() == SCSegmentKind2d::Line;
    }
    SCCurveEval2d SCPolylineCurve2d::Evaluate(double parameter, int derivativeOrder) const
    {
        SCCurveEval2d result{};
        result.point.x = std::numeric_limits<double>::quiet_NaN();
        const double eps = std::numeric_limits<double>::denorm_min();
        const std::size_t count = path_.SegmentCount();
        if ((!IsValid(eps) && !IsValid()) || !std::isfinite(parameter) || parameter < 0.0 || parameter > static_cast<double>(count))
            return result;
        const std::size_t index = parameter == static_cast<double>(count) ? count - 1 : static_cast<std::size_t>(parameter);
        const double local = parameter - static_cast<double>(index);
        const auto segment = path_.SegmentAt(index);
        const int order = std::clamp(derivativeOrder, 0, 1);
        if (segment->Kind() == SCSegmentKind2d::Line)
        {
            const auto first = path_.VertexAt(index);
            const auto second = path_.VertexAt((index + 1) % path_.PointCount());
            result.point = {std::lerp(first.x, second.x, local), std::lerp(first.y, second.y, local)};
            if (order >= 1) result.firstDerivative = second - first;
        }
        else if (const auto* arc = dynamic_cast<const SCArcSegment2d*>(segment.get()))
        {
            const double angle = arc->startAngle + arc->sweepAngle * local;
            const double sine = std::sin(angle), cosine = std::cos(angle);
            result.point = {arc->center.x + arc->radius * cosine, arc->center.y + arc->radius * sine};
            if (order >= 1) result.firstDerivative = {-arc->radius * arc->sweepAngle * sine,
                                                     arc->radius * arc->sweepAngle * cosine};
        }
        else return result;
        if (result.point.IsValid() && (order == 0 || result.firstDerivative.IsValid())) result.derivativeOrder = order;
        return result;
    }
    SCBox2d SCPolylineCurve2d::Bounds() const
    {
        const auto exactBounds = path_.Bounds(std::numeric_limits<double>::denorm_min());
        return exactBounds.IsValid() ? exactBounds : path_.Bounds();
    }
    std::unique_ptr<ISCCurve2d> SCPolylineCurve2d::Clone() const
    {
        return std::make_unique<SCPolylineCurve2d>(path_);
    }
}
