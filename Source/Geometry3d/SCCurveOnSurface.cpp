#include "Geometry3d/SCCurveOnSurface.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "Geometry3d/SCPlaneSurface.h"

namespace Geometry
{
    namespace
    {
        [[nodiscard]] bool HasValidLineUvCurve(const SCPolyline2d& uvCurve, double eps)
        {
            if (!uvCurve.IsValid(eps) || uvCurve.SegmentCount() == 0)
            {
                return false;
            }
            for (std::size_t index = 0; index < uvCurve.SegmentCount(); ++index)
            {
                const auto segment = uvCurve.SegmentAt(index);
                if (segment == nullptr || segment->Kind() != SCSegmentKind2d::Line)
                {
                    return false;
                }
            }
            return true;
        }

        [[nodiscard]] double PositiveParameterEpsilon(double eps)
        {
            // 折线显式入口要求正容差；用最小正值表达精确的零参数容差。
            return std::max(eps, std::numeric_limits<double>::denorm_min());
        }

        [[nodiscard]] SCGeometryTolerance3d EvaluationTolerance()
        {
            SCGeometryTolerance3d tolerance{};
            tolerance.parameterEpsilon = std::numeric_limits<double>::denorm_min();
            return tolerance;
        }
    }

    bool SCCurveOnSurface::IsValid(const SCGeometryTolerance3d& tolerance) const
    {
        if (!tolerance.IsValid() || supportSurface_ == nullptr || !supportSurface_->IsValid(tolerance))
        {
            return false;
        }
        const double uvEps = PositiveParameterEpsilon(tolerance.parameterEpsilon);
        if (!HasValidLineUvCurve(uvCurve_, uvEps))
        {
            return false;
        }

        if (dynamic_cast<const SCPlaneSurface*>(supportSurface_.get()) != nullptr)
        {
            return true;
        }

        const SCBox2d uvBounds = uvCurve_.Bounds(uvEps);
        const SCIntervald uRange = supportSurface_->URange();
        const SCIntervald vRange = supportSurface_->VRange();
        return uvBounds.IsValid() && uRange.IsValid() && vRange.IsValid() &&
               uRange.Contains(uvBounds.MinPoint().x, 0.0) &&
               uRange.Contains(uvBounds.MaxPoint().x, 0.0) &&
               vRange.Contains(uvBounds.MinPoint().y, 0.0) &&
               vRange.Contains(uvBounds.MaxPoint().y, 0.0);
    }

    SCIntervald SCCurveOnSurface::ParameterRange() const
    {
        return {0.0, static_cast<double>(uvCurve_.SegmentCount())};
    }

    bool SCCurveOnSurface::IsClosed(const SCGeometryTolerance3d& tolerance) const
    {
        return IsValid(tolerance) && PointAt(0.0).AlmostEquals(
            PointAt(ParameterRange().max), tolerance.distanceEpsilon);
    }

    bool SCCurveOnSurface::IsPeriodic(const SCGeometryTolerance3d& tolerance) const
    {
        (void)tolerance;
        return false;
    }

    SCPoint3d SCCurveOnSurface::PointAt(double parameter) const
    {
        return Evaluate(parameter, 0).point;
    }

    SCPoint2d SCCurveOnSurface::UvAt(double parameter) const
    {
        SCPoint2d invalid{};
        invalid.x = std::numeric_limits<double>::quiet_NaN();
        const std::size_t segmentCount = uvCurve_.SegmentCount();
        if (!HasValidLineUvCurve(uvCurve_, std::numeric_limits<double>::denorm_min()) ||
            !std::isfinite(parameter) || parameter < 0.0 ||
            parameter > static_cast<double>(segmentCount))
        {
            return invalid;
        }

        const std::size_t index = parameter == static_cast<double>(segmentCount)
                                      ? segmentCount - 1
                                      : static_cast<std::size_t>(parameter);
        const double local = parameter - static_cast<double>(index);
        const SCPoint2d first = uvCurve_.VertexAt(index);
        const SCPoint2d second = uvCurve_.VertexAt((index + 1) % uvCurve_.PointCount());
        return {std::lerp(first.x, second.x, local), std::lerp(first.y, second.y, local)};
    }

    SCCurveEval3d SCCurveOnSurface::Evaluate(double parameter, int derivativeOrder) const
    {
        SCCurveEval3d result{};
        result.point.x = std::numeric_limits<double>::quiet_NaN();
        const std::size_t segmentCount = uvCurve_.SegmentCount();
        if (!IsValid(EvaluationTolerance()) || !std::isfinite(parameter) || parameter < 0.0 ||
            parameter > static_cast<double>(segmentCount))
        {
            return result;
        }

        const std::size_t index = parameter == static_cast<double>(segmentCount)
                                      ? segmentCount - 1
                                      : static_cast<std::size_t>(parameter);
        const SCPoint2d first = uvCurve_.VertexAt(index);
        const SCPoint2d second = uvCurve_.VertexAt((index + 1) % uvCurve_.PointCount());
        const SCPoint2d uv = UvAt(parameter);

        const int requestedOrder = std::clamp(derivativeOrder, 0, 1);
        const SCSurfaceEval3d surfaceEval = supportSurface_->Evaluate(uv.x, uv.y, requestedOrder);
        result.point = surfaceEval.point;
        if (!result.point.IsValid())
        {
            return result;
        }
        if (requestedOrder >= 1 && surfaceEval.derivativeOrder >= 1 &&
            surfaceEval.derivativeU.IsValid() && surfaceEval.derivativeV.IsValid())
        {
            result.firstDerivative = surfaceEval.derivativeU * (second.x - first.x) +
                                     surfaceEval.derivativeV * (second.y - first.y);
            result.derivativeOrder = 1;
        }
        return result;
    }

    SCBox3d SCCurveOnSurface::Bounds() const
    {
        if (!IsValid(EvaluationTolerance()))
        {
            return {};
        }

        if (dynamic_cast<const SCPlaneSurface*>(supportSurface_.get()) == nullptr)
        {
            return supportSurface_->Bounds();
        }

        SCBox3d bounds{};
        for (std::size_t index = 0; index < uvCurve_.PointCount(); ++index)
        {
            bounds.ExpandToInclude(PointAt(index));
        }
        return bounds;
    }

    std::unique_ptr<ISCCurve3d> SCCurveOnSurface::Clone() const
    {
        if (supportSurface_ == nullptr)
        {
            return std::make_unique<SCCurveOnSurface>();
        }
        return std::make_unique<SCCurveOnSurface>(
            std::shared_ptr<ISCSurface>(supportSurface_->Clone().release()), uvCurve_);
    }
}  // namespace Geometry
