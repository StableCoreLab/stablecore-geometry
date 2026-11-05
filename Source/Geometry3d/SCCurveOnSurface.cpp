#include "Geometry3d/SCCurveOnSurface.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "Geometry3d/SCPlaneSurface.h"
#include "Types/Geometry2d/SCVector2.h"

namespace Geometry
{
    namespace
    {
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
        if (!uvCurve_ || !uvCurve_->IsValid(uvEps) || UvPieceCount() == 0)
        {
            return false;
        }

        if (!uvCurve_->Bounds().IsValid()) return false;
        const auto range = uvCurve_->ParameterRange();
        if (!range.IsValid() || !(range.min < range.max)) return false;
        double previous = range.min;
        for (std::size_t piece = 0; piece < UvPieceCount(); ++piece)
        {
            const auto part = uvCurve_->PieceRange(piece);
            if (!part.IsValid() || part.min != previous || !(part.min < part.max)) return false;
            previous = part.max;
        }
        if (previous != range.max) return false;
        if (dynamic_cast<const SCPlaneSurface*>(supportSurface_.get()) != nullptr) return true;

        const SCBox2d uvBounds = uvCurve_->Bounds();
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
        return uvCurve_ ? uvCurve_->ParameterRange() : SCIntervald{};
    }

    bool SCCurveOnSurface::IsClosed(const SCGeometryTolerance3d& tolerance) const
    {
        if (!IsValid(tolerance)) return false;
        const auto range = ParameterRange();
        // 显式校验通过后直接映射端点，避免无容差求值重新施加默认阈值。
        const auto first = uvCurve_->Evaluate(range.min, 0).point;
        const auto last = uvCurve_->Evaluate(range.max, 0).point;
        if (!first.IsValid() || !last.IsValid()) return false;
        return supportSurface_->PointAt(first.x, first.y).AlmostEquals(
            supportSurface_->PointAt(last.x, last.y), tolerance.distanceEpsilon);
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
        const auto range = ParameterRange();
        if (!uvCurve_ || (!uvCurve_->IsValid(std::numeric_limits<double>::denorm_min()) && !uvCurve_->IsValid()) ||
            !std::isfinite(parameter) || !range.Contains(parameter, 0.0))
        {
            return invalid;
        }

        return uvCurve_->Evaluate(parameter, 0).point;
    }

    SCCurveEval3d SCCurveOnSurface::Evaluate(double parameter, int derivativeOrder) const
    {
        return EvaluateInternal(parameter, derivativeOrder, EvaluationTolerance(), true);
    }

    SCCurveEval3d SCCurveOnSurface::Evaluate(double parameter, int derivativeOrder,
                                            const SCGeometryTolerance3d& tolerance) const
    {
        return EvaluateInternal(parameter, derivativeOrder, tolerance, false);
    }

    SCCurveEval3d SCCurveOnSurface::EvaluateInternal(double parameter, int derivativeOrder,
        const SCGeometryTolerance3d& tolerance, bool allowDefaultFallback) const
    {
        SCCurveEval3d result{};
        result.point.x = std::numeric_limits<double>::quiet_NaN();
        if ((!IsValid(tolerance) && !(allowDefaultFallback && IsValid())) || !std::isfinite(parameter) ||
            !ParameterRange().Contains(parameter, 0.0))
        {
            return result;
        }

        const int requestedOrder = std::clamp(derivativeOrder, 0, 1);
        const SCCurveEval2d uvSample = uvCurve_->Evaluate(parameter, requestedOrder);
        const SCPoint2d uv = uvSample.point;
        if (!uv.IsValid()) return result;

        const SCSurfaceEval3d surfaceEval = supportSurface_->Evaluate(uv.x, uv.y, requestedOrder);
        result.point = surfaceEval.point;
        if (!result.point.IsValid())
        {
            return result;
        }
        if (requestedOrder >= 1 && uvSample.derivativeOrder >= 1 && surfaceEval.derivativeOrder >= 1 &&
            surfaceEval.derivativeU.IsValid() && surfaceEval.derivativeV.IsValid())
        {
            result.firstDerivative = surfaceEval.derivativeU * uvSample.firstDerivative.x +
                                     surfaceEval.derivativeV * uvSample.firstDerivative.y;
            if (result.firstDerivative.IsValid()) result.derivativeOrder = 1;
        }
        return result;
    }

    SCBox3d SCCurveOnSurface::Bounds() const
    {
        if (!IsValid(EvaluationTolerance()) && !IsValid())
        {
            return {};
        }

        if (dynamic_cast<const SCPlaneSurface*>(supportSurface_.get()) == nullptr)
        {
            return supportSurface_->Bounds();
        }

        SCBox3d bounds{};
        const auto uvBounds = uvCurve_->Bounds();
        if (!uvBounds.IsValid()) return bounds;
        // 仿射平面映射 UV 保守盒的四角，覆盖曲线内部极值。
        for (const double u : {uvBounds.MinPoint().x, uvBounds.MaxPoint().x})
            for (const double v : {uvBounds.MinPoint().y, uvBounds.MaxPoint().y})
            {
                const auto point = supportSurface_->PointAt(u, v);
                if (!point.IsValid()) return {};
                bounds.ExpandToInclude(point);
            }
        return bounds;
    }

    std::unique_ptr<ISCCurve3d> SCCurveOnSurface::Clone() const
    {
        if (supportSurface_ == nullptr || uvCurve_ == nullptr)
        {
            return std::make_unique<SCCurveOnSurface>();
        }
        return std::make_unique<SCCurveOnSurface>(
            std::shared_ptr<ISCSurface>(supportSurface_->Clone().release()), *uvCurve_);
    }
}  // namespace Geometry
