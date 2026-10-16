#pragma once

#include <algorithm>
#include <memory>

#include "Geometry3d/ISCSurface.h"
#include "Support/StableNumerics.h"
#include "Geometry3d/Detail/SurfaceNumerics.h"

namespace Geometry
{
    class GEOMETRY_API SCRuledSurface final : public ISCSurface
    {
    public:
        SCRuledSurface() = default;
        SCRuledSurface(std::shared_ptr<ISCCurve3d> first,
                     std::shared_ptr<ISCCurve3d> second,
                     SCIntervald vRange = SCIntervald{0.0, 1.0})
            : first_(std::move(first)), second_(std::move(second)), vRange_(vRange)
        {
        }

        [[nodiscard]] static SCRuledSurface FromCurves(const ISCCurve3d& first,
                                                     const ISCCurve3d& second,
                                                     SCIntervald vRange = SCIntervald{0.0, 1.0})
        {
            return SCRuledSurface(first.Clone(), second.Clone(), vRange);
        }

        [[nodiscard]] bool IsValid(const SCGeometryTolerance3d& tolerance = {}) const override
        {
            return first_ != nullptr && second_ != nullptr && first_->IsValid(tolerance) &&
                   second_->IsValid(tolerance) && vRange_.IsValid();
        }

        [[nodiscard]] SCIntervald URange() const override
        {
            return first_ != nullptr ? first_->ParameterRange() : SCIntervald{};
        }

        [[nodiscard]] SCIntervald VRange() const override
        {
            return vRange_;
        }

        [[nodiscard]] SCPoint3d PointAt(double u, double v) const override
        {
            if (first_ == nullptr || second_ == nullptr || !vRange_.IsValid() || !std::isfinite(v))
            {
                return {std::numeric_limits<double>::quiet_NaN(), 0, 0};
            }

            const auto normalized = Detail::StableNormalizedParameter(vRange_.min, vRange_.max,
                std::clamp(v, vRange_.min, vRange_.max));
            if (!normalized) return {std::numeric_limits<double>::quiet_NaN(), 0, 0};
            const SCPoint3d start = first_->PointAt(u);
            const SCPoint3d end = second_->PointAt(u);
            if (!start.IsValid() || !end.IsValid()) return {std::numeric_limits<double>::quiet_NaN(), 0, 0};
            return {std::lerp(start.x,end.x,*normalized), std::lerp(start.y,end.y,*normalized),
                    std::lerp(start.z,end.z,*normalized)};
        }

        [[nodiscard]] SCSurfaceEval3d Evaluate(double u, double v, int derivativeOrder = 1) const override
        {
            SCSurfaceEval3d eval{};
            eval.point = PointAt(u, v);
            eval.derivativeOrder = std::clamp(derivativeOrder, 0, 1);
            if (eval.derivativeOrder >= 1 && first_ != nullptr && second_ != nullptr)
            {
                if (!eval.point.IsValid() || !vRange_.IsValid()) { eval.derivativeOrder=0; return eval; }
                const auto normalized=Detail::StableNormalizedParameter(vRange_.min,vRange_.max,
                    std::clamp(v,vRange_.min,vRange_.max));
                const auto span=Detail::StableSpan(vRange_.min,vRange_.max);
                if (!normalized || !span || *span==0) { eval.derivativeOrder=0; return eval; }
                const double weight = *normalized;
                const SCCurveEval3d firstEval = first_->Evaluate(u, 1);
                const SCCurveEval3d secondEval = second_->Evaluate(u, 1);
                if (firstEval.derivativeOrder<1 || secondEval.derivativeOrder<1 ||
                    !firstEval.firstDerivative.IsValid() || !secondEval.firstDerivative.IsValid() ||
                    !Detail::SameRoundedPosition(firstEval.point,first_->PointAt(u)) ||
                    !Detail::SameRoundedPosition(secondEval.point,second_->PointAt(u)))
                    { eval.derivativeOrder=0; return eval; }
                eval.derivativeU = {std::lerp(firstEval.firstDerivative.x,secondEval.firstDerivative.x,weight),
                    std::lerp(firstEval.firstDerivative.y,secondEval.firstDerivative.y,weight),
                    std::lerp(firstEval.firstDerivative.z,secondEval.firstDerivative.z,weight)};
                eval.derivativeV = (second_->PointAt(u) - first_->PointAt(u)) / *span;
                eval.normal = Cross(eval.derivativeU, eval.derivativeV);
                if (!eval.derivativeU.IsValid() || !eval.derivativeV.IsValid() || !eval.normal.IsValid())
                    eval.derivativeOrder=0;
            }
            return eval;
        }

        [[nodiscard]] SCBox3d Bounds() const override
        {
            SCBox3d bounds{};
            if (first_ != nullptr)
            {
                const SCBox3d firstBounds = first_->Bounds();
                if (firstBounds.IsValid())
                {
                    bounds.ExpandToInclude(firstBounds.MinPoint());
                    bounds.ExpandToInclude(firstBounds.MaxPoint());
                }
            }
            if (second_ != nullptr)
            {
                const SCBox3d secondBounds = second_->Bounds();
                if (secondBounds.IsValid())
                {
                    bounds.ExpandToInclude(secondBounds.MinPoint());
                    bounds.ExpandToInclude(secondBounds.MaxPoint());
                }
            }
            return bounds;
        }

        [[nodiscard]] std::unique_ptr<ISCSurface> Clone() const override
        {
            return std::make_unique<SCRuledSurface>(
                first_ != nullptr ? std::shared_ptr<ISCCurve3d>(first_->Clone().release()) : nullptr,
                second_ != nullptr ? std::shared_ptr<ISCCurve3d>(second_->Clone().release()) : nullptr,
                vRange_);
        }

        [[nodiscard]] const ISCCurve3d* FirstCurve() const
        {
            return first_.get();
        }

        [[nodiscard]] const ISCCurve3d* SecondCurve() const
        {
            return second_.get();
        }

    private:
        std::shared_ptr<ISCCurve3d> first_{};
        std::shared_ptr<ISCCurve3d> second_{};
        SCIntervald vRange_{0.0, 1.0};
    };
}  // namespace Geometry

