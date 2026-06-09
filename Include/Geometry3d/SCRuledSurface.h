#pragma once

#include <algorithm>
#include <memory>

#include "Geometry3d/ISCSurface.h"

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
            if (first_ == nullptr || second_ == nullptr)
            {
                return {};
            }

            const double normalized =
                vRange_.Length() <= Geometry::kDefaultEpsilon ? 0.0 : (v - vRange_.min) / vRange_.Length();
            const SCPoint3d start = first_->PointAt(u);
            const SCPoint3d end = second_->PointAt(u);
            return start + (end - start) * std::clamp(normalized, 0.0, 1.0);
        }

        [[nodiscard]] SCSurfaceEval3d Evaluate(double u, double v, int derivativeOrder = 1) const override
        {
            SCSurfaceEval3d eval{};
            eval.point = PointAt(u, v);
            eval.derivativeOrder = std::clamp(derivativeOrder, 0, 1);
            if (eval.derivativeOrder >= 1 && first_ != nullptr && second_ != nullptr)
            {
                const double normalized =
                    vRange_.Length() <= Geometry::kDefaultEpsilon ? 0.0 : (v - vRange_.min) / vRange_.Length();
                const double weight = std::clamp(normalized, 0.0, 1.0);
                const SCCurveEval3d firstEval = first_->Evaluate(u, 1);
                const SCCurveEval3d secondEval = second_->Evaluate(u, 1);
                eval.derivativeU =
                    firstEval.firstDerivative + (secondEval.firstDerivative - firstEval.firstDerivative) * weight;
                eval.derivativeV = second_->PointAt(u) - first_->PointAt(u);
                eval.normal = Cross(eval.derivativeU, eval.derivativeV);
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

