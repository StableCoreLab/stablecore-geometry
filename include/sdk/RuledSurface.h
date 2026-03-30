#pragma once

#include <algorithm>
#include <memory>

#include "sdk/Surface.h"

namespace geometry::sdk
{
class GEOMETRY_API RuledSurface final : public Surface
{
public:
    RuledSurface() = default;
    RuledSurface(std::shared_ptr<Curve3d> first, std::shared_ptr<Curve3d> second, Intervald vRange = Intervald{0.0, 1.0})
        : first_(std::move(first)), second_(std::move(second)), vRange_(vRange)
    {
    }

    [[nodiscard]] static RuledSurface FromCurves(
        const Curve3d& first,
        const Curve3d& second,
        Intervald vRange = Intervald{0.0, 1.0})
    {
        return RuledSurface(first.Clone(), second.Clone(), vRange);
    }

    [[nodiscard]] bool IsValid(const GeometryTolerance3d& tolerance = {}) const override
    {
        return first_ != nullptr && second_ != nullptr && first_->IsValid(tolerance) && second_->IsValid(tolerance) &&
               vRange_.IsValid();
    }

    [[nodiscard]] Intervald URange() const override
    {
        return first_ != nullptr ? first_->ParameterRange() : Intervald{};
    }

    [[nodiscard]] Intervald VRange() const override
    {
        return vRange_;
    }

    [[nodiscard]] Point3d PointAt(double u, double v) const override
    {
        if (first_ == nullptr || second_ == nullptr)
        {
            return {};
        }

        const double normalized =
            vRange_.Length() <= geometry::kDefaultEpsilon ? 0.0 : (v - vRange_.min) / vRange_.Length();
        const Point3d start = first_->PointAt(u);
        const Point3d end = second_->PointAt(u);
        return start + (end - start) * std::clamp(normalized, 0.0, 1.0);
    }

    [[nodiscard]] SurfaceEval3d Evaluate(double u, double v, int derivativeOrder = 1) const override
    {
        SurfaceEval3d eval{};
        eval.point = PointAt(u, v);
        eval.derivativeOrder = std::clamp(derivativeOrder, 0, 1);
        if (eval.derivativeOrder >= 1 && first_ != nullptr && second_ != nullptr)
        {
            const double normalized =
                vRange_.Length() <= geometry::kDefaultEpsilon ? 0.0 : (v - vRange_.min) / vRange_.Length();
            const double weight = std::clamp(normalized, 0.0, 1.0);
            const CurveEval3d firstEval = first_->Evaluate(u, 1);
            const CurveEval3d secondEval = second_->Evaluate(u, 1);
            eval.derivativeU =
                firstEval.firstDerivative + (secondEval.firstDerivative - firstEval.firstDerivative) * weight;
            eval.derivativeV = second_->PointAt(u) - first_->PointAt(u);
            eval.normal = Cross(eval.derivativeU, eval.derivativeV);
        }
        return eval;
    }

    [[nodiscard]] Box3d Bounds() const override
    {
        Box3d bounds{};
        if (first_ != nullptr)
        {
            const Box3d firstBounds = first_->Bounds();
            if (firstBounds.IsValid())
            {
                bounds.ExpandToInclude(firstBounds.MinPoint());
                bounds.ExpandToInclude(firstBounds.MaxPoint());
            }
        }
        if (second_ != nullptr)
        {
            const Box3d secondBounds = second_->Bounds();
            if (secondBounds.IsValid())
            {
                bounds.ExpandToInclude(secondBounds.MinPoint());
                bounds.ExpandToInclude(secondBounds.MaxPoint());
            }
        }
        return bounds;
    }

    [[nodiscard]] std::unique_ptr<Surface> Clone() const override
    {
        return std::make_unique<RuledSurface>(
            first_ != nullptr ? std::shared_ptr<Curve3d>(first_->Clone().release()) : nullptr,
            second_ != nullptr ? std::shared_ptr<Curve3d>(second_->Clone().release()) : nullptr,
            vRange_);
    }

    [[nodiscard]] const Curve3d* FirstCurve() const
    {
        return first_.get();
    }

    [[nodiscard]] const Curve3d* SecondCurve() const
    {
        return second_.get();
    }

private:
    std::shared_ptr<Curve3d> first_{};
    std::shared_ptr<Curve3d> second_{};
    Intervald vRange_{0.0, 1.0};
};
} // namespace geometry::sdk
