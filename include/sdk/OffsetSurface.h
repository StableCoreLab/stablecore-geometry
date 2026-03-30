#pragma once

#include <cmath>
#include <memory>

#include "sdk/Surface.h"

namespace geometry::sdk
{
class GEOMETRY_API OffsetSurface final : public Surface
{
public:
    OffsetSurface() = default;
    OffsetSurface(std::shared_ptr<Surface> baseSurface, double offsetDistance)
        : baseSurface_(std::move(baseSurface)), offsetDistance_(offsetDistance)
    {
    }

    [[nodiscard]] static OffsetSurface FromSurface(const Surface& surface, double offsetDistance)
    {
        return OffsetSurface(std::shared_ptr<Surface>(surface.Clone().release()), offsetDistance);
    }

    [[nodiscard]] bool IsValid(const GeometryTolerance3d& tolerance = {}) const override
    {
        return baseSurface_ != nullptr && baseSurface_->IsValid(tolerance) && std::isfinite(offsetDistance_);
    }

    [[nodiscard]] Intervald URange() const override
    {
        return baseSurface_ != nullptr ? baseSurface_->URange() : Intervald{};
    }

    [[nodiscard]] Intervald VRange() const override
    {
        return baseSurface_ != nullptr ? baseSurface_->VRange() : Intervald{};
    }

    [[nodiscard]] Point3d PointAt(double u, double v) const override
    {
        if (baseSurface_ == nullptr)
        {
            return {};
        }

        const SurfaceEval3d eval = baseSurface_->Evaluate(u, v, 1);
        const Vector3d unitNormal = eval.normal.Normalized(geometry::kDefaultEpsilon);
        return baseSurface_->PointAt(u, v) + unitNormal * offsetDistance_;
    }

    [[nodiscard]] SurfaceEval3d Evaluate(double u, double v, int derivativeOrder = 1) const override
    {
        SurfaceEval3d eval = baseSurface_ != nullptr ? baseSurface_->Evaluate(u, v, derivativeOrder) : SurfaceEval3d{};
        if (eval.derivativeOrder >= 0)
        {
            eval.point = PointAt(u, v);
        }
        return eval;
    }

    [[nodiscard]] Box3d Bounds() const override
    {
        if (baseSurface_ == nullptr)
        {
            return {};
        }

        Box3d bounds = baseSurface_->Bounds();
        if (!bounds.IsValid())
        {
            return bounds;
        }

        const double padding = std::abs(offsetDistance_);
        bounds.ExpandToInclude(bounds.MinPoint() + Vector3d{-padding, -padding, -padding});
        bounds.ExpandToInclude(bounds.MaxPoint() + Vector3d{padding, padding, padding});
        return bounds;
    }

    [[nodiscard]] std::unique_ptr<Surface> Clone() const override
    {
        return std::make_unique<OffsetSurface>(
            baseSurface_ != nullptr ? std::shared_ptr<Surface>(baseSurface_->Clone().release()) : nullptr,
            offsetDistance_);
    }

    [[nodiscard]] const Surface* BaseSurface() const
    {
        return baseSurface_.get();
    }

    [[nodiscard]] double OffsetDistance() const
    {
        return offsetDistance_;
    }

private:
    std::shared_ptr<Surface> baseSurface_{};
    double offsetDistance_{0.0};
};
} // namespace geometry::sdk
