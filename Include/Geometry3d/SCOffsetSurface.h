#pragma once

#include <cmath>
#include <memory>
#include <limits>

#include "Geometry3d/Detail/SurfaceNumerics.h"
#include "Geometry3d/ISCSurface.h"

namespace Geometry
{
    class GEOMETRY_API SCOffsetSurface final : public ISCSurface
    {
    public:
        SCOffsetSurface() = default;
        SCOffsetSurface(std::shared_ptr<ISCSurface> baseSurface, double offsetDistance)
            : baseSurface_(std::move(baseSurface)), offsetDistance_(offsetDistance)
        {
        }

        [[nodiscard]] static SCOffsetSurface FromSurface(const ISCSurface& surface, double offsetDistance)
        {
            return SCOffsetSurface(std::shared_ptr<ISCSurface>(surface.Clone().release()), offsetDistance);
        }

        [[nodiscard]] bool IsValid(const SCGeometryTolerance3d& tolerance = {}) const override
        {
            return baseSurface_ != nullptr && baseSurface_->IsValid(tolerance) && std::isfinite(offsetDistance_);
        }

        [[nodiscard]] SCIntervald URange() const override
        {
            return baseSurface_ != nullptr ? baseSurface_->URange() : SCIntervald{};
        }

        [[nodiscard]] SCIntervald VRange() const override
        {
            return baseSurface_ != nullptr ? baseSurface_->VRange() : SCIntervald{};
        }

        [[nodiscard]] SCPoint3d PointAt(double u, double v) const override
        {
            if (baseSurface_ == nullptr)
            {
                return {};
            }

            const SCPoint3d basePoint = baseSurface_->PointAt(u, v);
            if (!basePoint.IsValid() || offsetDistance_ == 0.0)
            {
                return basePoint;
            }
            const SCSurfaceEval3d eval = baseSurface_->Evaluate(u, v, 1);
            SCVector3d unitNormal;
            if (!Detail::TryUnitVector(eval.normal, unitNormal))
            {
                const double invalid = std::numeric_limits<double>::quiet_NaN();
                return {invalid, invalid, invalid};
            }
            return basePoint + unitNormal * offsetDistance_;
        }

        [[nodiscard]] SCSurfaceEval3d Evaluate(double u, double v, int derivativeOrder = 1) const override
        {
            SCSurfaceEval3d eval =
                baseSurface_ != nullptr ? baseSurface_->Evaluate(u, v, derivativeOrder) : SCSurfaceEval3d{};
            if (eval.derivativeOrder >= 0)
            {
                eval.point = PointAt(u, v);
            }
            return eval;
        }

        [[nodiscard]] SCBox3d Bounds() const override
        {
            if (baseSurface_ == nullptr)
            {
                return {};
            }

            SCBox3d bounds = baseSurface_->Bounds();
            if (!bounds.IsValid())
            {
                return bounds;
            }

            const double padding = std::abs(offsetDistance_);
            bounds.ExpandToInclude(bounds.MinPoint() + SCVector3d{-padding, -padding, -padding});
            bounds.ExpandToInclude(bounds.MaxPoint() + SCVector3d{padding, padding, padding});
            return bounds;
        }

        [[nodiscard]] std::unique_ptr<ISCSurface> Clone() const override
        {
            return std::make_unique<SCOffsetSurface>(
                baseSurface_ != nullptr ? std::shared_ptr<ISCSurface>(baseSurface_->Clone().release()) : nullptr,
                offsetDistance_);
        }

        [[nodiscard]] const ISCSurface* BaseSurface() const
        {
            return baseSurface_.get();
        }

        [[nodiscard]] double OffsetDistance() const
        {
            return offsetDistance_;
        }

    private:
        std::shared_ptr<ISCSurface> baseSurface_{};
        double offsetDistance_{0.0};
    };
}  // namespace Geometry

