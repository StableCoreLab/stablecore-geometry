#pragma once

#include <cstddef>
#include <memory>

#include "Geometry2d/SCPolyline2d.h"
#include "Geometry3d/ISCSurface.h"

namespace Geometry
{
    class GEOMETRY_API SCCurveOnSurface
    {
    public:
        SCCurveOnSurface() = default;
        SCCurveOnSurface(std::shared_ptr<ISCSurface> supportSurface, SCPolyline2d uvCurve)
            : supportSurface_(std::move(supportSurface)), uvCurve_(std::move(uvCurve))
        {
        }

        [[nodiscard]] bool IsValid(const SCGeometryTolerance3d& tolerance = {}) const
        {
            return supportSurface_ != nullptr && supportSurface_->IsValid(tolerance) && uvCurve_.IsValid();
        }

        [[nodiscard]] const ISCSurface* SupportSurface() const
        {
            return supportSurface_.get();
        }

        [[nodiscard]] const SCPolyline2d& UvCurve() const
        {
            return uvCurve_;
        }

        [[nodiscard]] std::size_t PointCount() const
        {
            return uvCurve_.PointCount();
        }

        [[nodiscard]] SCPoint2d UvPointAt(std::size_t index) const
        {
            return uvCurve_.PointAt(index);
        }

        [[nodiscard]] SCPoint3d PointAt(std::size_t index) const
        {
            if (supportSurface_ == nullptr || index >= uvCurve_.PointCount())
            {
                return {};
            }

            const SCPoint2d uv = uvCurve_.PointAt(index);
            return supportSurface_->PointAt(uv.x, uv.y);
        }

        [[nodiscard]] SCBox3d Bounds() const
        {
            SCBox3d bounds{};
            if (supportSurface_ == nullptr)
            {
                return bounds;
            }

            for (std::size_t i = 0; i < uvCurve_.PointCount(); ++i)
            {
                bounds.ExpandToInclude(PointAt(i));
            }
            return bounds;
        }

    private:
        std::shared_ptr<ISCSurface> supportSurface_{};
        SCPolyline2d uvCurve_{};
    };
}  // namespace Geometry

