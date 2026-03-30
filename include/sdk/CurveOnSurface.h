#pragma once

#include <cstddef>
#include <memory>

#include "sdk/Polyline2d.h"
#include "sdk/Surface.h"

namespace geometry::sdk
{
class GEOMETRY_API CurveOnSurface
{
public:
    CurveOnSurface() = default;
    CurveOnSurface(std::shared_ptr<Surface> supportSurface, Polyline2d uvCurve)
        : supportSurface_(std::move(supportSurface)), uvCurve_(std::move(uvCurve))
    {
    }

    [[nodiscard]] bool IsValid(const GeometryTolerance3d& tolerance = {}) const
    {
        return supportSurface_ != nullptr && supportSurface_->IsValid(tolerance) && uvCurve_.IsValid();
    }

    [[nodiscard]] const Surface* SupportSurface() const
    {
        return supportSurface_.get();
    }

    [[nodiscard]] const Polyline2d& UvCurve() const
    {
        return uvCurve_;
    }

    [[nodiscard]] std::size_t PointCount() const
    {
        return uvCurve_.PointCount();
    }

    [[nodiscard]] Point2d UvPointAt(std::size_t index) const
    {
        return uvCurve_.PointAt(index);
    }

    [[nodiscard]] Point3d PointAt(std::size_t index) const
    {
        if (supportSurface_ == nullptr || index >= uvCurve_.PointCount())
        {
            return {};
        }

        const Point2d uv = uvCurve_.PointAt(index);
        return supportSurface_->PointAt(uv.x, uv.y);
    }

    [[nodiscard]] Box3d Bounds() const
    {
        Box3d bounds{};
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
    std::shared_ptr<Surface> supportSurface_{};
    Polyline2d uvCurve_{};
};
} // namespace geometry::sdk
