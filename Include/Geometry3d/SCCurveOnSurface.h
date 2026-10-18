#pragma once

#include "Export/GeometryExport.h"

#include <cstddef>
#include <memory>
#include <type_traits>
#include <utility>

#include "Geometry2d/SCPolyline2d.h"
#include "Geometry3d/ISCCurve3d.h"
#include "Geometry3d/ISCSurface.h"

namespace Geometry
{
    class GEOMETRY_API SCCurveOnSurface final : public ISCCurve3d
    {
    public:
        SCCurveOnSurface() = default;
        SCCurveOnSurface(std::shared_ptr<ISCSurface> supportSurface, SCPolyline2d uvCurve)
            : supportSurface_(std::move(supportSurface)), uvCurve_(std::move(uvCurve))
        {
        }

        [[nodiscard]] bool IsValid(const SCGeometryTolerance3d& tolerance = {}) const override;
        [[nodiscard]] SCIntervald ParameterRange() const override;
        [[nodiscard]] bool IsClosed(const SCGeometryTolerance3d& tolerance = {}) const override;
        [[nodiscard]] bool IsPeriodic(const SCGeometryTolerance3d& tolerance = {}) const override;
        [[nodiscard]] SCPoint3d PointAt(double parameter) const override;
        [[nodiscard]] SCCurveEval3d Evaluate(double parameter, int derivativeOrder = 1) const override;
        [[nodiscard]] std::unique_ptr<ISCCurve3d> Clone() const override;

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
            return uvCurve_.VertexAt(index);
        }

        [[nodiscard]] SCPoint2d UvAt(double parameter) const;

        [[nodiscard]] SCPoint3d PointAt(std::size_t index) const
        {
            if (supportSurface_ == nullptr || index >= uvCurve_.PointCount())
            {
                return {};
            }

            const SCPoint2d uv = uvCurve_.VertexAt(index);
            return supportSurface_->PointAt(uv.x, uv.y);
        }

        template <typename T>
            requires (std::is_integral_v<T> && !std::is_same_v<T, std::size_t>)
        [[nodiscard]] SCPoint3d PointAt(T index) const
        {
            if constexpr (std::is_signed_v<T>)
            {
                if (index < 0)
                {
                    return {};
                }
            }
            return PointAt(static_cast<std::size_t>(index));
        }

        [[nodiscard]] SCBox3d Bounds() const override;

    private:
        std::shared_ptr<ISCSurface> supportSurface_{};
        SCPolyline2d uvCurve_{};
    };
}  // namespace Geometry

