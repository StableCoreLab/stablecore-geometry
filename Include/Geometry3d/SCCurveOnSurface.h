#pragma once

#include "Export/GeometryExport.h"

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "Geometry2d/SCPolylineCurve2d.h"
#include "Geometry3d/ISCCurve3d.h"
#include "Geometry3d/ISCSurface.h"

namespace Geometry
{
    class GEOMETRY_API SCCurveOnSurface final : public ISCCurve3d
    {
    public:
        SCCurveOnSurface() = default;
        SCCurveOnSurface(std::shared_ptr<ISCSurface> supportSurface, SCPolyline2d uvCurve)
            : supportSurface_(std::move(supportSurface)), uvCurve_(std::make_shared<SCPolylineCurve2d>(std::move(uvCurve)))
        {
        }

        SCCurveOnSurface(std::shared_ptr<ISCSurface> supportSurface, const ISCCurve2d& uvCurve)
            : supportSurface_(std::move(supportSurface)), uvCurve_(uvCurve.Clone()) {}

        [[nodiscard]] bool IsValid(const SCGeometryTolerance3d& tolerance = {}) const override;
        [[nodiscard]] SCIntervald ParameterRange() const override;
        [[nodiscard]] bool IsClosed(const SCGeometryTolerance3d& tolerance = {}) const override;
        [[nodiscard]] bool IsPeriodic(const SCGeometryTolerance3d& tolerance = {}) const override;
        [[nodiscard]] SCPoint3d PointAt(double parameter) const override;
        [[nodiscard]] SCCurveEval3d Evaluate(double parameter, int derivativeOrder = 1) const override;
        [[nodiscard]] SCCurveEval3d Evaluate(double parameter, int derivativeOrder,
                                            const SCGeometryTolerance3d& tolerance) const;
        [[nodiscard]] std::unique_ptr<ISCCurve3d> Clone() const override;

        [[nodiscard]] const ISCSurface* SupportSurface() const
        {
            return supportSurface_.get();
        }

        [[nodiscard]] const ISCCurve2d& UvGeometry() const
        {
            if (!uvCurve_) throw std::logic_error("SCCurveOnSurface has no UV geometry");
            return *uvCurve_;
        }

        [[nodiscard]] const SCPolyline2d* TryUvCurve() const
        {
            const auto* adapter = dynamic_cast<const SCPolylineCurve2d*>(uvCurve_.get());
            return adapter ? &adapter->Polyline() : nullptr;
        }
        [[nodiscard]] const SCPolyline2d& UvCurve() const
        {
            const auto* path = TryUvCurve();
            if (!path) throw std::logic_error("SCCurveOnSurface has no polyline UV view");
            return *path;
        }
        [[nodiscard]] std::size_t UvPieceCount() const { return uvCurve_ ? uvCurve_->PieceCount() : 0; }
        [[nodiscard]] double UvParameterAt(std::size_t piece, double local) const
        {
            return UvGeometry().ParameterAtPiece(piece, local);
        }

        [[nodiscard]] std::size_t PointCount() const
        {
            return UvCurve().PointCount();
        }

        [[nodiscard]] SCPoint2d UvPointAt(std::size_t index) const
        {
            return UvCurve().VertexAt(index);
        }

        [[nodiscard]] SCPoint2d UvAt(double parameter) const;

        [[nodiscard]] SCPoint3d PointAt(std::size_t index) const
        {
            if (supportSurface_ == nullptr || index >= UvCurve().PointCount())
            {
                return {};
            }

            const SCPoint2d uv = UvCurve().VertexAt(index);
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
        [[nodiscard]] SCCurveEval3d EvaluateInternal(double parameter, int derivativeOrder,
            const SCGeometryTolerance3d& tolerance, bool allowDefaultFallback) const;
        std::shared_ptr<ISCSurface> supportSurface_{};
        std::shared_ptr<const ISCCurve2d> uvCurve_{std::make_shared<SCPolylineCurve2d>()};
    };
}  // namespace Geometry

