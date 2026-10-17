#pragma once

#include "Export/GeometryExport.h"

#include <memory>
#include <utility>
#include <vector>

#include "Geometry3d/ISCSurface.h"

namespace Geometry
{
    class SCNurbsSurface;
    namespace Detail
    {
        [[nodiscard]] GEOMETRY_API SCSurfaceEval3d EvaluateValidatedNurbsSurface(const SCNurbsSurface& surface,
                                                                     double u, double v, int derivativeOrder);
    }

    class GEOMETRY_API SCNurbsSurface final : public ISCSurface
    {
    public:
        SCNurbsSurface() = default;
        SCNurbsSurface(int degreeU,
                     int degreeV,
                     std::size_t controlPointCountU,
                     std::size_t controlPointCountV,
                     std::vector<SCPoint3d> controlPoints,
                     std::vector<double> knotsU,
                     std::vector<double> knotsV)
            : degreeU_(degreeU),
              degreeV_(degreeV),
              controlPointCountU_(controlPointCountU),
              controlPointCountV_(controlPointCountV),
              controlPoints_(std::move(controlPoints)),
              knotsU_(std::move(knotsU)),
              knotsV_(std::move(knotsV))
        {
            weights_.assign(controlPoints_.size(), 1.0);
        }

        SCNurbsSurface(int degreeU, int degreeV, std::size_t controlPointCountU, std::size_t controlPointCountV,
                       std::vector<SCPoint3d> controlPoints, std::vector<double> knotsU,
                       std::vector<double> knotsV, std::vector<double> weights)
            : degreeU_(degreeU), degreeV_(degreeV), controlPointCountU_(controlPointCountU),
              controlPointCountV_(controlPointCountV), controlPoints_(std::move(controlPoints)),
              knotsU_(std::move(knotsU)), knotsV_(std::move(knotsV)), weights_(std::move(weights))
        {
        }

        [[nodiscard]] bool IsValid(const SCGeometryTolerance3d& tolerance = {}) const override;

        [[nodiscard]] SCIntervald URange() const override
        {
            return IsValid() ? SCIntervald{knotsU_[static_cast<std::size_t>(degreeU_)], knotsU_[controlPointCountU_]}
                             : SCIntervald{};
        }

        [[nodiscard]] SCIntervald VRange() const override
        {
            return IsValid() ? SCIntervald{knotsV_[static_cast<std::size_t>(degreeV_)], knotsV_[controlPointCountV_]}
                             : SCIntervald{};
        }

        [[nodiscard]] SCPoint3d PointAt(double u, double v) const override
        {
            return Evaluate(u, v, 0).point;
        }

        [[nodiscard]] SCSurfaceEval3d Evaluate(double u, double v, int derivativeOrder = 1) const override;

        [[nodiscard]] SCBox3d Bounds() const override
        {
            SCBox3d bounds{};
            for (const SCPoint3d& point : controlPoints_)
            {
                bounds.ExpandToInclude(point);
            }
            return bounds;
        }

        [[nodiscard]] std::unique_ptr<ISCSurface> Clone() const override
        {
            return std::make_unique<SCNurbsSurface>(*this);
        }

        [[nodiscard]] int DegreeU() const
        {
            return degreeU_;
        }

        [[nodiscard]] int DegreeV() const
        {
            return degreeV_;
        }

        [[nodiscard]] std::size_t ControlPointCountU() const
        {
            return controlPointCountU_;
        }

        [[nodiscard]] std::size_t ControlPointCountV() const
        {
            return controlPointCountV_;
        }

        [[nodiscard]] const std::vector<SCPoint3d>& ControlPoints() const
        {
            return controlPoints_;
        }

        [[nodiscard]] const std::vector<double>& KnotsU() const
        {
            return knotsU_;
        }

        [[nodiscard]] const std::vector<double>& KnotsV() const
        {
            return knotsV_;
        }

        [[nodiscard]] const std::vector<double>& Weights() const { return weights_; }

    private:
        friend SCSurfaceEval3d Detail::EvaluateValidatedNurbsSurface(const SCNurbsSurface&, double, double, int);

        int degreeU_{1};
        int degreeV_{1};
        std::size_t controlPointCountU_{0};
        std::size_t controlPointCountV_{0};
        std::vector<SCPoint3d> controlPoints_{};
        std::vector<double> knotsU_{0.0, 1.0};
        std::vector<double> knotsV_{0.0, 1.0};
        std::vector<double> weights_{};
    };

}  // namespace Geometry

