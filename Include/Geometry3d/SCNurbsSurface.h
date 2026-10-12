#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "Geometry3d/ISCSurface.h"
#include "Geometry3d/Detail/NurbsBasis.h"

namespace Geometry
{
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

        [[nodiscard]] bool IsValid(const SCGeometryTolerance3d& tolerance = {}) const override
        {
            (void)tolerance;
            if (controlPointCountU_ == 0 || controlPointCountV_ == 0 ||
                controlPoints_.size() / controlPointCountU_ != controlPointCountV_ ||
                controlPoints_.size() != controlPointCountU_ * controlPointCountV_ ||
                weights_.size() != controlPoints_.size() ||
                !Detail::ValidKnots(knotsU_, controlPointCountU_, degreeU_) ||
                !Detail::ValidKnots(knotsV_, controlPointCountV_, degreeV_))
            {
                return false;
            }

            for (const SCPoint3d& point : controlPoints_)
            {
                if (!point.IsValid())
                {
                    return false;
                }
            }

            for (double weight : weights_)
            {
                if (!std::isfinite(weight) || weight <= 0.0)
                {
                    return false;
                }
            }

            return true;
        }

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

        [[nodiscard]] SCSurfaceEval3d Evaluate(double u, double v, int derivativeOrder = 1) const override
        {
            derivativeOrder = std::clamp(derivativeOrder, 0, 1);
            SCSurfaceEval3d eval{};
            eval.derivativeOrder = derivativeOrder;
            if (!IsValid() || std::isnan(u) || std::isnan(v))
            {
                return eval;
            }
            const auto bu = Detail::EvaluateNurbsBasis(knotsU_, controlPointCountU_, degreeU_, u);
            const auto bv = Detail::EvaluateNurbsBasis(knotsV_, controlPointCountV_, degreeV_, v);
            std::vector<double> activeBasis(controlPoints_.size());
            for (std::size_t iv = 0; iv < controlPointCountV_; ++iv)
                for (std::size_t iu = 0; iu < controlPointCountU_; ++iu)
                    activeBasis[iv * controlPointCountU_ + iu] = bu.value[iu] * bv.value[iv];
            Detail::NurbsWeightedSum sumU(controlPoints_, weights_, activeBasis);
            Detail::NurbsWeightedSum sumV(controlPoints_, weights_, activeBasis);
            for (std::size_t iv = 0; iv < controlPointCountV_; ++iv)
            {
                for (std::size_t iu = 0; iu < controlPointCountU_; ++iu)
                {
                    const std::size_t index = iv * controlPointCountU_ + iu;
                    sumU.Add(controlPoints_[index], weights_[index], activeBasis[index],
                             derivativeOrder >= 1 ? bu.first[iu] * bv.value[iv] : 0.0);
                    if (derivativeOrder >= 1)
                        sumV.Add(controlPoints_[index], weights_[index], activeBasis[index],
                                 bu.value[iu] * bv.first[iv]);
                }
            }
            if (sumU.weight <= 0.0 || !std::isfinite(sumU.weight))
            {
                return eval;
            }
            const SCVector3d point = sumU.Point();
            eval.point = SCPoint3d{point.x, point.y, point.z};
            if (derivativeOrder >= 1)
            {
                eval.derivativeU = sumU.First();
                eval.derivativeV = sumV.First();
                eval.normal = Cross(eval.derivativeU, eval.derivativeV);
            }
            return eval;
        }

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

