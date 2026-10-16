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
    class SCNurbsSurface;
    namespace Detail
    {
        [[nodiscard]] SCSurfaceEval3d EvaluateValidatedNurbsSurface(const SCNurbsSurface& surface,
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
            eval.point.x=std::numeric_limits<double>::quiet_NaN();
            eval.derivativeOrder = derivativeOrder;
            if (!IsValid() || !std::isfinite(u) || !std::isfinite(v))
            {
                return eval;
            }
            return Detail::EvaluateValidatedNurbsSurface(*this, u, v, derivativeOrder);
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

    namespace Detail
    {
        [[nodiscard]] inline SCSurfaceEval3d EvaluateValidatedNurbsSurface(const SCNurbsSurface& surface,
                                                                            double u, double v, int derivativeOrder)
        {
            derivativeOrder = std::clamp(derivativeOrder, 0, 1);
            SCSurfaceEval3d eval{};
            eval.point.x=std::numeric_limits<double>::quiet_NaN();
            eval.derivativeOrder = derivativeOrder;
            if (!std::isfinite(u) || !std::isfinite(v)) return eval;

            // 双线性片使用相同基函数和加权求和，避免每次位置查询分配基函数矩阵。
            if (surface.degreeU_ == 1 && surface.degreeV_ == 1 && derivativeOrder == 0)
            {
                const auto basis = [](const std::vector<double>& knots, std::size_t count, double parameter,
                                      std::size_t& first, double (&values)[2]) {
                    const std::size_t span = EvaluateValidatedLinearNurbsBasis(knots,count,parameter,values);
                    first = span - 1;
                };
                std::size_t firstU, firstV;
                double bu[2], bv[2];
                basis(surface.knotsU_, surface.controlPointCountU_, u, firstU, bu);
                basis(surface.knotsV_, surface.controlPointCountV_, v, firstV, bv);
                const std::size_t row = firstV * surface.controlPointCountU_ + firstU;
                const std::size_t indices[]{row, row + 1, row + surface.controlPointCountU_, row + surface.controlPointCountU_ + 1};
                const double values[]{bu[0]*bv[0], bu[1]*bv[0], bu[0]*bv[1], bu[1]*bv[1]};
                NurbsWeightedSum sum(surface.controlPoints_, surface.weights_, std::span<const double>(values),
                                     std::span<const std::size_t>(indices));
                for (std::size_t i = 0; i < 4; ++i)
                    sum.Add(surface.controlPoints_[indices[i]], surface.weights_[indices[i]], values[i]);
                if (sum.weight <= 0.0 || !std::isfinite(sum.weight)) return eval;
                const auto point = sum.Point();
                eval.point = {point.x, point.y, point.z};
                return eval;
            }
            const auto bu = EvaluateLocalNurbsBasis(surface.knotsU_, surface.controlPointCountU_, surface.degreeU_, u);
            const auto bv = EvaluateLocalNurbsBasis(surface.knotsV_, surface.controlPointCountV_, surface.degreeV_, v);
            std::vector<std::size_t> activeIndices;
            std::vector<double> activeBasis;
            activeIndices.reserve(bu.value.size() * bv.value.size());
            activeBasis.reserve(activeIndices.capacity());
            for (std::size_t iv = 0; iv < bv.value.size(); ++iv)
            {
                for (std::size_t iu = 0; iu < bu.value.size(); ++iu)
                {
                    activeIndices.push_back((bv.firstControlIndex + iv) * surface.controlPointCountU_ +
                                            bu.firstControlIndex + iu);
                    activeBasis.push_back(bu.value[iu] * bv.value[iv]);
                }
            }
            NurbsWeightedSum sumU(surface.controlPoints_, surface.weights_, activeBasis, activeIndices);
            NurbsWeightedSum sumV(surface.controlPoints_, surface.weights_, activeBasis, activeIndices);
            for (std::size_t iv = 0; iv < bv.value.size(); ++iv)
            {
                for (std::size_t iu = 0; iu < bu.value.size(); ++iu)
                {
                    const std::size_t local = iv * bu.value.size() + iu;
                    const std::size_t index = activeIndices[local];
                    sumU.Add(surface.controlPoints_[index], surface.weights_[index], activeBasis[local],
                             derivativeOrder >= 1 ? bu.first[iu] * bv.value[iv] : 0.0);
                    if (derivativeOrder >= 1)
                    {
                        sumV.Add(surface.controlPoints_[index], surface.weights_[index], activeBasis[local],
                                 bu.value[iu] * bv.first[iv]);
                    }
                }
            }
            if (sumU.weight <= 0.0 || !std::isfinite(sumU.weight)) return eval;
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
    }
}  // namespace Geometry

