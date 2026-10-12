#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "Geometry3d/ISCCurve3d.h"
#include "Geometry3d/Detail/NurbsBasis.h"

namespace Geometry
{
    class GEOMETRY_API SCNurbsCurve3d final : public ISCCurve3d
    {
    public:
        SCNurbsCurve3d() = default;
        SCNurbsCurve3d(int degree, std::vector<SCPoint3d> controlPoints, std::vector<double> knots, bool periodic = false)
            : degree_(degree), controlPoints_(std::move(controlPoints)), knots_(std::move(knots)), periodic_(periodic)
        {
            weights_.assign(controlPoints_.size(), 1.0);
        }

        SCNurbsCurve3d(int degree, std::vector<SCPoint3d> controlPoints, std::vector<double> knots,
                       std::vector<double> weights, bool periodic = false)
            : degree_(degree), controlPoints_(std::move(controlPoints)), knots_(std::move(knots)),
              weights_(std::move(weights)), periodic_(periodic)
        {
        }

        [[nodiscard]] bool IsValid(const SCGeometryTolerance3d& tolerance = {}) const override
        {
            (void)tolerance;
            if (!Detail::ValidKnots(knots_, controlPoints_.size(), degree_) ||
                weights_.size() != controlPoints_.size())
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

        [[nodiscard]] SCIntervald ParameterRange() const override
        {
            return IsValid() ? SCIntervald{knots_[static_cast<std::size_t>(degree_)], knots_[controlPoints_.size()]}
                             : SCIntervald{};
        }

        [[nodiscard]] bool IsClosed(const SCGeometryTolerance3d& tolerance = {}) const override
        {
            return IsValid(tolerance) && PointAt(StartParameter()).AlmostEquals(
                PointAt(EndParameter()), tolerance.distanceEpsilon);
        }

        [[nodiscard]] bool IsPeriodic(const SCGeometryTolerance3d& tolerance = {}) const override
        {
            (void)tolerance;
            return periodic_;
        }

        [[nodiscard]] SCPoint3d PointAt(double parameter) const override
        {
            return Evaluate(parameter, 0).point;
        }

        [[nodiscard]] SCCurveEval3d Evaluate(double parameter, int derivativeOrder = 1) const override
        {
            derivativeOrder = std::clamp(derivativeOrder, 0, 2);
            SCCurveEval3d eval{};
            eval.derivativeOrder = derivativeOrder;
            if (!IsValid() || std::isnan(parameter))
            {
                return eval;
            }
            const auto basis = Detail::EvaluateNurbsBasis(knots_, controlPoints_.size(), degree_, parameter);
            Detail::NurbsWeightedSum sum(controlPoints_, weights_, basis.value);
            for (std::size_t i = 0; i < controlPoints_.size(); ++i)
            {
                sum.Add(controlPoints_[i], weights_[i], basis.value[i],
                        derivativeOrder >= 1 ? basis.first[i] : 0.0,
                        derivativeOrder >= 2 ? basis.second[i] : 0.0);
            }
            if (sum.weight <= 0.0 || !std::isfinite(sum.weight))
            {
                return eval;
            }
            const SCVector3d point = sum.Point();
            eval.point = SCPoint3d{point.x, point.y, point.z};
            if (derivativeOrder >= 1)
            {
                eval.firstDerivative = sum.First();
            }
            if (derivativeOrder >= 2)
            {
                eval.secondDerivative = sum.Second(eval.firstDerivative);
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

        [[nodiscard]] std::unique_ptr<ISCCurve3d> Clone() const override
        {
            return std::make_unique<SCNurbsCurve3d>(*this);
        }

        [[nodiscard]] int Degree() const
        {
            return degree_;
        }

        [[nodiscard]] const std::vector<SCPoint3d>& ControlPoints() const
        {
            return controlPoints_;
        }

        [[nodiscard]] const std::vector<double>& Knots() const
        {
            return knots_;
        }

        [[nodiscard]] const std::vector<double>& Weights() const { return weights_; }

    private:
        int degree_{1};
        std::vector<SCPoint3d> controlPoints_{};
        std::vector<double> knots_{0.0, 1.0};
        std::vector<double> weights_{};
        bool periodic_{false};
    };
}  // namespace Geometry

