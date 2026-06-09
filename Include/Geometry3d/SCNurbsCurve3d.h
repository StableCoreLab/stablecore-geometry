#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "Geometry3d/ISCCurve3d.h"

namespace Geometry
{
    class GEOMETRY_API SCNurbsCurve3d final : public ISCCurve3d
    {
    public:
        SCNurbsCurve3d() = default;
        SCNurbsCurve3d(int degree, std::vector<SCPoint3d> controlPoints, std::vector<double> knots, bool periodic = false)
            : degree_(degree), controlPoints_(std::move(controlPoints)), knots_(std::move(knots)), periodic_(periodic)
        {
        }

        [[nodiscard]] bool IsValid(const SCGeometryTolerance3d& tolerance = {}) const override
        {
            if (degree_ < 1 || controlPoints_.size() < 2 || knots_.size() != controlPoints_.size() + degree_ + 1)
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

            for (std::size_t i = 1; i < knots_.size(); ++i)
            {
                if (knots_[i] + tolerance.parameterEpsilon < knots_[i - 1])
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
            return controlPoints_.size() >= 2 &&
                   controlPoints_.front().AlmostEquals(controlPoints_.back(), tolerance.distanceEpsilon);
        }

        [[nodiscard]] bool IsPeriodic(const SCGeometryTolerance3d& tolerance = {}) const override
        {
            (void)tolerance;
            return periodic_;
        }

        [[nodiscard]] SCPoint3d PointAt(double parameter) const override
        {
            if (!IsValid())
            {
                return {};
            }

            const SCIntervald range = ParameterRange();
            const double clamped = std::clamp(parameter, range.min, range.max);
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            for (std::size_t i = 0; i < controlPoints_.size(); ++i)
            {
                const double basis = Basis(i, degree_, clamped);
                x += controlPoints_[i].x * basis;
                y += controlPoints_[i].y * basis;
                z += controlPoints_[i].z * basis;
            }
            return SCPoint3d{x, y, z};
        }

        [[nodiscard]] SCCurveEval3d Evaluate(double parameter, int derivativeOrder = 1) const override
        {
            derivativeOrder = std::clamp(derivativeOrder, 0, 2);
            SCCurveEval3d eval{};
            eval.point = PointAt(parameter);
            eval.derivativeOrder = derivativeOrder;
            if (derivativeOrder >= 1 && controlPoints_.size() >= 2)
            {
                const SCIntervald range = ParameterRange();
                const double step =
                    std::max(range.Length() / static_cast<double>(std::max<std::size_t>(controlPoints_.size() * 8, 8)),
                             Geometry::kDefaultEpsilon);
                const double prevParameter = std::max(range.min, parameter - step);
                const double nextParameter = std::min(range.max, parameter + step);
                const double denom = std::max(nextParameter - prevParameter, Geometry::kDefaultEpsilon);
                eval.firstDerivative = (PointAt(nextParameter) - PointAt(prevParameter)) / denom;
            }
            if (derivativeOrder >= 2)
            {
                const SCIntervald range = ParameterRange();
                const double step =
                    std::max(range.Length() / static_cast<double>(std::max<std::size_t>(controlPoints_.size() * 8, 8)),
                             Geometry::kDefaultEpsilon);
                const double prevParameter = std::max(range.min, parameter - step);
                const double nextParameter = std::min(range.max, parameter + step);
                const SCPoint3d previous = PointAt(prevParameter);
                const SCPoint3d current = PointAt(parameter);
                const SCPoint3d next = PointAt(nextParameter);
                const double denom = std::max(step * step, Geometry::kDefaultEpsilon);
                eval.secondDerivative = ((next - current) - (current - previous)) / denom;
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

    private:
        [[nodiscard]] double Basis(std::size_t index, int degree, double parameter) const
        {
            if (degree == 0)
            {
                const bool inHalfOpen = knots_[index] <= parameter && parameter < knots_[index + 1];
                const bool isRightBoundary =
                    std::abs(parameter - knots_[controlPoints_.size()]) <= Geometry::kDefaultEpsilon &&
                    index + 1 == controlPoints_.size();
                return (inHalfOpen || isRightBoundary) ? 1.0 : 0.0;
            }

            double left = 0.0;
            const double leftDenom = knots_[index + static_cast<std::size_t>(degree)] - knots_[index];
            if (std::abs(leftDenom) > Geometry::kDefaultEpsilon)
            {
                left = ((parameter - knots_[index]) / leftDenom) * Basis(index, degree - 1, parameter);
            }

            double right = 0.0;
            const double rightDenom = knots_[index + static_cast<std::size_t>(degree) + 1] - knots_[index + 1];
            if (std::abs(rightDenom) > Geometry::kDefaultEpsilon)
            {
                right = ((knots_[index + static_cast<std::size_t>(degree) + 1] - parameter) / rightDenom) *
                        Basis(index + 1, degree - 1, parameter);
            }

            return left + right;
        }

        int degree_{1};
        std::vector<SCPoint3d> controlPoints_{};
        std::vector<double> knots_{0.0, 1.0};
        bool periodic_{false};
    };
}  // namespace Geometry

