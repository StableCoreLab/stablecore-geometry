#pragma once

#include "Export/GeometryExport.h"

#include <memory>
#include <utility>
#include <vector>

#include "Geometry3d/ISCCurve3d.h"

namespace Geometry
{
    class SCNurbsCurve3d;
    namespace Detail
    {
        [[nodiscard]] GEOMETRY_API SCCurveEval3d EvaluateValidatedNurbsCurve(const SCNurbsCurve3d& curve,
                                                                 double parameter,
                                                                 int derivativeOrder);
    }

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

        [[nodiscard]] bool IsValid(const SCGeometryTolerance3d& tolerance = {}) const override;

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

        [[nodiscard]] SCCurveEval3d Evaluate(double parameter, int derivativeOrder = 1) const override;

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
        friend SCCurveEval3d Detail::EvaluateValidatedNurbsCurve(const SCNurbsCurve3d&, double, int);

        int degree_{1};
        std::vector<SCPoint3d> controlPoints_{};
        std::vector<double> knots_{0.0, 1.0};
        std::vector<double> weights_{};
        bool periodic_{false};
    };

}  // namespace Geometry

