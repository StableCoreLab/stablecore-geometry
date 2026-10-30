#include "Geometry3d/SCNurbsCurve3d.h"
#include "../Detail/NurbsBasis.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Geometry
{
    bool SCNurbsCurve3d::IsValid(const SCGeometryTolerance3d& tolerance) const
    {
        if (!tolerance.IsValid() || !Detail::ValidKnots(knots_, controlPoints_.size(), degree_) ||
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

    SCCurveEval3d SCNurbsCurve3d::Evaluate(double parameter, int derivativeOrder) const
    {
        derivativeOrder = std::clamp(derivativeOrder, 0, 2);
        SCCurveEval3d eval{};
        eval.point.x=std::numeric_limits<double>::quiet_NaN();
        eval.derivativeOrder = derivativeOrder;
        if (!IsValid() || !std::isfinite(parameter))
        {
            return eval;
        }
        return Detail::EvaluateValidatedNurbsCurve(*this, parameter, derivativeOrder);
    }

    namespace Detail
    {
        [[nodiscard]] SCCurveEval3d EvaluateValidatedNurbsCurve(const SCNurbsCurve3d& curve,
                                                                double parameter,
                                                                int derivativeOrder)
        {
            derivativeOrder = std::clamp(derivativeOrder, 0, 2);
            SCCurveEval3d eval{};
            eval.point.x=std::numeric_limits<double>::quiet_NaN();
            eval.derivativeOrder = derivativeOrder;
            if (!std::isfinite(parameter))
            {
                return eval;
            }
            const auto basis = EvaluateLocalNurbsBasis(
                curve.knots_, curve.controlPoints_.size(), curve.degree_, parameter);
            NurbsWeightedSum sum(curve.controlPoints_, curve.weights_, basis.value,
                                 basis.firstControlIndex, basis.value.size());
            for (std::size_t i = 0; i < basis.value.size(); ++i)
            {
                const std::size_t index = basis.firstControlIndex + i;
                sum.Add(curve.controlPoints_[index], curve.weights_[index], basis.value[i],
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
    }
}  // namespace Geometry
