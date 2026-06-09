#pragma once

#include <algorithm>
#include <memory>

#include "Geometry3d/ISCCurve3d.h"
#include "Types/Geometry3d/SCLine3d.h"

namespace Geometry
{
    class GEOMETRY_API SCLineCurve3d final : public ISCCurve3d
    {
    public:
        SCLineCurve3d() = default;
        SCLineCurve3d(SCLine3d line, SCIntervald parameterRange) : line_(line), parameterRange_(parameterRange)
        {
        }

        [[nodiscard]] static SCLineCurve3d FromLine(const SCLine3d& line, SCIntervald parameterRange = SCIntervald{-1.0, 1.0})
        {
            return SCLineCurve3d(line, parameterRange);
        }

        [[nodiscard]] bool IsValid(const SCGeometryTolerance3d& tolerance = {}) const override
        {
            return line_.IsValid(tolerance.distanceEpsilon) && parameterRange_.IsValid();
        }

        [[nodiscard]] SCIntervald ParameterRange() const override
        {
            return parameterRange_;
        }

        [[nodiscard]] bool IsClosed(const SCGeometryTolerance3d& tolerance = {}) const override
        {
            if (!IsValid(tolerance))
            {
                return false;
            }

            return PointAt(StartParameter()).AlmostEquals(PointAt(EndParameter()), tolerance.distanceEpsilon);
        }

        [[nodiscard]] bool IsPeriodic(const SCGeometryTolerance3d& tolerance = {}) const override
        {
            (void)tolerance;
            return false;
        }

        [[nodiscard]] SCPoint3d PointAt(double parameter) const override
        {
            return line_.PointAt(parameter);
        }

        [[nodiscard]] SCCurveEval3d Evaluate(double parameter, int derivativeOrder = 1) const override
        {
            derivativeOrder = std::clamp(derivativeOrder, 0, 2);
            SCCurveEval3d eval{};
            eval.point = PointAt(parameter);
            eval.derivativeOrder = derivativeOrder;
            if (derivativeOrder >= 1)
            {
                eval.firstDerivative = line_.direction;
            }
            if (derivativeOrder >= 2)
            {
                eval.secondDerivative = SCVector3d{};
            }
            return eval;
        }

        [[nodiscard]] SCBox3d Bounds() const override
        {
            SCBox3d bounds{};
            bounds.ExpandToInclude(PointAt(StartParameter()));
            bounds.ExpandToInclude(PointAt(EndParameter()));
            return bounds;
        }

        [[nodiscard]] std::unique_ptr<ISCCurve3d> Clone() const override
        {
            return std::make_unique<SCLineCurve3d>(*this);
        }

        [[nodiscard]] SCLine3d Line() const
        {
            return line_;
        }

    private:
        SCLine3d line_{};
        SCIntervald parameterRange_{-1.0, 1.0};
    };
}  // namespace Geometry

