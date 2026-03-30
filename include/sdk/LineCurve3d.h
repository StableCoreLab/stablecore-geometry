#pragma once

#include <algorithm>
#include <memory>

#include "sdk/Curve3d.h"

namespace geometry::sdk
{
class GEOMETRY_API LineCurve3d final : public Curve3d
{
public:
    LineCurve3d() = default;
    LineCurve3d(Line3d line, Intervald parameterRange) : line_(line), parameterRange_(parameterRange) {}

    [[nodiscard]] static LineCurve3d FromLine(const Line3d& line, Intervald parameterRange = Intervald{-1.0, 1.0})
    {
        return LineCurve3d(line, parameterRange);
    }

    [[nodiscard]] bool IsValid(const GeometryTolerance3d& tolerance = {}) const override
    {
        return line_.IsValid(tolerance.distanceEpsilon) && parameterRange_.IsValid();
    }

    [[nodiscard]] Intervald ParameterRange() const override
    {
        return parameterRange_;
    }

    [[nodiscard]] bool IsClosed(const GeometryTolerance3d& tolerance = {}) const override
    {
        if (!IsValid(tolerance))
        {
            return false;
        }

        return PointAt(StartT()).AlmostEquals(PointAt(EndT()), tolerance.distanceEpsilon);
    }

    [[nodiscard]] bool IsPeriodic(const GeometryTolerance3d& tolerance = {}) const override
    {
        (void)tolerance;
        return false;
    }

    [[nodiscard]] Point3d PointAt(double parameter) const override
    {
        return line_.PointAt(parameter);
    }

    [[nodiscard]] CurveEval3d Evaluate(double parameter, int derivativeOrder = 1) const override
    {
        derivativeOrder = std::clamp(derivativeOrder, 0, 2);
        CurveEval3d eval{};
        eval.point = PointAt(parameter);
        eval.derivativeOrder = derivativeOrder;
        if (derivativeOrder >= 1)
        {
            eval.firstDerivative = line_.direction;
        }
        if (derivativeOrder >= 2)
        {
            eval.secondDerivative = Vector3d{};
        }
        return eval;
    }

    [[nodiscard]] Box3d Bounds() const override
    {
        Box3d bounds{};
        bounds.ExpandToInclude(PointAt(StartT()));
        bounds.ExpandToInclude(PointAt(EndT()));
        return bounds;
    }

    [[nodiscard]] std::unique_ptr<Curve3d> Clone() const override
    {
        return std::make_unique<LineCurve3d>(*this);
    }

    [[nodiscard]] Line3d Line() const
    {
        return line_;
    }

private:
    Line3d line_{};
    Intervald parameterRange_{-1.0, 1.0};
};
} // namespace geometry::sdk
