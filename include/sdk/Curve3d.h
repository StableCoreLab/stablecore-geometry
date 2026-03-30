#pragma once

#include <memory>

#include "export/GeometryExport.h"
#include "sdk/GeometryTypes.h"

namespace geometry::sdk
{
class GEOMETRY_API Curve3d
{
public:
    virtual ~Curve3d() = default;

    [[nodiscard]] virtual bool IsValid(const GeometryTolerance3d& tolerance = {}) const = 0;
    [[nodiscard]] virtual Intervald ParameterRange() const = 0;
    [[nodiscard]] virtual bool IsClosed(const GeometryTolerance3d& tolerance = {}) const = 0;
    [[nodiscard]] virtual bool IsPeriodic(const GeometryTolerance3d& tolerance = {}) const = 0;
    [[nodiscard]] virtual Point3d PointAt(double parameter) const = 0;
    [[nodiscard]] virtual CurveEval3d Evaluate(double parameter, int derivativeOrder = 1) const = 0;
    [[nodiscard]] virtual Box3d Bounds() const = 0;
    [[nodiscard]] virtual std::unique_ptr<Curve3d> Clone() const = 0;

    [[nodiscard]] double StartT() const
    {
        return ParameterRange().min;
    }

    [[nodiscard]] double EndT() const
    {
        return ParameterRange().max;
    }
};
} // namespace geometry::sdk
