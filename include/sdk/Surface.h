#pragma once

#include <memory>

#include "export/GeometryExport.h"
#include "sdk/GeometryTypes.h"

namespace geometry::sdk
{
class GEOMETRY_API Surface
{
public:
    virtual ~Surface() = default;

    [[nodiscard]] virtual bool IsValid(const GeometryTolerance3d& tolerance = {}) const = 0;
    [[nodiscard]] virtual Intervald URange() const = 0;
    [[nodiscard]] virtual Intervald VRange() const = 0;
    [[nodiscard]] virtual Point3d PointAt(double u, double v) const = 0;
    [[nodiscard]] virtual SurfaceEval3d Evaluate(double u, double v, int derivativeOrder = 1) const = 0;
    [[nodiscard]] virtual Box3d Bounds() const = 0;
    [[nodiscard]] virtual std::unique_ptr<Surface> Clone() const = 0;

    [[nodiscard]] double StartU() const
    {
        return URange().min;
    }

    [[nodiscard]] double EndU() const
    {
        return URange().max;
    }

    [[nodiscard]] double StartV() const
    {
        return VRange().min;
    }

    [[nodiscard]] double EndV() const
    {
        return VRange().max;
    }
};
} // namespace geometry::sdk
