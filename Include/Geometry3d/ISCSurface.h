#pragma once

#include <memory>

#include "Core/GeometryTypesPrimitives.h"
#include "Export/GeometryExport.h"
#include "Types/Geometry3d/SCBox3.h"
#include "Types/Geometry3d/SCIntervald.h"

namespace Geometry
{
    class GEOMETRY_API ISCSurface
    {
    public:
        virtual ~ISCSurface() = default;

        [[nodiscard]] virtual bool IsValid(const SCGeometryTolerance3d& tolerance = {}) const = 0;
        [[nodiscard]] virtual SCIntervald URange() const = 0;
        [[nodiscard]] virtual SCIntervald VRange() const = 0;
        [[nodiscard]] virtual SCPoint3d PointAt(double u, double v) const = 0;
        [[nodiscard]] virtual SCSurfaceEval3d Evaluate(double u, double v, int derivativeOrder = 1) const = 0;
        [[nodiscard]] virtual SCBox3d Bounds() const = 0;
        [[nodiscard]] virtual std::unique_ptr<ISCSurface> Clone() const = 0;

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
}  // namespace Geometry

