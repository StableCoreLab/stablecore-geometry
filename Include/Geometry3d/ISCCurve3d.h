#pragma once

#include <memory>

#include "Core/GeometryTypesPrimitives.h"
#include "Export/GeometryExport.h"
#include "Types/Geometry3d/SCBox3.h"
#include "Types/Geometry3d/SCIntervald.h"

namespace Geometry
{
    class GEOMETRY_API ISCCurve3d
    {
    public:
        virtual ~ISCCurve3d() = default;

        [[nodiscard]] virtual bool IsValid(const SCGeometryTolerance3d& tolerance = {}) const = 0;
        [[nodiscard]] virtual SCIntervald ParameterRange() const = 0;
        [[nodiscard]] virtual bool IsClosed(const SCGeometryTolerance3d& tolerance = {}) const = 0;
        [[nodiscard]] virtual bool IsPeriodic(const SCGeometryTolerance3d& tolerance = {}) const = 0;
        [[nodiscard]] virtual SCPoint3d PointAt(double parameter) const = 0;
        [[nodiscard]] virtual SCCurveEval3d Evaluate(double parameter, int derivativeOrder = 1) const = 0;
        [[nodiscard]] virtual SCBox3d Bounds() const = 0;
        [[nodiscard]] virtual std::unique_ptr<ISCCurve3d> Clone() const = 0;

        [[nodiscard]] double StartParameter() const
        {
            return ParameterRange().min;
        }

        [[nodiscard]] double EndParameter() const
        {
            return ParameterRange().max;
        }

        [[nodiscard]] double StartT() const
        {
            return StartParameter();
        }

        [[nodiscard]] double EndT() const
        {
            return EndParameter();
        }
    };
}  // namespace Geometry

