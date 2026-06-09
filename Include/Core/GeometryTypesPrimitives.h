#pragma once

#include <cmath>
#include <sstream>
#include <string>

#include "Core/GeometryTypesFwd.h"
#include "Support/Epsilon.h"
#include "Types/Geometry3d/SCPoint3.h"
#include "Types/Geometry3d/SCVector3.h"

namespace Geometry
{
    struct GEOMETRY_API SCGeometryTolerance3d
    {
        double distanceEpsilon{Geometry::kDefaultEpsilon};
        double angleEpsilon{Geometry::kDefaultEpsilon};
        double parameterEpsilon{Geometry::kDefaultEpsilon};
        double boxPadding{Geometry::kDefaultEpsilon};

        [[nodiscard]] bool IsValid() const
        {
            return std::isfinite(distanceEpsilon) && distanceEpsilon >= 0.0 && std::isfinite(angleEpsilon) &&
                   angleEpsilon >= 0.0 && std::isfinite(parameterEpsilon) && parameterEpsilon >= 0.0 &&
                   std::isfinite(boxPadding) && boxPadding >= 0.0;
        }
    };

    struct GEOMETRY_API SCCurveEval3d
    {
        SCPoint3d point{};
        SCVector3d firstDerivative{};
        SCVector3d secondDerivative{};
        int derivativeOrder{0};

        [[nodiscard]] bool IsValid() const
        {
            if (!point.IsValid() || derivativeOrder < 0 || derivativeOrder > 2)
            {
                return false;
            }

            if (derivativeOrder >= 1 && !firstDerivative.IsValid())
            {
                return false;
            }

            if (derivativeOrder >= 2 && !secondDerivative.IsValid())
            {
                return false;
            }

            return true;
        }
    };

    struct GEOMETRY_API SCSurfaceEval3d
    {
        SCPoint3d point{};
        SCVector3d derivativeU{};
        SCVector3d derivativeV{};
        SCVector3d normal{};
        int derivativeOrder{0};

        [[nodiscard]] bool IsValid(double eps = Geometry::kDefaultEpsilon) const
        {
            if (!point.IsValid() || derivativeOrder < 0 || derivativeOrder > 1)
            {
                return false;
            }

            if (derivativeOrder >= 1)
            {
                if (!derivativeU.IsValid() || !derivativeV.IsValid() || !normal.IsValid())
                {
                    return false;
                }

                if (normal.Length() <= eps)
                {
                    return false;
                }
            }

            return true;
        }
    };
}  // namespace Geometry

