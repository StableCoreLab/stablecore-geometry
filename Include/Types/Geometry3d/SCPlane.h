#pragma once

#include <sstream>
#include <string>

#include "Types/Geometry3d/SCPoint3.h"
#include "Types/Geometry3d/SCVector3.h"

namespace Geometry
{
    struct SCPlane
    {
        SCPoint3d origin{};
        SCVector3d normal{};

        [[nodiscard]] static SCPlane FromPointAndNormal(const SCPoint3d& originValue, const SCVector3d& normalValue)
        {
            return SCPlane{originValue, normalValue};
        }

        [[nodiscard]] bool IsValid(double eps = kDefaultEpsilon) const
        {
            return origin.IsValid() && normal.IsValid() && normal.Length() > eps;
        }

        [[nodiscard]] SCVector3d UnitNormal(double eps = kDefaultEpsilon) const
        {
            return normal.Normalized(eps);
        }

        [[nodiscard]] double SignedDistanceTo(const SCPoint3d& point, double eps = kDefaultEpsilon) const
        {
            const SCVector3d unitNormal = UnitNormal(eps);
            return Dot(point - origin, unitNormal);
        }

        [[nodiscard]] std::string DebugString() const
        {
            std::ostringstream stream;
            stream << "SCPlane{origin=" << origin.DebugString() << ", normal=" << normal.DebugString() << "}";
            return stream.str();
        }
    };

}  // namespace Geometry
