#pragma once

#include <cmath>
#include <sstream>
#include <string>

#include "Types/Geometry3d/SCPoint3.h"
#include "Types/Geometry3d/SCVector3.h"

namespace Geometry
{
    struct SCLine3d
    {
        SCPoint3d origin{};
        SCVector3d direction{};

        [[nodiscard]] static SCLine3d FromOriginAndDirection(const SCPoint3d& originValue,
                                                              const SCVector3d& directionValue)
        {
            return SCLine3d{originValue, directionValue};
        }

        [[nodiscard]] bool IsValid(double eps = kDefaultEpsilon) const
        {
            const double length = direction.Length();
            return std::isfinite(eps) && eps >= 0.0 && origin.IsValid() && direction.IsValid()
                && std::isfinite(length) && length > eps;
        }

        [[nodiscard]] SCPoint3d PointAt(double parameter) const
        {
            return origin + direction * parameter;
        }

        [[nodiscard]] std::string DebugString() const
        {
            std::ostringstream stream;
            stream << "SCLine3d{origin=" << origin.DebugString() << ", direction=" << direction.DebugString() << "}";
            return stream.str();
        }
    };

}  // namespace Geometry
