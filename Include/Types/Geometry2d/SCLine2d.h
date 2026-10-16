#pragma once

#include <sstream>
#include <string>

#include "Export/GeometryExport.h"
#include "Support/Epsilon.h"
#include "Types/Geometry2d/SCPoint2.h"
#include "Types/Geometry2d/SCVector2.h"

namespace Geometry
{
    struct GEOMETRY_API SCLine2d
    {
        SCPoint2d origin{};
        SCVector2d direction{};

        [[nodiscard]] static SCLine2d FromOriginAndDirection(const SCPoint2d& originValue,
                                                              const SCVector2d& directionValue)
        {
            return SCLine2d{originValue, directionValue};
        }

        [[nodiscard]] static SCLine2d FromTwoPoints(const SCPoint2d& first, const SCPoint2d& second)
        {
            return SCLine2d{first, second - first};
        }

        [[nodiscard]] bool IsValid(double eps = kDefaultEpsilon) const
        {
            return origin.IsValid() && direction.IsValid() && direction.Length() > eps;
        }

        [[nodiscard]] SCPoint2d PointAt(double parameter) const
        {
            return origin + direction * parameter;
        }

        [[nodiscard]] std::string DebugString() const
        {
            std::ostringstream stream;
            stream << "SCLine2d{origin=" << origin.DebugString() << ", direction=" << direction.DebugString() << "}";
            return stream.str();
        }
    };
}  // namespace Geometry
