#pragma once

#include <sstream>
#include <string>

#include "Types/Geometry3d/SCDirection3d.h"
#include "Types/Geometry3d/SCPoint3.h"

namespace Geometry
{
    struct SCRay3d
    {
        SCPoint3d origin{};
        SCDirection3d direction{};

        [[nodiscard]] static SCRay3d FromOriginAndDirection(const SCPoint3d& originValue,
                                                             const SCDirection3d& directionValue)
        {
            return SCRay3d{originValue, directionValue};
        }

        [[nodiscard]] bool IsValid(double eps = kDefaultEpsilon) const
        {
            return origin.IsValid() && direction.IsValid(eps);
        }

        [[nodiscard]] SCPoint3d PointAt(double parameter) const
        {
            return origin + direction.Vector() * parameter;
        }

        [[nodiscard]] std::string DebugString() const
        {
            std::ostringstream stream;
            stream << "SCRay3d{origin=" << origin.DebugString() << ", direction=" << direction.DebugString() << "}";
            return stream.str();
        }
    };

}  // namespace Geometry
