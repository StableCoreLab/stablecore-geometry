#pragma once

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

#include "Support/Epsilon.h"
#include "Types/Geometry3d/SCVector3.h"

namespace Geometry
{
    struct SCDirection3d
    {
        SCVector3d value{};

        [[nodiscard]] static SCDirection3d FromVector(const SCVector3d& vector, double eps = kDefaultEpsilon)
        {
            SCDirection3d direction;
            if (vector.IsValid() && vector.Length() > eps)
            {
                direction.value = vector.Normalized(eps);
            }
            return direction;
        }

        [[nodiscard]] bool IsValid(double eps = kDefaultEpsilon) const
        {
            return value.IsValid() && value.Length() > eps;
        }

        [[nodiscard]] SCVector3d Vector() const
        {
            return value;
        }

        [[nodiscard]] SCDirection3d Reverse() const
        {
            return SCDirection3d::FromVector(-value);
        }

        [[nodiscard]] bool AlmostEquals(const SCDirection3d& other, double angleEps = kDefaultEpsilon) const
        {
            if (!IsValid() || !other.IsValid())
            {
                return false;
            }

            const double cosine = std::clamp(Dot(value, other.value), -1.0, 1.0);
            return cosine >= std::cos(angleEps);
        }

        [[nodiscard]] std::string DebugString() const
        {
            std::ostringstream stream;
            stream << "SCDirection3d{value=" << value.DebugString() << "}";
            return stream.str();
        }
    };

}  // namespace Geometry
