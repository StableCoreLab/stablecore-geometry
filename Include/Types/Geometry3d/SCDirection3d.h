#pragma once

#include <cmath>
#include <numbers>
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
            if (!std::isfinite(eps) || eps < 0.0 || !vector.IsValid())
            {
                return direction;
            }
            const double length = vector.Length();
            if (std::isfinite(length) && length > eps)
            {
                direction.value = vector.Normalized(eps);
            }
            return direction;
        }

        [[nodiscard]] bool IsValid(double eps = kDefaultEpsilon) const
        {
            if (!std::isfinite(eps) || eps < 0.0 || !value.IsValid())
            {
                return false;
            }
            const double length = value.Length();
            return std::isfinite(length) && length > eps;
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
            if (!IsValid() || !other.IsValid() || !std::isfinite(angleEps) || angleEps < 0.0)
            {
                return false;
            }

            if (angleEps >= std::numbers::pi_v<double>)
            {
                return true;
            }

            const SCVector3d first = value.Normalized();
            const SCVector3d second = other.value.Normalized();
            const double chord = std::hypot(first.x - second.x, first.y - second.y, first.z - second.z);
            return chord <= 2.0 * std::sin(angleEps * 0.5);
        }

        [[nodiscard]] std::string DebugString() const
        {
            std::ostringstream stream;
            stream << "SCDirection3d{value=" << value.DebugString() << "}";
            return stream.str();
        }
    };

}  // namespace Geometry
