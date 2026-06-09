#pragma once

#include <cmath>
#include <sstream>
#include <string>

#include "Support/Epsilon.h"
#include "Types/Geometry3d/SCDirection3d.h"
#include "Types/Geometry3d/SCMatrix3d.h"

namespace Geometry
{
    struct SCTransform3d
    {
        SCMatrix3d linear{SCMatrix3d::Identity()};
        SCVector3d translation{};

        [[nodiscard]] static constexpr SCTransform3d Identity()
        {
            return SCTransform3d{};
        }

        [[nodiscard]] static SCTransform3d Translation(const SCVector3d& offset)
        {
            return SCTransform3d{SCMatrix3d::Identity(), offset};
        }

        [[nodiscard]] static SCTransform3d Scale(const SCPoint3d& origin, double ratio)
        {
            const SCMatrix3d scale{ratio, 0.0, 0.0, 0.0, ratio, 0.0, 0.0, 0.0, ratio};
            const SCPoint3d movedOrigin = scale * origin;
            return SCTransform3d{scale, origin - movedOrigin};
        }

        [[nodiscard]] bool IsValid() const
        {
            return linear.IsValid() && translation.IsValid();
        }

        [[nodiscard]] SCPoint3d Apply(const SCPoint3d& point) const
        {
            return linear * point + translation;
        }

        [[nodiscard]] SCVector3d Apply(const SCVector3d& vector) const
        {
            return linear * vector;
        }

        [[nodiscard]] SCDirection3d Apply(const SCDirection3d& direction, double eps = kDefaultEpsilon) const
        {
            return SCDirection3d::FromVector(Apply(direction.Vector()), eps);
        }

        [[nodiscard]] SCTransform3d Inverse(double eps = kDefaultEpsilon) const
        {
            const SCMatrix3d inverseLinear = linear.Inverse(eps);
            if (!inverseLinear.IsInvertible(eps) && !linear.IsInvertible(eps))
            {
                return {};
            }
            return SCTransform3d{inverseLinear, -(inverseLinear * translation)};
        }

        [[nodiscard]] std::string DebugString() const
        {
            std::ostringstream stream;
            stream << "SCTransform3d{linear=" << linear.DebugString() << ", translation=" << translation.DebugString()
                   << "}";
            return stream.str();
        }
    };

    [[nodiscard]] inline SCTransform3d operator*(const SCTransform3d& lhs, const SCTransform3d& rhs)
    {
        return SCTransform3d{lhs.linear * rhs.linear, lhs.linear * rhs.translation + lhs.translation};
    }

}  // namespace Geometry
