#pragma once

#include <optional>
#include <sstream>
#include <string>

#include "Support/Epsilon.h"
#include "Types/Geometry2d/SCMatrix2d.h"
#include "Types/Geometry2d/SCPoint2.h"
#include "Types/Geometry2d/SCVector2.h"

namespace Geometry
{
    // 2D affine transform: linear (2x2) + translation. Points map by
    // linear*point + translation, vectors map by linear only.
    struct SCTransform2d
    {
        SCMatrix2d linear{SCMatrix2d::Identity()};
        SCVector2d translation{};

        [[nodiscard]] static constexpr SCTransform2d Identity()
        {
            return SCTransform2d{};
        }

        [[nodiscard]] static SCTransform2d Translation(const SCVector2d& offset)
        {
            return SCTransform2d{SCMatrix2d::Identity(), offset};
        }

        // Rotate around center by angleRadians (counterclockwise positive):
        // p -> center + R*(p - center). This represents rotation about center
        // only; the axis-grid "anchor alignment + rotation + origin translation"
        // must be built from the linear and translation terms directly.
        [[nodiscard]] static SCTransform2d Rotation(const SCPoint2d& center, double angleRadians)
        {
            const SCMatrix2d r = SCMatrix2d::Rotation(angleRadians);
            return SCTransform2d{r, center - r * center};
        }

        [[nodiscard]] static SCTransform2d Scale(const SCPoint2d& origin, double ratio)
        {
            const SCMatrix2d s{ratio, 0.0, 0.0, ratio};
            return SCTransform2d{s, origin - s * origin};
        }

        [[nodiscard]] bool IsValid() const
        {
            return linear.IsValid() && translation.IsValid();
        }

        [[nodiscard]] SCPoint2d Apply(const SCPoint2d& point) const
        {
            return linear * point + translation;
        }

        [[nodiscard]] SCVector2d Apply(const SCVector2d& vector) const
        {
            return linear * vector;
        }

        // Returns std::nullopt when non-invertible; SCTransform2d{} is the
        // identity transform and must not be used as a failure value.
        [[nodiscard]] std::optional<SCTransform2d> Inverse(double eps = kDefaultEpsilon) const
        {
            if (!linear.IsInvertible(eps))
            {
                return std::nullopt;
            }
            const SCMatrix2d inverseLinear = linear.Inverse(eps);
            return SCTransform2d{inverseLinear, -(inverseLinear * translation)};
        }

        [[nodiscard]] std::string DebugString() const
        {
            std::ostringstream stream;
            stream << "SCTransform2d{linear=" << linear.DebugString()
                   << ", translation=" << translation.DebugString() << "}";
            return stream.str();
        }
    };

    [[nodiscard]] inline SCTransform2d operator*(const SCTransform2d& lhs, const SCTransform2d& rhs)
    {
        // Composition: rhs first, then lhs (matches SCTransform3d).
        return SCTransform2d{lhs.linear * rhs.linear,
                             lhs.linear * rhs.translation + lhs.translation};
    }
}  // namespace Geometry
