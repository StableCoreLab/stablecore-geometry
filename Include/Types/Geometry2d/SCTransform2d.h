#pragma once

#include <cmath>
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

        // 关于过 linePoint、方向为 lineDirection 的直线反射；允许非单位方向。
        // 非法输入、方向长度不大于 eps 或结果不可表示时返回空值。
        [[nodiscard]] static std::optional<SCTransform2d> Reflection(
            const SCPoint2d& linePoint, const SCVector2d& lineDirection,
            double eps = kDefaultEpsilon)
        {
            if (!linePoint.IsValid() || !std::isfinite(eps) || eps <= 0.0)
                return std::nullopt;
            SCVector2d unit;
            if (!lineDirection.TryNormalized(unit, eps)) return std::nullopt;
            const double diagonal = unit.x * unit.x - unit.y * unit.y;
            const double offDiagonal = 2.0 * unit.x * unit.y;
            const SCMatrix2d reflection{diagonal, offDiagonal, offDiagonal, -diagonal};
            const SCTransform2d result{reflection, linePoint - reflection * linePoint};
            return result.IsValid() ? std::optional<SCTransform2d>{result} : std::nullopt;
        }

        [[nodiscard]] bool IsValid() const
        {
            return linear.IsValid() && translation.IsValid();
        }

        [[nodiscard]] bool IsIdentity(double eps = kDefaultEpsilon) const
        {
            return IsValid() && linear.AlmostEquals(SCMatrix2d::Identity(), eps) &&
                   translation.AlmostEquals(SCVector2d{}, eps);
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
            if (!IsValid())
            {
                return std::nullopt;
            }
            const std::optional<SCMatrix2d> inverseLinear = linear.Inverse(eps);
            if (!inverseLinear.has_value())
            {
                return std::nullopt;
            }
            const SCTransform2d result{*inverseLinear, -(*inverseLinear * translation)};
            return result.IsValid() ? std::optional<SCTransform2d>{result} : std::nullopt;
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
