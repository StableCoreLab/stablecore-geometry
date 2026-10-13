#pragma once

#include <cmath>
#include <optional>
#include <sstream>
#include <string>

#include "Support/Epsilon.h"
#include "Types/Geometry2d/SCPoint2.h"
#include "Types/Geometry2d/SCVector2.h"

namespace Geometry
{
    // 2x2 matrix, row-major: { m00 m01 ; m10 m11 }.
    struct SCMatrix2d
    {
        double m00{0.0};
        double m01{0.0};
        double m10{0.0};
        double m11{0.0};

        [[nodiscard]] static constexpr SCMatrix2d Identity()
        {
            return SCMatrix2d{1.0, 0.0, 0.0, 1.0};
        }

        [[nodiscard]] static constexpr SCMatrix2d FromRows(const SCVector2d& row0, const SCVector2d& row1)
        {
            return SCMatrix2d{row0.x, row0.y, row1.x, row1.y};
        }

        // 2D rotation matrix (counterclockwise positive).
        [[nodiscard]] static SCMatrix2d Rotation(double angleRadians)
        {
            const double c = std::cos(angleRadians);
            const double s = std::sin(angleRadians);
            return SCMatrix2d{c, -s, s, c};
        }

        [[nodiscard]] bool IsValid() const
        {
            return std::isfinite(m00) && std::isfinite(m01) && std::isfinite(m10) && std::isfinite(m11);
        }

        [[nodiscard]] double Determinant() const
        {
            return m00 * m11 - m01 * m10;
        }

        [[nodiscard]] bool AlmostEquals(const SCMatrix2d& other, double eps = kDefaultEpsilon) const
        {
            return std::isfinite(eps) && eps >= 0.0 && std::abs(m00 - other.m00) <= eps &&
                   std::abs(m01 - other.m01) <= eps && std::abs(m10 - other.m10) <= eps &&
                   std::abs(m11 - other.m11) <= eps;
        }

        [[nodiscard]] bool IsInvertible(double eps = kDefaultEpsilon) const
        {
            return Inverse(eps).has_value();
        }

        [[nodiscard]] SCMatrix2d Transpose() const
        {
            return SCMatrix2d{m00, m10, m01, m11};
        }

        // Returns std::nullopt for invalid input, invalid tolerance, or a
        // determinant within the supplied singularity tolerance.
        [[nodiscard]] std::optional<SCMatrix2d> Inverse(double eps = kDefaultEpsilon) const
        {
            if (!IsValid() || !std::isfinite(eps) || eps < 0.0)
            {
                return std::nullopt;
            }
            const double det = Determinant();
            if (!std::isfinite(det) || std::abs(det) <= eps)
            {
                return std::nullopt;
            }
            const SCMatrix2d result{m11 / det, -m01 / det, -m10 / det, m00 / det};
            return result.IsValid() ? std::optional<SCMatrix2d>{result} : std::nullopt;
        }

        [[nodiscard]] SCVector2d operator*(const SCVector2d& vector) const
        {
            return SCVector2d{m00 * vector.x + m01 * vector.y,
                              m10 * vector.x + m11 * vector.y};
        }

        [[nodiscard]] SCPoint2d operator*(const SCPoint2d& point) const
        {
            return SCPoint2d{m00 * point.x + m01 * point.y,
                             m10 * point.x + m11 * point.y};
        }

        [[nodiscard]] std::string DebugString() const
        {
            std::ostringstream stream;
            stream << "SCMatrix2d{[" << m00 << ", " << m01 << "], [" << m10 << ", " << m11 << "]}";
            return stream.str();
        }

        [[nodiscard]] constexpr bool operator==(const SCMatrix2d& other) const = default;
        [[nodiscard]] constexpr bool operator!=(const SCMatrix2d& other) const = default;
    };

    [[nodiscard]] inline SCMatrix2d operator*(const SCMatrix2d& lhs, const SCMatrix2d& rhs)
    {
        return SCMatrix2d{lhs.m00 * rhs.m00 + lhs.m01 * rhs.m10,
                          lhs.m00 * rhs.m01 + lhs.m01 * rhs.m11,
                          lhs.m10 * rhs.m00 + lhs.m11 * rhs.m10,
                          lhs.m10 * rhs.m01 + lhs.m11 * rhs.m11};
    }
}  // namespace Geometry
