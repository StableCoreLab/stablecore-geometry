#pragma once

#include <cmath>
#include <sstream>
#include <string>

#include "Support/Epsilon.h"
#include "Types/Geometry3d/SCPoint3.h"
#include "Types/Geometry3d/SCVector3.h"

namespace Geometry
{
    struct SCMatrix3d
    {
        double m00{0.0};
        double m01{0.0};
        double m02{0.0};
        double m10{0.0};
        double m11{0.0};
        double m12{0.0};
        double m20{0.0};
        double m21{0.0};
        double m22{0.0};

        [[nodiscard]] static constexpr SCMatrix3d Identity()
        {
            return SCMatrix3d{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        }

        [[nodiscard]] static constexpr SCMatrix3d FromRows(const SCVector3d& row0,
                                                           const SCVector3d& row1,
                                                           const SCVector3d& row2)
        {
            return SCMatrix3d{row0.x, row0.y, row0.z, row1.x, row1.y, row1.z, row2.x, row2.y, row2.z};
        }

        [[nodiscard]] bool IsValid() const
        {
            return std::isfinite(m00) && std::isfinite(m01) && std::isfinite(m02) && std::isfinite(m10) &&
                   std::isfinite(m11) && std::isfinite(m12) && std::isfinite(m20) && std::isfinite(m21) &&
                   std::isfinite(m22);
        }

        [[nodiscard]] double Determinant() const
        {
            return m00 * (m11 * m22 - m12 * m21) - m01 * (m10 * m22 - m12 * m20) + m02 * (m10 * m21 - m11 * m20);
        }

        [[nodiscard]] bool IsInvertible(double eps = kDefaultEpsilon) const
        {
            return IsValid() && std::abs(Determinant()) > eps;
        }

        [[nodiscard]] SCMatrix3d Transpose() const
        {
            return SCMatrix3d{m00, m10, m20, m01, m11, m21, m02, m12, m22};
        }

        [[nodiscard]] SCMatrix3d Inverse(double eps = kDefaultEpsilon) const
        {
            const double det = Determinant();
            if (!IsValid() || std::abs(det) <= eps)
            {
                return {};
            }

            const double invDet = 1.0 / det;
            return SCMatrix3d{(m11 * m22 - m12 * m21) * invDet,
                              (m02 * m21 - m01 * m22) * invDet,
                              (m01 * m12 - m02 * m11) * invDet,
                              (m12 * m20 - m10 * m22) * invDet,
                              (m00 * m22 - m02 * m20) * invDet,
                              (m02 * m10 - m00 * m12) * invDet,
                              (m10 * m21 - m11 * m20) * invDet,
                              (m01 * m20 - m00 * m21) * invDet,
                              (m00 * m11 - m01 * m10) * invDet};
        }

        [[nodiscard]] SCVector3d operator*(const SCVector3d& vector) const
        {
            return SCVector3d{m00 * vector.x + m01 * vector.y + m02 * vector.z,
                              m10 * vector.x + m11 * vector.y + m12 * vector.z,
                              m20 * vector.x + m21 * vector.y + m22 * vector.z};
        }

        [[nodiscard]] SCPoint3d operator*(const SCPoint3d& point) const
        {
            return SCPoint3d{m00 * point.x + m01 * point.y + m02 * point.z,
                             m10 * point.x + m11 * point.y + m12 * point.z,
                             m20 * point.x + m21 * point.y + m22 * point.z};
        }

        [[nodiscard]] std::string DebugString() const
        {
            std::ostringstream stream;
            stream << "SCMatrix3d{"
                   << "[" << m00 << ", " << m01 << ", " << m02 << "], "
                   << "[" << m10 << ", " << m11 << ", " << m12 << "], "
                   << "[" << m20 << ", " << m21 << ", " << m22 << "]}";
            return stream.str();
        }
    };

    [[nodiscard]] inline SCMatrix3d operator*(const SCMatrix3d& lhs, const SCMatrix3d& rhs)
    {
        return SCMatrix3d{lhs.m00 * rhs.m00 + lhs.m01 * rhs.m10 + lhs.m02 * rhs.m20,
                          lhs.m00 * rhs.m01 + lhs.m01 * rhs.m11 + lhs.m02 * rhs.m21,
                          lhs.m00 * rhs.m02 + lhs.m01 * rhs.m12 + lhs.m02 * rhs.m22,
                          lhs.m10 * rhs.m00 + lhs.m11 * rhs.m10 + lhs.m12 * rhs.m20,
                          lhs.m10 * rhs.m01 + lhs.m11 * rhs.m11 + lhs.m12 * rhs.m21,
                          lhs.m10 * rhs.m02 + lhs.m11 * rhs.m12 + lhs.m12 * rhs.m22,
                          lhs.m20 * rhs.m00 + lhs.m21 * rhs.m10 + lhs.m22 * rhs.m20,
                          lhs.m20 * rhs.m01 + lhs.m21 * rhs.m11 + lhs.m22 * rhs.m21,
                          lhs.m20 * rhs.m02 + lhs.m21 * rhs.m12 + lhs.m22 * rhs.m22};
    }

}  // namespace Geometry
