#pragma once

#include <cmath>
#include <optional>
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
            return Inverse(eps).has_value();
        }

        [[nodiscard]] SCMatrix3d Transpose() const
        {
            return SCMatrix3d{m00, m10, m20, m01, m11, m21, m02, m12, m22};
        }

        // Returns std::nullopt for invalid input, invalid tolerance, or a
        // determinant within the supplied singularity tolerance.
        [[nodiscard]] std::optional<SCMatrix3d> Inverse(double eps = kDefaultEpsilon) const
        {
            if (!IsValid() || !std::isfinite(eps) || eps < 0.0)
            {
                return std::nullopt;
            }

            struct ScaledValue
            {
                double mantissa;
                double correction;
                int exponent;
            };
            const auto normalize = [](double value, int exponent) -> ScaledValue
            {
                if (value == 0.0)
                {
                    return {0.0, 0.0, 0};
                }
                int adjustment = 0;
                return {std::frexp(value, &adjustment), 0.0, exponent + adjustment};
            };
            // Keep the rounded high part and its residual when products cancel.
            const auto compensated = [&](double high, double low, int exponent) -> ScaledValue
            {
                const double sum = high + low;
                const double part = sum - high;
                const double remainder = (high - (sum - part)) + (low - part);
                if (sum == 0.0)
                {
                    return normalize(remainder, exponent);
                }
                int adjustment = 0;
                const double mantissa = std::frexp(sum, &adjustment);
                return {mantissa, std::ldexp(remainder, -adjustment), exponent + adjustment};
            };
            const auto multiply = [&](ScaledValue lhs, ScaledValue rhs) -> ScaledValue
            {
                const double product = lhs.mantissa * rhs.mantissa;
                const double error = std::fma(lhs.mantissa, rhs.mantissa, -product);
                const double correction = error + lhs.mantissa * rhs.correction +
                                          lhs.correction * rhs.mantissa + lhs.correction * rhs.correction;
                return compensated(product, correction, lhs.exponent + rhs.exponent);
            };
            const auto add = [&](ScaledValue lhs, ScaledValue rhs) -> ScaledValue
            {
                if (lhs.mantissa == 0.0)
                {
                    return rhs;
                }
                if (rhs.mantissa == 0.0)
                {
                    return lhs;
                }
                const int exponent = lhs.exponent > rhs.exponent ? lhs.exponent : rhs.exponent;
                const double left = std::ldexp(lhs.mantissa, lhs.exponent - exponent);
                const double right = std::ldexp(rhs.mantissa, rhs.exponent - exponent);
                const double sum = left + right;
                const double part = sum - left;
                const double error = (left - (sum - part)) + (right - part);
                const double correction = error + std::ldexp(lhs.correction, lhs.exponent - exponent) +
                                          std::ldexp(rhs.correction, rhs.exponent - exponent);
                return compensated(sum, correction, exponent);
            };
            const auto difference = [&](ScaledValue lhs, ScaledValue rhs) -> ScaledValue
            {
                return add(lhs, {-rhs.mantissa, -rhs.correction, rhs.exponent});
            };
            const auto value = [&](double input) -> ScaledValue { return normalize(input, 0); };
            const auto minor = [&](double a, double b, double c, double d) -> ScaledValue
            {
                return difference(multiply(value(a), value(b)), multiply(value(c), value(d)));
            };

            const ScaledValue c00 = minor(m11, m22, m12, m21);
            const ScaledValue c01 = minor(m12, m20, m10, m22);
            const ScaledValue c02 = minor(m10, m21, m11, m20);
            const ScaledValue c10 = minor(m02, m21, m01, m22);
            const ScaledValue c11 = minor(m00, m22, m02, m20);
            const ScaledValue c12 = minor(m01, m20, m00, m21);
            const ScaledValue c20 = minor(m01, m12, m02, m11);
            const ScaledValue c21 = minor(m02, m10, m00, m12);
            const ScaledValue c22 = minor(m00, m11, m01, m10);
            ScaledValue terms[3]{multiply(value(m00), c00), multiply(value(m01), c01),
                                 multiply(value(m02), c02)};
            // Cancel large opposing terms before adding a small determinant residual.
            for (int i = 0; i < 2; ++i)
            {
                for (int j = i + 1; j < 3; ++j)
                {
                    if (terms[j].exponent > terms[i].exponent ||
                        (terms[j].exponent == terms[i].exponent &&
                         std::abs(terms[j].mantissa) > std::abs(terms[i].mantissa)))
                    {
                        const ScaledValue term = terms[i];
                        terms[i] = terms[j];
                        terms[j] = term;
                    }
                }
            }
            const ScaledValue determinant = add(add(terms[0], terms[1]), terms[2]);
            if (determinant.mantissa == 0.0)
            {
                return std::nullopt;
            }
            if (eps > 0.0)
            {
                const ScaledValue threshold = value(eps);
                if (determinant.exponent < threshold.exponent ||
                    (determinant.exponent == threshold.exponent &&
                     (std::abs(determinant.mantissa) < threshold.mantissa ||
                      (std::abs(determinant.mantissa) == threshold.mantissa &&
                       std::copysign(1.0, determinant.mantissa) * determinant.correction <= 0.0))))
                {
                    return std::nullopt;
                }
            }

            const auto divide = [&](ScaledValue numerator) -> double
            {
                if (numerator.mantissa == 0.0)
                {
                    return 0.0;
                }
                double quotient = numerator.mantissa / determinant.mantissa;
                const double remainder = std::fma(-quotient, determinant.mantissa, numerator.mantissa) +
                                         numerator.correction - quotient * determinant.correction;
                quotient += remainder / determinant.mantissa;
                return std::ldexp(quotient, numerator.exponent - determinant.exponent);
            };
            const SCMatrix3d result{divide(c00), divide(c10), divide(c20),
                                    divide(c01), divide(c11), divide(c21),
                                    divide(c02), divide(c12), divide(c22)};
            return result.IsValid() ? std::optional<SCMatrix3d>{result} : std::nullopt;
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
