#pragma once

#include <cmath>
#include <type_traits>

#include "Support/Epsilon.h"
#include "Types/Geometry2d/SCPoint2.h"

namespace Geometry
{
    template <typename T>
    using SCVectorLengthType = std::conditional_t<std::is_floating_point_v<T>, T, double>;

    template <typename T>
    struct SCVector2 : SCVec2Storage<T>
    {
        using SCVec2Storage<T>::SCVec2Storage;
        using LengthType = SCVectorLengthType<T>;

        [[nodiscard]] static constexpr SCVector2 FromXY(T xValue, T yValue)
        {
            return SCVector2(xValue, yValue);
        }

        [[nodiscard]] constexpr auto LengthSquared() const -> LengthType
        {
            return static_cast<LengthType>(this->x) * static_cast<LengthType>(this->x) +
                   static_cast<LengthType>(this->y) * static_cast<LengthType>(this->y);
        }

        [[nodiscard]] auto Length() const -> LengthType
        {
            using std::sqrt;
            return sqrt(LengthSquared());
        }

        [[nodiscard]] SCVector2 Normalized(double eps = kDefaultEpsilon) const
        {
            SCVector2<LengthType> normalized{};
            return TryNormalized(normalized, eps)
                       ? SCVector2{static_cast<T>(normalized.x), static_cast<T>(normalized.y)}
                       : SCVector2{};
        }

        template <typename TNormalized = LengthType>
        [[nodiscard]] bool TryNormalized(SCVector2<TNormalized>& normalized, double eps = kDefaultEpsilon) const
        {
            if constexpr (!std::is_floating_point_v<TNormalized>)
            {
                (void)normalized;
                (void)eps;
                return false;
            }
            else
            {
                if (!this->IsValid() || !std::isfinite(eps) || eps < 0.0)
                {
                    return false;
                }
                const LengthType length = Length();
                if (!std::isfinite(length) || length <= static_cast<LengthType>(eps))
                {
                    return false;
                }
                normalized = SCVector2<TNormalized>{static_cast<TNormalized>(static_cast<LengthType>(this->x) / length),
                                                    static_cast<TNormalized>(static_cast<LengthType>(this->y) / length)};
                return true;
            }
        }

        [[nodiscard]] std::string DebugString() const
        {
            return this->SCVec2Storage<T>::DebugString("SCVector2");
        }
    };

    template <typename T>
    [[nodiscard]] constexpr SCVector2<T> operator+(const SCVector2<T>& lhs, const SCVector2<T>& rhs)
    {
        return SCVector2<T>(lhs.x + rhs.x, lhs.y + rhs.y);
    }

    template <typename T>
    [[nodiscard]] constexpr SCVector2<T> operator-(const SCVector2<T>& lhs, const SCVector2<T>& rhs)
    {
        return SCVector2<T>(lhs.x - rhs.x, lhs.y - rhs.y);
    }

    template <typename T>
    [[nodiscard]] constexpr SCVector2<T> operator-(const SCVector2<T>& vector)
    {
        return SCVector2<T>(-vector.x, -vector.y);
    }

    template <typename T, typename Scalar, typename = std::enable_if_t<std::is_arithmetic_v<Scalar>>>
    [[nodiscard]] constexpr SCVector2<T> operator*(const SCVector2<T>& vector, Scalar scalar)
    {
        return SCVector2<T>(static_cast<T>(vector.x * scalar), static_cast<T>(vector.y * scalar));
    }

    template <typename Scalar, typename T, typename = std::enable_if_t<std::is_arithmetic_v<Scalar>>>
    [[nodiscard]] constexpr SCVector2<T> operator*(Scalar scalar, const SCVector2<T>& vector)
    {
        return vector * scalar;
    }

    template <typename T, typename Scalar, typename = std::enable_if_t<std::is_arithmetic_v<Scalar>>>
    [[nodiscard]] constexpr SCVector2<T> operator/(const SCVector2<T>& vector, Scalar scalar)
    {
        return SCVector2<T>(static_cast<T>(vector.x / scalar), static_cast<T>(vector.y / scalar));
    }

    template <typename T>
    [[nodiscard]] constexpr SCVector2<T> operator-(const SCPoint2<T>& lhs, const SCPoint2<T>& rhs)
    {
        return SCVector2<T>(lhs.x - rhs.x, lhs.y - rhs.y);
    }

    template <typename TLhs, typename TRhs>
    [[nodiscard]] constexpr auto Dot(const SCVector2<TLhs>& lhs, const SCVector2<TRhs>& rhs)
        -> std::common_type_t<TLhs, TRhs>
    {
        using ResultType = std::common_type_t<TLhs, TRhs>;
        return static_cast<ResultType>(lhs.x) * static_cast<ResultType>(rhs.x) +
               static_cast<ResultType>(lhs.y) * static_cast<ResultType>(rhs.y);
    }

    template <typename TLhs, typename TRhs>
    [[nodiscard]] constexpr auto Cross(const SCVector2<TLhs>& lhs, const SCVector2<TRhs>& rhs)
        -> std::common_type_t<TLhs, TRhs>
    {
        using ResultType = std::common_type_t<TLhs, TRhs>;
        return static_cast<ResultType>(lhs.x) * static_cast<ResultType>(rhs.y) -
               static_cast<ResultType>(lhs.y) * static_cast<ResultType>(rhs.x);
    }

    using SCVector2d = SCVector2<double>;
    using SCVector2i = SCVector2<int>;
}  // namespace Geometry
