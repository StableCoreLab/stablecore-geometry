#pragma once

#include <cmath>
#include <type_traits>

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

    using SCVector2d = SCVector2<double>;
    using SCVector2i = SCVector2<int>;
}  // namespace Geometry
