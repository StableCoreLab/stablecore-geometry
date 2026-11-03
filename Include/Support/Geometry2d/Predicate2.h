#pragma once

#include <cmath>
#include <limits>
#include <type_traits>

#include "Support/Epsilon.h"
#include "Types/Geometry2d/SCVector2.h"

namespace Geometry
{
    template <typename T>
    [[nodiscard]] constexpr T AbsValue(T value)
    {
        static_assert(std::is_signed<T>::value, "T must be signed");
        return value < T{} ? -value : value;
    }

    template <typename T>
    [[nodiscard]] constexpr bool IsZero(T value, double eps = kDefaultEpsilon)
    {
        if constexpr (std::is_floating_point_v<T>)
        {
            // 有限范围比较保留 C++20 constexpr；容差不窄化到较小浮点类型。
            return eps >= 0.0 && eps <= (std::numeric_limits<double>::max)() &&
                AbsValue(value) <= (std::numeric_limits<T>::max)() && AbsValue(value) <= eps;
        } else
        {
            static_assert(std::is_integral_v<T>, "IsZero only supports arithmetic scalar types.");
            (void)eps;
            return value == T{};
        }
    }

    template <typename T>
    [[nodiscard]] constexpr bool IsZero(const SCVector2<T>& vector, double eps = kDefaultEpsilon)
    {
        if constexpr (std::is_floating_point_v<T>)
        {
            return std::isfinite(eps) && eps >= 0 && std::isfinite(vector.x) && std::isfinite(vector.y) &&
                std::hypot(vector.x, vector.y) <= eps;
        } else
        {
            return vector.x == T{} && vector.y == T{};
        }
    }

    template <typename T>
    [[nodiscard]] constexpr bool IsEqual(T lhs, T rhs, double eps = kDefaultEpsilon)
    {
        if constexpr (std::is_floating_point_v<T>)
        {
            using Difference = std::common_type_t<T, double>;
            return eps >= 0.0 && eps <= (std::numeric_limits<double>::max)() &&
                AbsValue(lhs) <= (std::numeric_limits<T>::max)() &&
                AbsValue(rhs) <= (std::numeric_limits<T>::max)() &&
                AbsValue(static_cast<Difference>(lhs) - static_cast<Difference>(rhs)) <= eps;
        } else
        {
            (void)eps;
            return lhs == rhs;
        }
    }

    template <typename T>
    [[nodiscard]] constexpr bool IsEqual(const SCPoint2<T>& lhs, const SCPoint2<T>& rhs, double eps = kDefaultEpsilon)
    {
        return IsEqual(lhs.x, rhs.x, eps) && IsEqual(lhs.y, rhs.y, eps);
    }

    template <typename T>
    [[nodiscard]] constexpr bool IsEqual(const SCVector2<T>& lhs, const SCVector2<T>& rhs, double eps = kDefaultEpsilon)
    {
        return IsEqual(lhs.x, rhs.x, eps) && IsEqual(lhs.y, rhs.y, eps);
    }
}  // namespace Geometry
