#pragma once

#include <cmath>
#include <type_traits>

#include "common/Epsilon.h"
#include "types/Vector2.h"

namespace geometry
{
template <typename T>
//requires std::is_signed_v<T>
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
        return AbsValue(value) <= static_cast<T>(eps);
    }
    else
    {
        static_assert(std::is_integral_v<T>, "IsZero only supports arithmetic scalar types.");
        (void)eps;
        return value == T{};
    }
}

template <typename T>
[[nodiscard]] constexpr bool IsZero(const Vector2<T>& vector, double eps = kDefaultEpsilon)
{
    if constexpr (std::is_floating_point_v<T>)
    {
        return vector.LengthSquared() <= eps * eps;
    }
    else
    {
        return vector.x == T{} && vector.y == T{};
    }
}

template <typename TLhs, typename TRhs>
[[nodiscard]] constexpr auto Dot(const Vector2<TLhs>& lhs, const Vector2<TRhs>& rhs)
    -> std::common_type_t<TLhs, TRhs>
{
    using ResultType = std::common_type_t<TLhs, TRhs>;
    return static_cast<ResultType>(lhs.x) * static_cast<ResultType>(rhs.x) +
           static_cast<ResultType>(lhs.y) * static_cast<ResultType>(rhs.y);
}

template <typename TLhs, typename TRhs>
[[nodiscard]] constexpr auto Cross(const Vector2<TLhs>& lhs, const Vector2<TRhs>& rhs)
    -> std::common_type_t<TLhs, TRhs>
{
    using ResultType = std::common_type_t<TLhs, TRhs>;
    return static_cast<ResultType>(lhs.x) * static_cast<ResultType>(rhs.y) -
           static_cast<ResultType>(lhs.y) * static_cast<ResultType>(rhs.x);
}

template <typename T>
[[nodiscard]] constexpr bool IsEqual(T lhs, T rhs, double eps = kDefaultEpsilon)
{
    if constexpr (std::is_floating_point_v<T>)
    {
        return AbsValue(static_cast<T>(lhs - rhs)) <= static_cast<T>(eps);
    }
    else
    {
        (void)eps;
        return lhs == rhs;
    }
}

template <typename T>
[[nodiscard]] constexpr bool IsEqual(
    const Point2<T>& lhs,
    const Point2<T>& rhs,
    double eps = kDefaultEpsilon)
{
    return IsEqual(lhs.x, rhs.x, eps) && IsEqual(lhs.y, rhs.y, eps);
}

template <typename T>
[[nodiscard]] constexpr bool IsEqual(
    const Vector2<T>& lhs,
    const Vector2<T>& rhs,
    double eps = kDefaultEpsilon)
{
    return IsEqual(lhs.x, rhs.x, eps) && IsEqual(lhs.y, rhs.y, eps);
}
}
