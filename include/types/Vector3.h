#pragma once

#include <cmath>
#include <string>
#include <type_traits>

#include "types/Point3.h"

namespace geometry
{
template <typename T>
using VectorLengthType3 = std::conditional_t<std::is_floating_point_v<T>, T, double>;

template <typename T>
struct Vector3 : Vec3Storage<T>
{
    using Vec3Storage<T>::Vec3Storage;
    using LengthType = VectorLengthType3<T>;

    [[nodiscard]] static constexpr Vector3 FromXYZ(T xValue, T yValue, T zValue)
    {
        return Vector3(xValue, yValue, zValue);
    }

    [[nodiscard]] constexpr auto LengthSquared() const -> LengthType
    {
        return static_cast<LengthType>(this->x) * static_cast<LengthType>(this->x) +
               static_cast<LengthType>(this->y) * static_cast<LengthType>(this->y) +
               static_cast<LengthType>(this->z) * static_cast<LengthType>(this->z);
    }

    [[nodiscard]] auto Length() const -> LengthType
    {
        using std::sqrt;
        return sqrt(LengthSquared());
    }

    [[nodiscard]] Vector3 Normalized(double eps = kDefaultEpsilon) const
    {
        const auto length = Length();
        if (length <= eps)
        {
            return Vector3{};
        }
        return *this / length;
    }

    [[nodiscard]] std::string DebugString() const
    {
        return this->Vec3Storage<T>::DebugString("Vector3");
    }
};

template <typename T>
[[nodiscard]] constexpr Vector3<T> operator+(const Vector3<T>& lhs, const Vector3<T>& rhs)
{
    return Vector3<T>(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
}

template <typename T>
[[nodiscard]] constexpr Vector3<T> operator-(const Vector3<T>& lhs, const Vector3<T>& rhs)
{
    return Vector3<T>(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
}

template <typename T>
[[nodiscard]] constexpr Vector3<T> operator-(const Vector3<T>& vector)
{
    return Vector3<T>(-vector.x, -vector.y, -vector.z);
}

template <typename T, typename Scalar, typename = std::enable_if_t<std::is_arithmetic_v<Scalar>>>
[[nodiscard]] constexpr Vector3<T> operator*(const Vector3<T>& vector, Scalar scalar)
{
    return Vector3<T>(
        static_cast<T>(vector.x * scalar),
        static_cast<T>(vector.y * scalar),
        static_cast<T>(vector.z * scalar));
}

template <typename Scalar, typename T, typename = std::enable_if_t<std::is_arithmetic_v<Scalar>>>
[[nodiscard]] constexpr Vector3<T> operator*(Scalar scalar, const Vector3<T>& vector)
{
    return vector * scalar;
}

template <typename T, typename Scalar, typename = std::enable_if_t<std::is_arithmetic_v<Scalar>>>
[[nodiscard]] constexpr Vector3<T> operator/(const Vector3<T>& vector, Scalar scalar)
{
    return Vector3<T>(
        static_cast<T>(vector.x / scalar),
        static_cast<T>(vector.y / scalar),
        static_cast<T>(vector.z / scalar));
}

template <typename T>
[[nodiscard]] constexpr Vector3<T> operator-(const Point3<T>& lhs, const Point3<T>& rhs)
{
    return Vector3<T>(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
}

template <typename T>
[[nodiscard]] constexpr auto Dot(const Vector3<T>& lhs, const Vector3<T>& rhs) -> VectorLengthType3<T>
{
    return static_cast<VectorLengthType3<T>>(lhs.x) * static_cast<VectorLengthType3<T>>(rhs.x) +
           static_cast<VectorLengthType3<T>>(lhs.y) * static_cast<VectorLengthType3<T>>(rhs.y) +
           static_cast<VectorLengthType3<T>>(lhs.z) * static_cast<VectorLengthType3<T>>(rhs.z);
}

template <typename T>
[[nodiscard]] constexpr Vector3<T> Cross(const Vector3<T>& lhs, const Vector3<T>& rhs)
{
    return Vector3<T>(
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x);
}

using Vector3d = Vector3<double>;
using Vector3i = Vector3<int>;
} // namespace geometry
