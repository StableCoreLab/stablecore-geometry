#pragma once

#include <cmath>
#include <sstream>
#include <string>
#include <type_traits>

#include "common/Epsilon.h"

namespace geometry
{
template <typename T>
struct Vector3;

template <typename T>
struct Vec3Storage
{
    T x{};
    T y{};
    T z{};

    constexpr Vec3Storage() = default;
    constexpr Vec3Storage(T xValue, T yValue, T zValue) : x(xValue), y(yValue), z(zValue) {}

    constexpr void Set(T xValue, T yValue, T zValue)
    {
        x = xValue;
        y = yValue;
        z = zValue;
    }

    [[nodiscard]] constexpr bool IsValid() const
    {
        if constexpr (std::is_floating_point_v<T>)
        {
            return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
        }
        return true;
    }

    [[nodiscard]] constexpr bool AlmostEquals(
        const Vec3Storage& other,
        double eps = kDefaultEpsilon) const
    {
        if constexpr (std::is_floating_point_v<T>)
        {
            return std::abs(static_cast<double>(x - other.x)) <= eps &&
                   std::abs(static_cast<double>(y - other.y)) <= eps &&
                   std::abs(static_cast<double>(z - other.z)) <= eps;
        }

        (void)eps;
        return *this == other;
    }

    [[nodiscard]] std::string DebugString(const char* typeName) const
    {
        std::ostringstream stream;
        stream << typeName << "{x=" << x << ", y=" << y << ", z=" << z << "}";
        return stream.str();
    }

    [[nodiscard]] constexpr bool operator==(const Vec3Storage& other) const = default;
    [[nodiscard]] constexpr bool operator!=(const Vec3Storage& other) const = default;
};

template <typename T>
struct Point3 : Vec3Storage<T>
{
    using Vec3Storage<T>::Vec3Storage;

    [[nodiscard]] static constexpr Point3 FromXYZ(T xValue, T yValue, T zValue)
    {
        return Point3(xValue, yValue, zValue);
    }

    [[nodiscard]] std::string DebugString() const
    {
        return this->Vec3Storage<T>::DebugString("Point3");
    }
};

template <typename T>
[[nodiscard]] constexpr Point3<T> operator+(const Point3<T>& point, const Vector3<T>& vector)
{
    return Point3<T>(point.x + vector.x, point.y + vector.y, point.z + vector.z);
}

template <typename T>
[[nodiscard]] constexpr Point3<T> operator-(const Point3<T>& point, const Vector3<T>& vector)
{
    return Point3<T>(point.x - vector.x, point.y - vector.y, point.z - vector.z);
}

using Point3d = Point3<double>;
using Point3i = Point3<int>;
} // namespace geometry
