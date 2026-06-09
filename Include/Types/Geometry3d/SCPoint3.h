#pragma once

#include <cmath>
#include <sstream>
#include <string>
#include <type_traits>

#include "Support/Epsilon.h"

namespace Geometry
{
    template <typename T>
    struct SCVector3;

    template <typename T>
    struct SCVec3Storage
    {
        T x{};
        T y{};
        T z{};

        constexpr SCVec3Storage() = default;
        constexpr SCVec3Storage(T xValue, T yValue, T zValue) : x(xValue), y(yValue), z(zValue)
        {
        }

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

        [[nodiscard]] constexpr bool AlmostEquals(const SCVec3Storage& other, double eps = kDefaultEpsilon) const
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

        [[nodiscard]] constexpr bool operator==(const SCVec3Storage& other) const = default;
        [[nodiscard]] constexpr bool operator!=(const SCVec3Storage& other) const = default;
    };

    template <typename T>
    struct SCPoint3 : SCVec3Storage<T>
    {
        using SCVec3Storage<T>::SCVec3Storage;

        [[nodiscard]] static constexpr SCPoint3 FromXYZ(T xValue, T yValue, T zValue)
        {
            return SCPoint3(xValue, yValue, zValue);
        }

        [[nodiscard]] std::string DebugString() const
        {
            return this->SCVec3Storage<T>::DebugString("SCPoint3");
        }
    };

    template <typename T>
    [[nodiscard]] constexpr SCPoint3<T> operator+(const SCPoint3<T>& point, const SCVector3<T>& vector)
    {
        return SCPoint3<T>(point.x + vector.x, point.y + vector.y, point.z + vector.z);
    }

    template <typename T>
    [[nodiscard]] constexpr SCPoint3<T> operator-(const SCPoint3<T>& point, const SCVector3<T>& vector)
    {
        return SCPoint3<T>(point.x - vector.x, point.y - vector.y, point.z - vector.z);
    }

    using SCPoint3d = SCPoint3<double>;
}  // namespace Geometry
