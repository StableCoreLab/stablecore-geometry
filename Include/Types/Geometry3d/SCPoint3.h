#pragma once

#include <array>
#include <string>

#include "Support/Epsilon.h"
#include "Types/SCVecStorageOps.h"

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
            return Detail::CoordinateStorageIsValid(std::array<T, 3>{x, y, z});
        }

        [[nodiscard]] constexpr bool AlmostEquals(const SCVec3Storage& other, double eps = kDefaultEpsilon) const
        {
            return Detail::CoordinateStorageAlmostEquals(
                std::array<T, 3>{x, y, z}, std::array<T, 3>{other.x, other.y, other.z}, eps);
        }

        [[nodiscard]] std::string DebugString(const char* typeName) const
        {
            return Detail::CoordinateStorageDebugString(typeName, std::array<T, 3>{x, y, z});
        }

        [[nodiscard]] constexpr bool operator==(const SCVec3Storage& other) const = default;
        [[nodiscard]] constexpr bool operator!=(const SCVec3Storage& other) const = default;
    };

    template <typename T>
    struct SCPoint3 : SCVec3Storage<T>
    {
        using SCVec3Storage<T>::SCVec3Storage;
        using SCVec3Storage<T>::AlmostEquals;

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
    using SCPoint3i = SCPoint3<int>;
}  // namespace Geometry
