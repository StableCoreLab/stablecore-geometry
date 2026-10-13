#pragma once

#include <array>
#include <string>

#include "Support/Epsilon.h"
#include "Types/SCVecStorageOps.h"

namespace Geometry
{
    template <typename T>
    struct SCVector2;

    template <typename T>
    struct SCVec2Storage
    {
        T x{};
        T y{};

        constexpr SCVec2Storage() = default;
        constexpr SCVec2Storage(T xValue, T yValue) : x(xValue), y(yValue)
        {
        }

        constexpr void Set(T xValue, T yValue)
        {
            x = xValue;
            y = yValue;
        }

        [[nodiscard]] constexpr bool IsValid() const
        {
            return Detail::CoordinateStorageIsValid(std::array<T, 2>{x, y});
        }

        [[nodiscard]] constexpr bool AlmostEquals(const SCVec2Storage& other, double eps = kDefaultEpsilon) const
        {
            return Detail::CoordinateStorageAlmostEquals(
                std::array<T, 2>{x, y}, std::array<T, 2>{other.x, other.y}, eps);
        }

        [[nodiscard]] std::string DebugString(const char* typeName) const
        {
            return Detail::CoordinateStorageDebugString(typeName, std::array<T, 2>{x, y});
        }

        [[nodiscard]] constexpr bool operator==(const SCVec2Storage& other) const = default;
        [[nodiscard]] constexpr bool operator!=(const SCVec2Storage& other) const = default;
    };

    template <typename T>
    struct SCPoint2 : SCVec2Storage<T>
    {
        using SCVec2Storage<T>::SCVec2Storage;
        using SCVec2Storage<T>::AlmostEquals;

        [[nodiscard]] static constexpr SCPoint2 FromXY(T xValue, T yValue)
        {
            return SCPoint2(xValue, yValue);
        }

        [[nodiscard]] std::string DebugString() const
        {
            return this->SCVec2Storage<T>::DebugString("SCPoint2");
        }
    };

    template <typename T>
    [[nodiscard]] constexpr SCPoint2<T> operator+(const SCPoint2<T>& point, const SCVector2<T>& vector)
    {
        return SCPoint2<T>(point.x + vector.x, point.y + vector.y);
    }

    template <typename T>
    [[nodiscard]] constexpr SCPoint2<T> operator-(const SCPoint2<T>& point, const SCVector2<T>& vector)
    {
        return SCPoint2<T>(point.x - vector.x, point.y - vector.y);
    }

    using SCPoint2d = SCPoint2<double>;
    using SCPoint2i = SCPoint2<int>;
}  // namespace Geometry
