#pragma once

#include <cmath>
#include <sstream>
#include <string>
#include <type_traits>

#include "Support/Epsilon.h"

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
            if constexpr (std::is_floating_point_v<T>)
            {
                return std::isfinite(x) && std::isfinite(y);
            }

            return true;
        }

        [[nodiscard]] constexpr bool AlmostEquals(const SCVec2Storage& other, double eps = kDefaultEpsilon) const
        {
            if constexpr (std::is_floating_point_v<T>)
            {
                return std::abs(static_cast<double>(x - other.x)) <= eps &&
                       std::abs(static_cast<double>(y - other.y)) <= eps;
            }

            (void)eps;
            return *this == other;
        }

        [[nodiscard]] std::string DebugString(const char* typeName) const
        {
            std::ostringstream stream;
            stream << typeName << "{x=" << x << ", y=" << y << "}";
            return stream.str();
        }

        [[nodiscard]] constexpr bool operator==(const SCVec2Storage& other) const = default;
        [[nodiscard]] constexpr bool operator!=(const SCVec2Storage& other) const = default;
    };

    template <typename T>
    struct SCPoint2 : SCVec2Storage<T>
    {
        using SCVec2Storage<T>::SCVec2Storage;

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
