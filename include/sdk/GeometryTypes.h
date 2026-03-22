#pragma once

#include <cmath>
#include <cstdint>

#include "export/GeometryExport.h"

namespace geometry::sdk
{
struct GEOMETRY_API GeoPoint2d
{
    double x{0.0};
    double y{0.0};

    [[nodiscard]] static constexpr GeoPoint2d FromXY(double xValue, double yValue)
    {
        return GeoPoint2d{xValue, yValue};
    }

    [[nodiscard]] constexpr bool operator==(const GeoPoint2d& other) const = default;
    [[nodiscard]] constexpr bool operator!=(const GeoPoint2d& other) const = default;
};

struct GEOMETRY_API GeoVector2d
{
    double x{0.0};
    double y{0.0};

    [[nodiscard]] static constexpr GeoVector2d FromXY(double xValue, double yValue)
    {
        return GeoVector2d{xValue, yValue};
    }

    [[nodiscard]] constexpr double LengthSquared() const
    {
        return x * x + y * y;
    }

    [[nodiscard]] double Length() const
    {
        using std::sqrt;
        return sqrt(LengthSquared());
    }

    [[nodiscard]] constexpr bool operator==(const GeoVector2d& other) const = default;
    [[nodiscard]] constexpr bool operator!=(const GeoVector2d& other) const = default;
};

struct GEOMETRY_API GeoBox2d
{
    GeoPoint2d minPoint{};
    GeoPoint2d maxPoint{};
    bool isValid{false};
    std::uint8_t reserved0{0};
    std::uint8_t reserved1{0};
    std::uint8_t reserved2{0};

    [[nodiscard]] static constexpr GeoBox2d FromMinMax(
        const GeoPoint2d& minPointValue,
        const GeoPoint2d& maxPointValue)
    {
        return GeoBox2d{
            minPointValue,
            maxPointValue,
            minPointValue.x <= maxPointValue.x && minPointValue.y <= maxPointValue.y};
    }

    [[nodiscard]] constexpr bool IsValid() const
    {
        return isValid && minPoint.x <= maxPoint.x && minPoint.y <= maxPoint.y;
    }

    [[nodiscard]] constexpr double Width() const
    {
        return maxPoint.x - minPoint.x;
    }

    [[nodiscard]] constexpr double Height() const
    {
        return maxPoint.y - minPoint.y;
    }

    [[nodiscard]] constexpr GeoPoint2d Center() const
    {
        return GeoPoint2d{
            (minPoint.x + maxPoint.x) * 0.5,
            (minPoint.y + maxPoint.y) * 0.5};
    }

    [[nodiscard]] constexpr bool operator==(const GeoBox2d& other) const = default;
    [[nodiscard]] constexpr bool operator!=(const GeoBox2d& other) const = default;
};

[[nodiscard]] constexpr GeoPoint2d operator+(const GeoPoint2d& point, const GeoVector2d& vector)
{
    return GeoPoint2d{point.x + vector.x, point.y + vector.y};
}

[[nodiscard]] constexpr GeoPoint2d operator-(const GeoPoint2d& point, const GeoVector2d& vector)
{
    return GeoPoint2d{point.x - vector.x, point.y - vector.y};
}

[[nodiscard]] constexpr GeoVector2d operator-(const GeoPoint2d& lhs, const GeoPoint2d& rhs)
{
    return GeoVector2d{lhs.x - rhs.x, lhs.y - rhs.y};
}

[[nodiscard]] constexpr GeoVector2d operator+(const GeoVector2d& lhs, const GeoVector2d& rhs)
{
    return GeoVector2d{lhs.x + rhs.x, lhs.y + rhs.y};
}

[[nodiscard]] constexpr GeoVector2d operator-(const GeoVector2d& lhs, const GeoVector2d& rhs)
{
    return GeoVector2d{lhs.x - rhs.x, lhs.y - rhs.y};
}

[[nodiscard]] constexpr GeoVector2d operator-(const GeoVector2d& vector)
{
    return GeoVector2d{-vector.x, -vector.y};
}

[[nodiscard]] constexpr GeoVector2d operator*(const GeoVector2d& vector, double scalar)
{
    return GeoVector2d{vector.x * scalar, vector.y * scalar};
}

[[nodiscard]] constexpr GeoVector2d operator*(double scalar, const GeoVector2d& vector)
{
    return vector * scalar;
}

[[nodiscard]] constexpr GeoVector2d operator/(const GeoVector2d& vector, double scalar)
{
    return GeoVector2d{vector.x / scalar, vector.y / scalar};
}

using Point2d = GeoPoint2d;
using Vector2d = GeoVector2d;
using Box2d = GeoBox2d;
}
