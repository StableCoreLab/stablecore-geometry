#pragma once

#include "sdk/GeometryTypes.h"
#include "types/Box2.h"
#include "types/Point2.h"
#include "types/Vector2.h"

namespace geometry::sdk::detail
{
[[nodiscard]] inline geometry::Point2d ToInternal(const GeoPoint2d& point)
{
    return geometry::Point2d(point.x, point.y);
}

[[nodiscard]] inline geometry::Vector2d ToInternal(const GeoVector2d& vector)
{
    return geometry::Vector2d(vector.x, vector.y);
}

[[nodiscard]] inline geometry::Box2d ToInternal(const GeoBox2d& box)
{
    if (!box.isValid)
    {
        return geometry::Box2d();
    }

    return geometry::Box2d(ToInternal(box.minPoint), ToInternal(box.maxPoint));
}

[[nodiscard]] inline GeoPoint2d ToSdk(const geometry::Point2d& point)
{
    return GeoPoint2d{point.x, point.y};
}

[[nodiscard]] inline GeoVector2d ToSdk(const geometry::Vector2d& vector)
{
    return GeoVector2d{vector.x, vector.y};
}

[[nodiscard]] inline GeoBox2d ToSdk(const geometry::Box2d& box)
{
    if (!box.IsValid())
    {
        return GeoBox2d{};
    }

    return GeoBox2d{
        ToSdk(box.GetMinPoint()),
        ToSdk(box.GetMaxPoint()),
        true};
}
} // namespace geometry::sdk::detail
