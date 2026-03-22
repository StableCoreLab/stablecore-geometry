#pragma once

#include "export/GeometryExport.h"
#include "sdk/GeometryTypes.h"

namespace geometry::sdk
{
struct GEOMETRY_API ProjectionResult2d
{
    GeoPoint2d point{};
    double parameter{0.0};
    double distanceSquared{0.0};
    bool isOnSegment{false};
};

[[nodiscard]] GEOMETRY_API double DistanceSquared(const GeoPoint2d& lhs, const GeoPoint2d& rhs);
[[nodiscard]] GEOMETRY_API double Distance(const GeoPoint2d& lhs, const GeoPoint2d& rhs);
[[nodiscard]] GEOMETRY_API ProjectionResult2d ProjectPointToSegment(
    const GeoPoint2d& point,
    const GeoPoint2d& segmentStart,
    const GeoPoint2d& segmentEnd,
    bool clampToSegment = true);
[[nodiscard]] GEOMETRY_API bool Contains(
    const GeoBox2d& box,
    const GeoPoint2d& point,
    double eps = 1e-9);
[[nodiscard]] GEOMETRY_API bool Intersects(
    const GeoBox2d& lhs,
    const GeoBox2d& rhs,
    double eps = 1e-9);
} // namespace geometry::sdk
