#include "sdk/GeometryShapeOps.h"

#include <algorithm>
#include <cmath>
#include <vector>
#include <utility>

#include "common/Epsilon.h"
#include "sdk/GeometryAlgorithms.h"

namespace geometry::sdk
{
namespace
{
[[nodiscard]] double SignedArea(const Polyline2d& ring)
{
    if (!ring.IsClosed() || ring.PointCount() < 3)
    {
        return 0.0;
    }

    double sum = 0.0;
    for (std::size_t i = 0; i < ring.PointCount(); ++i)
    {
        const Point2d current = ring.PointAt(i);
        const Point2d next = ring.PointAt((i + 1) % ring.PointCount());
        sum += current.x * next.y - next.x * current.y;
    }

    return 0.5 * sum;
}

[[nodiscard]] Point2d RingCentroid(const Polyline2d& ring)
{
    const double signedArea = SignedArea(ring);
    if (std::abs(signedArea) <= geometry::kDefaultEpsilon)
    {
        return Point2d{};
    }

    double cx = 0.0;
    double cy = 0.0;
    for (std::size_t i = 0; i < ring.PointCount(); ++i)
    {
        const Point2d current = ring.PointAt(i);
        const Point2d next = ring.PointAt((i + 1) % ring.PointCount());
        const double cross = current.x * next.y - next.x * current.y;
        cx += (current.x + next.x) * cross;
        cy += (current.y + next.y) * cross;
    }

    return Point2d{cx / (6.0 * signedArea), cy / (6.0 * signedArea)};
}

} // namespace

double Perimeter(const Polygon2d& polygon)
{
    if (!polygon.IsValid())
    {
        return 0.0;
    }

    double total = polygon.OuterRing().Length();
    for (std::size_t i = 0; i < polygon.HoleCount(); ++i)
    {
        total += polygon.HoleAt(i).Length();
    }

    return total;
}

RingOrientation2d Orientation(const Polyline2d& ring)
{
    const double signedArea = SignedArea(ring);
    if (std::abs(signedArea) <= geometry::kDefaultEpsilon)
    {
        return RingOrientation2d::Unknown;
    }

    return signedArea > 0.0 ? RingOrientation2d::CounterClockwise : RingOrientation2d::Clockwise;
}

bool IsClockwise(const Polyline2d& ring)
{
    return Orientation(ring) == RingOrientation2d::Clockwise;
}

bool IsCounterClockwise(const Polyline2d& ring)
{
    return Orientation(ring) == RingOrientation2d::CounterClockwise;
}

Point2d Centroid(const Polygon2d& polygon)
{
    if (!polygon.IsValid())
    {
        return Point2d{};
    }

    const Polyline2d outer = polygon.OuterRing();
    const double outerArea = SignedArea(outer);
    if (std::abs(outerArea) <= geometry::kDefaultEpsilon)
    {
        return Point2d{};
    }

    Point2d outerCentroid = RingCentroid(outer);
    double weightedX = outerCentroid.x * outerArea;
    double weightedY = outerCentroid.y * outerArea;
    double totalSignedArea = outerArea;

    for (std::size_t i = 0; i < polygon.HoleCount(); ++i)
    {
        const Polyline2d hole = polygon.HoleAt(i);
        const double holeArea = SignedArea(hole);
        const Point2d holeCentroid = RingCentroid(hole);
        weightedX += holeCentroid.x * holeArea;
        weightedY += holeCentroid.y * holeArea;
        totalSignedArea += holeArea;
    }

    if (std::abs(totalSignedArea) <= geometry::kDefaultEpsilon)
    {
        return Point2d{};
    }

    return Point2d{weightedX / totalSignedArea, weightedY / totalSignedArea};
}

LineSegment2d Reverse(const LineSegment2d& segment)
{
    return LineSegment2d(segment.endPoint, segment.startPoint);
}

ArcSegment2d Reverse(const ArcSegment2d& segment)
{
    return ArcSegment2d(segment.center, segment.radius, segment.EndAngle(), -segment.sweepAngle);
}

Polyline2d Reverse(const Polyline2d& polyline)
{
    std::vector<Point2d> points;
    points.reserve(polyline.PointCount());
    for (std::size_t i = 0; i < polyline.PointCount(); ++i)
    {
        points.push_back(polyline.PointAt(polyline.PointCount() - 1 - i));
    }

    return Polyline2d(
        std::move(points),
        polyline.IsClosed() ? PolylineClosure::Closed : PolylineClosure::Open);
}

Polyline2d Close(const Polyline2d& polyline)
{
    if (polyline.IsClosed())
    {
        return polyline;
    }

    std::vector<Point2d> points;
    points.reserve(polyline.PointCount());
    for (std::size_t i = 0; i < polyline.PointCount(); ++i)
    {
        points.push_back(polyline.PointAt(i));
    }

    return Polyline2d(std::move(points), PolylineClosure::Closed);
}
} // namespace geometry::sdk
