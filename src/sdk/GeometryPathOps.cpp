#include "sdk/GeometryPathOps.h"

#include <algorithm>
#include <vector>

#include "algorithm/Predicate2.h"
#include "sdk/GeometryBoolean.h"
#include "sdk/GeometryProjection.h"
#include "sdk/GeometryRelation.h"
#include "sdk/GeometryShapeOps.h"

namespace geometry::sdk
{
namespace
{
[[nodiscard]] double SideValue(const Point2d& point, const LineSegment2d& line)
{
    return Cross(line.endPoint - line.startPoint, point - line.startPoint);
}

[[nodiscard]] Point2d IntersectInfiniteLines(const Point2d& a1, const Point2d& a2, const Point2d& b1, const Point2d& b2)
{
    const Vector2d r = a2 - a1;
    const Vector2d s = b2 - b1;
    const double denom = Cross(r, s);
    if (std::abs(denom) <= geometry::kDefaultEpsilon)
    {
        return a2;
    }
    const double t = Cross(b1 - a1, s) / denom;
    return a1 + r * t;
}

[[nodiscard]] Polyline2d ClipPolylineLength(const Polyline2d& polyline, double startLength, double endLength)
{
    if (!polyline.IsValid() || endLength < startLength)
    {
        return {};
    }

    const double total = Length(polyline);
    startLength = std::clamp(startLength, 0.0, total);
    endLength = std::clamp(endLength, 0.0, total);
    if (endLength < startLength + geometry::kDefaultEpsilon)
    {
        return {};
    }

    std::vector<Point2d> points;
    points.push_back(PointAtLength(LineSegment2d(polyline.PointAt(0), polyline.PointAt(1)), 0.0));
    points.clear();

    double cursor = 0.0;
    for (std::size_t i = 0; i + 1 < polyline.PointCount(); ++i)
    {
        const LineSegment2d segment(polyline.PointAt(i), polyline.PointAt(i + 1));
        const double segLen = segment.Length();
        const double segStart = cursor;
        const double segEnd = cursor + segLen;

        if (segEnd < startLength - geometry::kDefaultEpsilon)
        {
            cursor = segEnd;
            continue;
        }
        if (segStart > endLength + geometry::kDefaultEpsilon)
        {
            break;
        }

        const double localStart = std::max(0.0, startLength - segStart);
        const double localEnd = std::min(segLen, endLength - segStart);
        const Point2d startPoint = segment.PointAtLength(localStart, true);
        const Point2d endPoint = segment.PointAtLength(localEnd, true);
        if (points.empty() || !points.back().AlmostEquals(startPoint))
        {
            points.push_back(startPoint);
        }
        if (!points.back().AlmostEquals(endPoint))
        {
            points.push_back(endPoint);
        }

        cursor = segEnd;
    }

    return Polyline2d(std::move(points), PolylineClosure::Open);
}

[[nodiscard]] Polygon2d ClipPolygonHalfPlane(const Polygon2d& polygon, const LineSegment2d& cutter, bool keepLeft)
{
    if (!polygon.IsValid() || polygon.HoleCount() != 0)
    {
        return {};
    }

    std::vector<Point2d> input;
    input.reserve(polygon.OuterRing().PointCount());
    for (std::size_t i = 0; i < polygon.OuterRing().PointCount(); ++i)
    {
        input.push_back(polygon.OuterRing().PointAt(i));
    }

    std::vector<Point2d> output;
    if (input.empty())
    {
        return {};
    }

    Point2d prev = input.back();
    double prevSide = SideValue(prev, cutter);
    for (const Point2d& current : input)
    {
        const double currentSide = SideValue(current, cutter);
        const bool prevInside = keepLeft ? prevSide >= -geometry::kDefaultEpsilon : prevSide <= geometry::kDefaultEpsilon;
        const bool currentInside =
            keepLeft ? currentSide >= -geometry::kDefaultEpsilon : currentSide <= geometry::kDefaultEpsilon;

        if (currentInside)
        {
            if (!prevInside)
            {
                output.push_back(IntersectInfiniteLines(prev, current, cutter.startPoint, cutter.endPoint));
            }
            output.push_back(current);
        }
        else if (prevInside)
        {
            output.push_back(IntersectInfiniteLines(prev, current, cutter.startPoint, cutter.endPoint));
        }

        prev = current;
        prevSide = currentSide;
    }

    if (output.size() < 3)
    {
        return {};
    }

    return Polygon2d(Polyline2d(std::move(output), PolylineClosure::Closed));
}
} // namespace

Polyline2d SubPolyline(const Polyline2d& polyline, double startLength, double endLength)
{
    return ClipPolylineLength(polyline, startLength, endLength);
}

PolygonCutResult2d CutPolygon(const Polygon2d& polygon, const LineSegment2d& cutter, double eps)
{
    (void)eps;
    PolygonCutResult2d result;
    const Polygon2d left = ClipPolygonHalfPlane(polygon, cutter, true);
    const Polygon2d right = ClipPolygonHalfPlane(polygon, cutter, false);
    if (!left.IsValid() || !right.IsValid())
    {
        return result;
    }
    result.success = true;
    result.left.Add(Polygon2d(left));
    result.right.Add(Polygon2d(right));
    return result;
}

MultiPolygon2d BuildMultiPolygonByLines(const MultiPolyline2d& polylines, double eps)
{
    (void)eps;
    MultiPolygon2d result;
    for (std::size_t i = 0; i < polylines.Count(); ++i)
    {
        const Polyline2d& polyline = polylines[i];
        if (polyline.IsClosed())
        {
            Polygon2d polygon(polyline);
            if (polygon.IsValid())
            {
                result.Add(Polygon2d(polygon));
            }
        }
    }
    return result;
}
} // namespace geometry::sdk
