#include "sdk/Circle2d.h"

#include <cmath>
#include <numbers>
#include <sstream>
#include <vector>

#include "sdk/Polyline2d.h"

namespace geometry::sdk
{
Circle2d::Circle2d(const Point2d& centerValue, double radiusValue)
    : center(centerValue), radius(radiusValue)
{
}

bool Circle2d::IsValid() const
{
    return center.IsValid() && std::isfinite(radius) && radius > 0.0;
}

double Circle2d::Area() const
{
    return IsValid() ? std::numbers::pi_v<double> * radius * radius : 0.0;
}

double Circle2d::Perimeter() const
{
    return IsValid() ? 2.0 * std::numbers::pi_v<double> * radius : 0.0;
}

Box2d Circle2d::Bounds() const
{
    if (!IsValid())
    {
        return {};
    }

    return Box2d::FromMinMax(
        Point2d{center.x - radius, center.y - radius},
        Point2d{center.x + radius, center.y + radius});
}

Point2d Circle2d::PointAtAngle(double angleRadians) const
{
    return Point2d{center.x + radius * std::cos(angleRadians), center.y + radius * std::sin(angleRadians)};
}

Polygon2d Circle2d::ToPolygon(std::size_t segmentCount) const
{
    if (!IsValid() || segmentCount < 3)
    {
        return {};
    }

    std::vector<Point2d> points;
    points.reserve(segmentCount);
    for (std::size_t i = 0; i < segmentCount; ++i)
    {
        const double t = 2.0 * std::numbers::pi_v<double> * static_cast<double>(i) / static_cast<double>(segmentCount);
        points.push_back(PointAtAngle(t));
    }
    return Polygon2d(Polyline2d(std::move(points), PolylineClosure::Closed));
}

std::string Circle2d::DebugString() const
{
    std::ostringstream stream;
    stream << "Circle2d{center=" << center.DebugString() << ", radius=" << radius << "}";
    return stream.str();
}
} // namespace geometry::sdk
