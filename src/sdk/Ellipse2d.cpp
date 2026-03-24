#include "sdk/Ellipse2d.h"

#include <cmath>
#include <numbers>
#include <sstream>
#include <vector>

#include "sdk/Polyline2d.h"

namespace geometry::sdk
{
Ellipse2d::Ellipse2d(
    const Point2d& centerValue,
    double majorRadiusValue,
    double minorRadiusValue,
    double angleValue)
    : center(centerValue),
      majorRadius(majorRadiusValue),
      minorRadius(minorRadiusValue),
      angleRadians(angleValue)
{
}

bool Ellipse2d::IsValid() const
{
    return center.IsValid() && std::isfinite(majorRadius) && std::isfinite(minorRadius) &&
           std::isfinite(angleRadians) && majorRadius > 0.0 && minorRadius > 0.0;
}

double Ellipse2d::Area() const
{
    return IsValid() ? std::numbers::pi_v<double> * majorRadius * minorRadius : 0.0;
}

double Ellipse2d::Perimeter() const
{
    if (!IsValid())
    {
        return 0.0;
    }
    const double h = std::pow((majorRadius - minorRadius), 2.0) / std::pow((majorRadius + minorRadius), 2.0);
    return std::numbers::pi_v<double> * (majorRadius + minorRadius) *
           (1.0 + (3.0 * h) / (10.0 + std::sqrt(4.0 - 3.0 * h)));
}

Point2d Ellipse2d::PointAtAngle(double angleRadiansValue) const
{
    const double x = majorRadius * std::cos(angleRadiansValue);
    const double y = minorRadius * std::sin(angleRadiansValue);
    const double c = std::cos(angleRadians);
    const double s = std::sin(angleRadians);
    return Point2d{center.x + x * c - y * s, center.y + x * s + y * c};
}

Box2d Ellipse2d::Bounds(std::size_t segmentCount) const
{
    if (!IsValid())
    {
        return {};
    }
    Box2d box;
    const Polygon2d polygon = ToPolygon(segmentCount);
    return polygon.Bounds();
}

Polygon2d Ellipse2d::ToPolygon(std::size_t segmentCount) const
{
    if (!IsValid() || segmentCount < 8)
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

std::string Ellipse2d::DebugString() const
{
    std::ostringstream stream;
    stream << "Ellipse2d{center=" << center.DebugString()
           << ", majorRadius=" << majorRadius
           << ", minorRadius=" << minorRadius
           << ", angleRadians=" << angleRadians << "}";
    return stream.str();
}
} // namespace geometry::sdk
