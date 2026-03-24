#include "sdk/Rectangle2d.h"

#include <array>
#include <cmath>
#include <sstream>
#include <vector>

#include "sdk/Polyline2d.h"

namespace geometry::sdk
{
namespace
{
[[nodiscard]] Point2d RotateLocal(const Point2d& center, double angle, double x, double y)
{
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return Point2d{center.x + x * c - y * s, center.y + x * s + y * c};
}
} // namespace

Rectangle2d::Rectangle2d(const Point2d& centerValue, double widthValue, double heightValue, double angleValue)
    : center(centerValue), width(widthValue), height(heightValue), angleRadians(angleValue)
{
}

bool Rectangle2d::IsValid() const
{
    return center.IsValid() && std::isfinite(width) && std::isfinite(height) && std::isfinite(angleRadians) &&
           width > 0.0 && height > 0.0;
}

double Rectangle2d::Area() const
{
    return IsValid() ? width * height : 0.0;
}

double Rectangle2d::Perimeter() const
{
    return IsValid() ? 2.0 * (width + height) : 0.0;
}

std::array<Point2d, 4> Rectangle2d::Corners() const
{
    const double hx = width * 0.5;
    const double hy = height * 0.5;
    return {
        RotateLocal(center, angleRadians, -hx, -hy),
        RotateLocal(center, angleRadians, hx, -hy),
        RotateLocal(center, angleRadians, hx, hy),
        RotateLocal(center, angleRadians, -hx, hy)};
}

Box2d Rectangle2d::Bounds() const
{
    if (!IsValid())
    {
        return {};
    }

    Box2d box;
    for (const Point2d& corner : Corners())
    {
        box.ExpandToInclude(corner);
    }
    return box;
}

Polygon2d Rectangle2d::ToPolygon() const
{
    if (!IsValid())
    {
        return {};
    }
    const auto corners = Corners();
    return Polygon2d(Polyline2d(std::vector<Point2d>(corners.begin(), corners.end()), PolylineClosure::Closed));
}

std::string Rectangle2d::DebugString() const
{
    std::ostringstream stream;
    stream << "Rectangle2d{center=" << center.DebugString()
           << ", width=" << width
           << ", height=" << height
           << ", angleRadians=" << angleRadians << "}";
    return stream.str();
}
} // namespace geometry::sdk
