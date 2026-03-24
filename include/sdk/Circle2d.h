#pragma once

#include <string>

#include "export/GeometryExport.h"
#include "sdk/GeometryTypes.h"
#include "sdk/Polygon2d.h"

namespace geometry::sdk
{
class GEOMETRY_API Circle2d
{
public:
    Point2d center{};
    double radius{0.0};

    Circle2d() = default;
    Circle2d(const Point2d& centerValue, double radiusValue);

    [[nodiscard]] bool IsValid() const;
    [[nodiscard]] double Area() const;
    [[nodiscard]] double Perimeter() const;
    [[nodiscard]] Box2d Bounds() const;
    [[nodiscard]] Point2d PointAtAngle(double angleRadians) const;
    [[nodiscard]] Polygon2d ToPolygon(std::size_t segmentCount = 64) const;
    [[nodiscard]] std::string DebugString() const;
};
} // namespace geometry::sdk
