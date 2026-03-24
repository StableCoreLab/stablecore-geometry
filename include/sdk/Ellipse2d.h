#pragma once

#include <string>

#include "export/GeometryExport.h"
#include "sdk/GeometryTypes.h"
#include "sdk/Polygon2d.h"

namespace geometry::sdk
{
class GEOMETRY_API Ellipse2d
{
public:
    Point2d center{};
    double majorRadius{0.0};
    double minorRadius{0.0};
    double angleRadians{0.0};

    Ellipse2d() = default;
    Ellipse2d(
        const Point2d& centerValue,
        double majorRadiusValue,
        double minorRadiusValue,
        double angleValue = 0.0);

    [[nodiscard]] bool IsValid() const;
    [[nodiscard]] double Area() const;
    [[nodiscard]] double Perimeter() const;
    [[nodiscard]] Point2d PointAtAngle(double angleRadiansValue) const;
    [[nodiscard]] Box2d Bounds(std::size_t segmentCount = 128) const;
    [[nodiscard]] Polygon2d ToPolygon(std::size_t segmentCount = 128) const;
    [[nodiscard]] std::string DebugString() const;
};
} // namespace geometry::sdk
