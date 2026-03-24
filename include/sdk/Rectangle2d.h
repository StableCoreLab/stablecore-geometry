#pragma once

#include <array>
#include <string>

#include "export/GeometryExport.h"
#include "sdk/GeometryTypes.h"
#include "sdk/Polygon2d.h"

namespace geometry::sdk
{
class GEOMETRY_API Rectangle2d
{
public:
    Point2d center{};
    double width{0.0};
    double height{0.0};
    double angleRadians{0.0};

    Rectangle2d() = default;
    Rectangle2d(const Point2d& centerValue, double widthValue, double heightValue, double angleValue = 0.0);

    [[nodiscard]] bool IsValid() const;
    [[nodiscard]] double Area() const;
    [[nodiscard]] double Perimeter() const;
    [[nodiscard]] std::array<Point2d, 4> Corners() const;
    [[nodiscard]] Box2d Bounds() const;
    [[nodiscard]] Polygon2d ToPolygon() const;
    [[nodiscard]] std::string DebugString() const;
};
} // namespace geometry::sdk
