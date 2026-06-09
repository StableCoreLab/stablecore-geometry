#pragma once
#include <string>
#include "Export/GeometryExport.h"
#include "Geometry2d/SCPolygon2d.h"

namespace Geometry
{
    class GEOMETRY_API SCEllipse2d
    {
    public:
        SCPoint2d center{};
        double majorRadius{0.0};
        double minorRadius{0.0};
        double angleRadians{0.0};

        SCEllipse2d() = default;
        SCEllipse2d(const SCPoint2d& center, double majorRadius, double minorRadius, double angleRadians = 0.0);

        [[nodiscard]] bool IsValid() const;
        [[nodiscard]] double Area() const;
        [[nodiscard]] double Perimeter() const;
        [[nodiscard]] SCPoint2d PointAtAngle(double angleRadians) const;
        [[nodiscard]] SCBox2d Bounds(std::size_t segmentCount = 128) const;
        [[nodiscard]] SCPolygon2d ToPolygon(std::size_t segmentCount = 128) const;
        [[nodiscard]] std::string DebugString() const;
    };
}  // namespace Geometry
