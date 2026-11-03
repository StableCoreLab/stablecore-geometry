#pragma once

#include <cmath>
#include <cstddef>
#include <string>
#include "Export/GeometryExport.h"
#include "Geometry2d/SCPolygon2d.h"
#include "Support/Epsilon.h"

namespace Geometry
{
    class GEOMETRY_API SCCircle2d
    {
    public:
        SCPoint2d center{};
        double radius{0.0};

        SCCircle2d() = default;
        SCCircle2d(const SCPoint2d& center, double radius);

        [[nodiscard]] bool IsValid() const;
        [[nodiscard]] double Area() const;
        [[nodiscard]] double Perimeter() const;
        [[nodiscard]] SCBox2d Bounds() const;
        [[nodiscard]] SCPoint2d PointAtAngle(double angleRadians) const;
        // 等参数角采样；至少 3 段，默认 64 段，不提供弦误差保证。
        [[nodiscard]] SCPolygon2d ToPolygon(std::size_t segmentCount = 64) const;
        [[nodiscard]] bool AlmostEquals(const SCCircle2d& other, double eps = Geometry::kDefaultEpsilon) const
        {
            return std::isfinite(eps) && eps >= 0.0 && center.AlmostEquals(other.center, eps) &&
                   std::abs(radius - other.radius) <= eps;
        }
        [[nodiscard]] bool operator==(const SCCircle2d& other) const = default;
        [[nodiscard]] std::string DebugString() const;
    };
}  // namespace Geometry
