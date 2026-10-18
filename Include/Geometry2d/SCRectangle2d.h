#pragma once

#include <array>
#include <cmath>
#include <string>

#include "Export/GeometryExport.h"
#include "Geometry2d/SCPolygon2d.h"
#include "Support/Epsilon.h"
#include "Types/Geometry2d/SCBox2.h"
#include "Types/Geometry2d/SCPoint2.h"

namespace Geometry
{
    class GEOMETRY_API SCRectangle2d
    {
    public:
        SCPoint2d center{};
        double width{0.0};
        double height{0.0};
        double angleRadians{0.0};

        SCRectangle2d() = default;
        SCRectangle2d(const SCPoint2d& center, double width, double height, double angleRadians = 0.0);

        [[nodiscard]] bool IsValid() const;
        [[nodiscard]] double Area() const;
        [[nodiscard]] double Perimeter() const;
        [[nodiscard]] std::array<SCPoint2d, 4> Corners() const;
        [[nodiscard]] SCBox2d Bounds() const;
        [[nodiscard]] SCPolygon2d ToPolygon() const;
        [[nodiscard]] bool AlmostEquals(const SCRectangle2d& other,
                                        double distanceEps = Geometry::kDefaultEpsilon,
                                        double angleEps = Geometry::kDefaultEpsilon) const
        {
            return std::isfinite(distanceEps) && distanceEps >= 0.0 &&
                   std::isfinite(angleEps) && angleEps >= 0.0 &&
                   center.AlmostEquals(other.center, distanceEps) &&
                   std::abs(width - other.width) <= distanceEps &&
                   std::abs(height - other.height) <= distanceEps &&
                   std::abs(angleRadians - other.angleRadians) <= angleEps;
        }
        [[nodiscard]] bool operator==(const SCRectangle2d& other) const = default;
        [[nodiscard]] std::string DebugString() const;
    };
}  // namespace Geometry
