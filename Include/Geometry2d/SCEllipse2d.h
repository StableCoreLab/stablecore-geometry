#pragma once
#include <cmath>
#include <string>
#include "Export/GeometryExport.h"
#include "Geometry2d/SCPolygon2d.h"
#include "Support/Epsilon.h"

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
        [[nodiscard]] bool AlmostEquals(const SCEllipse2d& other,
                                        double distanceEps = Geometry::kDefaultEpsilon,
                                        double angleEps = Geometry::kDefaultAngleEpsilon) const
        {
            return std::isfinite(distanceEps) && distanceEps >= 0.0 &&
                   std::isfinite(angleEps) && angleEps >= 0.0 &&
                   center.AlmostEquals(other.center, distanceEps) &&
                   std::abs(majorRadius - other.majorRadius) <= distanceEps &&
                   std::abs(minorRadius - other.minorRadius) <= distanceEps &&
                   std::abs(angleRadians - other.angleRadians) <= angleEps;
        }
        [[nodiscard]] bool operator==(const SCEllipse2d& other) const = default;
        [[nodiscard]] std::string DebugString() const;
    };
}  // namespace Geometry
