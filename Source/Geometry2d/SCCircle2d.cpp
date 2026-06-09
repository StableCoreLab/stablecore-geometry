#include "Geometry2d/SCCircle2d.h"

#include <cmath>
#include <numbers>
#include <sstream>
#include <vector>

#include "Geometry2d/SCPolyline2d.h"

namespace Geometry
{
    SCCircle2d::SCCircle2d(const SCPoint2d& center, double radius) : center(center), radius(radius)
    {
    }

    bool SCCircle2d::IsValid() const
    {
        return center.IsValid() && std::isfinite(radius) && radius > 0.0;
    }

    double SCCircle2d::Area() const
    {
        return IsValid() ? std::numbers::pi_v<double> * radius * radius : 0.0;
    }

    double SCCircle2d::Perimeter() const
    {
        return IsValid() ? 2.0 * std::numbers::pi_v<double> * radius : 0.0;
    }

    SCBox2d SCCircle2d::Bounds() const
    {
        if (!IsValid())
        {
            return {};
        }

        return SCBox2d::FromMinMax(SCPoint2d{center.x - radius, center.y - radius},
                                 SCPoint2d{center.x + radius, center.y + radius});
    }

    SCPoint2d SCCircle2d::PointAtAngle(double angleRadians) const
    {
        return SCPoint2d{center.x + radius * std::cos(angleRadians), center.y + radius * std::sin(angleRadians)};
    }

    SCPolygon2d SCCircle2d::ToPolygon(std::size_t segmentCount) const
    {
        if (!IsValid() || segmentCount < 3)
        {
            return {};
        }

        std::vector<SCPoint2d> points;
        points.reserve(segmentCount);
        for (std::size_t i = 0; i < segmentCount; ++i)
        {
            const double t =
                2.0 * std::numbers::pi_v<double> * static_cast<double>(i) / static_cast<double>(segmentCount);
            points.push_back(PointAtAngle(t));
        }
        return SCPolygon2d(SCPolyline2d(std::move(points), SCPolylineClosure::Closed));
    }

    std::string SCCircle2d::DebugString() const
    {
        std::ostringstream stream;
        stream << "SCCircle2d{center=" << center.DebugString() << ", radius=" << radius << "}";
        return stream.str();
    }
}  // namespace Geometry
