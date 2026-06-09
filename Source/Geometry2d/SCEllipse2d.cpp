#include "Geometry2d/SCEllipse2d.h"

#include <cmath>
#include <numbers>
#include <sstream>
#include <vector>

#include "Geometry2d/SCPolyline2d.h"

namespace Geometry
{
    SCEllipse2d::SCEllipse2d(const SCPoint2d& center, double majorRadius, double minorRadius, double angleRadians)
        : center(center), majorRadius(majorRadius), minorRadius(minorRadius), angleRadians(angleRadians)
    {
    }

    bool SCEllipse2d::IsValid() const
    {
        return center.IsValid() && std::isfinite(majorRadius) && std::isfinite(minorRadius) &&
               std::isfinite(angleRadians) && majorRadius > 0.0 && minorRadius > 0.0;
    }

    double SCEllipse2d::Area() const
    {
        return IsValid() ? std::numbers::pi_v<double> * majorRadius * minorRadius : 0.0;
    }

    double SCEllipse2d::Perimeter() const
    {
        if (!IsValid())
        {
            return 0.0;
        }
        const double h = std::pow((majorRadius - minorRadius), 2.0) / std::pow((majorRadius + minorRadius), 2.0);
        return std::numbers::pi_v<double> * (majorRadius + minorRadius) *
               (1.0 + (3.0 * h) / (10.0 + std::sqrt(4.0 - 3.0 * h)));
    }

    SCPoint2d SCEllipse2d::PointAtAngle(double angleRadians) const
    {
        const double x = majorRadius * std::cos(angleRadians);
        const double y = minorRadius * std::sin(angleRadians);
        const double c = std::cos(this->angleRadians);
        const double s = std::sin(this->angleRadians);
        return SCPoint2d{center.x + x * c - y * s, center.y + x * s + y * c};
    }

    SCBox2d SCEllipse2d::Bounds(std::size_t segmentCount) const
    {
        if (!IsValid())
        {
            return {};
        }
        return ToPolygon(segmentCount).Bounds();
    }

    SCPolygon2d SCEllipse2d::ToPolygon(std::size_t segmentCount) const
    {
        if (!IsValid() || segmentCount < 8)
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

    std::string SCEllipse2d::DebugString() const
    {
        std::ostringstream stream;
        stream << "SCEllipse2d{center=" << center.DebugString() << ", majorRadius=" << majorRadius
               << ", minorRadius=" << minorRadius << ", angleRadians=" << angleRadians << "}";
        return stream.str();
    }
}  // namespace Geometry
