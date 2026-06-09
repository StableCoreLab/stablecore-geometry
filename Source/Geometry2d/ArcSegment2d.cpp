#include "Geometry2d/SCArcSegment2d.h"

#include <cmath>
#include <memory>
#include <numbers>
#include <sstream>

#include "Support/Epsilon.h"

namespace Geometry
{
    SCArcSegment2d::SCArcSegment2d(const SCPoint2d& center, double radius, double startAngle, double sweepAngle)
        : center(center), radius(radius), startAngle(startAngle), sweepAngle(sweepAngle)
    {
    }

    SCArcSegment2d::SCArcSegment2d(
        const SCPoint2d& center, double radius, double startAngle, double endAngle, SCArcDirection direction)
        : center(center), radius(radius), startAngle(startAngle), sweepAngle(endAngle - startAngle)
    {
        if (direction == SCArcDirection::CounterClockwise)
        {
            while (sweepAngle < 0.0)
            {
                sweepAngle += 2.0 * std::numbers::pi_v<double>;
            }
        } else
        {
            while (sweepAngle > 0.0)
            {
                sweepAngle -= 2.0 * std::numbers::pi_v<double>;
            }
        }
    }

    SCArcSegment2d SCArcSegment2d::FromCenterRadiusStartSweep(const SCPoint2d& center,
                                                          double radius,
                                                          double startAngle,
                                                          double sweepAngle)
    {
        return SCArcSegment2d(center, radius, startAngle, sweepAngle);
    }

    SCSegmentKind2 SCArcSegment2d::Kind() const
    {
        return SCSegmentKind2::Arc;
    }

    bool SCArcSegment2d::IsValid() const
    {
        return center.IsValid() && std::isfinite(radius) && std::isfinite(startAngle) && std::isfinite(sweepAngle) &&
               radius > 0.0 && std::abs(sweepAngle) > Geometry::kArcSegmentDefaultEpsilon &&
               std::abs(sweepAngle) <= 2.0 * std::numbers::pi_v<double> + Geometry::kArcSegmentDefaultEpsilon;
    }

    SCArcDirection SCArcSegment2d::Direction() const
    {
        return sweepAngle >= 0.0 ? SCArcDirection::CounterClockwise : SCArcDirection::Clockwise;
    }

    double SCArcSegment2d::EndAngle() const
    {
        return startAngle + sweepAngle;
    }

    SCPoint2d SCArcSegment2d::StartPoint() const
    {
        return PointAtAngle(startAngle);
    }

    SCPoint2d SCArcSegment2d::EndPoint() const
    {
        return PointAtAngle(EndAngle());
    }

    double SCArcSegment2d::Length() const
    {
        if (!IsValid())
        {
            return 0.0;
        }

        return std::abs(sweepAngle) * radius;
    }

    SCBox2d SCArcSegment2d::Bounds() const
    {
        if (!IsValid())
        {
            return SCBox2d{};
        }

        SCBox2d box;
        box.ExpandToInclude(StartPoint());
        box.ExpandToInclude(EndPoint());

        constexpr double kCriticalAngles[] = {
            0.0, std::numbers::pi_v<double> * 0.5, std::numbers::pi_v<double>, std::numbers::pi_v<double> * 1.5};

        for (double angle : kCriticalAngles)
        {
            if (IsAngleOnArc(angle))
            {
                box.ExpandToInclude(PointAtAngle(angle));
            }
        }

        return box;
    }

    SCPoint2d SCArcSegment2d::PointAt(double parameter) const
    {
        return PointAtLength(parameter * Length(), false);
    }

    SCPoint2d SCArcSegment2d::PointAtLength(double distanceFromStart, bool clampToSegment) const
    {
        if (!IsValid())
        {
            return StartPoint();
        }

        const double length = Length();
        if (length <= 0.0)
        {
            return StartPoint();
        }

        if (clampToSegment)
        {
            if (distanceFromStart < 0.0)
            {
                distanceFromStart = 0.0;
            } else if (distanceFromStart > length)
            {
                distanceFromStart = length;
            }
        }

        return PointAtAngle(startAngle + sweepAngle * (distanceFromStart / length));
    }

    bool SCArcSegment2d::AlmostEquals(const SCArcSegment2d& other, double eps) const
    {
        return center.AlmostEquals(other.center, eps) && std::abs(radius - other.radius) <= eps &&
               std::abs(startAngle - other.startAngle) <= eps && std::abs(sweepAngle - other.sweepAngle) <= eps;
    }

    std::string SCArcSegment2d::DebugString() const
    {
        std::ostringstream stream;
        stream << "SCArcSegment2d{center=" << center.DebugString() << ", radius=" << radius
               << ", startAngle=" << startAngle << ", sweepAngle=" << sweepAngle << "}";
        return stream.str();
    }

    std::unique_ptr<ISCSegment2d> SCArcSegment2d::Clone() const
    {
        return std::make_unique<SCArcSegment2d>(*this);
    }

    double SCArcSegment2d::NormalizeAngle(double angle)
    {
        angle = std::fmod(angle, 2.0 * std::numbers::pi_v<double>);
        if (angle < 0.0)
        {
            angle += 2.0 * std::numbers::pi_v<double>;
        }
        return angle;
    }

    bool SCArcSegment2d::IsAngleOnArc(double candidateAngle) const
    {
        if (sweepAngle >= 0.0)
        {
            const double delta = NormalizeAngle(candidateAngle - startAngle);
            return delta <= sweepAngle + Geometry::kArcSegmentDefaultEpsilon;
        }

        const double delta = NormalizeAngle(startAngle - candidateAngle);
        return delta <= (-sweepAngle) + Geometry::kArcSegmentDefaultEpsilon;
    }

    SCPoint2d SCArcSegment2d::PointAtAngle(double angle) const
    {
        return SCPoint2d{center.x + radius * std::cos(angle), center.y + radius * std::sin(angle)};
    }
}  // namespace Geometry
