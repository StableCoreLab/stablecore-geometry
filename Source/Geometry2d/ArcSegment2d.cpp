#include "Geometry2d/SCArcSegment2d.h"

#include <algorithm>
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
        return IsValidForTolerance(Geometry::kDefaultAngleEpsilon, 0.0,
                                   Geometry::kDefaultAngleEpsilon);
    }

    bool SCArcSegment2d::IsValid(double eps) const
    {
        return std::isfinite(eps) && eps > 0.0 && std::isfinite(radius) && radius > 0.0 &&
               IsValidForTolerance(0.0, eps, eps / radius);
    }

    bool SCArcSegment2d::IsValidForTolerance(double minimumSweep, double minimumLength,
                                              double angleEps) const
    {
        if (!center.IsValid() || !std::isfinite(radius) || !std::isfinite(startAngle) ||
            !std::isfinite(sweepAngle) || !(radius > 0.0) ||
            !(std::abs(sweepAngle) > minimumSweep) ||
            std::abs(sweepAngle) > 2.0 * std::numbers::pi_v<double> + Geometry::kDefaultAngleEpsilon ||
            !StartPoint().IsValid() || !EndPoint().IsValid())
        {
            return false;
        }
        const double length = std::abs(sweepAngle) * radius;
        if (!std::isfinite(length) || length <= minimumLength)
        {
            return false;
        }
        for (double angle : Geometry::kArcBoundsCriticalAngles)
        {
            if (IsAngleOnArc(angle, angleEps) && !PointAtAngle(angle).IsValid())
            {
                return false;
            }
        }
        return true;
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

    double SCArcSegment2d::Length(double eps) const
    {
        return IsValid(eps) ? std::abs(sweepAngle) * radius : 0.0;
    }

    SCBox2d SCArcSegment2d::Bounds() const
    {
        if (!IsValid())
        {
            return SCBox2d{};
        }

        return BoundsForAngleTolerance(Geometry::kDefaultAngleEpsilon);
    }

    SCBox2d SCArcSegment2d::Bounds(double eps) const
    {
        if (!IsValid(eps))
        {
            return SCBox2d{};
        }

        return BoundsForAngleTolerance(eps / radius);
    }

    SCBox2d SCArcSegment2d::BoundsForAngleTolerance(double angleEps) const
    {
        SCBox2d box;
        box.ExpandToInclude(StartPoint());
        box.ExpandToInclude(EndPoint());

        for (double angle : Geometry::kArcBoundsCriticalAngles)
        {
            if (IsAngleOnArc(angle, angleEps))
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

    SCPoint2d SCArcSegment2d::PointAt(double parameter, double eps) const
    {
        if (!std::isfinite(parameter) || !IsValid(eps))
        {
            return StartPoint();
        }
        const SCPoint2d point = PointAtAngle(startAngle + sweepAngle * parameter);
        return point.IsValid() ? point : StartPoint();
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

    SCPoint2d SCArcSegment2d::PointAtLength(double distanceFromStart, bool clampToSegment, double eps) const
    {
        if (!std::isfinite(distanceFromStart) || !IsValid(eps))
        {
            return StartPoint();
        }
        const double length = std::abs(sweepAngle) * radius;
        if (clampToSegment)
        {
            distanceFromStart = std::clamp(distanceFromStart, 0.0, length);
        }
        return PointAt(distanceFromStart / length, eps);
    }

    bool SCArcSegment2d::AlmostEquals(const SCArcSegment2d& other, double eps) const
    {
        return AlmostEquals(other, eps, eps);
    }

    bool SCArcSegment2d::AlmostEquals(const SCArcSegment2d& other, double distanceEps, double angleEps) const
    {
        return std::isfinite(distanceEps) && distanceEps >= 0.0 &&
               std::isfinite(angleEps) && angleEps >= 0.0 &&
               center.AlmostEquals(other.center, distanceEps) &&
               std::abs(radius - other.radius) <= distanceEps &&
               std::abs(startAngle - other.startAngle) <= angleEps &&
               std::abs(sweepAngle - other.sweepAngle) <= angleEps;
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

    bool SCArcSegment2d::ContainsAngle(double angle) const
    {
        return IsAngleOnArc(angle);
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
        return IsAngleOnArc(candidateAngle, Geometry::kDefaultAngleEpsilon);
    }

    bool SCArcSegment2d::IsAngleOnArc(double candidateAngle, double angleEps) const
    {
        if (sweepAngle >= 0.0)
        {
            const double delta = NormalizeAngle(candidateAngle - startAngle);
            return delta <= sweepAngle + angleEps;
        }

        const double delta = NormalizeAngle(startAngle - candidateAngle);
        return delta <= (-sweepAngle) + angleEps;
    }

    SCPoint2d SCArcSegment2d::PointAtAngle(double angle) const
    {
        return SCPoint2d{center.x + radius * std::cos(angle), center.y + radius * std::sin(angle)};
    }
}  // namespace Geometry
