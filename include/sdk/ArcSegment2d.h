#pragma once

#include <memory>
#include <string>

#include "export/GeometryExport.h"
#include "sdk/Segment2d.h"
#include "sdk/GeometryTypes.h"
#include "types/Segment2.h"

namespace geometry::sdk
{
using geometry::ArcDirection;

class GEOMETRY_API ArcSegment2d : public Segment2d
{
public:
    Point2d center{};
    double radius{0.0};
    double startAngle{0.0};
    double sweepAngle{0.0};

    ArcSegment2d() = default;
    ArcSegment2d(
        const Point2d& centerValue,
        double radiusValue,
        double startAngleValue,
        double sweepAngleValue);

    [[nodiscard]] static ArcSegment2d FromCenterRadiusStartSweep(
        const Point2d& centerValue,
        double radiusValue,
        double startAngleValue,
        double sweepAngleValue);

    [[nodiscard]] SegmentKind2 Kind() const override;
    [[nodiscard]] bool IsValid() const override;
    [[nodiscard]] ArcDirection Direction() const;
    [[nodiscard]] double EndAngle() const;
    [[nodiscard]] Point2d StartPoint() const override;
    [[nodiscard]] Point2d EndPoint() const override;
    [[nodiscard]] double Length() const override;
    [[nodiscard]] Box2d Bounds() const override;
    [[nodiscard]] Point2d PointAt(double parameter) const override;
    [[nodiscard]] Point2d PointAtLength(double distanceFromStart, bool clampToSegment = false) const override;
    [[nodiscard]] bool AlmostEquals(const ArcSegment2d& other, double eps = geometry::kDefaultEpsilon) const;
    [[nodiscard]] std::string DebugString() const override;
    [[nodiscard]] std::unique_ptr<Segment2d> Clone() const override;

    [[nodiscard]] constexpr bool operator==(const ArcSegment2d& other) const = default;
    [[nodiscard]] constexpr bool operator!=(const ArcSegment2d& other) const = default;

private:
    [[nodiscard]] static double NormalizeAngle(double angle);
    [[nodiscard]] bool IsAngleOnArc(double candidateAngle) const;
    [[nodiscard]] Point2d PointAtAngle(double angle) const;
};
} // namespace geometry::sdk
