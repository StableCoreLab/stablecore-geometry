#pragma once

#include <memory>
#include <string>

#include "Export/GeometryExport.h"
#include "Geometry2d/ISCSegment2d.h"
#include "Geometry2d/SegmentTypes.h"

namespace Geometry
{
    class GEOMETRY_API SCArcSegment2d : public ISCSegment2d
    {
    public:
        SCPoint2d center{};
        double radius{0.0};
        double startAngle{0.0};
        double sweepAngle{0.0};

        SCArcSegment2d() = default;
        SCArcSegment2d(const SCPoint2d& center, double radius, double startAngle, double sweepAngle);
        SCArcSegment2d(const SCPoint2d& center, double radius, double startAngle, double endAngle, SCArcDirection direction);

        [[nodiscard]] static SCArcSegment2d FromCenterRadiusStartSweep(const SCPoint2d& center,
                                                                     double radius,
                                                                     double startAngle,
                                                                     double sweepAngle);

        [[nodiscard]] SCSegmentKind2 Kind() const override;
        [[nodiscard]] bool IsValid() const override;
        [[nodiscard]] SCArcDirection Direction() const;
        [[nodiscard]] double EndAngle() const;
        [[nodiscard]] SCPoint2d StartPoint() const override;
        [[nodiscard]] SCPoint2d EndPoint() const override;
        [[nodiscard]] double Length() const override;
        [[nodiscard]] SCBox2d Bounds() const override;
        [[nodiscard]] SCPoint2d PointAt(double parameter) const override;
        [[nodiscard]] SCPoint2d PointAtLength(double distanceFromStart, bool clampToSegment = false) const override;
        [[nodiscard]] bool AlmostEquals(const SCArcSegment2d& other, double eps = Geometry::kDefaultEpsilon) const;
        [[nodiscard]] std::string DebugString() const override;
        [[nodiscard]] std::unique_ptr<ISCSegment2d> Clone() const override;

        [[nodiscard]] constexpr bool operator==(const SCArcSegment2d& other) const = default;
        [[nodiscard]] constexpr bool operator!=(const SCArcSegment2d& other) const = default;

    private:
        [[nodiscard]] static double NormalizeAngle(double angle);
        [[nodiscard]] bool IsAngleOnArc(double candidateAngle) const;
        [[nodiscard]] SCPoint2d PointAtAngle(double angle) const;
    };
}  // namespace Geometry

