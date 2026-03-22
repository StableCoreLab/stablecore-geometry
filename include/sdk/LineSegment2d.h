#pragma once

#include <memory>
#include <string>

#include "export/GeometryExport.h"
#include "sdk/GeometryTypes.h"
#include "sdk/Segment2d.h"

namespace geometry::sdk
{
class GEOMETRY_API LineSegment2d : public Segment2d
{
public:
    Point2d startPoint{};
    Point2d endPoint{};

    LineSegment2d() = default;
    LineSegment2d(const Point2d& startPointValue, const Point2d& endPointValue);

    [[nodiscard]] static LineSegment2d FromEndpoints(
        const Point2d& startPointValue,
        const Point2d& endPointValue);

    [[nodiscard]] SegmentKind2 Kind() const override;
    [[nodiscard]] bool IsValid() const override;
    [[nodiscard]] Point2d StartPoint() const override;
    [[nodiscard]] Point2d EndPoint() const override;
    [[nodiscard]] double Length() const override;
    [[nodiscard]] Box2d Bounds() const override;
    [[nodiscard]] Point2d PointAt(double parameter) const override;
    [[nodiscard]] Point2d PointAtLength(double distanceFromStart, bool clampToSegment = false) const override;
    [[nodiscard]] bool AlmostEquals(const LineSegment2d& other, double eps = geometry::kDefaultEpsilon) const;
    [[nodiscard]] std::string DebugString() const override;
    [[nodiscard]] std::unique_ptr<Segment2d> Clone() const override;

    [[nodiscard]] constexpr bool operator==(const LineSegment2d& other) const = default;
    [[nodiscard]] constexpr bool operator!=(const LineSegment2d& other) const = default;
};
} // namespace geometry::sdk
