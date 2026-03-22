#pragma once

#include <memory>
#include <string>

#include "export/GeometryExport.h"
#include "sdk/GeometryTypes.h"
#include "types/Segment2.h"

namespace geometry::sdk
{
using geometry::SegmentKind2;

class GEOMETRY_API Segment2d
{
public:
    virtual ~Segment2d() = default;

    [[nodiscard]] virtual SegmentKind2 Kind() const = 0;
    [[nodiscard]] virtual bool IsValid() const = 0;
    [[nodiscard]] virtual Point2d StartPoint() const = 0;
    [[nodiscard]] virtual Point2d EndPoint() const = 0;
    [[nodiscard]] virtual double Length() const = 0;
    [[nodiscard]] virtual Box2d Bounds() const = 0;
    [[nodiscard]] virtual Point2d PointAt(double parameter) const = 0;
    [[nodiscard]] virtual Point2d PointAtLength(double distanceFromStart, bool clampToSegment = false) const = 0;
    [[nodiscard]] virtual std::string DebugString() const = 0;
    [[nodiscard]] virtual std::unique_ptr<Segment2d> Clone() const = 0;
};
} // namespace geometry::sdk
