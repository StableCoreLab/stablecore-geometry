#pragma once

#include <memory>
#include <string>

#include "Export/GeometryExport.h"
#include "Geometry2d/SegmentTypes.h"
#include "Types/Geometry2d/SCBox2.h"
#include "Types/Geometry2d/SCPoint2.h"

namespace Geometry
{
    class GEOMETRY_API ISCSegment2d
    {
    public:
        virtual ~ISCSegment2d() = default;

        [[nodiscard]] virtual SCSegmentKind2 Kind() const = 0;
        [[nodiscard]] virtual bool IsValid() const = 0;
        [[nodiscard]] virtual SCPoint2d StartPoint() const = 0;
        [[nodiscard]] virtual SCPoint2d EndPoint() const = 0;
        [[nodiscard]] virtual double Length() const = 0;
        [[nodiscard]] virtual SCBox2d Bounds() const = 0;
        [[nodiscard]] virtual SCPoint2d PointAt(double parameter) const = 0;
        [[nodiscard]] virtual SCPoint2d PointAtLength(double distanceFromStart, bool clampToSegment = false) const = 0;
        [[nodiscard]] virtual std::string DebugString() const = 0;
        [[nodiscard]] virtual std::unique_ptr<ISCSegment2d> Clone() const = 0;
    };
}  // namespace Geometry
