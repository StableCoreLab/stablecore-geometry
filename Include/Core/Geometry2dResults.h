#pragma once

#include <cstddef>
#include <cmath>
#include <sstream>
#include <string>
#include "Export/GeometryExport.h"
#include "Types/Geometry2d/SCPoint2.h"
#include "Support/Epsilon.h"

namespace Geometry
{
    struct GEOMETRY_API SCSegmentProjection2d
    {
        SCPoint2d point{};
        double parameter{0.0};
        double distanceSquared{0.0};
        bool isOnSegment{false};

        [[nodiscard]] bool IsValid() const
        {
            return point.IsValid() && std::isfinite(parameter) && std::isfinite(distanceSquared) &&
                   distanceSquared >= 0.0;
        }

        [[nodiscard]] bool AlmostEquals(const SCSegmentProjection2d& other, double eps = Geometry::kDefaultEpsilon) const
        {
            return point.AlmostEquals(other.point, eps) && std::abs(parameter - other.parameter) <= eps &&
                   std::abs(distanceSquared - other.distanceSquared) <= eps && isOnSegment == other.isOnSegment;
        }

        [[nodiscard]] std::string DebugString() const
        {
            std::ostringstream stream;
            stream << "SCSegmentProjection2d{point=" << point.DebugString() << ", parameter=" << parameter
                   << ", distanceSquared=" << distanceSquared << ", isOnSegment=" << (isOnSegment ? "true" : "false")
                   << "}";
            return stream.str();
        }
    };

    struct GEOMETRY_API SCClosestPoints2d
    {
        SCPoint2d firstPoint{};
        SCPoint2d secondPoint{};
        double parameterOnFirst{0.0};
        double parameterOnSecond{0.0};
        double distanceSquared{0.0};

        [[nodiscard]] bool IsValid() const
        {
            return firstPoint.IsValid() && secondPoint.IsValid() && distanceSquared >= 0.0;
        }
    };

    struct GEOMETRY_API SCSnapResult2d
    {
        bool snapped{false};
        SCPoint2d point{};
        double distanceSquared{0.0};
        std::size_t segmentIndex{0};
        double parameter{0.0};

        [[nodiscard]] bool IsValid() const
        {
            return !snapped || point.IsValid();
        }
    };
}  // namespace Geometry
