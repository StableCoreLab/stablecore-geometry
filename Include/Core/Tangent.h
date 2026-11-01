#pragma once

#include <array>
#include <cmath>

#include "Export/GeometryExport.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCCircle2d.h"
#include "Support/Epsilon.h"
#include "Types/Geometry2d/SCPoint2.h"

namespace Geometry
{
    struct GEOMETRY_API SCTangentPoints2d
    {
        std::size_t pointCount{0};
        std::array<SCPoint2d, 2> points{};
        std::array<double, 2> angles{0.0, 0.0};
        bool pointInsideCircle{false};

        [[nodiscard]] bool IsValid() const
        {
            if (pointCount > points.size() || (pointInsideCircle && pointCount != 0))
            {
                return false;
            }
            for (std::size_t index = 0; index < pointCount; ++index)
            {
                if (!points[index].IsValid() || !std::isfinite(angles[index]))
                {
                    return false;
                }
            }
            return true;
        }
    };

    [[nodiscard]] GEOMETRY_API SCTangentPoints2d TangentPoints(const SCPoint2d& point,
                                                               const SCCircle2d& circle,
                                                               double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API SCTangentPoints2d TangentPoints(const SCPoint2d& point,
                                                               const SCArcSegment2d& arc,
                                                               double eps = Geometry::kDefaultEpsilon);
}  // namespace Geometry
