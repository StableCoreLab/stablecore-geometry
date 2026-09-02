#pragma once

#include <vector>

#include "Types/Geometry2d/SCPoint2.h"

namespace Geometry::Detail
{
    [[nodiscard]] inline std::vector<SCPoint2d> NormalizeRingVertices2d(std::vector<SCPoint2d> points,
                                                                        const double tolerance)
    {
        std::vector<SCPoint2d> normalized;
        normalized.reserve(points.size());
        for (const SCPoint2d& point : points)
        {
            if (normalized.empty() || !normalized.back().AlmostEquals(point, tolerance))
            {
                normalized.push_back(point);
            }
        }

        while (normalized.size() >= 2 && normalized.front().AlmostEquals(normalized.back(), tolerance))
        {
            normalized.pop_back();
        }

        return normalized;
    }
}  // namespace Geometry::Detail
