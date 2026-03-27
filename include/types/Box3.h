#pragma once

#include <algorithm>
#include <sstream>
#include <string>

#include "types/Point3.h"

namespace geometry
{
struct Box3d
{
    Point3d minPoint{};
    Point3d maxPoint{};
    bool valid{false};

    [[nodiscard]] static constexpr Box3d FromMinMax(const Point3d& minValue, const Point3d& maxValue)
    {
        return Box3d{minValue, maxValue, true};
    }

    [[nodiscard]] bool IsValid() const
    {
        return valid && minPoint.IsValid() && maxPoint.IsValid() && minPoint.x <= maxPoint.x &&
               minPoint.y <= maxPoint.y && minPoint.z <= maxPoint.z;
    }

    [[nodiscard]] Point3d MinPoint() const { return minPoint; }
    [[nodiscard]] Point3d MaxPoint() const { return maxPoint; }

    [[nodiscard]] double Width() const { return maxPoint.x - minPoint.x; }
    [[nodiscard]] double Height() const { return maxPoint.y - minPoint.y; }
    [[nodiscard]] double Depth() const { return maxPoint.z - minPoint.z; }

    [[nodiscard]] Point3d Center() const
    {
        return Point3d{(minPoint.x + maxPoint.x) * 0.5, (minPoint.y + maxPoint.y) * 0.5, (minPoint.z + maxPoint.z) * 0.5};
    }

    void ExpandToInclude(const Point3d& point)
    {
        if (!IsValid())
        {
            minPoint = point;
            maxPoint = point;
            valid = point.IsValid();
            return;
        }

        minPoint.x = std::min(minPoint.x, point.x);
        minPoint.y = std::min(minPoint.y, point.y);
        minPoint.z = std::min(minPoint.z, point.z);
        maxPoint.x = std::max(maxPoint.x, point.x);
        maxPoint.y = std::max(maxPoint.y, point.y);
        maxPoint.z = std::max(maxPoint.z, point.z);
    }

    [[nodiscard]] std::string DebugString() const
    {
        std::ostringstream stream;
        stream << "Box3d{minPoint=" << minPoint.DebugString() << ", maxPoint=" << maxPoint.DebugString()
               << ", valid=" << (valid ? "true" : "false") << "}";
        return stream.str();
    }
};
} // namespace geometry
