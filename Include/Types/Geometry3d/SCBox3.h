#pragma once

#include <algorithm>
#include <sstream>
#include <string>

#include "Types/Geometry3d/SCPoint3.h"

namespace Geometry
{
    struct SCBox3d
    {
        SCPoint3d minPoint{};
        SCPoint3d maxPoint{};
        bool valid{false};

        [[nodiscard]] static constexpr SCBox3d FromMinMax(const SCPoint3d& minValue, const SCPoint3d& maxValue)
        {
            return SCBox3d{minValue, maxValue, true};
        }

        [[nodiscard]] bool IsValid() const
        {
            return valid && minPoint.IsValid() && maxPoint.IsValid() && minPoint.x <= maxPoint.x &&
                   minPoint.y <= maxPoint.y && minPoint.z <= maxPoint.z;
        }

        [[nodiscard]] SCPoint3d MinPoint() const
        {
            return minPoint;
        }

        [[nodiscard]] SCPoint3d MaxPoint() const
        {
            return maxPoint;
        }

        [[nodiscard]] double Width() const
        {
            return maxPoint.x - minPoint.x;
        }

        [[nodiscard]] double Height() const
        {
            return maxPoint.y - minPoint.y;
        }

        [[nodiscard]] double Depth() const
        {
            return maxPoint.z - minPoint.z;
        }

        [[nodiscard]] SCPoint3d Center() const
        {
            return SCPoint3d{
                (minPoint.x + maxPoint.x) * 0.5, (minPoint.y + maxPoint.y) * 0.5, (minPoint.z + maxPoint.z) * 0.5};
        }

        void ExpandToInclude(const SCPoint3d& point)
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
            stream << "SCBox3d{minPoint=" << minPoint.DebugString() << ", maxPoint=" << maxPoint.DebugString()
                   << ", valid=" << (valid ? "true" : "false") << "}";
            return stream.str();
        }
    };

}  // namespace Geometry
