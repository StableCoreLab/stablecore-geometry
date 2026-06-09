#pragma once

#include <algorithm>
#include <sstream>
#include <string>

#include "Types/Geometry3d/SCBox3.h"
#include "Types/Geometry3d/SCDirection3d.h"

namespace Geometry
{
    struct SCLineSegment3d
    {
        SCPoint3d startPoint{};
        SCPoint3d endPoint{};

        [[nodiscard]] static SCLineSegment3d FromStartEnd(const SCPoint3d& startValue, const SCPoint3d& endValue)
        {
            return SCLineSegment3d{startValue, endValue};
        }

        [[nodiscard]] bool IsValid(double eps = kDefaultEpsilon) const
        {
            return startPoint.IsValid() && endPoint.IsValid() && (endPoint - startPoint).Length() > eps;
        }

        [[nodiscard]] double Length() const
        {
            return (endPoint - startPoint).Length();
        }

        [[nodiscard]] SCBox3d Bounds() const
        {
            if (!IsValid())
            {
                return {};
            }

            SCBox3d box;
            box.ExpandToInclude(startPoint);
            box.ExpandToInclude(endPoint);
            return box;
        }

        [[nodiscard]] SCPoint3d PointAt(double parameter) const
        {
            return startPoint + (endPoint - startPoint) * parameter;
        }

        [[nodiscard]] SCPoint3d PointAtLength(double distance, bool clampToSegment = false) const
        {
            if (!IsValid())
            {
                return startPoint;
            }

            const double length = Length();
            if (length <= kDefaultEpsilon)
            {
                return startPoint;
            }

            if (clampToSegment)
            {
                distance = std::clamp(distance, 0.0, length);
            }
            return PointAt(distance / length);
        }

        [[nodiscard]] SCDirection3d Direction(double eps = kDefaultEpsilon) const
        {
            return SCDirection3d::FromVector(endPoint - startPoint, eps);
        }

        [[nodiscard]] std::string DebugString() const
        {
            std::ostringstream stream;
            stream << "SCLineSegment3d{startPoint=" << startPoint.DebugString() << ", endPoint=" << endPoint.DebugString()
                   << "}";
            return stream.str();
        }
    };

}  // namespace Geometry
