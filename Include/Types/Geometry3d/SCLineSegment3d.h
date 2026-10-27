#pragma once

#include <algorithm>
#include <cmath>
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
            if (!std::isfinite(eps) || eps < 0.0 || !startPoint.IsValid() || !endPoint.IsValid())
            {
                return false;
            }
            const double length = Length();
            return std::isfinite(length) && length > eps;
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

        [[nodiscard]] SCPoint3d PointAtLength(double distance, bool clampToSegment, double eps) const
        {
            if (!std::isfinite(eps) || eps <= 0.0 || !std::isfinite(distance) || !IsValid(eps))
            {
                return startPoint;
            }

            const double length = Length();
            if (clampToSegment)
            {
                distance = std::clamp(distance, 0.0, length);
            }
            const double parameter = distance / length;
            if (!std::isfinite(parameter))
            {
                return startPoint;
            }
            const SCPoint3d point = PointAt(parameter);
            return point.IsValid() ? point : startPoint;
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
