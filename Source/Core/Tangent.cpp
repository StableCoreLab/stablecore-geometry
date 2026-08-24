#include "Core/Tangent.h"

#include <algorithm>
#include <cmath>

#include "Types/Geometry2d/SCVector2.h"

namespace Geometry
{
    namespace
    {
        [[nodiscard]] SCTangentPoints2d TangentPointsImpl(const SCPoint2d& point,
                                                          const SCPoint2d& center,
                                                          double radius,
                                                          double eps)
        {
            SCTangentPoints2d result{};
            if (!(radius > 0.0) || !point.IsValid() || !center.IsValid())
            {
                return result;
            }

            const SCVector2d delta = point - center;
            const double distanceSquared = delta.LengthSquared();
            if (distanceSquared <= eps * eps)
            {
                result.pointInsideCircle = true;
                return result;
            }

            const double distance = std::sqrt(distanceSquared);
            if (distance < radius - eps)
            {
                result.pointInsideCircle = true;
                return result;
            }

            const double angle = std::atan2(delta.y, delta.x);
            if (std::abs(distance - radius) <= eps)
            {
                result.pointCount = 1;
                result.points[0] = point;
                result.angles[0] = angle;
                return result;
            }

            const double ratio = std::clamp(radius / distance, -1.0, 1.0);
            const double offset = std::acos(ratio);
            result.pointCount = 2;
            result.angles[0] = angle - offset;
            result.angles[1] = angle + offset;
            result.points[0] = center + SCVector2d{radius * std::cos(result.angles[0]), radius * std::sin(result.angles[0])};
            result.points[1] = center + SCVector2d{radius * std::cos(result.angles[1]), radius * std::sin(result.angles[1])};

            if (result.angles[0] > result.angles[1])
            {
                std::swap(result.angles[0], result.angles[1]);
                std::swap(result.points[0], result.points[1]);
            }

            return result;
        }
    }  // namespace

    SCTangentPoints2d TangentPoints(const SCPoint2d& point, const SCCircle2d& circle, double eps)
    {
        return TangentPointsImpl(point, circle.center, circle.radius, eps);
    }

    SCTangentPoints2d TangentPoints(const SCPoint2d& point, const SCArcSegment2d& arc, double eps)
    {
        SCTangentPoints2d result = TangentPointsImpl(point, arc.center, arc.radius, eps);
        if (result.pointCount == 0)
        {
            return result;
        }

        std::size_t writeIndex = 0;
        for (std::size_t index = 0; index < result.pointCount; ++index)
        {
            if (arc.ContainsAngle(result.angles[index]))
            {
                if (writeIndex != index)
                {
                    result.points[writeIndex] = result.points[index];
                    result.angles[writeIndex] = result.angles[index];
                }
                ++writeIndex;
            }
        }

        result.pointCount = writeIndex;
        return result;
    }
}  // namespace Geometry
