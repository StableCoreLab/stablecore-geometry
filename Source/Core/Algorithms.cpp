#include "Core/Algorithms.h"

#include <algorithm>
#include <cmath>

#include "Support/Epsilon.h"
#include "Support/Geometry2d/Predicate2.h"

namespace Geometry
{
    namespace
    {
        [[nodiscard]] Geometry::SCPoint2d Interpolate(const Geometry::SCPoint2d& startPoint,
                                                    const Geometry::SCPoint2d& endPoint,
                                                    double parameter)
        {
            return Geometry::SCPoint2d(startPoint.x + (endPoint.x - startPoint.x) * parameter,
                                     startPoint.y + (endPoint.y - startPoint.y) * parameter);
        }
    }  // namespace

    SCSegmentProjection2d ProjectPointToSegment(const SCPoint2d& point,
                                              const SCPoint2d& segmentStart,
                                              const SCPoint2d& segmentEnd,
                                              bool clampToSegment)
    {
        const Geometry::SCPoint2d internalPoint = point;
        const Geometry::SCPoint2d internalStart = segmentStart;
        const Geometry::SCPoint2d internalEnd = segmentEnd;
        const Geometry::SCVector2d segmentVector = internalEnd - internalStart;
        const double segmentLengthSquared = segmentVector.LengthSquared();

        if (Geometry::IsZero(segmentLengthSquared))
        {
            return SCSegmentProjection2d{segmentStart, 0.0, DistanceSquared(point, segmentStart), true};
        }

        const Geometry::SCVector2d startToPoint = internalPoint - internalStart;
        const double rawParameter = Geometry::Dot(startToPoint, segmentVector) / segmentLengthSquared;
        const double parameter = clampToSegment ? std::clamp(rawParameter, 0.0, 1.0) : rawParameter;
        const Geometry::SCPoint2d projectedPoint = Interpolate(internalStart, internalEnd, parameter);

        return SCSegmentProjection2d{projectedPoint,
                                   parameter,
                                   DistanceSquared(point, projectedPoint),
                                   clampToSegment || (rawParameter >= -Geometry::kAlgorithmsDefaultEpsilon &&
                                                      rawParameter <= 1.0 + Geometry::kAlgorithmsDefaultEpsilon)};
    }
}  // namespace Geometry

