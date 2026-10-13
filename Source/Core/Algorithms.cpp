#include "Core/Algorithms.h"

#include "Support/Geometry2d/Predicate2.h"

namespace Geometry
{
    SCSegmentProjection2d ProjectPointToSegment(const SCPoint2d& point,
                                              const SCPoint2d& segmentStart,
                                              const SCPoint2d& segmentEnd,
                                              bool clampToSegment)
    {
        if (IsZero((segmentEnd - segmentStart).LengthSquared()))
        {
            return {segmentStart, 0.0, DistanceSquared(point, segmentStart), true};
        }

        const SCLineSegment2d segment = SCLineSegment2d::FromEndpoints(segmentStart, segmentEnd);
        SCSegmentProjection2d projection = ProjectPointToLineSegment(point, segment, clampToSegment);
        projection.isOnSegment = clampToSegment || projection.isOnSegment;
        return projection;
    }
}  // namespace Geometry

