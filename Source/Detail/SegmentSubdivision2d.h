#pragma once

#include <cstddef>
#include <vector>

#include "Core/Intersection.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "SegmentParameters2d.h"

namespace Geometry::Detail
{
    struct SubdividedLineSegment2d
    {
        std::size_t sourceIndex{0};
        SCPoint2d start{};
        SCPoint2d end{};
    };

    // Keeps only the geometric subdivision result. Callers retain ownership of
    // source-specific segment state and choose their own parameter tolerance.
    template <typename TParameterTolerance>
    [[nodiscard]] std::vector<SubdividedLineSegment2d> SubdivideLineSegments(
        const std::vector<SCLineSegment2d>& segments,
        const double intersectionEpsilon,
        const TParameterTolerance& parameterTolerance)
    {
        std::vector<std::vector<double>> parameters(segments.size(), std::vector<double>{0.0, 1.0});
        for (std::size_t i = 0; i < segments.size(); ++i)
        {
            for (std::size_t j = i + 1; j < segments.size(); ++j)
            {
                const SCSegmentIntersection2d intersection = Intersect(segments[i], segments[j], intersectionEpsilon);
                if (!intersection.HasIntersection())
                {
                    continue;
                }

                for (std::size_t k = 0; k < intersection.pointCount; ++k)
                {
                    AddClampedParameter(parameters[i], intersection.points[k].parameterOnFirst, intersectionEpsilon);
                    AddClampedParameter(parameters[j], intersection.points[k].parameterOnSecond, intersectionEpsilon);
                }
            }
        }

        std::vector<SubdividedLineSegment2d> subdivisions;
        for (std::size_t i = 0; i < segments.size(); ++i)
        {
            const SCLineSegment2d& segment = segments[i];
            const double parameterTol = parameterTolerance(segment);
            const std::vector<double> compacted = CompactSortedParameters(parameters[i], parameterTol);

            for (std::size_t k = 0; k + 1 < compacted.size(); ++k)
            {
                if (compacted[k + 1] <= compacted[k] + parameterTol)
                {
                    continue;
                }

                SCPoint2d start = segment.PointAt(compacted[k]);
                SCPoint2d end = segment.PointAt(compacted[k + 1]);
                if (compacted[k] <= parameterTol)
                {
                    start = segment.startPoint;
                } else if (compacted[k] >= 1.0 - parameterTol)
                {
                    start = segment.endPoint;
                }

                if (compacted[k + 1] <= parameterTol)
                {
                    end = segment.startPoint;
                } else if (compacted[k + 1] >= 1.0 - parameterTol)
                {
                    end = segment.endPoint;
                }

                if (!start.AlmostEquals(end, intersectionEpsilon))
                {
                    subdivisions.push_back(SubdividedLineSegment2d{i, start, end});
                }
            }
        }

        return subdivisions;
    }
}  // namespace Geometry::Detail
