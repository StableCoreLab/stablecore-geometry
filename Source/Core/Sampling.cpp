#include "Core/Sampling.h"

#include <algorithm>
#include <cmath>

namespace Geometry
{
    std::vector<SCPoint2d> Sample(const SCLineSegment2d& segment, std::size_t partCount)
    {
        std::vector<SCPoint2d> points;
        if (partCount == 0)
        {
            return points;
        }
        points.reserve(partCount + 1);
        for (std::size_t i = 0; i <= partCount; ++i)
        {
            points.push_back(segment.PointAt(static_cast<double>(i) / static_cast<double>(partCount)));
        }
        return points;
    }

    std::vector<SCPoint2d> Sample(const SCArcSegment2d& segment, std::size_t partCount)
    {
        std::vector<SCPoint2d> points;
        if (partCount == 0)
        {
            return points;
        }
        points.reserve(partCount + 1);
        for (std::size_t i = 0; i <= partCount; ++i)
        {
            points.push_back(segment.PointAt(static_cast<double>(i) / static_cast<double>(partCount)));
        }
        return points;
    }

    std::vector<SCPoint2d> Sample(const ISCSegment2d& segment, std::size_t partCount)
    {
        std::vector<SCPoint2d> points;
        if (partCount == 0)
        {
            return points;
        }
        points.reserve(partCount + 1);
        for (std::size_t i = 0; i <= partCount; ++i)
        {
            points.push_back(segment.PointAt(static_cast<double>(i) / static_cast<double>(partCount)));
        }
        return points;
    }

    std::vector<SCPoint2d> SampleByMaxAngle(const SCArcSegment2d& segment, double maxAngleRadians)
    {
        if (maxAngleRadians <= 0.0)
        {
            return {segment.StartPoint(), segment.EndPoint()};
        }

        const std::size_t parts = std::max<std::size_t>(
            1, static_cast<std::size_t>(std::ceil(std::abs(segment.sweepAngle) / maxAngleRadians)));
        return Sample(segment, parts);
    }
}  // namespace Geometry
