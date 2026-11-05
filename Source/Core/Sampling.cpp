#include "Core/Sampling.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace Geometry
{
    namespace
    {
        [[nodiscard]] std::vector<SCPoint2d> SampleSegment(const ISCSegment2d& segment, std::size_t partCount)
        {
            std::vector<SCPoint2d> points;
            if (partCount == 0)
            {
                return points;
            }
            if (partCount >= points.max_size())
            {
                throw std::length_error("sample point count exceeds vector capacity");
            }
            points.reserve(partCount + 1);
            for (std::size_t i = 0; i <= partCount; ++i)
            {
                points.push_back(segment.PointAt(static_cast<double>(i) / static_cast<double>(partCount)));
            }
            return points;
        }
    }  // namespace

    std::vector<SCPoint2d> Sample(const SCLineSegment2d& segment, std::size_t partCount)
    {
        return SampleSegment(segment, partCount);
    }

    std::vector<SCPoint2d> Sample(const SCArcSegment2d& segment, std::size_t partCount)
    {
        return SampleSegment(segment, partCount);
    }

    std::vector<SCPoint2d> Sample(const ISCSegment2d& segment, std::size_t partCount)
    {
        return SampleSegment(segment, partCount);
    }

    std::vector<SCPoint2d> SampleByMaxAngle(const SCArcSegment2d& segment, double maxAngleRadians)
    {
        if (!std::isfinite(maxAngleRadians) || maxAngleRadians <= 0.0)
        {
            return {segment.StartPoint(), segment.EndPoint()};
        }

        const double requiredParts = std::ceil(std::abs(segment.sweepAngle) / maxAngleRadians);
        if (!std::isfinite(requiredParts) ||
            requiredParts >= static_cast<double>(std::numeric_limits<std::size_t>::max()))
        {
            throw std::length_error("angular sample count is not representable");
        }
        const std::size_t parts = std::max<std::size_t>(1, static_cast<std::size_t>(requiredParts));
        return Sample(segment, parts);
    }
}  // namespace Geometry
