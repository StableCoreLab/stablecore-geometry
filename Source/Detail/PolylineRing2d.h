#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "Geometry2d/SCLineSegment2d.h"
#include "Geometry2d/SCPolyline2d.h"

namespace Geometry::Detail
{
    [[nodiscard]] inline SCPolyline2d MakeVertexPreservingRing(const std::vector<SCPoint2d>& points)
    {
        std::vector<std::shared_ptr<ISCSegment2d>> segments;
        segments.reserve(points.size());
        for (std::size_t i = 0; i < points.size(); ++i)
        {
            segments.push_back(std::make_shared<SCLineSegment2d>(points[i], points[(i + 1) % points.size()]));
        }
        return SCPolyline2d(std::move(segments), SCPolylineClosure::Closed);
    }
}  // namespace Geometry::Detail
