#pragma once

#include <cstddef>
#include <numbers>
#include <utility>
#include <vector>

#include "Geometry2d/SCPolygon2d.h"
#include "Geometry2d/SCPolyline2d.h"
#include "Types/Geometry2d/SCPoint2.h"

namespace Geometry::Detail
{
    // 等参数角采样，闭合边由 SCPolyline2d 表达，不重复首顶点。
    template <typename Shape>
    [[nodiscard]] SCPolygon2d SampleClosedShapePolygon2d(const Shape& shape,
                                                        std::size_t segmentCount,
                                                        std::size_t minimumSegmentCount)
    {
        if (!shape.IsValid() || segmentCount < minimumSegmentCount)
        {
            return {};
        }
        std::vector<SCPoint2d> points;
        points.reserve(segmentCount);
        for (std::size_t i = 0; i < segmentCount; ++i)
        {
            const double angle = 2.0 * std::numbers::pi_v<double> *
                static_cast<double>(i) / static_cast<double>(segmentCount);
            points.push_back(shape.PointAtAngle(angle));
        }
        return SCPolygon2d(SCPolyline2d(std::move(points), SCPolylineClosure::Closed));
    }
}
