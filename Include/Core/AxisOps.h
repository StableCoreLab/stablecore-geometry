#pragma once

#include <memory>
#include <span>

#include "Core/Results.h"
#include "Export/GeometryExport.h"
#include "Geometry2d/ISCSegment2d.h"
#include "Types/Geometry2d/SCPoint2.h"

namespace Geometry
{
    [[nodiscard]] GEOMETRY_API SCAxisSample2d SampleAxis(const ISCSegment2d& segment, double parameter);
    [[nodiscard]] GEOMETRY_API SCAxisSample2d SampleAxisAtLength(const ISCSegment2d& segment,
                                                               double length,
                                                               bool clampToSegment = true);
    [[nodiscard]] GEOMETRY_API SCAxisProjection2d ProjectPointToAxis(const SCPoint2d& point, const ISCSegment2d& segment);

    [[nodiscard]] GEOMETRY_API std::unique_ptr<ISCSegment2d> Reverse(const ISCSegment2d& segment);

    [[nodiscard]] GEOMETRY_API SCSegmentSplit2d SplitSegment(const ISCSegment2d& segment, double parameter);
    [[nodiscard]] GEOMETRY_API SCSegmentTrim2d TrimSegment(const ISCSegment2d& segment,
                                                         double startParameter,
                                                         double endParameter);

    [[nodiscard]] GEOMETRY_API SCSnapResult2d SnapPointToSegments(const SCPoint2d& point,
                                                                std::span<const ISCSegment2d* const> segments,
                                                                double maxDistance);
}  // namespace Geometry

