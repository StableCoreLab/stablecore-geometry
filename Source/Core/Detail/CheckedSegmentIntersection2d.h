#pragma once

#include "SegmentKernel2d.h"
#include "Geometry2d/ISCSegment2d.h"

namespace Geometry::Detail
{
    enum class SCCheckedGeometryStatus2d
    {
        Success,
        NoIntersection,
        InvalidInput,
        NumericalIndeterminate
    };

    struct SCCheckedIntersection2d
    {
        SCCheckedGeometryStatus2d status{SCCheckedGeometryStatus2d::InvalidInput};
        SCSegmentIntersection2d intersection{};
    };

    [[nodiscard]] SCCheckedIntersection2d CheckedIntersectSegments(const ISCSegment2d& first,
                                                                    const ISCSegment2d& second,
                                                                    double tolerance);
}
