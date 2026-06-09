#pragma once

#include <vector>

#include "Export/GeometryExport.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Geometry2d/ISCSegment2d.h"
#include "Types/Geometry2d/SCPoint2.h"

namespace Geometry
{
    [[nodiscard]] GEOMETRY_API std::vector<SCPoint2d> Sample(const SCLineSegment2d& segment, std::size_t partCount);
    [[nodiscard]] GEOMETRY_API std::vector<SCPoint2d> Sample(const SCArcSegment2d& segment, std::size_t partCount);
    [[nodiscard]] GEOMETRY_API std::vector<SCPoint2d> Sample(const ISCSegment2d& segment, std::size_t partCount);
    [[nodiscard]] GEOMETRY_API std::vector<SCPoint2d> SampleByMaxAngle(const SCArcSegment2d& segment,
                                                                     double maxAngleRadians);
}  // namespace Geometry
