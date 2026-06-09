#pragma once

#include "Core/GeometryTypes.h"
#include "Export/GeometryExport.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Geometry2d/ISCSegment2d.h"
#include "Support/Epsilon.h"

namespace Geometry
{
    [[nodiscard]] GEOMETRY_API double DistanceSquared(const SCPoint2d& lhs, const SCPoint2d& rhs);
    [[nodiscard]] GEOMETRY_API double Distance(const SCPoint2d& lhs, const SCPoint2d& rhs);

    [[nodiscard]] GEOMETRY_API double DistanceSquared(const SCPoint2d& point, const SCLineSegment2d& segment);
    [[nodiscard]] GEOMETRY_API double DistanceSquared(const SCPoint2d& point, const SCArcSegment2d& segment);
    [[nodiscard]] GEOMETRY_API double DistanceSquared(const SCPoint2d& point, const ISCSegment2d& segment);

    [[nodiscard]] GEOMETRY_API double Distance(const SCPoint2d& point, const SCLineSegment2d& segment);
    [[nodiscard]] GEOMETRY_API double Distance(const SCPoint2d& point, const SCArcSegment2d& segment);
    [[nodiscard]] GEOMETRY_API double Distance(const SCPoint2d& point, const ISCSegment2d& segment);

    [[nodiscard]] GEOMETRY_API bool Contains(const SCBox2d& box,
                                             const SCPoint2d& point,
                                             double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API bool Intersects(const SCBox2d& lhs,
                                               const SCBox2d& rhs,
                                               double eps = Geometry::kDefaultEpsilon);
}  // namespace Geometry
