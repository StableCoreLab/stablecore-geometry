#pragma once

#include <span>

#include "Core/GeometryTypesPrimitives.h"
#include "Core/Geometry3dResults.h"
#include "Export/GeometryExport.h"
#include "Geometry3d/ISCCurve3d.h"
#include "Geometry3d/SCLineCurve3d.h"
#include "Types/Geometry3d/SCLineSegment3d.h"
#include "Types/Geometry3d/SCPoint3.h"

namespace Geometry
{
    // 3D snap result is declared in Geometry3dResults.h.
    [[nodiscard]] GEOMETRY_API SCSnapResult3d SnapPointToSegments3d(const SCPoint3d& point,
                                                                    std::span<const ISCCurve3d* const> curves,
                                                                    double maxDistance,
                                                                    const SCGeometryTolerance3d& tolerance = {});

    [[nodiscard]] GEOMETRY_API SCSnapResult3d SnapPointToSegments3d(const SCPoint3d& point,
                                                                    std::span<const SCLineSegment3d* const> segments,
                                                                    double maxDistance,
                                                                    const SCGeometryTolerance3d& tolerance = {});
}  // namespace Geometry
