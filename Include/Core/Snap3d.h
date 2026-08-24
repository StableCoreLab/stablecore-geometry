#pragma once

#include <span>

#include "Core/GeometryTypesPrimitives.h"
#include "Export/GeometryExport.h"
#include "Geometry3d/ISCCurve3d.h"
#include "Geometry3d/SCLineCurve3d.h"
#include "Types/Geometry3d/SCLineSegment3d.h"
#include "Types/Geometry3d/SCPoint3.h"

namespace Geometry
{
    struct GEOMETRY_API SCSnapResult3d
    {
        bool snapped{false};
        SCPoint3d point{};
        double distanceSquared{0.0};
        std::size_t segmentIndex{0};
        double parameter{0.0};

        [[nodiscard]] bool IsValid() const
        {
            return !snapped || point.IsValid();
        }
    };

    [[nodiscard]] GEOMETRY_API SCSnapResult3d SnapPointToSegments3d(const SCPoint3d& point,
                                                                    std::span<const ISCCurve3d* const> curves,
                                                                    double maxDistance,
                                                                    const SCGeometryTolerance3d& tolerance = {});

    [[nodiscard]] GEOMETRY_API SCSnapResult3d SnapPointToSegments3d(const SCPoint3d& point,
                                                                    std::span<const SCLineSegment3d* const> segments,
                                                                    double maxDistance,
                                                                    const SCGeometryTolerance3d& tolerance = {});
}  // namespace Geometry
