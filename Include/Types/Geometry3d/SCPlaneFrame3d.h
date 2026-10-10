#pragma once

#include "Export/GeometryExport.h"
#include "Types/Geometry3d/SCPoint3.h"
#include "Types/Geometry3d/SCVector3.h"

namespace Geometry
{
    struct GEOMETRY_API SCPlaneFrame3d
    {
        SCPoint3d origin{};
        SCVector3d u{};
        SCVector3d v{};
        SCVector3d n{};
    };
}
