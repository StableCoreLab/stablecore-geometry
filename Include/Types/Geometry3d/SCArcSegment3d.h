#pragma once

#include "Export/GeometryExport.h"
#include "Types/Geometry3d/SCPoint3.h"
#include "Types/Geometry3d/SCVector3.h"

namespace Geometry
{
    struct GEOMETRY_API SCArcSegment3d
    {
        SCPoint3d center{};
        SCVector3d unitNormal{};
        SCVector3d startRadialUnit{};
        double radius{0.0};
        double signedSweepRad{0.0};
    };
}
