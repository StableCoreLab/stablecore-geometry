#pragma once

#include <cstddef>

#include "Export/GeometryExport.h"

namespace Geometry
{
    template <typename T>
    struct SCPoint2;

    template <typename T>
    struct SCVector2;

    template <typename T>
    struct SCBox2;

    template <typename T>
    struct SCPoint3;

    template <typename T>
    struct SCVector3;

    template <typename T>
    class SCBox3;

    using SCBox3d = SCBox3<double>;

    struct SCIntervald;

    struct SCLine3d;

    struct SCLineSegment3d;

    struct SCMatrix3d;

    struct SCPlane;

    struct SCRay3d;

    struct SCTransform3d;

    struct SCTriangle3d;

    struct SCDirection3d;

    struct SCGeometryTolerance3d;
    struct SCCurveEval3d;
    struct SCSurfaceEval3d;

}  // namespace Geometry

