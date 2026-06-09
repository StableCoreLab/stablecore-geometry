#pragma once

#include "Geometry.h"

namespace Geometry::Test
{
    inline Geometry::PolyhedronBody BuildUnitCubeBody()
    {
        using Geometry::SCPlane;
        using Geometry::SCPoint3d;
        using Geometry::PolyhedronBody;
        using Geometry::PolyhedronFace3d;
        using Geometry::PolyhedronLoop3d;
        using Geometry::SCVector3d;

        return Geometry::PolyhedronBody({
            PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, -1.0}),
                             PolyhedronLoop3d({
                                 SCPoint3d{0.0, 0.0, 0.0},
                                 SCPoint3d{0.0, 1.0, 0.0},
                                 SCPoint3d{1.0, 1.0, 0.0},
                                 SCPoint3d{1.0, 0.0, 0.0},
                             })),
            PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 1.0}, SCVector3d{0.0, 0.0, 1.0}),
                             PolyhedronLoop3d({
                                 SCPoint3d{0.0, 0.0, 1.0},
                                 SCPoint3d{1.0, 0.0, 1.0},
                                 SCPoint3d{1.0, 1.0, 1.0},
                                 SCPoint3d{0.0, 1.0, 1.0},
                             })),
            PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, -1.0, 0.0}),
                             PolyhedronLoop3d({
                                 SCPoint3d{0.0, 0.0, 0.0},
                                 SCPoint3d{1.0, 0.0, 0.0},
                                 SCPoint3d{1.0, 0.0, 1.0},
                                 SCPoint3d{0.0, 0.0, 1.0},
                             })),
            PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{1.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
                             PolyhedronLoop3d({
                                 SCPoint3d{1.0, 0.0, 0.0},
                                 SCPoint3d{1.0, 1.0, 0.0},
                                 SCPoint3d{1.0, 1.0, 1.0},
                                 SCPoint3d{1.0, 0.0, 1.0},
                             })),
            PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 1.0, 0.0}, SCVector3d{0.0, 1.0, 0.0}),
                             PolyhedronLoop3d({
                                 SCPoint3d{0.0, 1.0, 0.0},
                                 SCPoint3d{0.0, 1.0, 1.0},
                                 SCPoint3d{1.0, 1.0, 1.0},
                                 SCPoint3d{1.0, 1.0, 0.0},
                             })),
            PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{-1.0, 0.0, 0.0}),
                             PolyhedronLoop3d({
                                 SCPoint3d{0.0, 0.0, 0.0},
                                 SCPoint3d{0.0, 0.0, 1.0},
                                 SCPoint3d{0.0, 1.0, 1.0},
                                 SCPoint3d{0.0, 1.0, 0.0},
                             })),
        });
    }
}  // namespace Geometry::Test

