#pragma once

#include <memory>
#include <vector>

#include "Geometry.h"

namespace Geometry::Test
{
    inline Geometry::SCPlane BuildSupportPlaneAtZ5()
    {
        return Geometry::SCPlane::FromPointAndNormal(Geometry::SCPoint3d{0.0, 0.0, 5.0},
                                                     Geometry::SCVector3d{0.0, 0.0, 1.0});
    }

    inline Geometry::SCPlaneSurface BuildFinitePlaneSurfaceAtZ5()
    {
        return Geometry::SCPlaneSurface::FromPlane(
            BuildSupportPlaneAtZ5(), Geometry::SCIntervald{-2.0, 2.0}, Geometry::SCIntervald{-3.0, 1.0});
    }

    inline Geometry::SCNurbsSurface BuildUnitNurbsSurface()
    {
        return Geometry::SCNurbsSurface(
            1,
            1,
            2,
            2,
            {Geometry::SCPoint3d{0.0, 0.0, 0.0}, Geometry::SCPoint3d{2.0, 0.0, 0.0},
             Geometry::SCPoint3d{0.0, 2.0, 0.0}, Geometry::SCPoint3d{2.0, 2.0, 0.0}},
            {0.0, 0.0, 1.0, 1.0},
            {0.0, 0.0, 1.0, 1.0});
    }

    inline std::vector<Geometry::SCBrepVertex> BuildPlanarBrepVerticesAtZ5()
    {
        return {Geometry::SCBrepVertex(Geometry::SCPoint3d{0.0, 0.0, 5.0}),
                Geometry::SCBrepVertex(Geometry::SCPoint3d{2.0, 0.0, 5.0}),
                Geometry::SCBrepVertex(Geometry::SCPoint3d{2.0, 2.0, 5.0}),
                Geometry::SCBrepVertex(Geometry::SCPoint3d{0.0, 2.0, 5.0})};
    }

    inline std::vector<Geometry::SCBrepEdge> BuildPlanarBrepEdgesAtZ5()
    {
        using namespace Geometry;
        return {SCBrepEdge(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                             SCLine3d::FromOriginAndDirection(SCPoint3d{0.0, 0.0, 5.0}, SCVector3d{2.0, 0.0, 0.0}),
                             SCIntervald{0.0, 1.0})), 0, 1),
                SCBrepEdge(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                             SCLine3d::FromOriginAndDirection(SCPoint3d{2.0, 0.0, 5.0}, SCVector3d{0.0, 2.0, 0.0}),
                             SCIntervald{0.0, 1.0})), 1, 2),
                SCBrepEdge(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                             SCLine3d::FromOriginAndDirection(SCPoint3d{2.0, 2.0, 5.0}, SCVector3d{-2.0, 0.0, 0.0}),
                             SCIntervald{0.0, 1.0})), 2, 3),
                SCBrepEdge(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                             SCLine3d::FromOriginAndDirection(SCPoint3d{0.0, 2.0, 5.0}, SCVector3d{0.0, -2.0, 0.0}),
                             SCIntervald{0.0, 1.0})), 3, 0)};
    }

    inline Geometry::SCBrepLoop BuildPlanarBrepOuterLoop()
    {
        return Geometry::SCBrepLoop({Geometry::SCBrepCoedge(0, false), Geometry::SCBrepCoedge(1, false),
                                     Geometry::SCBrepCoedge(2, false), Geometry::SCBrepCoedge(3, false)});
    }

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
