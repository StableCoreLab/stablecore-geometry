#include "Geometry.h"

#include <array>
#include <memory>
#include <numbers>

#if defined(SCGEOMETRY_EXPECT_SHARED) != defined(GEOMETRY_USE_DLL)
#error "The installed SCGeometry target publishes the wrong DLL import definition."
#endif

int main()
{
    using namespace Geometry;
    const SCLineSegment2d first{{0.0, 0.0}, {10.0, 0.0}};
    const SCLineSegment2d second{{2.0, 0.0}, {8.0, 0.0}};
    const auto projection = ClassifyParallelSegmentProjection(
        first, second, SCParallelSegmentProjectionTolerance2d{});
    const SCBox2d box = SCBox2d::FromMinMax({0.0, 0.0}, {10.0, 10.0});
    const SCPolyline2d path({{2.0, 2.0}, {8.0, 8.0}}, SCPolylineClosure::Open);
    const SCPolygon2d polygon(SCPolyline2d(
        {{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}}, SCPolylineClosure::Closed));
    const auto area = QueryPolygonPositiveAreaIntersection(polygon, polygon);
    const SCPathGeometryTolerance pathTolerance{1e-8, 1e-8, 1e-12};
    const std::array<SCPoint3d, 3> planePoints{{{0,0,0}, {1,0,0}, {0,1,0}}};
    const auto fittedPlane = FitPlane(planePoints, pathTolerance);
    const auto arc = CreateArcSegment3d({2,0,0}, {0,2,0}, {0,0,0}, {0,0,1},
                                        2.0, std::numbers::pi_v<double> / 2.0, pathTolerance);
    const SCPlaneFrame3d xy{{0,0,0}, {1,0,0}, {0,1,0}, {0,0,1}};
    const auto projectedArc = arc.success ? ProjectArcToFrame(arc.arc, xy, pathTolerance)
                                          : SCArcProjectionResult{};
    const auto mappedOffset = OffsetWithMapping(path, 0.0, {4.0}, pathTolerance);
    const SCLineSegment2d cornerExit{{10.0, 0.0}, {10.0, 1.0}};
    const auto corner = EvaluateCornerGeometry(first, cornerExit, pathTolerance);
    const auto legacyOffset = Offset(first, 1.0);
    const SCArcSegment2d planarArc({0.0, 0.0}, 1.0, 0.0, std::numbers::pi_v<double> / 2.0);
    const auto normalized = Normalize(path);
    const SCNurbsCurve3d nurbsCurve(1, {{0, 0, 0}, {2, 0, 0}}, {0, 0, 1, 1});
    const SCNurbsSurface nurbsSurface(1, 1, 2, 2,
                                      {{0, 0, 0}, {2, 0, 0}, {0, 2, 0}, {2, 2, 0}},
                                      {0, 0, 1, 1}, {0, 0, 1, 1});
    const bool nurbsAvailable = nurbsCurve.IsValid() && nurbsSurface.IsValid() &&
        nurbsCurve.PointAt(0.5).AlmostEquals({1, 0, 0}) &&
        nurbsSurface.PointAt(0.5, 0.5).AlmostEquals({1, 1, 0});
    const auto curveSurface = std::make_shared<SCPlaneSurface>(SCPlaneSurface::FromPlane(
        SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1}), {-2, 2}, {-2, 2}));
    const SCCurveOnSurface composedCurve(curveSurface, SCPolyline2d({{0, 0}, {1, 0}}));
    const ISCCurve3d& genericCurve = composedCurve;
    const bool genericCurveAvailable = genericCurve.IsValid() &&
        genericCurve.PointAt(0.5).AlmostEquals({0.5, 0, 0});
    const SCOffsetSurface offsetSurface(curveSurface, 0.2);
    const auto offsetEval = offsetSurface.Evaluate(0.0, 0.0, 1);
    const bool offsetAvailable = offsetEval.IsValid() &&
        offsetEval.point.AlmostEquals({0.0, 0.0, 0.2});
    return projection == SCParallelSegmentProjectionRelation2d::PositiveLengthIntersection &&
                   Contains(box, path) && Intersects(box, path) && area.success &&
                   area.hasPositiveAreaIntersection && fittedPlane.success && arc.success &&
                   projectedArc.success && mappedOffset.success && corner.success &&
                   legacyOffset.IsValid() && planarArc.AlmostEquals(planarArc, kDefaultEpsilon, kDefaultAngleEpsilon) &&
                   normalized.IsValid() && nurbsAvailable && genericCurveAvailable &&
                   offsetAvailable
               ? 0
               : 1;
}
