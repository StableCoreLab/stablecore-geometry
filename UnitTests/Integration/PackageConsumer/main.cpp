#include "Geometry.h"

#include <array>
#include <numbers>

#ifndef GEOMETRY_USE_DLL
#error "The installed SCGeometry target must publish GEOMETRY_USE_DLL."
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
    const auto normalized = Normalize(path);
    return projection == SCParallelSegmentProjectionRelation2d::PositiveLengthIntersection &&
                   Contains(box, path) && Intersects(box, path) && area.success &&
                   area.hasPositiveAreaIntersection && fittedPlane.success && arc.success &&
                   projectedArc.success && mappedOffset.success && corner.success &&
                   legacyOffset.IsValid() && normalized.IsValid()
               ? 0
               : 1;
}
