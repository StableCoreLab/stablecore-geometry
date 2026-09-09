#include "Geometry.h"

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
    return projection == SCParallelSegmentProjectionRelation2d::PositiveLengthIntersection &&
                   Contains(box, path) && Intersects(box, path) && area.success && area.hasPositiveAreaIntersection
               ? 0
               : 1;
}
