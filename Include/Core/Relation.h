#pragma once

#include "Brep/SCBrepBody.h"
#include "Brep/SCBrepFace.h"
#include "Brep/PolyhedronBody.h"
#include "Brep/PolyhedronFace3d.h"
#include "Core/GeometryTypes.h"
#include "Export/GeometryExport.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Geometry2d/SCPolygon2d.h"
#include "Geometry2d/SCPolyline2d.h"
#include "Geometry2d/ISCSegment2d.h"
#include "Geometry3d/ISCCurve3d.h"
#include "Geometry3d/SCCurveOnSurface.h"
#include "Geometry3d/TriangleMesh.h"
#include "Support/Epsilon.h"

namespace Geometry
{
    enum class SCPointContainment2d
    {
        Outside,
        OnBoundary,
        Inside
    };

    enum class SCPointPlaneSide3d
    {
        Below,
        OnPlane,
        Above
    };

    [[nodiscard]] GEOMETRY_API SCPointContainment2d LocatePoint(const SCPoint2d& point,
                                                              const SCLineSegment2d& segment,
                                                              double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API SCPointContainment2d LocatePoint(const SCPoint2d& point,
                                                              const SCArcSegment2d& segment,
                                                              double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API SCPointContainment2d LocatePoint(const SCPoint2d& point,
                                                              const ISCSegment2d& segment,
                                                              double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API SCPointContainment2d LocatePoint(const SCPoint2d& point,
                                                              const SCPolyline2d& polyline,
                                                              double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API SCPointContainment2d LocatePoint(const SCPoint2d& point,
                                                              const SCPolygon2d& polygon,
                                                              double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API SCPointPlaneSide3d LocatePoint(const SCPoint3d& point,
                                                            const SCPlane& plane,
                                                            const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCPointContainment2d LocatePoint(const SCPoint3d& point,
                                                              const ISCCurve3d& curve,
                                                              const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCPointContainment2d LocatePoint(const SCPoint3d& point,
                                                              const SCCurveOnSurface& curveOnSurface,
                                                              const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCPointContainment2d LocatePoint(const SCPoint3d& point,
                                                              const PolyhedronFace3d& face,
                                                              const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCPointContainment2d LocatePoint(const SCPoint3d& point,
                                                              const PolyhedronBody& body,
                                                              const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCPointContainment2d LocatePoint(const SCPoint3d& point,
                                                              const SCBrepFace& face,
                                                              const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCPointContainment2d LocatePoint(const SCPoint3d& point,
                                                              const SCBrepBody& body,
                                                              const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCPointContainment2d LocatePoint(const SCPoint3d& point,
                                                              const TriangleMesh& mesh,
                                                              const SCGeometryTolerance3d& tolerance = {});

    [[nodiscard]] GEOMETRY_API bool IsParallel(const SCLineSegment2d& first,
                                               const SCLineSegment2d& second,
                                               double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API bool IsAntiParallel(const SCLineSegment2d& first,
                                                   const SCLineSegment2d& second,
                                                   double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API bool IsSameDirection(const SCLineSegment2d& first,
                                                    const SCLineSegment2d& second,
                                                    double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API bool IsEqual(const SCLineSegment2d& first,
                                            const SCLineSegment2d& second,
                                            bool ignoreDirection = true,
                                            double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API bool IsParallel(const SCVector3d& first,
                                               const SCVector3d& second,
                                               const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API bool IsPerpendicular(const SCVector3d& first,
                                                    const SCVector3d& second,
                                                    const SCGeometryTolerance3d& tolerance = {});
}  // namespace Geometry


