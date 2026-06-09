#include "Core/Relation.h"

#include <cmath>

#include "Core/Intersection.h"
#include "Core/Projection.h"

namespace Geometry
{
    SCPointPlaneSide3d LocatePoint(const SCPoint3d& point, const SCPlane& plane, const SCGeometryTolerance3d& tolerance)
    {
        if (!plane.IsValid(tolerance.distanceEpsilon))
        {
            return SCPointPlaneSide3d::OnPlane;
        }

        const double signedDistance = plane.SignedDistanceTo(point, tolerance.distanceEpsilon);
        if (signedDistance > tolerance.distanceEpsilon)
        {
            return SCPointPlaneSide3d::Above;
        }
        if (signedDistance < -tolerance.distanceEpsilon)
        {
            return SCPointPlaneSide3d::Below;
        }
        return SCPointPlaneSide3d::OnPlane;
    }

    SCPointContainment2d LocatePoint(const SCPoint3d& point, const ISCCurve3d& curve, const SCGeometryTolerance3d& tolerance)
    {
        if (!curve.IsValid(tolerance))
        {
            return SCPointContainment2d::Outside;
        }

        const SCCurveProjection3d projection = ProjectPointToCurve(point, curve, tolerance);
        if (!projection.success)
        {
            return SCPointContainment2d::Outside;
        }

        return projection.distanceSquared <= tolerance.distanceEpsilon * tolerance.distanceEpsilon
                   ? SCPointContainment2d::OnBoundary
                   : SCPointContainment2d::Outside;
    }

    SCPointContainment2d LocatePoint(const SCPoint3d& point,
                                   const SCCurveOnSurface& curveOnSurface,
                                   const SCGeometryTolerance3d& tolerance)
    {
        if (!curveOnSurface.IsValid(tolerance))
        {
            return SCPointContainment2d::Outside;
        }

        const SCCurveOnSurfaceProjection3d projection = ProjectPointToCurveOnSurface(point, curveOnSurface, tolerance);
        if (!projection.success)
        {
            return SCPointContainment2d::Outside;
        }

        return projection.distanceSquared <= tolerance.distanceEpsilon * tolerance.distanceEpsilon
                   ? SCPointContainment2d::OnBoundary
                   : SCPointContainment2d::Outside;
    }

    SCPointContainment2d LocatePoint(const SCPoint3d& point,
                                   const PolyhedronFace3d& face,
                                   const SCGeometryTolerance3d& tolerance)
    {
        if (!face.IsValid(tolerance.distanceEpsilon))
        {
            return SCPointContainment2d::Outside;
        }

        if (LocatePoint(point, face.SupportPlane(), tolerance) != SCPointPlaneSide3d::OnPlane)
        {
            return SCPointContainment2d::Outside;
        }

        const SCFaceProjection3d projection = ProjectFaceToPolygon2d(face, tolerance);
        if (!projection.success)
        {
            return SCPointContainment2d::Outside;
        }

        const SCVector3d delta = point - projection.origin;
        return LocatePoint(SCPoint2d{Dot(delta, projection.uAxis), Dot(delta, projection.vAxis)},
                           projection.polygon,
                           tolerance.distanceEpsilon);
    }

    SCPointContainment2d LocatePoint(const SCPoint3d& point,
                                   const PolyhedronBody& body,
                                   const SCGeometryTolerance3d& tolerance)
    {
        if (!body.IsValid(tolerance.distanceEpsilon))
        {
            return SCPointContainment2d::Outside;
        }

        const SCPolyhedronBodyProjection3d projection = ProjectPointToPolyhedronBody(point, body, tolerance);
        if (projection.success &&
            projection.projection.distanceSquared <= tolerance.distanceEpsilon * tolerance.distanceEpsilon)
        {
            return SCPointContainment2d::OnBoundary;
        }

        const SCLine3d probe = SCLine3d::FromOriginAndDirection(point, SCVector3d{1.0, 0.3713906763541037, 0.52999894000318});
        const SCLinePolyhedronBodyIntersection3d intersections = Intersect(probe, body, tolerance);
        if (!intersections.intersects)
        {
            return SCPointContainment2d::Outside;
        }

        std::size_t crossingCount = 0;
        for (const SCLinePolyhedronFaceIntersection3d& hit : intersections.hits)
        {
            if (hit.lineParameter > tolerance.parameterEpsilon)
            {
                ++crossingCount;
            }
        }

        return (crossingCount % 2 == 1) ? SCPointContainment2d::Inside : SCPointContainment2d::Outside;
    }

    SCPointContainment2d LocatePoint(const SCPoint3d& point, const SCBrepFace& face, const SCGeometryTolerance3d& tolerance)
    {
        if (!face.IsValid(tolerance) || face.SupportSurface() == nullptr)
        {
            return SCPointContainment2d::Outside;
        }

        const SCBrepFaceProjection3d projection = ProjectPointToBrepFace(point, face, tolerance);
        if (!projection.success || projection.distanceSquared > tolerance.distanceEpsilon * tolerance.distanceEpsilon)
        {
            return SCPointContainment2d::Outside;
        }

        if (projection.onBoundary)
        {
            return SCPointContainment2d::OnBoundary;
        }

        return projection.onTrimmedFace ? SCPointContainment2d::Inside : SCPointContainment2d::Outside;
    }

    SCPointContainment2d LocatePoint(const SCPoint3d& point, const SCBrepBody& body, const SCGeometryTolerance3d& tolerance)
    {
        if (!body.IsValid(tolerance))
        {
            return SCPointContainment2d::Outside;
        }

        const SCBrepBodyProjection3d projection = ProjectPointToBrepBody(point, body, tolerance);
        if (projection.success &&
            projection.projection.distanceSquared <= tolerance.distanceEpsilon * tolerance.distanceEpsilon)
        {
            return SCPointContainment2d::OnBoundary;
        }

        const SCLine3d probe = SCLine3d::FromOriginAndDirection(point, SCVector3d{1.0, 0.3713906763541037, 0.52999894000318});
        const SCLineBrepBodyIntersection3d intersections = Intersect(probe, body, tolerance);
        if (!intersections.intersects)
        {
            return SCPointContainment2d::Outside;
        }

        std::size_t crossingCount = 0;
        for (const SCLineBrepFaceIntersection3d& hit : intersections.hits)
        {
            if (hit.lineParameter > tolerance.parameterEpsilon)
            {
                ++crossingCount;
            }
        }

        return (crossingCount % 2 == 1) ? SCPointContainment2d::Inside : SCPointContainment2d::Outside;
    }

    SCPointContainment2d LocatePoint(const SCPoint3d& point, const TriangleMesh& mesh, const SCGeometryTolerance3d& tolerance)
    {
        if (!mesh.IsValid(tolerance.distanceEpsilon))
        {
            return SCPointContainment2d::Outside;
        }

        const SCTriangleMeshProjection3d projection = ProjectPointToTriangleMesh(point, mesh, tolerance);
        if (projection.success && projection.distanceSquared <= tolerance.distanceEpsilon * tolerance.distanceEpsilon)
        {
            return SCPointContainment2d::OnBoundary;
        }

        const SCLine3d probe = SCLine3d::FromOriginAndDirection(point, SCVector3d{1.0, 0.3713906763541037, 0.52999894000318});
        const SCLineTriangleMeshIntersection3d intersections = Intersect(probe, mesh, tolerance);
        if (!intersections.intersects)
        {
            return SCPointContainment2d::Outside;
        }

        std::size_t crossingCount = 0;
        for (double parameter : intersections.lineParameters)
        {
            if (parameter > tolerance.parameterEpsilon)
            {
                ++crossingCount;
            }
        }

        return (crossingCount % 2 == 1) ? SCPointContainment2d::Inside : SCPointContainment2d::Outside;
    }

    bool IsParallel(const SCVector3d& first, const SCVector3d& second, const SCGeometryTolerance3d& tolerance)
    {
        const double firstLength = first.Length();
        const double secondLength = second.Length();
        if (firstLength <= tolerance.distanceEpsilon || secondLength <= tolerance.distanceEpsilon)
        {
            return false;
        }

        const SCVector3d cross = Cross(first, second);
        return cross.Length() <= tolerance.angleEpsilon * firstLength * secondLength + tolerance.distanceEpsilon;
    }

    bool IsPerpendicular(const SCVector3d& first, const SCVector3d& second, const SCGeometryTolerance3d& tolerance)
    {
        const double firstLength = first.Length();
        const double secondLength = second.Length();
        if (firstLength <= tolerance.distanceEpsilon || secondLength <= tolerance.distanceEpsilon)
        {
            return false;
        }

        return std::abs(Dot(first, second)) <=
               tolerance.angleEpsilon * firstLength * secondLength + tolerance.distanceEpsilon;
    }
}  // namespace Geometry


