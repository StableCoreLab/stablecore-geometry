#include "Core/Relation.h"

#include <cmath>

#include "Core/Intersection.h"
#include "Core/Projection.h"
#include "Brep/PolyhedronBody.h"
#include "Brep/PolyhedronFace3d.h"
#include "Brep/SCBrepBody.h"
#include "Brep/SCBrepFace.h"
#include "Geometry3d/ISCCurve3d.h"
#include "Geometry3d/TriangleMesh.h"

namespace Geometry
{
    namespace
    {
        const SCVector3d kContainmentProbeDirection{1.0, 0.3713906763541037, 0.52999894000318};

        template <typename ProjectBody, typename IntersectBody>
        [[nodiscard]] SCPointContainment3d LocatePointInClosedBody(const SCPoint3d& point,
                                                                   const SCGeometryTolerance3d& tolerance,
                                                                   ProjectBody&& projectBody,
                                                                   IntersectBody&& intersectBody)
        {
            const auto projection = projectBody();
            if (projection.success &&
                projection.projection.distanceSquared <= tolerance.distanceEpsilon * tolerance.distanceEpsilon)
            {
                return SCPointContainment3d::OnBoundary;
            }

            const SCLine3d probe = SCLine3d::FromOriginAndDirection(point, kContainmentProbeDirection);
            const auto intersections = intersectBody(probe);
            if (!intersections.intersects)
            {
                return SCPointContainment3d::Outside;
            }

            std::size_t crossingCount = 0;
            for (const auto& hit : intersections.hits)
            {
                if (hit.lineParameter > tolerance.parameterEpsilon)
                {
                    ++crossingCount;
                }
            }
            return (crossingCount % 2 == 1) ? SCPointContainment3d::Inside : SCPointContainment3d::Outside;
        }
    }  // namespace

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

    SCPointContainment3d LocatePoint(const SCPoint3d& point, const ISCCurve3d& curve, const SCGeometryTolerance3d& tolerance)
    {
        if (!curve.IsValid(tolerance))
        {
            return SCPointContainment3d::Outside;
        }

        const SCCurveProjection3d projection = ProjectPointToCurve(point, curve, tolerance);
        if (!projection.success)
        {
            return SCPointContainment3d::Outside;
        }

        return projection.distanceSquared <= tolerance.distanceEpsilon * tolerance.distanceEpsilon
                   ? SCPointContainment3d::OnBoundary
                   : SCPointContainment3d::Outside;
    }

    SCPointContainment3d LocatePoint(const SCPoint3d& point,
                                   const SCCurveOnSurface& curveOnSurface,
                                   const SCGeometryTolerance3d& tolerance)
    {
        if (!curveOnSurface.IsValid(tolerance))
        {
            return SCPointContainment3d::Outside;
        }

        const SCCurveOnSurfaceProjection3d projection = ProjectPointToCurveOnSurface(point, curveOnSurface, tolerance);
        if (!projection.success)
        {
            return SCPointContainment3d::Outside;
        }

        return projection.distanceSquared <= tolerance.distanceEpsilon * tolerance.distanceEpsilon
                   ? SCPointContainment3d::OnBoundary
                   : SCPointContainment3d::Outside;
    }

    SCPointContainment3d LocatePoint(const SCPoint3d& point,
                                   const PolyhedronFace3d& face,
                                   const SCGeometryTolerance3d& tolerance)
    {
        if (!face.IsValid(tolerance))
        {
            return SCPointContainment3d::Outside;
        }

        if (LocatePoint(point, face.SupportPlane(), tolerance) != SCPointPlaneSide3d::OnPlane)
        {
            return SCPointContainment3d::Outside;
        }

        const SCFaceProjection3d projection = ProjectFaceToPolygon2d(face, tolerance);
        if (!projection.success)
        {
            return SCPointContainment3d::Outside;
        }

        const SCVector3d delta = point - projection.origin;
        const SCPointContainment2d containment = LocatePoint(SCPoint2d{Dot(delta, projection.uAxis), Dot(delta, projection.vAxis)},
                                                               projection.polygon,
                                                               tolerance.distanceEpsilon);
        return containment == SCPointContainment2d::Inside
                   ? SCPointContainment3d::Inside
                   : (containment == SCPointContainment2d::OnBoundary ? SCPointContainment3d::OnBoundary
                                                                     : SCPointContainment3d::Outside);
    }

    SCPointContainment3d LocatePoint(const SCPoint3d& point,
                                   const PolyhedronBody& body,
                                   const SCGeometryTolerance3d& tolerance)
    {
        if (!tolerance.IsValid() || !body.IsValid(tolerance))
        {
            return SCPointContainment3d::Outside;
        }
        return LocatePointInClosedBody(
            point,
            tolerance,
            [&] { return ProjectPointToPolyhedronBody(point, body, tolerance); },
            [&](const SCLine3d& probe) { return Intersect(probe, body, tolerance); });
    }

    SCPointContainment3d LocatePoint(const SCPoint3d& point, const SCBrepFace& face, const SCGeometryTolerance3d& tolerance)
    {
        if (!face.IsValid(tolerance) || face.SupportSurface() == nullptr)
        {
            return SCPointContainment3d::Outside;
        }

        const SCBrepFaceProjection3d projection = ProjectPointToBrepFace(point, face, tolerance);
        if (!projection.success || projection.distanceSquared > tolerance.distanceEpsilon * tolerance.distanceEpsilon)
        {
            return SCPointContainment3d::Outside;
        }

        if (projection.onBoundary)
        {
            return SCPointContainment3d::OnBoundary;
        }

        return projection.onTrimmedFace ? SCPointContainment3d::Inside : SCPointContainment3d::Outside;
    }

    SCPointContainment3d LocatePoint(const SCPoint3d& point, const SCBrepBody& body, const SCGeometryTolerance3d& tolerance)
    {
        if (!body.IsValid(tolerance))
        {
            return SCPointContainment3d::Outside;
        }

        return LocatePointInClosedBody(
            point,
            tolerance,
            [&] { return ProjectPointToBrepBody(point, body, tolerance); },
            [&](const SCLine3d& probe) { return Intersect(probe, body, tolerance); });
    }

    SCPointContainment3d LocatePoint(const SCPoint3d& point, const TriangleMesh& mesh, const SCGeometryTolerance3d& tolerance)
    {
        if (!mesh.IsValid(tolerance.distanceEpsilon))
        {
            return SCPointContainment3d::Outside;
        }

        const SCTriangleMeshProjection3d projection = ProjectPointToTriangleMesh(point, mesh, tolerance);
        if (projection.success && projection.distanceSquared <= tolerance.distanceEpsilon * tolerance.distanceEpsilon)
        {
            return SCPointContainment3d::OnBoundary;
        }

        const SCLine3d probe = SCLine3d::FromOriginAndDirection(point, kContainmentProbeDirection);
        const SCLineTriangleMeshIntersection3d intersections = Intersect(probe, mesh, tolerance);
        if (!intersections.intersects)
        {
            return SCPointContainment3d::Outside;
        }

        std::size_t crossingCount = 0;
        for (double parameter : intersections.lineParameters)
        {
            if (parameter > tolerance.parameterEpsilon)
            {
                ++crossingCount;
            }
        }

        return (crossingCount % 2 == 1) ? SCPointContainment3d::Inside : SCPointContainment3d::Outside;
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


