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
#include "Geometry3d/Detail/SurfaceNumerics.h"

namespace Geometry
{
    SCPointPlaneSide3d LocatePoint(const SCPoint3d& point, const SCPlane& target,
        const SCGeometryTolerance3d& tolerance)
    {
        const auto result=LocatePointWithStatus(point,target,tolerance);
        return result.payloadValid ? result.payload : SCPointPlaneSide3d::OnPlane;
    }

    SCPointContainment3d LocatePoint(const SCPoint3d& point, const ISCCurve3d& target,
        const SCGeometryTolerance3d& tolerance)
    {
        const auto result=LocatePointWithStatus(point,target,tolerance);
        return result.payloadValid ? result.payload : SCPointContainment3d::Outside;
    }

    SCPointContainment3d LocatePoint(const SCPoint3d& point, const SCCurveOnSurface& target,
        const SCGeometryTolerance3d& tolerance)
    {
        const auto result=LocatePointWithStatus(point,target,tolerance);
        return result.payloadValid ? result.payload : SCPointContainment3d::Outside;
    }

    SCPointContainment3d LocatePoint(const SCPoint3d& point, const PolyhedronFace3d& target,
        const SCGeometryTolerance3d& tolerance)
    {
        const auto result=LocatePointWithStatus(point,target,tolerance);
        return result.payloadValid ? result.payload : SCPointContainment3d::Outside;
    }

    SCPointContainment3d LocatePoint(const SCPoint3d& point, const PolyhedronBody& target,
        const SCGeometryTolerance3d& tolerance)
    {
        const auto result=LocatePointWithStatus(point,target,tolerance);
        return result.payloadValid ? result.payload : SCPointContainment3d::Outside;
    }

    SCPointContainment3d LocatePoint(const SCPoint3d& point, const SCBrepFace& target,
        const SCGeometryTolerance3d& tolerance)
    {
        const auto result=LocatePointWithStatus(point,target,tolerance);
        return result.payloadValid ? result.payload : SCPointContainment3d::Outside;
    }

    SCPointContainment3d LocatePoint(const SCPoint3d& point, const SCBrepBody& target,
        const SCGeometryTolerance3d& tolerance)
    {
        const auto result=LocatePointWithStatus(point,target,tolerance);
        return result.payloadValid ? result.payload : SCPointContainment3d::Outside;
    }

    SCPointContainment3d LocatePoint(const SCPoint3d& point, const TriangleMesh& target,
        const SCGeometryTolerance3d& tolerance)
    {
        const auto result=LocatePointWithStatus(point,target,tolerance);
        return result.payloadValid ? result.payload : SCPointContainment3d::Outside;
    }

    bool IsParallel(const SCVector3d& first, const SCVector3d& second, const SCGeometryTolerance3d& tolerance)
    {
        SCVector3d a,b;
        if (!tolerance.IsValid() || !Detail::TryUnitVector(first,a) || !Detail::TryUnitVector(second,b))
        {
            return false;
        }

        const SCVector3d cross = Cross(a,b);
        return std::hypot(cross.x,cross.y,cross.z) <= tolerance.angleEpsilon;
    }

    bool IsPerpendicular(const SCVector3d& first, const SCVector3d& second, const SCGeometryTolerance3d& tolerance)
    {
        SCVector3d a,b;
        if (!tolerance.IsValid() || !Detail::TryUnitVector(first,a) || !Detail::TryUnitVector(second,b))
        {
            return false;
        }

        return std::abs(Dot(a,b)) <= tolerance.angleEpsilon;
    }
}  // namespace Geometry


