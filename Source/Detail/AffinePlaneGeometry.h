#pragma once

#include <cmath>

#include "Geometry3d/Detail/SurfaceNumerics.h"
#include "Geometry3d/SCPlaneSurface.h"
#include "Types/Geometry2d/SCPoint2.h"

namespace Geometry::Detail
{
    [[nodiscard]] inline bool TryActualPlane(const SCPlaneSurface& surface,
                                             const SCGeometryTolerance3d& tolerance,
                                             SCPlane& plane)
    {
        if (!surface.IsValid(tolerance))
        {
            return false;
        }
        SCVector3d unitU, unitV, storedNormal;
        if (!TryUnitVector(surface.UAxis(), unitU) || !TryUnitVector(surface.VAxis(), unitV) ||
            !TryUnitVector(surface.SupportPlane().normal, storedNormal))
        {
            return false;
        }
        SCVector3d actualNormal;
        if (!TryUnitVector(Cross(unitU, unitV), actualNormal))
        {
            return false;
        }
        const double alignment = Dot(actualNormal, storedNormal);
        if (!std::isfinite(alignment) || alignment == 0.0)
        {
            return false;
        }
        if (alignment < 0.0)
        {
            actualNormal = actualNormal * -1.0;
        }
        plane = SCPlane::FromPointAndNormal(surface.PointAt(0.0, 0.0), actualNormal);
        return plane.IsValid(0.0);
    }

    [[nodiscard]] inline bool TryAffinePlaneUv(const SCPlaneSurface& surface,
                                               const SCPoint3d& point,
                                               const SCGeometryTolerance3d& tolerance,
                                               SCPoint2d& uv)
    {
        if (!surface.IsValid(tolerance) || !point.IsValid())
        {
            return false;
        }
        const SCVector3d uAxis = surface.UAxis();
        const SCVector3d vAxis = surface.VAxis();
        SCVector3d unitU, unitV;
        if (!TryUnitVector(uAxis, unitU) || !TryUnitVector(vAxis, unitV))
        {
            return false;
        }
        const SCVector3d cross = Cross(unitU, unitV);
        const double sine = std::hypot(cross.x, cross.y, cross.z);
        const double uLength = std::hypot(uAxis.x, uAxis.y, uAxis.z);
        const double vLength = std::hypot(vAxis.x, vAxis.y, vAxis.z);
        if (!(sine > tolerance.angleEpsilon) || !std::isfinite(uLength) || !std::isfinite(vLength) ||
            !(uLength > 0.0) || !(vLength > 0.0))
        {
            return false;
        }
        const SCVector3d delta = point - surface.PointAt(0.0, 0.0);
        if (!delta.IsValid())
        {
            return false;
        }
        const double cosine = Dot(unitU, unitV);
        const double projectedU = Dot(delta, unitU);
        const double projectedV = Dot(delta, unitV);
        const double denominator = sine * sine;
        const double u = std::fma(-cosine, projectedV, projectedU) / (uLength * denominator);
        const double v = std::fma(-cosine, projectedU, projectedV) / (vLength * denominator);
        if (!std::isfinite(u) || !std::isfinite(v))
        {
            return false;
        }
        uv = SCPoint2d{u, v};
        return true;
    }
}
