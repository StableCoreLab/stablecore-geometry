#include "Geometry3d/SCPlaneSurface.h"

#include <algorithm>
#include <cmath>

namespace Geometry
{
    namespace
    {
        [[nodiscard]] SCVector3d MakePerpendicularVector(const SCVector3d& normal, double eps)
        {
            SCVector3d normalized;
            if (!Detail::TryUnitVector(normal, normalized))
            {
                return SCVector3d{};
            }

            // Prefer world-aligned in-plane axes when possible so plane UV
            // coordinates remain stable and intuitive, especially for
            // axis-aligned support planes.
            SCVector3d referenceAxis{1.0, 0.0, 0.0};
            if (std::abs(Dot(normalized, referenceAxis)) >= 1.0 - eps)
            {
                referenceAxis = SCVector3d{0.0, 1.0, 0.0};
            }

            const SCVector3d inPlane = referenceAxis - normalized * Dot(referenceAxis, normalized);
            SCVector3d unitInPlane;
            return Detail::TryUnitVector(inPlane, unitInPlane) ? unitInPlane : SCVector3d{};
        }
    }  // namespace

    SCPlaneSurface SCPlaneSurface::FromPlane(const SCPlane& plane, SCIntervald uRange, SCIntervald vRange, double eps)
    {
        SCVector3d unitNormal;
        if (!std::isfinite(eps) || eps < 0.0 || !plane.origin.IsValid() ||
            !Detail::TryUnitVector(plane.normal, unitNormal))
        {
            return SCPlaneSurface(plane, {}, {}, uRange, vRange);
        }
        const SCVector3d uAxis = MakePerpendicularVector(plane.normal, eps);
        SCVector3d unitU, vAxis;
        if (!Detail::TryUnitVector(uAxis, unitU) ||
            !Detail::TryUnitVector(Cross(unitNormal, unitU), vAxis))
        {
            return SCPlaneSurface(plane, {}, {}, uRange, vRange);
        }
        return SCPlaneSurface(plane, uAxis, vAxis, uRange, vRange);
    }
}  // namespace Geometry
