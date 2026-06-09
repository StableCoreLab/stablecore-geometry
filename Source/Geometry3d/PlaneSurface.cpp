#include "Geometry3d/SCPlaneSurface.h"

#include <algorithm>
#include <cmath>

namespace Geometry
{
    namespace
    {
        [[nodiscard]] SCVector3d MakePerpendicularVector(const SCVector3d& normal, double eps)
        {
            const SCVector3d normalized = normal.Normalized(eps);
            if (normalized.Length() <= eps)
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
            return inPlane.Normalized(eps);
        }
    }  // namespace

    SCPlaneSurface SCPlaneSurface::FromPlane(const SCPlane& plane, SCIntervald uRange, SCIntervald vRange, double eps)
    {
        const SCVector3d uAxis = MakePerpendicularVector(plane.normal, eps);
        const SCVector3d vAxis = Cross(plane.UnitNormal(eps), uAxis).Normalized(eps);
        return SCPlaneSurface(plane, uAxis, vAxis, uRange, vRange);
    }
}  // namespace Geometry
