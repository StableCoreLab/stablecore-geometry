#include "Core/PointOps.h"

namespace Geometry
{
    SCPoint2d Midpoint(const SCPoint2d& first, const SCPoint2d& second) noexcept
    {
        return SCPoint2d{first.x + (second.x - first.x) * 0.5,
                         first.y + (second.y - first.y) * 0.5};
    }

    SCPoint3d Midpoint(const SCPoint3d& first, const SCPoint3d& second) noexcept
    {
        return SCPoint3d{first.x + (second.x - first.x) * 0.5,
                         first.y + (second.y - first.y) * 0.5,
                         first.z + (second.z - first.z) * 0.5};
    }
}  // namespace Geometry
