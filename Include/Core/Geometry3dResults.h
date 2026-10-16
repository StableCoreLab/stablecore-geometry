#pragma once

#include <cstddef>

#include "Export/GeometryExport.h"
#include "Types/Geometry3d/SCPoint3.h"

namespace Geometry
{
    struct GEOMETRY_API SCSnapResult3d
    {
        bool snapped{false};
        SCPoint3d point{};
        double distanceSquared{0.0};
        std::size_t segmentIndex{0};
        double parameter{0.0};

        [[nodiscard]] bool IsValid() const
        {
            return !snapped || point.IsValid();
        }
    };

}  // namespace Geometry
