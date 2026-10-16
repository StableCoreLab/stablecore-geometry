#pragma once

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

#include "Types/Geometry3d/SCPoint3.h"
#include "Types/Geometry3d/SCVector3.h"

namespace Geometry
{
    struct SCPlane
    {
        SCPoint3d origin{};
        SCVector3d normal{};

        [[nodiscard]] static SCPlane FromPointAndNormal(const SCPoint3d& originValue, const SCVector3d& normalValue)
        {
            return SCPlane{originValue, normalValue};
        }

        [[nodiscard]] bool IsValid(double eps = kDefaultEpsilon) const
        {
            return origin.IsValid() && normal.IsValid() && normal.Length() > eps;
        }

        [[nodiscard]] SCVector3d UnitNormal(double eps = kDefaultEpsilon) const
        {
            if (!IsValid(eps)) return {};
            const double scale=std::max({std::abs(normal.x),std::abs(normal.y),std::abs(normal.z)});
            const SCVector3d scaled=normal/scale;
            return scaled/std::hypot(scaled.x,scaled.y,scaled.z);
        }

        [[nodiscard]] double SignedDistanceTo(const SCPoint3d& point, double eps = kDefaultEpsilon) const
        {
            const SCVector3d unitNormal = UnitNormal(eps);
            const double direct=Dot(point-origin,unitNormal);
            if (std::isfinite(direct) || !point.IsValid() || !origin.IsValid()) return direct;
            // 仅在中间差值或点积溢出时缩放重算，保留正常尺度的舍入路径。
            const double scale=std::max({std::abs(point.x),std::abs(point.y),std::abs(point.z),
                std::abs(origin.x),std::abs(origin.y),std::abs(origin.z)});
            if (!(scale>0) || !std::isfinite(scale)) return direct;
            const SCVector3d delta{point.x/scale-origin.x/scale,
                                   point.y/scale-origin.y/scale,
                                   point.z/scale-origin.z/scale};
            return Dot(delta,unitNormal)*scale;
        }

        [[nodiscard]] std::string DebugString() const
        {
            std::ostringstream stream;
            stream << "SCPlane{origin=" << origin.DebugString() << ", normal=" << normal.DebugString() << "}";
            return stream.str();
        }
    };

}  // namespace Geometry
