#pragma once

#include <sstream>
#include <string>

#include "types/Point3.h"
#include "types/Vector3.h"

namespace geometry
{
struct Plane
{
    Point3d origin{};
    Vector3d normal{};

    [[nodiscard]] static Plane FromPointAndNormal(const Point3d& originValue, const Vector3d& normalValue)
    {
        return Plane{originValue, normalValue};
    }

    [[nodiscard]] bool IsValid(double eps = kDefaultEpsilon) const
    {
        return origin.IsValid() && normal.IsValid() && normal.Length() > eps;
    }

    [[nodiscard]] Vector3d UnitNormal(double eps = kDefaultEpsilon) const
    {
        return normal.Normalized(eps);
    }

    [[nodiscard]] double SignedDistanceTo(const Point3d& point, double eps = kDefaultEpsilon) const
    {
        const Vector3d unitNormal = UnitNormal(eps);
        return Dot(point - origin, unitNormal);
    }

    [[nodiscard]] std::string DebugString() const
    {
        std::ostringstream stream;
        stream << "Plane{origin=" << origin.DebugString() << ", normal=" << normal.DebugString() << "}";
        return stream.str();
    }
};
} // namespace geometry
