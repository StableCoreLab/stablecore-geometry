#pragma once

#include <sstream>
#include <string>

#include "types/Point3.h"
#include "types/Vector3.h"

namespace geometry
{
struct Line3d
{
    Point3d origin{};
    Vector3d direction{};

    [[nodiscard]] static Line3d FromOriginAndDirection(const Point3d& originValue, const Vector3d& directionValue)
    {
        return Line3d{originValue, directionValue};
    }

    [[nodiscard]] bool IsValid(double eps = kDefaultEpsilon) const
    {
        return origin.IsValid() && direction.IsValid() && direction.Length() > eps;
    }

    [[nodiscard]] Point3d PointAt(double parameter) const
    {
        return origin + direction * parameter;
    }

    [[nodiscard]] std::string DebugString() const
    {
        std::ostringstream stream;
        stream << "Line3d{origin=" << origin.DebugString() << ", direction=" << direction.DebugString() << "}";
        return stream.str();
    }
};
} // namespace geometry
