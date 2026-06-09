#pragma once

#include <sstream>
#include <string>

#include "Types/Geometry3d/SCBox3.h"
#include "Types/Geometry3d/SCPlane.h"

namespace Geometry
{
    struct SCTriangle3d
    {
        SCPoint3d a{};
        SCPoint3d b{};
        SCPoint3d c{};

        [[nodiscard]] bool IsDegenerate(double eps = kDefaultEpsilon) const
        {
            return Cross(b - a, c - a).Length() <= eps;
        }

        [[nodiscard]] bool IsValid(double eps = kDefaultEpsilon) const
        {
            return a.IsValid() && b.IsValid() && c.IsValid() && !IsDegenerate(eps);
        }

        [[nodiscard]] SCBox3d Bounds() const
        {
            if (!a.IsValid() || !b.IsValid() || !c.IsValid())
            {
                return {};
            }

            SCBox3d box;
            box.ExpandToInclude(a);
            box.ExpandToInclude(b);
            box.ExpandToInclude(c);
            return box;
        }

        [[nodiscard]] SCVector3d Normal() const
        {
            return Cross(b - a, c - a);
        }

        [[nodiscard]] double Area() const
        {
            return 0.5 * Normal().Length();
        }

        [[nodiscard]] SCPoint3d Centroid() const
        {
            return SCPoint3d{(a.x + b.x + c.x) / 3.0, (a.y + b.y + c.y) / 3.0, (a.z + b.z + c.z) / 3.0};
        }

        [[nodiscard]] SCPlane Plane() const
        {
            return Geometry::SCPlane::FromPointAndNormal(a, Normal());
        }

        [[nodiscard]] std::string DebugString() const
        {
            std::ostringstream stream;
            stream << "SCTriangle3d{a=" << a.DebugString() << ", b=" << b.DebugString() << ", c=" << c.DebugString()
                   << "}";
            return stream.str();
        }
    };

}  // namespace Geometry
