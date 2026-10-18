#pragma once

#include <cmath>
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
            const SCVector3d ab = b - a;
            const SCVector3d bc = c - b;
            const SCVector3d ca = a - c;
            const double abLength = std::hypot(ab.x, ab.y, ab.z);
            const double bcLength = std::hypot(bc.x, bc.y, bc.z);
            const double caLength = std::hypot(ca.x, ca.y, ca.z);

            const SCVector3d* base = &ab;
            SCVector3d opposite = c - a;
            double baseLength = abLength;
            if (bcLength > baseLength)
            {
                base = &bc;
                opposite = a - b;
                baseLength = bcLength;
            }
            if (caLength > baseLength)
            {
                base = &ca;
                opposite = b - c;
                baseLength = caLength;
            }
            if (!(baseLength > 0.0)) return true;

            const SCVector3d normal = Cross(*base / baseLength, opposite);
            return std::hypot(normal.x, normal.y, normal.z) <= eps;
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
