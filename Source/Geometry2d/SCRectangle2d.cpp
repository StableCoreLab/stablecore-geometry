#include "Geometry2d/SCRectangle2d.h"

#include <array>
#include <cmath>
#include <sstream>
#include <vector>

#include "Geometry2d/SCPolyline2d.h"

namespace Geometry
{
    namespace
    {
        [[nodiscard]] SCPoint2d RotateLocal(const SCPoint2d& center, double angle, double x, double y)
        {
            const double c = std::cos(angle);
            const double s = std::sin(angle);
            return SCPoint2d{center.x + x * c - y * s, center.y + x * s + y * c};
        }
    }  // namespace

    SCRectangle2d::SCRectangle2d(const SCPoint2d& center, double width, double height, double angleRadians)
        : center(center), width(width), height(height), angleRadians(angleRadians)
    {
    }

    bool SCRectangle2d::IsValid() const
    {
        return center.IsValid() && std::isfinite(width) && std::isfinite(height) && std::isfinite(angleRadians) &&
               width > 0.0 && height > 0.0;
    }

    double SCRectangle2d::Area() const
    {
        return IsValid() ? width * height : 0.0;
    }

    double SCRectangle2d::Perimeter() const
    {
        return IsValid() ? 2.0 * (width + height) : 0.0;
    }

    std::array<SCPoint2d, 4> SCRectangle2d::Corners() const
    {
        const double hx = width * 0.5;
        const double hy = height * 0.5;
        return {RotateLocal(center, angleRadians, -hx, -hy),
                RotateLocal(center, angleRadians, hx, -hy),
                RotateLocal(center, angleRadians, hx, hy),
                RotateLocal(center, angleRadians, -hx, hy)};
    }

    SCBox2d SCRectangle2d::Bounds() const
    {
        if (!IsValid())
        {
            return {};
        }

        SCBox2d box;
        for (const SCPoint2d& corner : Corners())
        {
            box.ExpandToInclude(corner);
        }
        return box;
    }

    SCPolygon2d SCRectangle2d::ToPolygon() const
    {
        if (!IsValid())
        {
            return {};
        }
        const auto corners = Corners();
        return SCPolygon2d(SCPolyline2d(std::vector<SCPoint2d>(corners.begin(), corners.end()), SCPolylineClosure::Closed));
    }

    std::string SCRectangle2d::DebugString() const
    {
        std::ostringstream stream;
        stream << "SCRectangle2d{center=" << center.DebugString() << ", width=" << width << ", height=" << height
               << ", angleRadians=" << angleRadians << "}";
        return stream.str();
    }
}  // namespace Geometry
