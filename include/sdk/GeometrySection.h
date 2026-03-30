#pragma once

#include <cstddef>
#include <vector>

#include "export/GeometryExport.h"
#include "sdk/GeometryTypes.h"
#include "sdk/Polygon2d.h"
#include "sdk/PolyhedronBody.h"

namespace geometry::sdk
{
enum class SectionIssue3d
{
    None,
    InvalidPlane,
    InvalidBody,
    MeshConversionFailed,
    CoplanarGeometryUnsupported,
    OpenContour,
    NonManifoldContour,
    InvalidContour
};

struct GEOMETRY_API SectionPolyline3d
{
    bool closed{false};
    std::vector<Point3d> points{};

    [[nodiscard]] bool IsValid(double eps = geometry::kDefaultEpsilon) const
    {
        if (closed ? points.size() < 3 : points.size() < 2)
        {
            return false;
        }

        for (std::size_t i = 0; i < points.size(); ++i)
        {
            if (!points[i].IsValid())
            {
                return false;
            }

            if (i > 0 && points[i].AlmostEquals(points[i - 1], eps))
            {
                return false;
            }
        }

        return true;
    }
};

struct GEOMETRY_API PolyhedronSection3d
{
    bool success{false};
    SectionIssue3d issue{SectionIssue3d::None};
    std::vector<LineSegment3d> segments{};
    std::vector<SectionPolyline3d> contours{};
    std::vector<Polygon2d> polygons{};
    Point3d origin{};
    Vector3d uAxis{};
    Vector3d vAxis{};

    [[nodiscard]] bool IsValid(double eps = geometry::kDefaultEpsilon) const
    {
        if (!success)
        {
            return true;
        }

        if (!origin.IsValid() || !uAxis.IsValid() || !vAxis.IsValid() || uAxis.Length() <= eps || vAxis.Length() <= eps)
        {
            return false;
        }

        for (const LineSegment3d& segment : segments)
        {
            if (!segment.IsValid(eps))
            {
                return false;
            }
        }

        for (const SectionPolyline3d& contour : contours)
        {
            if (!contour.IsValid(eps))
            {
                return false;
            }
        }

        for (const Polygon2d& polygon : polygons)
        {
            if (!polygon.IsValid())
            {
                return false;
            }
        }

        return true;
    }
};

[[nodiscard]] GEOMETRY_API PolyhedronSection3d Section(
    const PolyhedronBody& body,
    const Plane& plane,
    const GeometryTolerance3d& tolerance = {});
} // namespace geometry::sdk
