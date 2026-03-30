#pragma once

#include <cstddef>
#include <string>
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

enum class SectionFaceRebuildIssue3d
{
    None,
    InvalidSection,
    InvalidPolygon
};

enum class SectionBodyRebuildIssue3d
{
    None,
    InvalidSection,
    FaceRebuildFailed
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

struct GEOMETRY_API SectionFaceRebuild3d
{
    bool success{false};
    SectionFaceRebuildIssue3d issue{SectionFaceRebuildIssue3d::None};
    std::vector<PolyhedronFace3d> faces{};

    [[nodiscard]] bool IsValid(double eps = geometry::kDefaultEpsilon) const
    {
        if (!success)
        {
            return true;
        }

        for (const PolyhedronFace3d& face : faces)
        {
            if (!face.IsValid(eps))
            {
                return false;
            }
        }

        return true;
    }
};

struct GEOMETRY_API SectionBodyRebuild3d
{
    bool success{false};
    SectionBodyRebuildIssue3d issue{SectionBodyRebuildIssue3d::None};
    PolyhedronBody body{};

    [[nodiscard]] bool IsValid(double eps = geometry::kDefaultEpsilon) const
    {
        return !success || body.IsValid(eps) || body.IsEmpty();
    }
};

struct GEOMETRY_API SectionTopologyNode3d
{
    std::size_t polygonIndex{0};
    std::size_t parentIndex{static_cast<std::size_t>(-1)};
    std::size_t depth{0};
    std::vector<std::size_t> children{};
};

class GEOMETRY_API SectionTopology3d
{
public:
    [[nodiscard]] bool IsValid() const
    {
        return valid_;
    }

    [[nodiscard]] std::size_t Count() const
    {
        return nodes_.size();
    }

    [[nodiscard]] bool IsEmpty() const
    {
        return nodes_.empty();
    }

    [[nodiscard]] const SectionTopologyNode3d& Node(std::size_t index) const
    {
        return nodes_.at(index);
    }

    [[nodiscard]] std::size_t ParentOf(std::size_t index) const
    {
        return nodes_.at(index).parentIndex;
    }

    [[nodiscard]] const std::vector<std::size_t>& ChildrenOf(std::size_t index) const
    {
        return nodes_.at(index).children;
    }

    [[nodiscard]] const std::vector<std::size_t>& Roots() const
    {
        return roots_;
    }

    [[nodiscard]] std::string DebugString() const;

private:
    friend GEOMETRY_API SectionTopology3d BuildSectionTopology(
        const PolyhedronSection3d& section,
        double eps);

    bool valid_{false};
    std::vector<SectionTopologyNode3d> nodes_{};
    std::vector<std::size_t> roots_{};
};

[[nodiscard]] GEOMETRY_API PolyhedronSection3d Section(
    const PolyhedronBody& body,
    const Plane& plane,
    const GeometryTolerance3d& tolerance = {});

[[nodiscard]] GEOMETRY_API SectionFaceRebuild3d RebuildSectionFaces(
    const PolyhedronSection3d& section,
    double eps = 1e-9);

[[nodiscard]] GEOMETRY_API SectionBodyRebuild3d RebuildSectionBody(
    const PolyhedronSection3d& section,
    double eps = 1e-9);

[[nodiscard]] GEOMETRY_API SectionTopology3d BuildSectionTopology(
    const PolyhedronSection3d& section,
    double eps = 1e-9);
} // namespace geometry::sdk
