#include "sdk/GeometryValidation.h"

#include "sdk/GeometryIntersection.h"
#include "sdk/GeometryRelation.h"
#include "sdk/GeometryShapeOps.h"

namespace geometry::sdk
{
bool HasSelfIntersection(const Polyline2d& ring, double eps)
{
    if (!ring.IsClosed() || ring.PointCount() < 4)
    {
        return false;
    }

    const std::size_t n = ring.PointCount();
    for (std::size_t i = 0; i < n; ++i)
    {
        const LineSegment2d first(ring.PointAt(i), ring.PointAt((i + 1) % n));
        for (std::size_t j = i + 1; j < n; ++j)
        {
            if (j == i || j == (i + 1) % n || (i == 0 && j + 1 == n))
            {
                continue;
            }

            const LineSegment2d second(ring.PointAt(j), ring.PointAt((j + 1) % n));
            if (HasIntersection(first, second, eps))
            {
                return true;
            }
        }
    }

    return false;
}

PolygonValidation2d Validate(const Polyline2d& ring, double eps)
{
    if (!ring.IsClosed() || ring.PointCount() < 3)
    {
        return {false, !ring.IsClosed() ? PolygonValidationIssue2d::NotClosed
                                        : PolygonValidationIssue2d::TooFewPoints};
    }
    if (Orientation(ring) == RingOrientation2d::Unknown)
    {
        return {false, PolygonValidationIssue2d::InvalidOrientation};
    }
    if (HasSelfIntersection(ring, eps))
    {
        return {false, PolygonValidationIssue2d::SelfIntersection};
    }
    return {true, PolygonValidationIssue2d::None};
}

PolygonValidation2d Validate(const Polygon2d& polygon, double eps)
{
    if (!polygon.IsValid())
    {
        if (!polygon.OuterRing().IsClosed())
        {
            return {false, PolygonValidationIssue2d::NotClosed};
        }
        return {false, PolygonValidationIssue2d::InvalidOrientation};
    }

    PolygonValidation2d outer = Validate(polygon.OuterRing(), eps);
    if (!outer.valid)
    {
        return outer;
    }

    for (std::size_t i = 0; i < polygon.HoleCount(); ++i)
    {
        PolygonValidation2d hole = Validate(polygon.HoleAt(i), eps);
        if (!hole.valid)
        {
            return hole;
        }
    }

    return {true, PolygonValidationIssue2d::None};
}

MeshValidation3d Validate(const TriangleMesh& mesh, double eps)
{
    const auto& vertices = mesh.Vertices();
    const auto& triangles = mesh.Triangles();

    for (std::size_t i = 0; i < vertices.size(); ++i)
    {
        if (!vertices[i].IsValid())
        {
            return {false, MeshValidationIssue3d::InvalidVertex, i};
        }
    }

    for (std::size_t i = 0; i < triangles.size(); ++i)
    {
        const TriangleMesh::TriangleIndices& tri = triangles[i];
        if (tri[0] >= vertices.size() || tri[1] >= vertices.size() || tri[2] >= vertices.size())
        {
            return {false, MeshValidationIssue3d::InvalidIndex, i};
        }

        if (tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2])
        {
            return {false, MeshValidationIssue3d::DuplicateIndex, i};
        }

        if (Triangle3d{vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]}.IsDegenerate(eps))
        {
            return {false, MeshValidationIssue3d::DegenerateTriangle, i};
        }
    }

    return {true, MeshValidationIssue3d::None, 0};
}
} // namespace geometry::sdk
