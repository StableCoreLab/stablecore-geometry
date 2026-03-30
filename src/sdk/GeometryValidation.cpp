#include "sdk/GeometryValidation.h"

#include <map>

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

PolyhedronValidation3d Validate(const PolyhedronBody& body, double eps)
{
    if (body.IsEmpty())
    {
        return {false, PolyhedronValidationIssue3d::EmptyBody, 0};
    }

    for (std::size_t i = 0; i < body.FaceCount(); ++i)
    {
        if (!body.FaceAt(i).IsValid(eps))
        {
            return {false, PolyhedronValidationIssue3d::InvalidFace, i};
        }
    }

    return {true, PolyhedronValidationIssue3d::None, 0};
}

namespace
{
struct UndirectedEdgeKey
{
    std::size_t first{0};
    std::size_t second{0};

    [[nodiscard]] bool operator<(const UndirectedEdgeKey& other) const
    {
        return first < other.first || (first == other.first && second < other.second);
    }
};

[[nodiscard]] UndirectedEdgeKey MakeUndirectedEdgeKey(std::size_t a, std::size_t b)
{
    return a < b ? UndirectedEdgeKey{a, b} : UndirectedEdgeKey{b, a};
}
} // namespace

BrepValidation3d Validate(const BrepBody& body, const GeometryTolerance3d& tolerance)
{
    if (body.IsEmpty())
    {
        return {false, BrepValidationIssue3d::EmptyBody, 0};
    }

    for (std::size_t i = 0; i < body.VertexCount(); ++i)
    {
        if (!body.VertexAt(i).IsValid())
        {
            return {false, BrepValidationIssue3d::InvalidVertex, i};
        }
    }

    for (std::size_t i = 0; i < body.EdgeCount(); ++i)
    {
        if (!body.EdgeAt(i).IsValid(tolerance))
        {
            return {false, BrepValidationIssue3d::InvalidEdge, i};
        }
    }

    for (std::size_t i = 0; i < body.ShellCount(); ++i)
    {
        if (!body.ShellAt(i).IsValid(tolerance))
        {
            return {false, BrepValidationIssue3d::InvalidShell, i};
        }
    }

    if (!body.IsValid(tolerance))
    {
        return {false, BrepValidationIssue3d::InvalidShell, 0};
    }

    std::map<UndirectedEdgeKey, std::size_t> edgeUseCounts;
    for (std::size_t shellIndex = 0; shellIndex < body.ShellCount(); ++shellIndex)
    {
        const BrepShell shell = body.ShellAt(shellIndex);
        for (std::size_t faceIndex = 0; faceIndex < shell.FaceCount(); ++faceIndex)
        {
            const BrepFace face = shell.FaceAt(faceIndex);
            auto accumulateLoop = [&](const BrepLoop& loop)
            {
                for (const BrepCoedge& coedge : loop.Coedges())
                {
                    const BrepEdge edge = body.EdgeAt(coedge.EdgeIndex());
                    ++edgeUseCounts[MakeUndirectedEdgeKey(edge.StartVertexIndex(), edge.EndVertexIndex())];
                }
            };

            accumulateLoop(face.OuterLoop());
            for (const BrepLoop& hole : face.HoleLoops())
            {
                accumulateLoop(hole);
            }
        }
    }

    for (std::size_t edgeIndex = 0; edgeIndex < body.EdgeCount(); ++edgeIndex)
    {
        const BrepEdge edge = body.EdgeAt(edgeIndex);
        const auto it = edgeUseCounts.find(MakeUndirectedEdgeKey(edge.StartVertexIndex(), edge.EndVertexIndex()));
        if (it == edgeUseCounts.end() || it->second == 0 || it->second > 2)
        {
            return {false, BrepValidationIssue3d::InvalidFaceAdjacency, edgeIndex};
        }
    }

    return {true, BrepValidationIssue3d::None, 0};
}

SectionValidation3d Validate(const PolyhedronSection3d& section, double eps)
{
    if (!section.success)
    {
        return {false, SectionValidationIssue3d::InvalidSectionState, 0};
    }

    if (!section.origin.IsValid() ||
        !section.uAxis.IsValid() ||
        !section.vAxis.IsValid() ||
        section.uAxis.Length() <= eps ||
        section.vAxis.Length() <= eps ||
        Cross(section.uAxis, section.vAxis).Length() <= eps)
    {
        return {false, SectionValidationIssue3d::InvalidBasis, 0};
    }

    for (std::size_t i = 0; i < section.segments.size(); ++i)
    {
        if (!section.segments[i].IsValid(eps))
        {
            return {false, SectionValidationIssue3d::InvalidSegment, i};
        }
    }

    for (std::size_t i = 0; i < section.contours.size(); ++i)
    {
        if (!section.contours[i].IsValid(eps))
        {
            return {false, SectionValidationIssue3d::InvalidContour, i};
        }
    }

    for (std::size_t i = 0; i < section.polygons.size(); ++i)
    {
        if (!section.polygons[i].IsValid())
        {
            return {false, SectionValidationIssue3d::InvalidPolygon, i};
        }
    }

    return {true, SectionValidationIssue3d::None, 0};
}
} // namespace geometry::sdk
