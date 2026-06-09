#include "Core/Validation.h"

#include <map>
#include <memory>

#include "Core/Intersection.h"
#include "Core/Relation.h"
#include "Core/ShapeOps.h"

namespace Geometry
{
    bool HasSelfIntersection(const SCPolyline2d& ring, double eps)
    {
        if (!ring.IsClosed() || ring.SegmentCount() < 3)
        {
            return false;
        }

        const std::size_t n = ring.SegmentCount();
        std::vector<std::unique_ptr<ISCSegment2d>> segments;
        segments.reserve(n);
        for (std::size_t i = 0; i < n; ++i)
        {
            segments.push_back(ring.SegmentAt(i));
        }

        for (std::size_t i = 0; i < n; ++i)
        {
            if (segments[i] == nullptr)
            {
                continue;
            }
            for (std::size_t j = i + 1; j < n; ++j)
            {
                if (j == i || j == (i + 1) % n || (i == 0 && j + 1 == n))
                {
                    continue;
                }

                if (segments[j] == nullptr)
                {
                    continue;
                }

                if (HasIntersection(*segments[i], *segments[j], eps))
                {
                    return true;
                }
            }
        }

        return false;
    }

    SCPolygonValidation2d Validate(const SCPolyline2d& ring, double eps)
    {
        if (!ring.IsClosed() || ring.SegmentCount() == 0)
        {
            return {false,
                    !ring.IsClosed() ? SCPolygonValidationIssue2d::NotClosed : SCPolygonValidationIssue2d::TooFewPoints};
        }
        if (Orientation(ring) == SCRingOrientation2d::Unknown)
        {
            return {false, SCPolygonValidationIssue2d::InvalidOrientation};
        }
        if (HasSelfIntersection(ring, eps))
        {
            return {false, SCPolygonValidationIssue2d::SelfIntersection};
        }
        return {true, SCPolygonValidationIssue2d::None};
    }

    SCPolygonValidation2d Validate(const SCPolygon2d& polygon, double eps)
    {
        if (!polygon.IsValid())
        {
            if (!polygon.OuterRing().IsClosed())
            {
                return {false, SCPolygonValidationIssue2d::NotClosed};
            }
            return {false, SCPolygonValidationIssue2d::InvalidOrientation};
        }

        SCPolygonValidation2d outer = Validate(polygon.OuterRing(), eps);
        if (!outer.valid)
        {
            return outer;
        }

        for (std::size_t i = 0; i < polygon.HoleCount(); ++i)
        {
            SCPolygonValidation2d hole = Validate(polygon.HoleAt(i), eps);
            if (!hole.valid)
            {
                return hole;
            }
        }

        return {true, SCPolygonValidationIssue2d::None};
    }

    SCMeshValidation3d Validate(const TriangleMesh& mesh, double eps)
    {
        const auto& vertices = mesh.Vertices();
        const auto& triangles = mesh.Triangles();

        for (std::size_t i = 0; i < vertices.size(); ++i)
        {
            if (!vertices[i].IsValid())
            {
                return {false, SCMeshValidationIssue3d::InvalidVertex, i};
            }
        }

        for (std::size_t i = 0; i < triangles.size(); ++i)
        {
            const TriangleMesh::TriangleIndices& tri = triangles[i];
            if (tri[0] >= vertices.size() || tri[1] >= vertices.size() || tri[2] >= vertices.size())
            {
                return {false, SCMeshValidationIssue3d::InvalidIndex, i};
            }

            if (tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2])
            {
                return {false, SCMeshValidationIssue3d::DuplicateIndex, i};
            }

            if (SCTriangle3d{vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]}.IsDegenerate(eps))
            {
                return {false, SCMeshValidationIssue3d::DegenerateTriangle, i};
            }
        }

        return {true, SCMeshValidationIssue3d::None, 0};
    }

    SCPolyhedronValidation3d Validate(const PolyhedronBody& body, double eps)
    {
        if (body.IsEmpty())
        {
            return {false, SCPolyhedronValidationIssue3d::EmptyBody, 0};
        }

        for (std::size_t i = 0; i < body.FaceCount(); ++i)
        {
            if (!body.FaceAt(i).IsValid(eps))
            {
                return {false, SCPolyhedronValidationIssue3d::InvalidFace, i};
            }
        }

        return {true, SCPolyhedronValidationIssue3d::None, 0};
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
    }  // namespace

    SCBrepValidation3d Validate(const SCBrepBody& body, const SCGeometryTolerance3d& tolerance)
    {
        if (body.IsEmpty())
        {
            return {false, SCBrepValidationIssue3d::EmptyBody, 0};
        }

        for (std::size_t i = 0; i < body.VertexCount(); ++i)
        {
            if (!body.VertexAt(i).IsValid())
            {
                return {false, SCBrepValidationIssue3d::InvalidVertex, i};
            }
        }

        for (std::size_t i = 0; i < body.EdgeCount(); ++i)
        {
            if (!body.EdgeAt(i).IsValid(tolerance))
            {
                return {false, SCBrepValidationIssue3d::InvalidEdge, i};
            }
        }

        for (std::size_t i = 0; i < body.ShellCount(); ++i)
        {
            if (!body.ShellAt(i).IsValid(tolerance))
            {
                return {false, SCBrepValidationIssue3d::InvalidShell, i};
            }
        }

        if (!body.IsValid(tolerance))
        {
            return {false, SCBrepValidationIssue3d::InvalidShell, 0};
        }

        std::map<UndirectedEdgeKey, std::size_t> edgeUseCounts;
        for (std::size_t shellIndex = 0; shellIndex < body.ShellCount(); ++shellIndex)
        {
            const SCBrepShell shell = body.ShellAt(shellIndex);
            for (std::size_t faceIndex = 0; faceIndex < shell.FaceCount(); ++faceIndex)
            {
                const SCBrepFace face = shell.FaceAt(faceIndex);
                auto accumulateLoop = [&](const SCBrepLoop& loop) {
                    for (const SCBrepCoedge& coedge : loop.Coedges())
                    {
                        const SCBrepEdge edge = body.EdgeAt(coedge.EdgeIndex());
                        ++edgeUseCounts[MakeUndirectedEdgeKey(edge.StartVertexIndex(), edge.EndVertexIndex())];
                    }
                };

                accumulateLoop(face.OuterLoop());
                for (const SCBrepLoop& hole : face.HoleLoops())
                {
                    accumulateLoop(hole);
                }
            }
        }

        for (std::size_t edgeIndex = 0; edgeIndex < body.EdgeCount(); ++edgeIndex)
        {
            const SCBrepEdge edge = body.EdgeAt(edgeIndex);
            const auto it = edgeUseCounts.find(MakeUndirectedEdgeKey(edge.StartVertexIndex(), edge.EndVertexIndex()));
            if (it == edgeUseCounts.end() || it->second == 0 || it->second > 2)
            {
                return {false, SCBrepValidationIssue3d::InvalidFaceAdjacency, edgeIndex};
            }
        }

        return {true, SCBrepValidationIssue3d::None, 0};
    }

    SCSectionValidation3d Validate(const SCPolyhedronSection3d& section, double eps)
    {
        if (!section.success)
        {
            return {false, SCSectionValidationIssue3d::InvalidSectionState, 0};
        }

        if (!section.origin.IsValid() || !section.uAxis.IsValid() || !section.vAxis.IsValid() ||
            section.uAxis.Length() <= eps || section.vAxis.Length() <= eps ||
            Cross(section.uAxis, section.vAxis).Length() <= eps)
        {
            return {false, SCSectionValidationIssue3d::InvalidBasis, 0};
        }

        for (std::size_t i = 0; i < section.segments.size(); ++i)
        {
            if (!section.segments[i].IsValid(eps))
            {
                return {false, SCSectionValidationIssue3d::InvalidSegment, i};
            }
        }

        for (std::size_t i = 0; i < section.contours.size(); ++i)
        {
            if (!section.contours[i].IsValid(eps))
            {
                return {false, SCSectionValidationIssue3d::InvalidContour, i};
            }
        }

        for (std::size_t i = 0; i < section.polygons.size(); ++i)
        {
            if (!section.polygons[i].IsValid())
            {
                return {false, SCSectionValidationIssue3d::InvalidPolygon, i};
            }
        }

        return {true, SCSectionValidationIssue3d::None, 0};
    }
}  // namespace Geometry


