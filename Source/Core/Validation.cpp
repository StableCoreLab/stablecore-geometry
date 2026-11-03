#include "Core/Validation.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "../Detail/RingIntegral2d.h"
#include "Support/StableNumerics.h"
#include "Core/Intersection.h"
#include "Core/Relation.h"
#include "Core/Section.h"
#include "Core/ShapeOps.h"
#include "Brep/PolyhedronBody.h"
#include "Brep/SCBrepBody.h"
#include "Geometry3d/TriangleMesh.h"
#include "Detail/SegmentKernel2d.h"

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
                if (segments[j] == nullptr)
                {
                    continue;
                }

                if (j == (i + 1) % n || (i == 0 && j + 1 == n))
                {
                    if (segments[i]->Kind() == SCSegmentKind2d::Line &&
                        segments[j]->Kind() == SCSegmentKind2d::Line)
                    {
                        const bool wraps = i == 0 && j + 1 == n;
                        const SCPoint2d start = wraps ? segments[j]->StartPoint() : segments[i]->StartPoint();
                        const SCPoint2d shared = wraps ? segments[j]->EndPoint() : segments[i]->EndPoint();
                        const SCPoint2d end = wraps ? segments[i]->EndPoint() : segments[j]->EndPoint();
                        const double firstX = shared.x - start.x;
                        const double firstY = shared.y - start.y;
                        const double secondX = end.x - shared.x;
                        const double secondY = end.y - shared.y;
                        if (firstX * secondX + firstY * secondY < 0.0 &&
                            firstX * secondY - firstY * secondX == 0.0)
                        {
                            return true;
                        }
                    }
                    continue;
                }

                if (!IntersectAll(*segments[i], *segments[j], eps).empty())
                {
                    return true;
                }
            }
        }

        return false;
    }

    SCPolygonValidation2d Validate(const SCPolyline2d& ring, double eps)
    {
        if (!(eps > 0.0) || !std::isfinite(eps))
        {
            return {false, SCPolygonValidationIssue2d::InvalidTolerance};
        }
        if (!ring.IsClosed() || ring.SegmentCount() == 0)
        {
            return {false,
                    !ring.IsClosed() ? SCPolygonValidationIssue2d::NotClosed : SCPolygonValidationIssue2d::TooFewPoints};
        }
        if (Orientation(ring) == SCRingOrientation2d::Unknown)
        {
            // 公开方向入口保留固定面积阈值；校验时按调用者距离容差判断小尺度环。
            const double area = std::abs(Detail::ComputeSignedArea(ring, eps));
            const auto absolute = Detail::StableProduct(eps, eps, 64);
            const auto relative = Detail::StableProduct(ring.Length(), eps, 2);
            if (!std::isfinite(area) || !absolute || !relative ||
                area <= std::max(*absolute, *relative))
            {
                return {false, SCPolygonValidationIssue2d::InvalidOrientation};
            }
        }
        if (HasSelfIntersection(ring, eps))
        {
            return {false, SCPolygonValidationIssue2d::SelfIntersection};
        }
        return {true, SCPolygonValidationIssue2d::None};
    }

    SCPolygonValidation2d Validate(const SCPolygon2d& polygon, double eps)
    {
        if (!(eps > 0.0) || !std::isfinite(eps))
        {
            return {false, SCPolygonValidationIssue2d::InvalidTolerance};
        }
        if (!polygon.IsValid(eps))
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
        if (!(eps > 0.0) || !std::isfinite(eps))
        {
            return {false, SCMeshValidationIssue3d::InvalidTolerance, 0};
        }
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
        if (!(eps > 0.0) || !std::isfinite(eps))
        {
            return {false, SCPolyhedronValidationIssue3d::InvalidTolerance, 0};
        }
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

    SCBrepValidation3d Validate(const SCBrepBody& body, const SCGeometryTolerance3d& tolerance)
    {
        if (!tolerance.IsValid())
        {
            return {false, SCBrepValidationIssue3d::InvalidTolerance, 0};
        }
        if (body.IsEmpty())
        {
            return {false, SCBrepValidationIssue3d::EmptyBody, 0};
        }

        for (std::size_t i = 0; i < body.VertexCount(); ++i)
        {
            if (!body.VertexAtRef(i).IsValid())
            {
                return {false, SCBrepValidationIssue3d::InvalidVertex, i};
            }
        }

        for (std::size_t i = 0; i < body.EdgeCount(); ++i)
        {
            if (!body.EdgeAtRef(i).IsValid(tolerance))
            {
                return {false, SCBrepValidationIssue3d::InvalidEdge, i};
            }
        }

        for (std::size_t i = 0; i < body.ShellCount(); ++i)
        {
            if (!body.ShellAtRef(i).IsValid(tolerance))
            {
                return {false, SCBrepValidationIssue3d::InvalidShell, i};
            }
        }

        if (!body.IsValid(tolerance))
        {
            return {false, SCBrepValidationIssue3d::InvalidShell, 0};
        }

        std::vector<std::size_t> edgeUseCounts(body.EdgeCount(), 0);
        for (std::size_t shellIndex = 0; shellIndex < body.ShellCount(); ++shellIndex)
        {
            const SCBrepShell& shell = body.ShellAtRef(shellIndex);
            for (std::size_t faceIndex = 0; faceIndex < shell.FaceCount(); ++faceIndex)
            {
                const SCBrepFace& face = shell.FaceAtRef(faceIndex);
                auto accumulateLoop = [&](const SCBrepLoop& loop) {
                    for (const SCBrepCoedge& coedge : loop.Coedges())
                    {
                        ++edgeUseCounts[coedge.EdgeIndex()];
                    }
                };

                accumulateLoop(face.OuterLoopRef());
                for (const SCBrepLoop& hole : face.HoleLoops())
                {
                    accumulateLoop(hole);
                }
            }
        }

        for (std::size_t edgeIndex = 0; edgeIndex < body.EdgeCount(); ++edgeIndex)
        {
            if (edgeUseCounts[edgeIndex] == 0 || edgeUseCounts[edgeIndex] > 2)
            {
                return {false, SCBrepValidationIssue3d::InvalidFaceAdjacency, edgeIndex};
            }
        }

        return {true, SCBrepValidationIssue3d::None, 0};
    }

    SCSectionValidation3d Validate(const SCPolyhedronSection3d& section, double eps)
    {
        if (!(eps > 0.0) || !std::isfinite(eps))
        {
            return {false, SCSectionValidationIssue3d::InvalidTolerance, 0};
        }
        if (!section.success)
        {
            return {false, SCSectionValidationIssue3d::InvalidSectionState, 0};
        }

        if (!section.origin.IsValid() || !section.uAxis.IsValid() || !section.vAxis.IsValid() ||
            section.uAxis.Length() <= eps || section.vAxis.Length() <= eps ||
            SCTriangle3d{{0.0, 0.0, 0.0},
                         {section.uAxis.x, section.uAxis.y, section.uAxis.z},
                         {section.vAxis.x, section.vAxis.y, section.vAxis.z}}.IsDegenerate(eps))
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
            if (!section.polygons[i].IsValid(eps))
            {
                return {false, SCSectionValidationIssue3d::InvalidPolygon, i};
            }
        }

        return {true, SCSectionValidationIssue3d::None, 0};
    }
}  // namespace Geometry

