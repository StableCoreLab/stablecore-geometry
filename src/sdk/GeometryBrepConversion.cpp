#include "sdk/GeometryBrepConversion.h"

#include <memory>
#include <vector>

#include "sdk/LineCurve3d.h"
#include "sdk/PlaneSurface.h"

namespace geometry::sdk
{
namespace
{
[[nodiscard]] bool AppendLoopVerticesFromBody(
    const BrepBody& body,
    const BrepLoop& loop,
    std::vector<Point3d>& vertices,
    double eps)
{
    vertices.clear();
    if (!loop.IsValid())
    {
        return false;
    }

    vertices.reserve(loop.CoedgeCount());
    for (std::size_t i = 0; i < loop.CoedgeCount(); ++i)
    {
        const BrepCoedge coedge = loop.CoedgeAt(i);
        if (coedge.EdgeIndex() >= body.EdgeCount())
        {
            return false;
        }

        const BrepEdge edge = body.EdgeAt(coedge.EdgeIndex());
        const std::size_t vertexIndex = coedge.Reversed() ? edge.EndVertexIndex() : edge.StartVertexIndex();
        if (vertexIndex >= body.VertexCount())
        {
            return false;
        }

        const Point3d point = body.VertexAt(vertexIndex).Point();
        if (vertices.empty() || !vertices.back().AlmostEquals(point, eps))
        {
            vertices.push_back(point);
        }
    }

    while (vertices.size() >= 2 && vertices.front().AlmostEquals(vertices.back(), eps))
    {
        vertices.pop_back();
    }

    return vertices.size() >= 3;
}

[[nodiscard]] bool BuildLoopFromTrim(
    const CurveOnSurface& trim,
    std::vector<Point3d>& vertices,
    double eps)
{
    if (!trim.IsValid() || trim.PointCount() < 3)
    {
        return false;
    }

    vertices.clear();
    vertices.reserve(trim.PointCount());
    for (std::size_t i = 0; i < trim.PointCount(); ++i)
    {
        const Point3d point = trim.PointAt(i);
        if (!point.IsValid())
        {
            return false;
        }

        if (vertices.empty() || !vertices.back().AlmostEquals(point, eps))
        {
            vertices.push_back(point);
        }
    }

    while (vertices.size() >= 2 && vertices.front().AlmostEquals(vertices.back(), eps))
    {
        vertices.pop_back();
    }

    return vertices.size() >= 3;
}

[[nodiscard]] bool BuildPolyhedronFaceFromBodyFace(
    const BrepBody& body,
    const BrepFace& face,
    PolyhedronFace3d& polyFace,
    double eps)
{
    const auto* planeSurface = dynamic_cast<const PlaneSurface*>(face.SupportSurface());
    if (planeSurface == nullptr)
    {
        return false;
    }

    std::vector<Point3d> outerVertices;
    if (!AppendLoopVerticesFromBody(body, face.OuterLoop(), outerVertices, eps))
    {
        return false;
    }

    std::vector<PolyhedronLoop3d> holes;
    holes.reserve(face.HoleCount());
    for (std::size_t i = 0; i < face.HoleCount(); ++i)
    {
        std::vector<Point3d> holeVertices;
        if (!AppendLoopVerticesFromBody(body, face.HoleAt(i), holeVertices, eps))
        {
            return false;
        }
        holes.emplace_back(std::move(holeVertices));
    }

    polyFace = PolyhedronFace3d(
        planeSurface->SupportPlane(),
        PolyhedronLoop3d(std::move(outerVertices)),
        std::move(holes));
    return polyFace.IsValid(eps);
}

[[nodiscard]] Point2d ProjectPointToPlaneUv(const Point3d& point, const PlaneSurface& planeSurface)
{
    const Plane plane = planeSurface.SupportPlane();
    const Vector3d delta = point - plane.origin;
    const Vector3d uAxis = planeSurface.UAxis();
    const Vector3d vAxis = planeSurface.VAxis();
    const double uDenom = std::max(uAxis.LengthSquared(), geometry::kDefaultEpsilon);
    const double vDenom = std::max(vAxis.LengthSquared(), geometry::kDefaultEpsilon);
    return Point2d{
        Dot(delta, uAxis) / uDenom,
        Dot(delta, vAxis) / vDenom};
}

[[nodiscard]] bool AppendBrepLoopFromPolyLoop(
    const PolyhedronLoop3d& polyLoop,
    std::vector<BrepVertex>& vertices,
    std::vector<BrepEdge>& edges,
    BrepLoop& loop,
    std::vector<Point2d>& uvPoints)
{
    if (!polyLoop.IsValid())
    {
        return false;
    }

    const std::size_t vertexBase = vertices.size();
    const std::size_t vertexCount = polyLoop.VertexCount();
    vertices.reserve(vertexBase + vertexCount);
    for (std::size_t i = 0; i < vertexCount; ++i)
    {
        vertices.emplace_back(polyLoop.VertexAt(i));
    }

    std::vector<BrepCoedge> coedges;
    coedges.reserve(vertexCount);
    for (std::size_t i = 0; i < vertexCount; ++i)
    {
        const std::size_t next = (i + 1) % vertexCount;
        const Point3d first = polyLoop.VertexAt(i);
        const Point3d second = polyLoop.VertexAt(next);
        edges.emplace_back(
            std::make_shared<LineCurve3d>(LineCurve3d::FromLine(
                Line3d::FromOriginAndDirection(first, second - first),
                Intervald{0.0, 1.0})),
            vertexBase + i,
            vertexBase + next);
        coedges.emplace_back(edges.size() - 1, false);
    }

    loop = BrepLoop(std::move(coedges));
    uvPoints.clear();
    uvPoints.reserve(vertexCount);
    return loop.IsValid();
}

[[nodiscard]] bool BuildFaceWithRefitSupportPlane(
    const PolyhedronFace3d& face,
    PolyhedronFace3d& repairedFace,
    double eps)
{
    const PolyhedronLoop3d outer = face.OuterLoop();
    if (!outer.IsValid(eps) || outer.VertexCount() < 3)
    {
        return false;
    }

    const Point3d p0 = outer.VertexAt(0);
    const Point3d p1 = outer.VertexAt(1);
    const Point3d p2 = outer.VertexAt(2);
    const Vector3d normal = Cross(p1 - p0, p2 - p0);
    if (normal.Length() <= eps)
    {
        return false;
    }

    std::vector<PolyhedronLoop3d> holes;
    holes.reserve(face.HoleCount());
    for (std::size_t i = 0; i < face.HoleCount(); ++i)
    {
        const PolyhedronLoop3d hole = face.HoleAt(i);
        if (!hole.IsValid(eps))
        {
            return false;
        }
        holes.push_back(hole);
    }

    repairedFace = PolyhedronFace3d(Plane::FromPointAndNormal(p0, normal), outer, std::move(holes));
    return repairedFace.IsValid(eps);
}

[[nodiscard]] bool TryRepairPolyhedronBodyForBrepConversion(
    const PolyhedronBody& body,
    PolyhedronBody& repairedBody,
    double eps)
{
    if (body.IsEmpty())
    {
        return false;
    }

    std::vector<PolyhedronFace3d> repairedFaces;
    repairedFaces.reserve(body.FaceCount());
    for (std::size_t i = 0; i < body.FaceCount(); ++i)
    {
        PolyhedronFace3d repairedFace{};
        if (!BuildFaceWithRefitSupportPlane(body.FaceAt(i), repairedFace, eps))
        {
            return false;
        }
        repairedFaces.push_back(std::move(repairedFace));
    }

    repairedBody = PolyhedronBody(std::move(repairedFaces));
    return repairedBody.IsValid(eps);
}
} // namespace

BrepFaceConversion3d ConvertToPolyhedronFace(const BrepFace& face, double eps)
{
    const GeometryTolerance3d tolerance{eps, eps, eps};
    if (!face.IsValid(tolerance))
    {
        return {false, BrepConversionIssue3d::InvalidFace, {}};
    }

    const auto* planeSurface = dynamic_cast<const PlaneSurface*>(face.SupportSurface());
    if (planeSurface == nullptr)
    {
        return {false, BrepConversionIssue3d::UnsupportedSurface, {}};
    }

    std::vector<Point3d> outerVertices;
    if (!BuildLoopFromTrim(face.OuterTrim(), outerVertices, eps))
    {
        return {false, BrepConversionIssue3d::InvalidTrim, {}};
    }

    std::vector<PolyhedronLoop3d> holes;
    holes.reserve(face.HoleCount());
    for (const CurveOnSurface& trim : face.HoleTrims())
    {
        std::vector<Point3d> holeVertices;
        if (!BuildLoopFromTrim(trim, holeVertices, eps))
        {
            return {false, BrepConversionIssue3d::InvalidTrim, {}};
        }

        holes.emplace_back(std::move(holeVertices));
    }

    PolyhedronFace3d polyFace(
        planeSurface->SupportPlane(),
        PolyhedronLoop3d(std::move(outerVertices)),
        std::move(holes));
    if (!polyFace.IsValid(eps))
    {
        return {false, BrepConversionIssue3d::InvalidTrim, {}};
    }

    return {true, BrepConversionIssue3d::None, std::move(polyFace)};
}

BrepBodyConversion3d ConvertToPolyhedronBody(const BrepBody& body, double eps)
{
    const GeometryTolerance3d tolerance{eps, eps, eps};
    if (!body.IsValid(tolerance))
    {
        return {false, BrepConversionIssue3d::InvalidBody, 0, {}};
    }

    std::vector<PolyhedronFace3d> faces;
    faces.reserve(body.FaceCount());
    std::size_t faceIndex = 0;
    for (std::size_t shellIndex = 0; shellIndex < body.ShellCount(); ++shellIndex)
    {
        const BrepShell shell = body.ShellAt(shellIndex);
        for (std::size_t localFaceIndex = 0; localFaceIndex < shell.FaceCount(); ++localFaceIndex, ++faceIndex)
        {
            const BrepFace face = shell.FaceAt(localFaceIndex);
            BrepFaceConversion3d converted = ConvertToPolyhedronFace(face, eps);
            if (!converted.success)
            {
                PolyhedronFace3d polyFace{};
                if (!BuildPolyhedronFaceFromBodyFace(body, face, polyFace, eps))
                {
                    return {false, converted.issue, faceIndex, {}};
                }

                converted = {true, BrepConversionIssue3d::None, std::move(polyFace)};
            }
            faces.push_back(converted.face);
        }
    }

    PolyhedronBody polyBody(std::move(faces));
    if (!polyBody.IsValid(eps))
    {
        return {false, BrepConversionIssue3d::InvalidBody, 0, {}};
    }

    return {true, BrepConversionIssue3d::None, 0, std::move(polyBody)};
}

PolyhedronBrepBodyConversion3d ConvertToBrepBody(const PolyhedronBody& body, double eps)
{
    PolyhedronBody sourceBody = body;
    if (!sourceBody.IsValid(eps) && !TryRepairPolyhedronBodyForBrepConversion(body, sourceBody, eps))
    {
        return {false, BrepConversionIssue3d::InvalidBody, 0, {}};
    }

    std::vector<BrepVertex> vertices;
    std::vector<BrepEdge> edges;
    std::vector<BrepFace> faces;
    faces.reserve(sourceBody.FaceCount());

    for (std::size_t faceIndex = 0; faceIndex < sourceBody.FaceCount(); ++faceIndex)
    {
        const PolyhedronFace3d face = sourceBody.FaceAt(faceIndex);
        if (!face.IsValid(eps))
        {
            return {false, BrepConversionIssue3d::InvalidFace, faceIndex, {}};
        }

        const PlaneSurface planeSurface = PlaneSurface::FromPlane(face.SupportPlane());
        auto surface = std::make_shared<PlaneSurface>(planeSurface);

        BrepLoop outerLoop;
        std::vector<Point2d> outerUv;
        if (!AppendBrepLoopFromPolyLoop(face.OuterLoop(), vertices, edges, outerLoop, outerUv))
        {
            return {false, BrepConversionIssue3d::InvalidFace, faceIndex, {}};
        }

        for (std::size_t i = 0; i < face.OuterLoop().VertexCount(); ++i)
        {
            outerUv.push_back(ProjectPointToPlaneUv(face.OuterLoop().VertexAt(i), planeSurface));
        }
        CurveOnSurface outerTrim(surface, Polyline2d(std::move(outerUv), PolylineClosure::Closed));

        std::vector<BrepLoop> holeLoops;
        std::vector<CurveOnSurface> holeTrims;
        holeLoops.reserve(face.HoleCount());
        holeTrims.reserve(face.HoleCount());
        for (std::size_t holeIndex = 0; holeIndex < face.HoleCount(); ++holeIndex)
        {
            const PolyhedronLoop3d hole = face.HoleAt(holeIndex);
            BrepLoop holeLoop;
            std::vector<Point2d> holeUv;
            if (!AppendBrepLoopFromPolyLoop(hole, vertices, edges, holeLoop, holeUv))
            {
                return {false, BrepConversionIssue3d::InvalidFace, faceIndex, {}};
            }

            for (std::size_t i = 0; i < hole.VertexCount(); ++i)
            {
                holeUv.push_back(ProjectPointToPlaneUv(hole.VertexAt(i), planeSurface));
            }

            holeLoops.push_back(std::move(holeLoop));
            holeTrims.emplace_back(surface, Polyline2d(std::move(holeUv), PolylineClosure::Closed));
        }

        BrepFace brepFace(surface, outerLoop, std::move(holeLoops), std::move(outerTrim), std::move(holeTrims));
        if (!brepFace.IsValid(GeometryTolerance3d{eps, eps, eps}))
        {
            return {false, BrepConversionIssue3d::InvalidFace, faceIndex, {}};
        }

        faces.push_back(std::move(brepFace));
    }

    BrepBody brepBody(std::move(vertices), std::move(edges), {BrepShell(std::move(faces), false)});
    if (!brepBody.IsValid(GeometryTolerance3d{eps, eps, eps}))
    {
        return {false, BrepConversionIssue3d::InvalidBody, 0, {}};
    }

    return {true, BrepConversionIssue3d::None, 0, std::move(brepBody)};
}
} // namespace geometry::sdk
