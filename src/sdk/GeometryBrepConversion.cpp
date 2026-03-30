#include "sdk/GeometryBrepConversion.h"

#include <memory>
#include <vector>

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
} // namespace geometry::sdk
