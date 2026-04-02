#include "sdk/GeometryBrepConversion.h"

#include <limits>
#include <memory>
#include <unordered_map>
#include <vector>

#include "sdk/LineCurve3d.h"
#include "sdk/PlaneSurface.h"

namespace geometry::sdk
{
namespace
{
struct FaceLoopRepresentativeIds
{
    std::vector<std::size_t> outer;
    std::vector<std::vector<std::size_t>> holes;
};

[[nodiscard]] std::size_t FindOrAddRepresentativePoint(
    const Point3d& point,
    std::vector<Point3d>& representativePoints,
    double eps)
{
    for (std::size_t i = 0; i < representativePoints.size(); ++i)
    {
        if (representativePoints[i].AlmostEquals(point, eps))
        {
            return i;
        }
    }

    representativePoints.push_back(point);
    return representativePoints.size() - 1;
}

void BuildBodyLoopRepresentativeIds(
    const PolyhedronBody& body,
    std::vector<FaceLoopRepresentativeIds>& ids,
    double eps)
{
    ids.clear();
    ids.resize(body.FaceCount());

    std::vector<Point3d> representativePoints;
    for (std::size_t faceIndex = 0; faceIndex < body.FaceCount(); ++faceIndex)
    {
        const PolyhedronFace3d face = body.FaceAt(faceIndex);
        auto& faceIds = ids[faceIndex];

        faceIds.outer.reserve(face.OuterLoop().VertexCount());
        for (std::size_t i = 0; i < face.OuterLoop().VertexCount(); ++i)
        {
            faceIds.outer.push_back(FindOrAddRepresentativePoint(face.OuterLoop().VertexAt(i), representativePoints, eps));
        }

        faceIds.holes.resize(face.HoleCount());
        for (std::size_t holeIndex = 0; holeIndex < face.HoleCount(); ++holeIndex)
        {
            const PolyhedronLoop3d hole = face.HoleAt(holeIndex);
            auto& holeIds = faceIds.holes[holeIndex];
            holeIds.reserve(hole.VertexCount());
            for (std::size_t i = 0; i < hole.VertexCount(); ++i)
            {
                holeIds.push_back(FindOrAddRepresentativePoint(hole.VertexAt(i), representativePoints, eps));
            }
        }
    }
}

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

[[nodiscard]] std::size_t FindOrAddBrepVertex(
    const Point3d& point,
    std::vector<BrepVertex>& vertices,
    double eps)
{
    for (std::size_t i = 0; i < vertices.size(); ++i)
    {
        if (vertices[i].Point().AlmostEquals(point, eps))
        {
            return i;
        }
    }

    vertices.emplace_back(point);
    return vertices.size() - 1;
}

[[nodiscard]] bool FindReusableBrepEdge(
    const std::vector<BrepEdge>& edges,
    std::size_t startVertexIndex,
    std::size_t endVertexIndex,
    std::size_t& edgeIndex,
    bool& reversed)
{
    for (std::size_t i = 0; i < edges.size(); ++i)
    {
        const BrepEdge& edge = edges[i];
        if (edge.StartVertexIndex() == startVertexIndex && edge.EndVertexIndex() == endVertexIndex)
        {
            edgeIndex = i;
            reversed = false;
            return true;
        }

        if (edge.StartVertexIndex() == endVertexIndex && edge.EndVertexIndex() == startVertexIndex)
        {
            edgeIndex = i;
            reversed = true;
            return true;
        }
    }

    return false;
}

[[nodiscard]] bool AppendSharedBrepLoopFromPolyLoop(
    const PolyhedronLoop3d& polyLoop,
    std::vector<BrepVertex>& vertices,
    std::vector<BrepEdge>& edges,
    BrepLoop& loop,
    std::vector<Point2d>& uvPoints,
    const std::vector<std::size_t>* representativeIds,
    std::unordered_map<std::size_t, std::size_t>* representativeToVertexIndex,
    double eps)
{
    if (!polyLoop.IsValid(eps))
    {
        return false;
    }

    const std::size_t vertexCount = polyLoop.VertexCount();
    std::vector<std::size_t> loopVertexIndices;
    loopVertexIndices.reserve(vertexCount);

    const bool hasRepresentativeIds =
        representativeIds != nullptr && representativeToVertexIndex != nullptr && representativeIds->size() == vertexCount;

    for (std::size_t i = 0; i < vertexCount; ++i)
    {
        const Point3d point = polyLoop.VertexAt(i);
        if (hasRepresentativeIds)
        {
            const std::size_t representativeId = (*representativeIds)[i];
            const auto found = representativeToVertexIndex->find(representativeId);
            if (found != representativeToVertexIndex->end())
            {
                loopVertexIndices.push_back(found->second);
            }
            else
            {
                const std::size_t vertexIndex = FindOrAddBrepVertex(point, vertices, eps);
                (*representativeToVertexIndex)[representativeId] = vertexIndex;
                loopVertexIndices.push_back(vertexIndex);
            }
        }
        else
        {
            loopVertexIndices.push_back(FindOrAddBrepVertex(point, vertices, eps));
        }
    }

    std::vector<BrepCoedge> coedges;
    coedges.reserve(vertexCount);
    for (std::size_t i = 0; i < vertexCount; ++i)
    {
        const std::size_t next = (i + 1) % vertexCount;
        const std::size_t startVertexIndex = loopVertexIndices[i];
        const std::size_t endVertexIndex = loopVertexIndices[next];

        std::size_t edgeIndex = static_cast<std::size_t>(-1);
        bool reversed = false;
        if (!FindReusableBrepEdge(edges, startVertexIndex, endVertexIndex, edgeIndex, reversed))
        {
            const Point3d first = vertices[startVertexIndex].Point();
            const Point3d second = vertices[endVertexIndex].Point();
            edges.emplace_back(
                std::make_shared<LineCurve3d>(LineCurve3d::FromLine(
                    Line3d::FromOriginAndDirection(first, second - first),
                    Intervald{0.0, 1.0})),
                startVertexIndex,
                endVertexIndex);
            edgeIndex = edges.size() - 1;
        }

        coedges.emplace_back(edgeIndex, reversed);
    }

    loop = BrepLoop(std::move(coedges));
    uvPoints.clear();
    uvPoints.reserve(vertexCount);
    return loop.IsValid();
}

[[nodiscard]] bool ComputeSharedShellClosed(
    const std::vector<BrepFace>& faces)
{
    std::vector<std::size_t> edgeUseCounts;
    for (const BrepFace& face : faces)
    {
        for (const BrepCoedge& coedge : face.OuterLoop().Coedges())
        {
            if (coedge.EdgeIndex() >= edgeUseCounts.size())
            {
                edgeUseCounts.resize(coedge.EdgeIndex() + 1, 0);
            }
            ++edgeUseCounts[coedge.EdgeIndex()];
        }

        for (const BrepLoop& holeLoop : face.HoleLoops())
        {
            for (const BrepCoedge& coedge : holeLoop.Coedges())
            {
                if (coedge.EdgeIndex() >= edgeUseCounts.size())
                {
                    edgeUseCounts.resize(coedge.EdgeIndex() + 1, 0);
                }
                ++edgeUseCounts[coedge.EdgeIndex()];
            }
        }
    }

    if (edgeUseCounts.empty())
    {
        return false;
    }

    for (const std::size_t count : edgeUseCounts)
    {
        if (count != 2)
        {
            return false;
        }
    }

    return true;
}

[[nodiscard]] bool BuildFaceWithRefitSupportPlane(
    const PolyhedronFace3d& face,
    PolyhedronFace3d& repairedFace,
    double eps,
    const std::vector<bool>& preferredOuterVertices)
{
    auto normalizeLoop = [&](const PolyhedronLoop3d& loop, PolyhedronLoop3d& normalized, std::vector<std::size_t>* keptIndices) {
        std::vector<Point3d> vertices;
        if (keptIndices != nullptr)
        {
            keptIndices->clear();
            keptIndices->reserve(loop.VertexCount());
        }

        vertices.reserve(loop.VertexCount());
        for (std::size_t i = 0; i < loop.VertexCount(); ++i)
        {
            const Point3d point = loop.VertexAt(i);
            if (vertices.empty() || !vertices.back().AlmostEquals(point, eps))
            {
                vertices.push_back(point);
                if (keptIndices != nullptr)
                {
                    keptIndices->push_back(i);
                }
            }
        }

        while (vertices.size() >= 2 && vertices.front().AlmostEquals(vertices.back(), eps))
        {
            vertices.pop_back();
            if (keptIndices != nullptr && !keptIndices->empty())
            {
                keptIndices->pop_back();
            }
        }

        normalized = PolyhedronLoop3d(std::move(vertices));
        return normalized.IsValid(eps);
    };

    PolyhedronLoop3d outer{};
    std::vector<std::size_t> outerKeptIndices;
    if (!normalizeLoop(face.OuterLoop(), outer, &outerKeptIndices) || outer.VertexCount() < 3)
    {
        return false;
    }

    std::vector<bool> normalizedPreferredOuter(outer.VertexCount(), false);
    if (!preferredOuterVertices.empty())
    {
        for (std::size_t i = 0; i < outerKeptIndices.size(); ++i)
        {
            const std::size_t sourceIndex = outerKeptIndices[i];
            if (sourceIndex < preferredOuterVertices.size())
            {
                normalizedPreferredOuter[i] = preferredOuterVertices[sourceIndex];
            }
        }
    }

    auto maxLoopScaleSquared = [&](const PolyhedronLoop3d& loop) {
        double maxDistanceSquared = 0.0;
        for (std::size_t i = 0; i < loop.VertexCount(); ++i)
        {
            for (std::size_t j = i + 1; j < loop.VertexCount(); ++j)
            {
                const double distanceSquared = (loop.VertexAt(i) - loop.VertexAt(j)).LengthSquared();
                if (distanceSquared > maxDistanceSquared)
                {
                    maxDistanceSquared = distanceSquared;
                }
            }
        }

        return std::max(maxDistanceSquared, eps * eps);
    };

    bool foundSupport = false;
    Point3d p0{};
    Vector3d normal{};
    double bestNormalLength = -1.0;
    Point3d bestP0{};
    Vector3d bestNormal{};

    int bestPreferredCount = -1;
    double bestPreferredNormalLength = -1.0;
    Point3d preferredP0{};
    Vector3d preferredNormal{};

    for (std::size_t i = 0; i + 2 < outer.VertexCount(); ++i)
    {
        const Point3d a = outer.VertexAt(i);
        for (std::size_t j = i + 1; j + 1 < outer.VertexCount(); ++j)
        {
            const Point3d b = outer.VertexAt(j);
            for (std::size_t k = j + 1; k < outer.VertexCount(); ++k)
            {
                const Point3d c = outer.VertexAt(k);
                const Vector3d candidateNormal = Cross(b - a, c - a);
                const double candidateLength = candidateNormal.Length();
                if (candidateLength > bestNormalLength)
                {
                    bestNormalLength = candidateLength;
                    bestP0 = a;
                    bestNormal = candidateNormal;
                }

                if (candidateLength > eps)
                {
                    const int preferredCount =
                        static_cast<int>(normalizedPreferredOuter[i]) +
                        static_cast<int>(normalizedPreferredOuter[j]) +
                        static_cast<int>(normalizedPreferredOuter[k]);

                    if (!foundSupport || preferredCount > bestPreferredCount ||
                        (preferredCount == bestPreferredCount && candidateLength > bestPreferredNormalLength))
                    {
                        foundSupport = true;
                        bestPreferredCount = preferredCount;
                        bestPreferredNormalLength = candidateLength;
                        preferredP0 = a;
                        preferredNormal = candidateNormal;
                    }
                }
            }
        }
    }

    if (foundSupport)
    {
        p0 = preferredP0;
        normal = preferredNormal;
    }
    else
    {
        const double scaleAwareThreshold = maxLoopScaleSquared(outer) * eps;
        if (bestNormalLength <= scaleAwareThreshold)
        {
            return false;
        }

        p0 = bestP0;
        normal = bestNormal;
        foundSupport = true;
    }

    const Plane refitPlane = Plane::FromPointAndNormal(p0, normal);

    std::vector<PolyhedronLoop3d> holes;
    holes.reserve(face.HoleCount());
    for (std::size_t i = 0; i < face.HoleCount(); ++i)
    {
        PolyhedronLoop3d hole{};
        if (!normalizeLoop(face.HoleAt(i), hole, nullptr))
        {
            return false;
        }
        holes.push_back(hole);
    }

    repairedFace = PolyhedronFace3d(refitPlane, outer, holes);
    if (repairedFace.IsValid(eps))
    {
        return true;
    }

    auto projectLoopToPlane = [&](const PolyhedronLoop3d& loop) {
        std::vector<Point3d> projected;
        projected.reserve(loop.VertexCount());
        const Vector3d unitNormal = refitPlane.UnitNormal(eps);
        for (std::size_t i = 0; i < loop.VertexCount(); ++i)
        {
            const Point3d point = loop.VertexAt(i);
            const double signedDistance = refitPlane.SignedDistanceTo(point, eps);
            projected.push_back(point - unitNormal * signedDistance);
        }
        return PolyhedronLoop3d(std::move(projected));
    };

    PolyhedronLoop3d projectedOuter = projectLoopToPlane(outer);
    if (!projectedOuter.IsValid(eps))
    {
        return false;
    }

    std::vector<PolyhedronLoop3d> projectedHoles;
    projectedHoles.reserve(holes.size());
    for (const PolyhedronLoop3d& hole : holes)
    {
        PolyhedronLoop3d projectedHole = projectLoopToPlane(hole);
        if (!projectedHole.IsValid(eps))
        {
            return false;
        }
        projectedHoles.push_back(std::move(projectedHole));
    }

    repairedFace = PolyhedronFace3d(refitPlane, std::move(projectedOuter), std::move(projectedHoles));
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

    std::vector<std::vector<bool>> facePreferredOuterVertices(body.FaceCount());
    {
        std::vector<Point3d> representatives;
        std::vector<std::size_t> representativeCounts;
        std::vector<std::vector<std::size_t>> faceRepresentativeIndices(body.FaceCount());

        for (std::size_t faceIndex = 0; faceIndex < body.FaceCount(); ++faceIndex)
        {
            const PolyhedronLoop3d outer = body.FaceAt(faceIndex).OuterLoop();
            auto& indices = faceRepresentativeIndices[faceIndex];
            indices.reserve(outer.VertexCount());

            for (std::size_t vertexIndex = 0; vertexIndex < outer.VertexCount(); ++vertexIndex)
            {
                const Point3d point = outer.VertexAt(vertexIndex);

                std::size_t representativeIndex = static_cast<std::size_t>(-1);
                for (std::size_t i = 0; i < representatives.size(); ++i)
                {
                    if (representatives[i].AlmostEquals(point, eps))
                    {
                        representativeIndex = i;
                        break;
                    }
                }

                if (representativeIndex == static_cast<std::size_t>(-1))
                {
                    representatives.push_back(point);
                    representativeCounts.push_back(0);
                    representativeIndex = representatives.size() - 1;
                }

                ++representativeCounts[representativeIndex];
                indices.push_back(representativeIndex);
            }
        }

        for (std::size_t faceIndex = 0; faceIndex < body.FaceCount(); ++faceIndex)
        {
            const auto& indices = faceRepresentativeIndices[faceIndex];
            auto& preferred = facePreferredOuterVertices[faceIndex];
            preferred.resize(indices.size(), false);
            for (std::size_t i = 0; i < indices.size(); ++i)
            {
                preferred[i] = representativeCounts[indices[i]] > 1;
            }
        }
    }

    std::vector<PolyhedronFace3d> repairedFaces;
    repairedFaces.reserve(body.FaceCount());
    for (std::size_t i = 0; i < body.FaceCount(); ++i)
    {
        PolyhedronFace3d repairedFace{};
        if (!BuildFaceWithRefitSupportPlane(body.FaceAt(i), repairedFace, eps, facePreferredOuterVertices[i]))
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
    std::vector<FaceLoopRepresentativeIds> sourceRepresentativeIds;
    BuildBodyLoopRepresentativeIds(body, sourceRepresentativeIds, eps);

    PolyhedronBody sourceBody = body;
    if (!sourceBody.IsValid(eps) && !TryRepairPolyhedronBodyForBrepConversion(body, sourceBody, eps))
    {
        return {false, BrepConversionIssue3d::InvalidBody, 0, {}};
    }

    std::vector<BrepVertex> vertices;
    std::vector<BrepEdge> edges;
    std::vector<BrepFace> faces;
    std::unordered_map<std::size_t, std::size_t> representativeToVertexIndex;
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

        const std::vector<std::size_t>* outerRepresentativeIds = nullptr;
        if (faceIndex < sourceRepresentativeIds.size())
        {
            const auto& candidateOuterIds = sourceRepresentativeIds[faceIndex].outer;
            if (candidateOuterIds.size() == face.OuterLoop().VertexCount())
            {
                outerRepresentativeIds = &candidateOuterIds;
            }
        }

        BrepLoop outerLoop;
        std::vector<Point2d> outerUv;
        if (!AppendSharedBrepLoopFromPolyLoop(
                face.OuterLoop(),
                vertices,
                edges,
                outerLoop,
                outerUv,
                outerRepresentativeIds,
                &representativeToVertexIndex,
                eps))
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

            const std::vector<std::size_t>* holeRepresentativeIds = nullptr;
            if (faceIndex < sourceRepresentativeIds.size() &&
                holeIndex < sourceRepresentativeIds[faceIndex].holes.size())
            {
                const auto& candidateHoleIds = sourceRepresentativeIds[faceIndex].holes[holeIndex];
                if (candidateHoleIds.size() == hole.VertexCount())
                {
                    holeRepresentativeIds = &candidateHoleIds;
                }
            }

            BrepLoop holeLoop;
            std::vector<Point2d> holeUv;
            if (!AppendSharedBrepLoopFromPolyLoop(
                    hole,
                    vertices,
                    edges,
                    holeLoop,
                    holeUv,
                    holeRepresentativeIds,
                    &representativeToVertexIndex,
                    eps))
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

    const bool shellClosed = ComputeSharedShellClosed(faces);
    BrepBody brepBody(std::move(vertices), std::move(edges), {BrepShell(std::move(faces), shellClosed)});
    if (!brepBody.IsValid(GeometryTolerance3d{eps, eps, eps}))
    {
        return {false, BrepConversionIssue3d::InvalidBody, 0, {}};
    }

    return {true, BrepConversionIssue3d::None, 0, std::move(brepBody)};
}
} // namespace geometry::sdk
