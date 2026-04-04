#include "sdk/GeometryBrepConversion.h"

#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <unordered_map>
#include <utility>
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

struct RepresentativePointAccumulator
{
    Vector3d sum{};
    std::size_t count{0};
};

struct NonPlanarRepairPassResult
{
    PolyhedronBody body{};
    std::vector<FaceLoopRepresentativeIds> representativeIds;
    std::unordered_map<std::size_t, Point3d> representativeTargetPoints;
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

template <typename FaceAccessor>
[[nodiscard]] bool ComputeRepresentativeTargetPointsForFaceRange(
    std::size_t faceCount,
    const FaceAccessor& faceAt,
    const std::vector<FaceLoopRepresentativeIds>& representativeIds,
    std::unordered_map<std::size_t, Point3d>& representativeTargetPoints)
{
    representativeTargetPoints.clear();
    if (representativeIds.size() != faceCount)
    {
        return false;
    }

    std::unordered_map<std::size_t, RepresentativePointAccumulator> accumulators;

    auto accumulateLoop = [&](const PolyhedronLoop3d& loop, const std::vector<std::size_t>& loopIds) {
        if (loop.VertexCount() != loopIds.size())
        {
            return false;
        }

        for (std::size_t i = 0; i < loop.VertexCount(); ++i)
        {
            const Point3d point = loop.VertexAt(i);
            auto& accumulator = accumulators[loopIds[i]];
            accumulator.sum = accumulator.sum + (point - Point3d{});
            ++accumulator.count;
        }
        return true;
    };

    for (std::size_t faceIndex = 0; faceIndex < faceCount; ++faceIndex)
    {
        const PolyhedronFace3d face = faceAt(faceIndex);
        const FaceLoopRepresentativeIds& ids = representativeIds[faceIndex];

        if (!accumulateLoop(face.OuterLoop(), ids.outer))
        {
            return false;
        }

        if (ids.holes.size() != face.HoleCount())
        {
            return false;
        }

        for (std::size_t holeIndex = 0; holeIndex < face.HoleCount(); ++holeIndex)
        {
            if (!accumulateLoop(face.HoleAt(holeIndex), ids.holes[holeIndex]))
            {
                return false;
            }
        }
    }

    representativeTargetPoints.reserve(accumulators.size());
    for (const auto& [id, accumulator] : accumulators)
    {
        if (accumulator.count == 0)
        {
            return false;
        }

        const Vector3d average = accumulator.sum / static_cast<double>(accumulator.count);
        representativeTargetPoints.emplace(id, Point3d{average.x, average.y, average.z});
    }

    return true;
}

[[nodiscard]] bool ExecuteRepresentativeTargetAggregationPass(
    const std::vector<PolyhedronFace3d>& faces,
    const std::vector<FaceLoopRepresentativeIds>& representativeIds,
    std::unordered_map<std::size_t, Point3d>& representativeTargetPoints)
{
    return ComputeRepresentativeTargetPointsForFaceRange(
        faces.size(),
        [&faces](const std::size_t faceIndex) -> const PolyhedronFace3d& { return faces[faceIndex]; },
        representativeIds,
        representativeTargetPoints);
}

[[nodiscard]] bool ExecuteRepresentativeTargetAggregationPass(
    const PolyhedronBody& body,
    const std::vector<FaceLoopRepresentativeIds>& representativeIds,
    std::unordered_map<std::size_t, Point3d>& representativeTargetPoints)
{
    return ComputeRepresentativeTargetPointsForFaceRange(
        body.FaceCount(),
        [&body](const std::size_t faceIndex) { return body.FaceAt(faceIndex); },
        representativeIds,
        representativeTargetPoints);
}

[[nodiscard]] bool ExecuteCrossFaceSnappingPass(
    const std::vector<FaceLoopRepresentativeIds>& representativeIds,
    const std::unordered_map<std::size_t, Point3d>& representativeTargetPoints,
    std::vector<PolyhedronFace3d>& faces,
    double eps)
{
    if (representativeIds.size() != faces.size())
    {
        return false;
    }

    auto snapLoopToFacePlane = [&](const PolyhedronLoop3d& loop, const std::vector<std::size_t>& loopIds, const Plane& plane) {
        std::vector<Point3d> snapped;
        snapped.reserve(loop.VertexCount());

        const Vector3d unitNormal = plane.UnitNormal(eps);
        for (std::size_t i = 0; i < loop.VertexCount(); ++i)
        {
            const auto targetIt = representativeTargetPoints.find(loopIds[i]);
            const Point3d target = targetIt != representativeTargetPoints.end() ? targetIt->second : loop.VertexAt(i);

            // Keep each snapped point on this face support plane.
            const double signedDistance = plane.SignedDistanceTo(target, eps);
            snapped.push_back(target - unitNormal * signedDistance);
        }

        return PolyhedronLoop3d(std::move(snapped));
    };

    for (std::size_t faceIndex = 0; faceIndex < faces.size(); ++faceIndex)
    {
        const PolyhedronFace3d& face = faces[faceIndex];
        const FaceLoopRepresentativeIds& ids = representativeIds[faceIndex];
        const Plane plane = face.SupportPlane();

        PolyhedronLoop3d outer = snapLoopToFacePlane(face.OuterLoop(), ids.outer, plane);
        if (!outer.IsValid(eps))
        {
            return false;
        }

        std::vector<PolyhedronLoop3d> holes;
        holes.reserve(face.HoleCount());
        for (std::size_t holeIndex = 0; holeIndex < face.HoleCount(); ++holeIndex)
        {
            PolyhedronLoop3d hole = snapLoopToFacePlane(face.HoleAt(holeIndex), ids.holes[holeIndex], plane);
            if (!hole.IsValid(eps))
            {
                return false;
            }
            holes.push_back(std::move(hole));
        }

        PolyhedronFace3d snappedFace(plane, std::move(outer), std::move(holes));
        if (!snappedFace.IsValid(eps))
        {
            return false;
        }

        faces[faceIndex] = std::move(snappedFace);
    }

    return true;
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
    const std::unordered_map<std::size_t, Point3d>* representativeTargetPoints,
    std::unordered_map<std::size_t, std::size_t>* representativeToVertexIndex,
    double eps)
{
    try
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
                if (i >= representativeIds->size())
                {
                    return false;
                }
                
                const std::size_t representativeId = (*representativeIds)[i];
                const auto found = representativeToVertexIndex->find(representativeId);
                if (found != representativeToVertexIndex->end())
                {
                    if (found->second >= vertices.size())
                    {
                        return false;
                    }

                    loopVertexIndices.push_back(found->second);
                }
                else
                {
                    Point3d representativePoint = point;
                    if (representativeTargetPoints != nullptr)
                    {
                        const auto representativePointIt = representativeTargetPoints->find(representativeId);
                        if (representativePointIt != representativeTargetPoints->end())
                        {
                            representativePoint = representativePointIt->second;
                        }
                    }

                    const std::size_t vertexIndex = FindOrAddBrepVertex(representativePoint, vertices, eps);
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
        
        if (loopVertexIndices.size() != vertexCount)
        {
            return false;
        }
        
        for (std::size_t i = 0; i < vertexCount; ++i)
        {
            const std::size_t next = (i + 1) % vertexCount;
            if (i >= loopVertexIndices.size() || next >= loopVertexIndices.size())
            {
                return false;
            }
            
            const std::size_t startVertexIndex = loopVertexIndices[i];
            const std::size_t endVertexIndex = loopVertexIndices[next];
            if (startVertexIndex >= vertices.size() || endVertexIndex >= vertices.size())
            {
                return false;
            }

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
    catch (const std::exception&)
    {
        return false;
    }
}

void BuildSharedOuterVertexPreferenceMask(
    const PolyhedronBody& body,
    std::vector<std::vector<bool>>& facePreferredOuterVertices,
    double eps)
{
    facePreferredOuterVertices.clear();
    facePreferredOuterVertices.resize(body.FaceCount());

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
    const std::vector<bool>& preferredOuterVertices,
    const std::vector<std::size_t>* sourceOuterRepresentativeIds,
    const std::vector<std::vector<std::size_t>>* sourceHoleRepresentativeIds,
    FaceLoopRepresentativeIds* repairedRepresentativeIds)
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

    std::vector<std::size_t> normalizedOuterRepresentativeIds;
    if (sourceOuterRepresentativeIds != nullptr && sourceOuterRepresentativeIds->size() == face.OuterLoop().VertexCount())
    {
        normalizedOuterRepresentativeIds.reserve(outerKeptIndices.size());
        for (const std::size_t sourceIndex : outerKeptIndices)
        {
            normalizedOuterRepresentativeIds.push_back((*sourceOuterRepresentativeIds)[sourceIndex]);
        }
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

    std::vector<PolyhedronLoop3d> holes;
    std::vector<std::vector<std::size_t>> normalizedHoleRepresentativeIds;
    holes.reserve(face.HoleCount());
    normalizedHoleRepresentativeIds.reserve(face.HoleCount());
    for (std::size_t i = 0; i < face.HoleCount(); ++i)
    {
        PolyhedronLoop3d hole{};
        std::vector<std::size_t> keptIndices;
        if (!normalizeLoop(face.HoleAt(i), hole, &keptIndices))
        {
            return false;
        }
        holes.push_back(hole);

        std::vector<std::size_t> normalizedHoleIds;
        if (sourceHoleRepresentativeIds != nullptr &&
            i < sourceHoleRepresentativeIds->size() &&
            (*sourceHoleRepresentativeIds)[i].size() == face.HoleAt(i).VertexCount())
        {
            const auto& sourceHoleIds = (*sourceHoleRepresentativeIds)[i];
            normalizedHoleIds.reserve(keptIndices.size());
            for (const std::size_t sourceIndex : keptIndices)
            {
                normalizedHoleIds.push_back(sourceHoleIds[sourceIndex]);
            }
        }
        normalizedHoleRepresentativeIds.push_back(std::move(normalizedHoleIds));
    }

    bool foundSupport = false;
    Point3d p0{};
    Vector3d normal{};
    double bestNormalLength = -1.0;
    Point3d bestP0{};
    Vector3d bestNormal{};
    double bestTotalDistance = std::numeric_limits<double>::infinity();
    double bestMaxDistance = std::numeric_limits<double>::infinity();
    int bestPreferredCount = -1;
    double bestPreferredNormalLength = -1.0;

    struct CandidatePoint
    {
        Point3d point;
        bool preferred{false};
    };

    std::vector<CandidatePoint> candidatePoints;
    candidatePoints.reserve(
        outer.VertexCount() +
        std::accumulate(
            holes.begin(),
            holes.end(),
            std::size_t{0},
            [](std::size_t total, const PolyhedronLoop3d& loop) { return total + loop.VertexCount(); }));
    for (std::size_t i = 0; i < outer.VertexCount(); ++i)
    {
        candidatePoints.push_back(CandidatePoint{outer.VertexAt(i), normalizedPreferredOuter[i]});
    }
    for (const PolyhedronLoop3d& hole : holes)
    {
        for (std::size_t i = 0; i < hole.VertexCount(); ++i)
        {
            candidatePoints.push_back(CandidatePoint{hole.VertexAt(i), false});
        }
    }

    auto scorePlane = [&](const Plane& plane) {
        double totalDistance = 0.0;
        double maxDistance = 0.0;

        const auto accumulateLoopDistance = [&](const PolyhedronLoop3d& loop) {
            for (std::size_t vertexIndex = 0; vertexIndex < loop.VertexCount(); ++vertexIndex)
            {
                const double distance = std::abs(plane.SignedDistanceTo(loop.VertexAt(vertexIndex), eps));
                totalDistance += distance;
                maxDistance = std::max(maxDistance, distance);
            }
        };

        accumulateLoopDistance(outer);
        for (const PolyhedronLoop3d& hole : holes)
        {
            accumulateLoopDistance(hole);
        }

        return std::pair<double, double>{totalDistance, maxDistance};
    };

    for (std::size_t i = 0; i + 2 < candidatePoints.size(); ++i)
    {
        const Point3d a = candidatePoints[i].point;
        for (std::size_t j = i + 1; j + 1 < candidatePoints.size(); ++j)
        {
            const Point3d b = candidatePoints[j].point;
            for (std::size_t k = j + 1; k < candidatePoints.size(); ++k)
            {
                const Point3d c = candidatePoints[k].point;
                const Vector3d candidateNormal = Cross(b - a, c - a);
                const double candidateLength = candidateNormal.Length();
                if (candidateLength > bestNormalLength)
                {
                    bestNormalLength = candidateLength;
                    bestP0 = a;
                    bestNormal = candidateNormal;
                }

                if (candidateLength <= eps)
                {
                    continue;
                }

                const Plane candidatePlane = Plane::FromPointAndNormal(a, candidateNormal);
                const auto [totalDistance, maxDistance] = scorePlane(candidatePlane);
                const int preferredCount =
                    static_cast<int>(candidatePoints[i].preferred) +
                    static_cast<int>(candidatePoints[j].preferred) +
                    static_cast<int>(candidatePoints[k].preferred);

                const bool betterFit =
                    !foundSupport ||
                    totalDistance + eps < bestTotalDistance ||
                    (std::abs(totalDistance - bestTotalDistance) <= eps &&
                     (maxDistance + eps < bestMaxDistance ||
                      (std::abs(maxDistance - bestMaxDistance) <= eps &&
                       (preferredCount > bestPreferredCount ||
                        (preferredCount == bestPreferredCount &&
                         candidateLength > bestPreferredNormalLength)))));
                if (betterFit)
                {
                    foundSupport = true;
                    p0 = a;
                    normal = candidateNormal;
                    bestTotalDistance = totalDistance;
                    bestMaxDistance = maxDistance;
                    bestPreferredCount = preferredCount;
                    bestPreferredNormalLength = candidateLength;
                }
            }
        }
    }

    if (!foundSupport)
    {
        const double scaleAwareThreshold = maxLoopScaleSquared(outer) * eps;
        if (bestNormalLength <= scaleAwareThreshold)
        {
            return false;
        }

        p0 = bestP0;
        normal = bestNormal;
    }

    const Plane refitPlane = Plane::FromPointAndNormal(p0, normal);

    repairedFace = PolyhedronFace3d(refitPlane, outer, holes);
    if (repairedFace.IsValid(eps))
    {
        if (repairedRepresentativeIds != nullptr)
        {
            repairedRepresentativeIds->outer = std::move(normalizedOuterRepresentativeIds);
            repairedRepresentativeIds->holes = std::move(normalizedHoleRepresentativeIds);
        }
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
    if (!repairedFace.IsValid(eps))
    {
        return false;
    }

    if (repairedRepresentativeIds != nullptr)
    {
        repairedRepresentativeIds->outer = std::move(normalizedOuterRepresentativeIds);
        repairedRepresentativeIds->holes = std::move(normalizedHoleRepresentativeIds);
    }
    return true;
}

[[nodiscard]] bool ExecuteSupportPlaneScoringPass(
    const PolyhedronBody& body,
    const std::vector<std::vector<bool>>& facePreferredOuterVertices,
    const std::vector<FaceLoopRepresentativeIds>* sourceRepresentativeIds,
    std::vector<PolyhedronFace3d>& repairedFaces,
    std::vector<FaceLoopRepresentativeIds>& repairedRepresentativeIds,
    double eps)
{
    repairedFaces.clear();
    repairedFaces.reserve(body.FaceCount());
    repairedRepresentativeIds.clear();
    repairedRepresentativeIds.resize(body.FaceCount());
    const std::vector<bool> emptyPreferredOuterVertices;

    for (std::size_t faceIndex = 0; faceIndex < body.FaceCount(); ++faceIndex)
    {
        const std::vector<bool>& preferredOuterVertices =
            faceIndex < facePreferredOuterVertices.size()
                ? facePreferredOuterVertices[faceIndex]
                : emptyPreferredOuterVertices;
        const std::vector<std::size_t>* sourceOuterIds = nullptr;
        const std::vector<std::vector<std::size_t>>* sourceHoleIds = nullptr;
        if (sourceRepresentativeIds != nullptr && faceIndex < sourceRepresentativeIds->size())
        {
            sourceOuterIds = &(*sourceRepresentativeIds)[faceIndex].outer;
            sourceHoleIds = &(*sourceRepresentativeIds)[faceIndex].holes;
        }

        FaceLoopRepresentativeIds repairedIds;
        PolyhedronFace3d repairedFace{};
        if (!BuildFaceWithRefitSupportPlane(
                body.FaceAt(faceIndex),
                repairedFace,
                eps,
                preferredOuterVertices,
                sourceOuterIds,
                sourceHoleIds,
                sourceRepresentativeIds != nullptr ? &repairedIds : nullptr))
        {
            return false;
        }

        repairedFaces.push_back(std::move(repairedFace));
        if (sourceRepresentativeIds != nullptr)
        {
            repairedRepresentativeIds[faceIndex] = std::move(repairedIds);
        }
    }

    if (sourceRepresentativeIds == nullptr)
    {
        repairedRepresentativeIds.clear();
    }

    return true;
}

[[nodiscard]] bool ExecuteTopologyReconciliationPass(
    const std::vector<PolyhedronFace3d>& faces,
    const std::vector<FaceLoopRepresentativeIds>& representativeIds,
    NonPlanarRepairPassResult& repairResult,
    double eps)
{
    PolyhedronBody repairedBody(faces);
    if (!repairedBody.IsValid(eps))
    {
        return false;
    }

    repairResult.body = std::move(repairedBody);
    repairResult.representativeIds = representativeIds;
    repairResult.representativeTargetPoints.clear();
    if (!representativeIds.empty() &&
        !ExecuteRepresentativeTargetAggregationPass(
            repairResult.body,
            repairResult.representativeIds,
            repairResult.representativeTargetPoints))
    {
        return false;
    }

    return true;
}

[[nodiscard]] bool TryRepairPolyhedronBodyForBrepConversion(
    const PolyhedronBody& body,
    const std::vector<FaceLoopRepresentativeIds>* sourceRepresentativeIds,
    NonPlanarRepairPassResult& repairResult,
    double eps)
{
    if (body.IsEmpty())
    {
        return false;
    }

    std::vector<std::vector<bool>> facePreferredOuterVertices;
    BuildSharedOuterVertexPreferenceMask(body, facePreferredOuterVertices, eps);

    // Support-plane scoring pass: select a refit support plane per face and
    // normalize loop vertices / representative ids.
    std::vector<PolyhedronFace3d> repairedFaces;
    std::vector<FaceLoopRepresentativeIds> repairedRepresentativeIds;
    if (!ExecuteSupportPlaneScoringPass(
            body,
            facePreferredOuterVertices,
            sourceRepresentativeIds,
            repairedFaces,
            repairedRepresentativeIds,
            eps))
    {
        return false;
    }

    if (repairedRepresentativeIds.empty())
    {
        repairResult = {};
        repairResult.body = PolyhedronBody(std::move(repairedFaces));
        return repairResult.body.IsValid(eps);
    }

    // Establish a topology-reconciled baseline before iterative
    // representative aggregation / cross-face snapping.
    if (!ExecuteTopologyReconciliationPass(
            repairedFaces,
            repairedRepresentativeIds,
            repairResult,
            eps))
    {
        return false;
    }

    std::vector<PolyhedronFace3d> currentFaces = repairedFaces;
    for (int iteration = 0; iteration < 2; ++iteration)
    {
        // Representative target aggregation pass.
        std::unordered_map<std::size_t, Point3d> representativeTargetPoints;
        if (!ExecuteRepresentativeTargetAggregationPass(
                currentFaces,
                repairedRepresentativeIds,
                representativeTargetPoints))
        {
            break;
        }

        // Cross-face snapping pass.
        std::vector<PolyhedronFace3d> snappedFaces = currentFaces;
        if (!ExecuteCrossFaceSnappingPass(
                repairedRepresentativeIds,
                representativeTargetPoints,
                snappedFaces,
                eps))
        {
            break;
        }

        // Topology reconciliation pass.
        NonPlanarRepairPassResult snappedResult;
        if (!ExecuteTopologyReconciliationPass(
                snappedFaces,
                repairedRepresentativeIds,
                snappedResult,
                eps))
        {
            break;
        }

        currentFaces = std::move(snappedFaces);
        repairResult = std::move(snappedResult);
    }

    return true;
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
    try
    {
    std::vector<FaceLoopRepresentativeIds> sourceRepresentativeIds;
    BuildBodyLoopRepresentativeIds(body, sourceRepresentativeIds, eps);

    NonPlanarRepairPassResult repairResult;
    PolyhedronBody sourceBody = body;
    const bool requiresRepair = !sourceBody.IsValid(eps);
    if (requiresRepair &&
        !TryRepairPolyhedronBodyForBrepConversion(
            body,
            &sourceRepresentativeIds,
            repairResult,
            eps))
    {
        return {false, BrepConversionIssue3d::InvalidBody, 0, {}};
    }

    if (requiresRepair)
    {
        sourceBody = repairResult.body;
    }

    const std::vector<FaceLoopRepresentativeIds>& representativeIds =
        requiresRepair ? repairResult.representativeIds : sourceRepresentativeIds;

    std::unordered_map<std::size_t, Point3d> representativeTargetPoints;
    bool hasRepresentativeTargetPoints = false;
    if (requiresRepair)
    {
        representativeTargetPoints = repairResult.representativeTargetPoints;
        hasRepresentativeTargetPoints =
            !representativeTargetPoints.empty() ||
            ExecuteRepresentativeTargetAggregationPass(sourceBody, representativeIds, representativeTargetPoints);
    }
    else
    {
        hasRepresentativeTargetPoints =
            ExecuteRepresentativeTargetAggregationPass(sourceBody, representativeIds, representativeTargetPoints);
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
        if (faceIndex < representativeIds.size())
        {
            const auto& candidateOuterIds = representativeIds[faceIndex].outer;
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
                hasRepresentativeTargetPoints ? &representativeTargetPoints : nullptr,
                &representativeToVertexIndex,
                eps))
        {
            return {false, BrepConversionIssue3d::InvalidFace, faceIndex, {}};
        }

        try
        {
            for (std::size_t i = 0; i < face.OuterLoop().VertexCount(); ++i)
            {
                outerUv.push_back(ProjectPointToPlaneUv(face.OuterLoop().VertexAt(i), planeSurface));
            }
        }
        catch (const std::exception&)
        {
            return {false, BrepConversionIssue3d::InvalidFace, faceIndex, {}};
        }
        
        if (outerUv.size() != face.OuterLoop().VertexCount())
        {
            return {false, BrepConversionIssue3d::InvalidFace, faceIndex, {}};
        }
        
        CurveOnSurface outerTrim;
        try
        {
            outerTrim = CurveOnSurface(surface, Polyline2d(std::move(outerUv), PolylineClosure::Closed));
        }
        catch (const std::exception&)
        {
            return {false, BrepConversionIssue3d::InvalidFace, faceIndex, {}};
        }

        std::vector<BrepLoop> holeLoops;
        std::vector<CurveOnSurface> holeTrims;
        holeLoops.reserve(face.HoleCount());
        holeTrims.reserve(face.HoleCount());
        for (std::size_t holeIndex = 0; holeIndex < face.HoleCount(); ++holeIndex)
        {
            try
            {
                const PolyhedronLoop3d hole = face.HoleAt(holeIndex);

            const std::vector<std::size_t>* holeRepresentativeIds = nullptr;
            if (faceIndex < representativeIds.size() &&
                holeIndex < representativeIds[faceIndex].holes.size())
            {
                const auto& candidateHoleIds = representativeIds[faceIndex].holes[holeIndex];
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
                    hasRepresentativeTargetPoints ? &representativeTargetPoints : nullptr,
                    &representativeToVertexIndex,
                    eps))
            {
                return {false, BrepConversionIssue3d::InvalidFace, faceIndex, {}};
            }

            try
            {
                for (std::size_t i = 0; i < hole.VertexCount(); ++i)
                {
                    holeUv.push_back(ProjectPointToPlaneUv(hole.VertexAt(i), planeSurface));
                }
            }
            catch (const std::exception&)
            {
                return {false, BrepConversionIssue3d::InvalidFace, faceIndex, {}};
            }
            
            if (holeUv.size() != hole.VertexCount())
            {
                return {false, BrepConversionIssue3d::InvalidFace, faceIndex, {}};
            }

            holeLoops.push_back(std::move(holeLoop));
            holeTrims.emplace_back(surface, Polyline2d(std::move(holeUv), PolylineClosure::Closed));
            }
            catch (const std::exception&)
            {
                return {false, BrepConversionIssue3d::InvalidFace, faceIndex, {}};
            }
        }

        try
        {
            BrepFace brepFace(surface, outerLoop, std::move(holeLoops), std::move(outerTrim), std::move(holeTrims));
            if (!brepFace.IsValid(GeometryTolerance3d{eps, eps, eps}))
            {
                return {false, BrepConversionIssue3d::InvalidFace, faceIndex, {}};
            }

            faces.push_back(std::move(brepFace));
        }
        catch (const std::exception&)
        {
            return {false, BrepConversionIssue3d::InvalidFace, faceIndex, {}};
        }
    }

    try
    {
        const bool shellClosed = ComputeSharedShellClosed(faces);
        BrepBody brepBody(std::move(vertices), std::move(edges), {BrepShell(std::move(faces), shellClosed)});
        if (!brepBody.IsValid(GeometryTolerance3d{eps, eps, eps}))
        {
            return {false, BrepConversionIssue3d::InvalidBody, 0, {}};
        }

        return {true, BrepConversionIssue3d::None, 0, std::move(brepBody)};
    }
    catch (const std::exception&)
    {
        return {false, BrepConversionIssue3d::InvalidBody, 0, {}};
    }
    }
    catch (const std::exception&)
    {
        return {false, BrepConversionIssue3d::InvalidBody, 0, {}};
    }
}
} // namespace geometry::sdk
