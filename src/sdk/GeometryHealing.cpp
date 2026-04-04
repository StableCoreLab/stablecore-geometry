#include "sdk/GeometryHealing.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "sdk/GeometryMeshRepair.h"
#include "sdk/GeometryRelation.h"
#include "sdk/GeometryShapeOps.h"
#include "sdk/PlaneSurface.h"
#include "sdk/GeometryValidation.h"

namespace geometry::sdk
{
namespace
{
constexpr std::size_t kInvalidIndex = static_cast<std::size_t>(-1);

HealingIssue3d MapMeshRepairIssue(const MeshRepairIssue3d issue)
{
    switch (issue)
    {
    case MeshRepairIssue3d::None:
        return HealingIssue3d::None;
    case MeshRepairIssue3d::InvalidMesh:
        return HealingIssue3d::InvalidMesh;
    default:
        return HealingIssue3d::RepairFailed;
    }
}

namespace trim_backfill
{
[[nodiscard]] bool AppendLoopVertices(
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

[[nodiscard]] bool BuildTrimFromLoop(
    const BrepBody& body,
    const BrepLoop& loop,
    const PlaneSurface& planeSurface,
    CurveOnSurface& trim,
    double eps)
{
    std::vector<Point3d> loopVertices;
    if (!AppendLoopVertices(body, loop, loopVertices, eps))
    {
        return false;
    }

    std::vector<Point2d> uvPoints;
    uvPoints.reserve(loopVertices.size());
    for (const Point3d& point : loopVertices)
    {
        uvPoints.push_back(ProjectPointToPlaneUv(point, planeSurface));
    }

    trim = CurveOnSurface(
        std::shared_ptr<Surface>(planeSurface.Clone().release()),
        Polyline2d(std::move(uvPoints), PolylineClosure::Closed));
    return trim.IsValid(GeometryTolerance3d{eps, eps, eps});
}

[[nodiscard]] bool BuildHealedFace(
    const BrepBody& body,
    const BrepFace& face,
    BrepFace& healedFace,
    double eps)
{
    const auto* planeSurface = dynamic_cast<const PlaneSurface*>(face.SupportSurface());
    if (planeSurface == nullptr)
    {
        return false;
    }

    CurveOnSurface outerTrim = face.OuterTrim();
    if (!outerTrim.IsValid(GeometryTolerance3d{eps, eps, eps}))
    {
        if (!BuildTrimFromLoop(body, face.OuterLoop(), *planeSurface, outerTrim, eps))
        {
            return false;
        }
    }

    std::vector<CurveOnSurface> holeTrims = face.HoleTrims();
    if (holeTrims.size() != face.HoleCount())
    {
        holeTrims.clear();
    }
    if (holeTrims.size() < face.HoleCount())
    {
        holeTrims.resize(face.HoleCount());
    }

    for (std::size_t i = 0; i < face.HoleCount(); ++i)
    {
        if (holeTrims[i].IsValid(GeometryTolerance3d{eps, eps, eps}))
        {
            continue;
        }

        if (!BuildTrimFromLoop(body, face.HoleAt(i), *planeSurface, holeTrims[i], eps))
        {
            return false;
        }
    }

    healedFace = BrepFace(
        std::shared_ptr<Surface>(face.SupportSurface()->Clone().release()),
        face.OuterLoop(),
        std::vector<BrepLoop>(face.HoleLoops().begin(), face.HoleLoops().end()),
        std::move(outerTrim),
        std::move(holeTrims));
    return healedFace.IsValid(GeometryTolerance3d{eps, eps, eps});
}

} // namespace trim_backfill

[[nodiscard]] bool ComputeShellClosed(
    const BrepShell& shell)
{
    std::map<std::size_t, std::size_t> edgeUseCount;
    for (const BrepFace& face : shell.Faces())
    {
        for (const BrepCoedge& coedge : face.OuterLoop().Coedges())
        {
            ++edgeUseCount[coedge.EdgeIndex()];
        }
        for (const BrepLoop& hole : face.HoleLoops())
        {
            for (const BrepCoedge& coedge : hole.Coedges())
            {
                ++edgeUseCount[coedge.EdgeIndex()];
            }
        }
    }

    if (edgeUseCount.empty())
    {
        return false;
    }

    for (const auto& [_, count] : edgeUseCount)
    {
        if (count != 2)
        {
            return false;
        }
    }

return true;
}

namespace aggressive
{
[[nodiscard]] BrepLoop ReversedLoop(const BrepLoop& loop);
}

namespace shell_cap
{
struct BoundaryHalfEdge
{
    BrepCoedge coedge{};
    std::size_t startVertexIndex{kInvalidIndex};
    std::size_t endVertexIndex{kInvalidIndex};
};

struct BoundaryLoopInfo
{
    BrepLoop loop{};
    Polyline2d normalizedUvRing{};
    Point2d samplePoint{};
    double area{0.0};
};

void AccumulateLoopEdgeUseCounts(
    const BrepLoop& loop,
    std::map<std::size_t, std::size_t>& edgeUseCount)
{
    for (const BrepCoedge& coedge : loop.Coedges())
    {
        ++edgeUseCount[coedge.EdgeIndex()];
    }
}

void AccumulateShellEdgeUseCounts(
    const BrepShell& shell,
    std::map<std::size_t, std::size_t>& edgeUseCount)
{
    for (const BrepFace& face : shell.Faces())
    {
        AccumulateLoopEdgeUseCounts(face.OuterLoop(), edgeUseCount);
        for (const BrepLoop& hole : face.HoleLoops())
        {
            AccumulateLoopEdgeUseCounts(hole, edgeUseCount);
        }
    }
}

[[nodiscard]] bool TryGetCoedgeVertexIndices(
    const BrepBody& body,
    const BrepCoedge& coedge,
    std::size_t& startVertexIndex,
    std::size_t& endVertexIndex)
{
    if (coedge.EdgeIndex() >= body.EdgeCount())
    {
        return false;
    }

    const BrepEdge edge = body.EdgeAt(coedge.EdgeIndex());
    startVertexIndex = coedge.Reversed() ? edge.EndVertexIndex() : edge.StartVertexIndex();
    endVertexIndex = coedge.Reversed() ? edge.StartVertexIndex() : edge.EndVertexIndex();
    return startVertexIndex < body.VertexCount() && endVertexIndex < body.VertexCount();
}

void AppendUniqueLoopVertexIndices(
    const BrepBody& body,
    const BrepLoop& loop,
    std::vector<std::size_t>& vertexIndices)
{
    for (const BrepCoedge& coedge : loop.Coedges())
    {
        std::size_t startVertexIndex = kInvalidIndex;
        std::size_t endVertexIndex = kInvalidIndex;
        if (!TryGetCoedgeVertexIndices(body, coedge, startVertexIndex, endVertexIndex))
        {
            continue;
        }

        if (std::find(vertexIndices.begin(), vertexIndices.end(), startVertexIndex) == vertexIndices.end())
        {
            vertexIndices.push_back(startVertexIndex);
        }
        if (std::find(vertexIndices.begin(), vertexIndices.end(), endVertexIndex) == vertexIndices.end())
        {
            vertexIndices.push_back(endVertexIndex);
        }
    }
}

[[nodiscard]] bool TryBuildStandaloneShellPlaneSurface(
    const BrepBody& body,
    const BrepShell& shell,
    const GeometryTolerance3d& tolerance,
    PlaneSurface& planeSurface)
{
    const double eps = std::max(tolerance.distanceEpsilon, geometry::kDefaultEpsilon);
    std::vector<std::size_t> vertexIndices;
    vertexIndices.reserve(shell.FaceCount() * 8);

    Vector3d referenceNormal{};
    bool hasReferenceNormal = false;
    for (const BrepFace& face : shell.Faces())
    {
        const auto* facePlaneSurface = dynamic_cast<const PlaneSurface*>(face.SupportSurface());
        if (facePlaneSurface == nullptr)
        {
            return false;
        }

        const Vector3d faceNormal = facePlaneSurface->SupportPlane().UnitNormal(eps);
        if (!hasReferenceNormal)
        {
            referenceNormal = faceNormal;
            hasReferenceNormal = true;
        }
        else if (!IsParallel(referenceNormal, faceNormal, tolerance))
        {
            return false;
        }

        AppendUniqueLoopVertexIndices(body, face.OuterLoop(), vertexIndices);
        for (const BrepLoop& hole : face.HoleLoops())
        {
            AppendUniqueLoopVertexIndices(body, hole, vertexIndices);
        }
    }

    if (!hasReferenceNormal || vertexIndices.size() < 3)
    {
        return false;
    }

    Point3d origin{};
    Point3d firstPoint{};
    Point3d secondPoint{};
    bool foundPlane = false;
    for (std::size_t i = 0; i < vertexIndices.size() && !foundPlane; ++i)
    {
        origin = body.VertexAt(vertexIndices[i]).Point();
        for (std::size_t j = i + 1; j < vertexIndices.size() && !foundPlane; ++j)
        {
            firstPoint = body.VertexAt(vertexIndices[j]).Point();
            if (origin.AlmostEquals(firstPoint, eps))
            {
                continue;
            }

            for (std::size_t k = j + 1; k < vertexIndices.size(); ++k)
            {
                secondPoint = body.VertexAt(vertexIndices[k]).Point();
                const Vector3d computedNormal = Cross(firstPoint - origin, secondPoint - origin);
                if (computedNormal.Length() <= eps)
                {
                    continue;
                }

                Vector3d alignedNormal = computedNormal;
                if (Dot(alignedNormal, referenceNormal) < 0.0)
                {
                    alignedNormal = alignedNormal * -1.0;
                }

                const Plane shellPlane = Plane::FromPointAndNormal(origin, alignedNormal);
                const double planeTolerance = std::max(tolerance.distanceEpsilon, 16.0 * eps);
                bool allCoplanar = true;
                for (const std::size_t vertexIndex : vertexIndices)
                {
                    if (std::abs(shellPlane.SignedDistanceTo(body.VertexAt(vertexIndex).Point(), eps)) > planeTolerance)
                    {
                        allCoplanar = false;
                        break;
                    }
                }

                if (!allCoplanar)
                {
                    continue;
                }

                planeSurface = PlaneSurface::FromPlane(shellPlane);
                foundPlane = planeSurface.IsValid(tolerance);
                break;
            }
        }
    }

    return foundPlane;
}

[[nodiscard]] Polyline2d NormalizeRingOrientationForShellCap(
    const Polyline2d& ring,
    bool counterClockwise)
{
    const RingOrientation2d orientation = Orientation(ring);
    if (orientation == RingOrientation2d::Unknown)
    {
        return {};
    }

    if ((counterClockwise && orientation == RingOrientation2d::Clockwise) ||
        (!counterClockwise && orientation == RingOrientation2d::CounterClockwise))
    {
        return Reverse(ring);
    }

    return ring;
}

[[nodiscard]] bool BuildBoundaryLoopInfos(
    const BrepBody& body,
    const BrepShell& shell,
    const std::map<std::size_t, std::size_t>& edgeUseCount,
    const PlaneSurface& planeSurface,
    double eps,
    std::vector<BoundaryLoopInfo>& boundaryLoops)
{
    boundaryLoops.clear();

    std::vector<BoundaryHalfEdge> halfEdges;
    auto collectBoundaryFromLoop = [&](const BrepLoop& loop) {
        for (const BrepCoedge& coedge : loop.Coedges())
        {
            const auto countIt = edgeUseCount.find(coedge.EdgeIndex());
            if (countIt == edgeUseCount.end() || countIt->second != 1U)
            {
                continue;
            }

            std::size_t startVertexIndex = kInvalidIndex;
            std::size_t endVertexIndex = kInvalidIndex;
            if (!TryGetCoedgeVertexIndices(body, coedge, startVertexIndex, endVertexIndex))
            {
                return false;
            }

            halfEdges.push_back(BoundaryHalfEdge{coedge, startVertexIndex, endVertexIndex});
        }

        return true;
    };

    for (const BrepFace& face : shell.Faces())
    {
        if (!collectBoundaryFromLoop(face.OuterLoop()))
        {
            return false;
        }
        for (const BrepLoop& hole : face.HoleLoops())
        {
            if (!collectBoundaryFromLoop(hole))
            {
                return false;
            }
        }
    }

    if (halfEdges.empty())
    {
        return false;
    }

    std::map<std::size_t, std::vector<std::size_t>> outgoing;
    std::map<std::size_t, std::vector<std::size_t>> incoming;
    for (std::size_t index = 0; index < halfEdges.size(); ++index)
    {
        outgoing[halfEdges[index].startVertexIndex].push_back(index);
        incoming[halfEdges[index].endVertexIndex].push_back(index);
    }

    for (const auto& [vertexIndex, outgoingEdges] : outgoing)
    {
        const auto incomingIt = incoming.find(vertexIndex);
        if (incomingIt == incoming.end() || outgoingEdges.size() != 1U || incomingIt->second.size() != 1U)
        {
            return false;
        }
    }

    std::vector<bool> visited(halfEdges.size(), false);
    for (std::size_t halfEdgeIndex = 0; halfEdgeIndex < halfEdges.size(); ++halfEdgeIndex)
    {
        if (visited[halfEdgeIndex])
        {
            continue;
        }

        std::vector<BrepCoedge> loopCoedges;
        std::vector<Point3d> loopVertices;
        const std::size_t startVertexIndex = halfEdges[halfEdgeIndex].startVertexIndex;
        std::size_t currentIndex = halfEdgeIndex;
        while (true)
        {
            if (visited[currentIndex])
            {
                return false;
            }

            const BoundaryHalfEdge& halfEdge = halfEdges[currentIndex];
            visited[currentIndex] = true;
            loopCoedges.push_back(halfEdge.coedge);
            loopVertices.push_back(body.VertexAt(halfEdge.startVertexIndex).Point());

            if (halfEdge.endVertexIndex == startVertexIndex)
            {
                break;
            }

            const auto nextIt = outgoing.find(halfEdge.endVertexIndex);
            if (nextIt == outgoing.end() || nextIt->second.size() != 1U)
            {
                return false;
            }

            currentIndex = nextIt->second.front();
        }

        if (loopCoedges.size() < 3U || loopVertices.size() < 3U)
        {
            return false;
        }

        std::vector<Point2d> uvPoints;
        uvPoints.reserve(loopVertices.size());
        for (const Point3d& point : loopVertices)
        {
            uvPoints.push_back(trim_backfill::ProjectPointToPlaneUv(point, planeSurface));
        }

        const Polyline2d uvRing(std::move(uvPoints), PolylineClosure::Closed);
        const Polyline2d normalizedUvRing = NormalizeRingOrientationForShellCap(uvRing, true);
        if (!normalizedUvRing.IsValid())
        {
            return false;
        }

        const Polygon2d projectedPolygon(normalizedUvRing);
        if (!projectedPolygon.IsValid())
        {
            return false;
        }

        const Point2d samplePoint = Centroid(projectedPolygon);
        if (!samplePoint.IsValid() || LocatePoint(samplePoint, projectedPolygon, eps) == PointContainment2d::Outside)
        {
            return false;
        }

        boundaryLoops.push_back(BoundaryLoopInfo{
            BrepLoop(std::move(loopCoedges)),
            normalizedUvRing,
            samplePoint,
            Area(projectedPolygon)});
    }

    return !boundaryLoops.empty();
}

[[nodiscard]] bool BuildBoundaryCapFaces(
    const BrepBody& body,
    const std::vector<BoundaryLoopInfo>& boundaryLoops,
    const PlaneSurface& planeSurface,
    const GeometryTolerance3d& tolerance,
    std::vector<BrepFace>& capFaces)
{
    capFaces.clear();
    if (boundaryLoops.empty())
    {
        return false;
    }

    std::vector<Polygon2d> projectedPolygons;
    projectedPolygons.reserve(boundaryLoops.size());
    for (const BoundaryLoopInfo& loopInfo : boundaryLoops)
    {
        const Polygon2d polygon(loopInfo.normalizedUvRing);
        if (!polygon.IsValid())
        {
            return false;
        }
        projectedPolygons.push_back(polygon);
    }

    std::vector<std::size_t> parents(boundaryLoops.size(), kInvalidIndex);
    std::vector<std::size_t> depths(boundaryLoops.size(), 0U);
    for (std::size_t loopIndex = 0; loopIndex < boundaryLoops.size(); ++loopIndex)
    {
        std::size_t parentIndex = kInvalidIndex;
        double parentArea = 0.0;
        for (std::size_t candidateIndex = 0; candidateIndex < boundaryLoops.size(); ++candidateIndex)
        {
            if (candidateIndex == loopIndex || boundaryLoops[candidateIndex].area <= boundaryLoops[loopIndex].area)
            {
                continue;
            }

            const PointContainment2d containment =
                LocatePoint(boundaryLoops[loopIndex].samplePoint, projectedPolygons[candidateIndex], tolerance.distanceEpsilon);
            if (containment == PointContainment2d::Outside)
            {
                continue;
            }

            if (parentIndex == kInvalidIndex || boundaryLoops[candidateIndex].area < parentArea)
            {
                parentIndex = candidateIndex;
                parentArea = boundaryLoops[candidateIndex].area;
            }
        }

        parents[loopIndex] = parentIndex;
    }

    for (std::size_t loopIndex = 0; loopIndex < boundaryLoops.size(); ++loopIndex)
    {
        std::size_t depth = 0;
        std::size_t parentIndex = parents[loopIndex];
        while (parentIndex != kInvalidIndex)
        {
            ++depth;
            parentIndex = parents[parentIndex];
        }
        depths[loopIndex] = depth;
    }

    for (std::size_t loopIndex = 0; loopIndex < boundaryLoops.size(); ++loopIndex)
    {
        if (depths[loopIndex] % 2U != 0U)
        {
            continue;
        }

        const BrepLoop outerLoop = aggressive::ReversedLoop(boundaryLoops[loopIndex].loop);
        CurveOnSurface outerTrim{};
        if (!trim_backfill::BuildTrimFromLoop(body, outerLoop, planeSurface, outerTrim, tolerance.distanceEpsilon))
        {
            return false;
        }

        std::vector<BrepLoop> holeLoops;
        std::vector<CurveOnSurface> holeTrims;
        for (std::size_t childIndex = 0; childIndex < boundaryLoops.size(); ++childIndex)
        {
            if (parents[childIndex] != loopIndex || depths[childIndex] != depths[loopIndex] + 1U)
            {
                continue;
            }

            const BrepLoop holeLoop = aggressive::ReversedLoop(boundaryLoops[childIndex].loop);
            CurveOnSurface holeTrim{};
            if (!trim_backfill::BuildTrimFromLoop(body, holeLoop, planeSurface, holeTrim, tolerance.distanceEpsilon))
            {
                return false;
            }

            holeLoops.push_back(holeLoop);
            holeTrims.push_back(std::move(holeTrim));
        }

        BrepFace capFace(
            std::shared_ptr<Surface>(planeSurface.Clone().release()),
            outerLoop,
            std::move(holeLoops),
            std::move(outerTrim),
            std::move(holeTrims));
        if (!capFace.IsValid(tolerance))
        {
            return false;
        }

        capFaces.push_back(std::move(capFace));
    }

    return !capFaces.empty();
}

[[nodiscard]] bool TryCloseStandaloneShellWithBoundaryCaps(
    const BrepBody& body,
    const BrepShell& shell,
    const GeometryTolerance3d& tolerance,
    const std::map<std::size_t, std::size_t>& edgeUseCount,
    BrepShell& repairedShell)
{
    PlaneSurface shellPlaneSurface{};
    if (!TryBuildStandaloneShellPlaneSurface(body, shell, tolerance, shellPlaneSurface))
    {
        return false;
    }

    std::vector<BoundaryLoopInfo> boundaryLoops;
    if (!BuildBoundaryLoopInfos(body, shell, edgeUseCount, shellPlaneSurface, tolerance.distanceEpsilon, boundaryLoops))
    {
        return false;
    }

    std::vector<BrepFace> capFaces;
    if (!BuildBoundaryCapFaces(body, boundaryLoops, shellPlaneSurface, tolerance, capFaces))
    {
        return false;
    }

    std::vector<BrepFace> closedFaces = shell.Faces();
    closedFaces.insert(closedFaces.end(), capFaces.begin(), capFaces.end());
    repairedShell = BrepShell(std::move(closedFaces), true);
    return repairedShell.IsValid(tolerance);
}

} // namespace shell_cap

namespace aggressive
{

[[nodiscard]] bool NeedsBrepHealing(const BrepBody& body, const GeometryTolerance3d& tolerance)
{
    for (std::size_t shellIndex = 0; shellIndex < body.ShellCount(); ++shellIndex)
    {
        const BrepShell shell = body.ShellAt(shellIndex);
        if (!shell.IsClosed())
        {
            return true;
        }

        for (std::size_t faceIndex = 0; faceIndex < shell.FaceCount(); ++faceIndex)
        {
            const BrepFace face = shell.FaceAt(faceIndex);
            if (!face.OuterTrim().IsValid(tolerance) || face.HoleTrims().size() != face.HoleCount())
            {
                return true;
            }

            for (const CurveOnSurface& trim : face.HoleTrims())
            {
                if (!trim.IsValid(tolerance))
                {
                    return true;
                }
            }
        }
    }

    return false;
}

[[nodiscard]] BrepLoop ReversedLoop(const BrepLoop& loop)
{
    std::vector<BrepCoedge> reversed;
    reversed.reserve(loop.CoedgeCount());
    for (std::size_t i = 0; i < loop.CoedgeCount(); ++i)
    {
        const BrepCoedge coedge = loop.CoedgeAt(loop.CoedgeCount() - 1 - i);
        reversed.emplace_back(coedge.EdgeIndex(), !coedge.Reversed());
    }
    return BrepLoop(std::move(reversed));
}

[[nodiscard]] std::vector<BrepLoop> ReversedLoops(const std::vector<BrepLoop>& loops)
{
    std::vector<BrepLoop> reversed;
    reversed.reserve(loops.size());
    for (const BrepLoop& loop : loops)
    {
        reversed.push_back(ReversedLoop(loop));
    }
    return reversed;
}

[[nodiscard]] bool TryAggressivelyCloseShells(
    const BrepBody& body,
    const GeometryTolerance3d& tolerance,
    BrepBody& repairedBody)
{
    std::vector<BrepShell> repairedShells;
    repairedShells.reserve(body.ShellCount());
    bool changed = false;

    for (std::size_t shellIndex = 0; shellIndex < body.ShellCount(); ++shellIndex)
    {
        const BrepShell shell = body.ShellAt(shellIndex);
        if (shell.IsClosed())
        {
            repairedShells.push_back(shell);
            continue;
        }

        std::map<std::size_t, std::size_t> edgeUseCount;
        bool eligible = true;
        bool hasBoundaryEdge = false;
        bool hasInteriorSharedEdge = false;
        for (std::size_t faceIndex = 0; faceIndex < shell.FaceCount() && eligible; ++faceIndex)
        {
            const BrepFace face = shell.FaceAt(faceIndex);
            if (dynamic_cast<const PlaneSurface*>(face.SupportSurface()) == nullptr)
            {
                eligible = false;
                break;
            }
        }

        shell_cap::AccumulateShellEdgeUseCounts(shell, edgeUseCount);

        if (!eligible || edgeUseCount.empty())
        {
            repairedShells.push_back(shell);
            continue;
        }

        for (const auto& [_, count] : edgeUseCount)
        {
            if (count > 2 || count == 0)
            {
                eligible = false;
                break;
            }

            if (count == 1)
            {
                hasBoundaryEdge = true;
            }
            else if (count == 2)
            {
                hasInteriorSharedEdge = true;
            }
        }

        if (!hasBoundaryEdge)
        {
            eligible = false;
        }

        if (!eligible)
        {
            repairedShells.push_back(shell);
            continue;
        }

        if (hasInteriorSharedEdge)
        {
            BrepShell cappedShell{};
            if (body.ShellCount() == 1 &&
                shell_cap::TryCloseStandaloneShellWithBoundaryCaps(body, shell, tolerance, edgeUseCount, cappedShell))
            {
                repairedShells.push_back(std::move(cappedShell));
                changed = true;
                continue;
            }

            repairedShells.push_back(shell);
            continue;
        }

        std::vector<BrepFace> closedFaces;
        closedFaces.reserve(shell.FaceCount() * 2);
        for (std::size_t faceIndex = 0; faceIndex < shell.FaceCount(); ++faceIndex)
        {
            const BrepFace frontFace = shell.FaceAt(faceIndex);
            closedFaces.push_back(frontFace);

            const BrepLoop reversedOuter = ReversedLoop(frontFace.OuterLoop());
            const std::vector<BrepLoop> reversedHoles = ReversedLoops(frontFace.HoleLoops());
            BrepFace backFace(
                std::shared_ptr<Surface>(frontFace.SupportSurface()->Clone().release()),
                reversedOuter,
                reversedHoles,
                frontFace.OuterTrim(),
                frontFace.HoleTrims());
            if (!backFace.IsValid(tolerance))
            {
                eligible = false;
                break;
            }

            closedFaces.push_back(std::move(backFace));
        }

        if (!eligible)
        {
            repairedShells.push_back(shell);
            continue;
        }

        repairedShells.emplace_back(std::move(closedFaces), true);
        changed = true;
    }

    if (!changed)
    {
        return false;
    }

    repairedBody = BrepBody(body.Vertices(), body.Edges(), std::move(repairedShells));
    return repairedBody.IsValid(tolerance);
}
} // namespace aggressive
} // namespace

MeshHealing3d Heal(const TriangleMesh& mesh, double eps)
{
    const MeshValidation3d validation = Validate(mesh, eps);
    if (!validation.valid)
    {
        return {false, HealingIssue3d::InvalidMesh, {}};
    }

    const TriangleMeshRepair3d closed = ClosePlanarBoundaryLoops(mesh, eps);
    if (closed.success)
    {
        return {true, HealingIssue3d::None, std::move(closed.mesh)};
    }

    const TriangleMeshRepair3d oriented = OrientTriangleMeshConsistently(mesh, eps);
    if (oriented.success)
    {
        return {true, HealingIssue3d::None, std::move(oriented.mesh)};
    }

    return {false, MapMeshRepairIssue(oriented.issue), {}};
}

PolyhedronHealing3d Heal(const PolyhedronBody& body, double eps)
{
    const PolyhedronValidation3d validation = Validate(body, eps);
    if (!validation.valid)
    {
        return {false, HealingIssue3d::InvalidPolyhedron, {}};
    }

    return {true, HealingIssue3d::None, body};
}

BrepHealing3d Heal(const BrepBody& body, const GeometryTolerance3d& tolerance)
{
    return Heal(body, tolerance, HealingPolicy3d::Conservative);
}

BrepHealing3d Heal(
    const BrepBody& body,
    const GeometryTolerance3d& tolerance,
    HealingPolicy3d policy)
{
    const BrepValidation3d validation = Validate(body, tolerance);
    if (validation.valid && !aggressive::NeedsBrepHealing(body, tolerance))
    {
        return {true, HealingIssue3d::None, body};
    }

    if (validation.issue == BrepValidationIssue3d::EmptyBody)
    {
        return {false, HealingIssue3d::InvalidBrep, {}};
    }

    std::vector<BrepShell> healedShells;
    healedShells.reserve(body.ShellCount());
    for (std::size_t shellIndex = 0; shellIndex < body.ShellCount(); ++shellIndex)
    {
        const BrepShell shell = body.ShellAt(shellIndex);
        std::vector<BrepFace> healedFaces;
        healedFaces.reserve(shell.FaceCount());
        for (std::size_t faceIndex = 0; faceIndex < shell.FaceCount(); ++faceIndex)
        {
            BrepFace healedFace{};
            if (!trim_backfill::BuildHealedFace(body, shell.FaceAt(faceIndex), healedFace, tolerance.distanceEpsilon))
            {
                return {false, HealingIssue3d::RepairFailed, {}};
            }
            healedFaces.push_back(std::move(healedFace));
        }

        BrepShell healedShell(std::move(healedFaces), ComputeShellClosed(shell));
        if (!healedShell.IsValid(tolerance))
        {
            return {false, HealingIssue3d::RepairFailed, {}};
        }
        healedShells.push_back(std::move(healedShell));
    }

    BrepBody healedBody(body.Vertices(), body.Edges(), std::move(healedShells));
    if (!healedBody.IsValid(tolerance))
    {
        return {false, HealingIssue3d::RepairFailed, {}};
    }

    if (policy == HealingPolicy3d::Aggressive)
    {
        BrepBody aggressivelyHealedBody{};
        if (aggressive::TryAggressivelyCloseShells(healedBody, tolerance, aggressivelyHealedBody))
        {
            return {true, HealingIssue3d::None, std::move(aggressivelyHealedBody)};
        }
    }

    return {true, HealingIssue3d::None, std::move(healedBody)};
}
} // namespace geometry::sdk
