#include "sdk/GeometryHealing.h"

#include <algorithm>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "sdk/GeometryMeshRepair.h"
#include "sdk/PlaneSurface.h"
#include "sdk/GeometryValidation.h"

namespace geometry::sdk
{
namespace
{
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
    const BrepValidation3d validation = Validate(body, tolerance);
    if (validation.valid && !NeedsBrepHealing(body, tolerance))
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
            if (!BuildHealedFace(body, shell.FaceAt(faceIndex), healedFace, tolerance.distanceEpsilon))
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
    return {true, HealingIssue3d::None, std::move(healedBody)};
}
} // namespace geometry::sdk
