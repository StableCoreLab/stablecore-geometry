#include "sdk/GeometryBrepEditing.h"

#include <vector>

namespace geometry::sdk
{
BrepLoopEdit3d InsertCoedge(const BrepLoop& loop, std::size_t index, const BrepCoedge& coedge)
{
    if (!loop.IsValid() || !coedge.IsValid())
    {
        return {false, BrepLoopEditIssue3d::InvalidLoop, {}};
    }

    if (index > loop.CoedgeCount())
    {
        return {false, BrepLoopEditIssue3d::InvalidIndex, {}};
    }

    std::vector<BrepCoedge> coedges = loop.Coedges();
    coedges.insert(coedges.begin() + static_cast<std::ptrdiff_t>(index), coedge);

    BrepLoop edited(std::move(coedges));
    if (!edited.IsValid())
    {
        return {false, BrepLoopEditIssue3d::InvalidResult, {}};
    }

    return {true, BrepLoopEditIssue3d::None, std::move(edited)};
}

BrepLoopEdit3d RemoveCoedge(const BrepLoop& loop, std::size_t index)
{
    if (!loop.IsValid())
    {
        return {false, BrepLoopEditIssue3d::InvalidLoop, {}};
    }

    if (index >= loop.CoedgeCount())
    {
        return {false, BrepLoopEditIssue3d::InvalidIndex, {}};
    }

    std::vector<BrepCoedge> coedges = loop.Coedges();
    coedges.erase(coedges.begin() + static_cast<std::ptrdiff_t>(index));

    BrepLoop edited(std::move(coedges));
    if (!edited.IsValid())
    {
        return {false, BrepLoopEditIssue3d::InvalidResult, {}};
    }

    return {true, BrepLoopEditIssue3d::None, std::move(edited)};
}

BrepLoopEdit3d FlipCoedgeDirection(const BrepLoop& loop, std::size_t index)
{
    if (!loop.IsValid())
    {
        return {false, BrepLoopEditIssue3d::InvalidLoop, {}};
    }

    if (index >= loop.CoedgeCount())
    {
        return {false, BrepLoopEditIssue3d::InvalidIndex, {}};
    }

    std::vector<BrepCoedge> coedges = loop.Coedges();
    const BrepCoedge target = coedges[index];
    coedges[index] = BrepCoedge(target.EdgeIndex(), !target.Reversed());

    BrepLoop edited(std::move(coedges));
    if (!edited.IsValid())
    {
        return {false, BrepLoopEditIssue3d::InvalidResult, {}};
    }

    return {true, BrepLoopEditIssue3d::None, std::move(edited)};
}

BrepFaceEdit3d ReplaceOuterLoop(
    const BrepFace& face,
    const BrepLoop& outerLoop,
    CurveOnSurface outerTrim,
    const GeometryTolerance3d& tolerance)
{
    if (!face.IsValid(tolerance) || !outerLoop.IsValid())
    {
        return {false, BrepTopologyEditIssue3d::InvalidFace, {}};
    }

    if (outerTrim.SupportSurface() == nullptr)
    {
        outerTrim = face.OuterTrim();
    }

    BrepFace edited(
        std::shared_ptr<Surface>(face.SupportSurface()->Clone().release()),
        outerLoop,
        std::vector<BrepLoop>(face.HoleLoops().begin(), face.HoleLoops().end()),
        std::move(outerTrim),
        face.HoleTrims());
    if (!edited.IsValid(tolerance))
    {
        return {false, BrepTopologyEditIssue3d::InvalidResult, {}};
    }

    return {true, BrepTopologyEditIssue3d::None, std::move(edited)};
}

BrepShellEdit3d ReplaceFace(
    const BrepShell& shell,
    std::size_t faceIndex,
    const BrepFace& face,
    const GeometryTolerance3d& tolerance)
{
    if (!shell.IsValid(tolerance) || !face.IsValid(tolerance))
    {
        return {false, BrepTopologyEditIssue3d::InvalidShell, {}};
    }

    if (faceIndex >= shell.FaceCount())
    {
        return {false, BrepTopologyEditIssue3d::InvalidIndex, {}};
    }

    std::vector<BrepFace> faces = shell.Faces();
    faces[faceIndex] = face;
    BrepShell edited(std::move(faces), shell.IsClosed());
    if (!edited.IsValid(tolerance))
    {
        return {false, BrepTopologyEditIssue3d::InvalidResult, {}};
    }

    return {true, BrepTopologyEditIssue3d::None, std::move(edited)};
}

BrepBodyEdit3d ReplaceShell(
    const BrepBody& body,
    std::size_t shellIndex,
    const BrepShell& shell,
    const GeometryTolerance3d& tolerance)
{
    if (!body.IsValid(tolerance) || !shell.IsValid(tolerance))
    {
        return {false, BrepTopologyEditIssue3d::InvalidBody, {}};
    }

    if (shellIndex >= body.ShellCount())
    {
        return {false, BrepTopologyEditIssue3d::InvalidIndex, {}};
    }

    std::vector<BrepShell> shells = body.Shells();
    shells[shellIndex] = shell;
    BrepBody edited(body.Vertices(), body.Edges(), std::move(shells));
    if (!edited.IsValid(tolerance))
    {
        return {false, BrepTopologyEditIssue3d::InvalidResult, {}};
    }

    return {true, BrepTopologyEditIssue3d::None, std::move(edited)};
}
} // namespace geometry::sdk
