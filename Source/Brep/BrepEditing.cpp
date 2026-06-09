#include "Brep/BrepEditing.h"

#include <vector>

namespace Geometry
{
    BrepLoopEdit3d InsertCoedge(const SCBrepLoop& loop, std::size_t index, const SCBrepCoedge& coedge)
    {
        if (!loop.IsValid() || !coedge.IsValid())
        {
            return {false, BrepLoopEditIssue3d::InvalidLoop, {}};
        }

        if (index > loop.CoedgeCount())
        {
            return {false, BrepLoopEditIssue3d::InvalidIndex, {}};
        }

        std::vector<SCBrepCoedge> coedges = loop.Coedges();
        coedges.insert(coedges.begin() + static_cast<std::ptrdiff_t>(index), coedge);

        SCBrepLoop edited(std::move(coedges));
        if (!edited.IsValid())
        {
            return {false, BrepLoopEditIssue3d::InvalidResult, {}};
        }

        return {true, BrepLoopEditIssue3d::None, std::move(edited)};
    }

    BrepLoopEdit3d RemoveCoedge(const SCBrepLoop& loop, std::size_t index)
    {
        if (!loop.IsValid())
        {
            return {false, BrepLoopEditIssue3d::InvalidLoop, {}};
        }

        if (index >= loop.CoedgeCount())
        {
            return {false, BrepLoopEditIssue3d::InvalidIndex, {}};
        }

        std::vector<SCBrepCoedge> coedges = loop.Coedges();
        coedges.erase(coedges.begin() + static_cast<std::ptrdiff_t>(index));

        SCBrepLoop edited(std::move(coedges));
        if (!edited.IsValid())
        {
            return {false, BrepLoopEditIssue3d::InvalidResult, {}};
        }

        return {true, BrepLoopEditIssue3d::None, std::move(edited)};
    }

    BrepLoopEdit3d FlipCoedgeDirection(const SCBrepLoop& loop, std::size_t index)
    {
        if (!loop.IsValid())
        {
            return {false, BrepLoopEditIssue3d::InvalidLoop, {}};
        }

        if (index >= loop.CoedgeCount())
        {
            return {false, BrepLoopEditIssue3d::InvalidIndex, {}};
        }

        std::vector<SCBrepCoedge> coedges = loop.Coedges();
        const SCBrepCoedge target = coedges[index];
        coedges[index] = SCBrepCoedge(target.EdgeIndex(), !target.Reversed());

        SCBrepLoop edited(std::move(coedges));
        if (!edited.IsValid())
        {
            return {false, BrepLoopEditIssue3d::InvalidResult, {}};
        }

        return {true, BrepLoopEditIssue3d::None, std::move(edited)};
    }

    BrepFaceEdit3d ReplaceOuterLoop(const SCBrepFace& face,
                                    const SCBrepLoop& outerLoop,
                                    SCCurveOnSurface outerTrim,
                                    const SCGeometryTolerance3d& tolerance)
    {
        if (!face.IsValid(tolerance) || !outerLoop.IsValid())
        {
            return {false, BrepTopologyEditIssue3d::InvalidFace, {}};
        }

        if (outerTrim.SupportSurface() == nullptr)
        {
            outerTrim = face.OuterTrim();
        }

        SCBrepFace edited(std::shared_ptr<ISCSurface>(face.SupportSurface()->Clone().release()),
                        outerLoop,
                        std::vector<SCBrepLoop>(face.HoleLoops().begin(), face.HoleLoops().end()),
                        std::move(outerTrim),
                        face.HoleTrims());
        if (!edited.IsValid(tolerance))
        {
            return {false, BrepTopologyEditIssue3d::InvalidResult, {}};
        }

        return {true, BrepTopologyEditIssue3d::None, std::move(edited)};
    }

    BrepShellEdit3d ReplaceFace(const SCBrepShell& shell,
                                std::size_t faceIndex,
                                const SCBrepFace& face,
                                const SCGeometryTolerance3d& tolerance)
    {
        if (!shell.IsValid(tolerance) || !face.IsValid(tolerance))
        {
            return {false, BrepTopologyEditIssue3d::InvalidShell, {}};
        }

        if (faceIndex >= shell.FaceCount())
        {
            return {false, BrepTopologyEditIssue3d::InvalidIndex, {}};
        }

        std::vector<SCBrepFace> faces = shell.Faces();
        faces[faceIndex] = face;
        SCBrepShell edited(std::move(faces), shell.IsClosed());
        if (!edited.IsValid(tolerance))
        {
            return {false, BrepTopologyEditIssue3d::InvalidResult, {}};
        }

        return {true, BrepTopologyEditIssue3d::None, std::move(edited)};
    }

    BrepBodyEdit3d ReplaceShell(const SCBrepBody& body,
                                std::size_t shellIndex,
                                const SCBrepShell& shell,
                                const SCGeometryTolerance3d& tolerance)
    {
        if (!body.IsValid(tolerance) || !shell.IsValid(tolerance))
        {
            return {false, BrepTopologyEditIssue3d::InvalidBody, {}};
        }

        if (shellIndex >= body.ShellCount())
        {
            return {false, BrepTopologyEditIssue3d::InvalidIndex, {}};
        }

        std::vector<SCBrepShell> shells = body.Shells();
        shells[shellIndex] = shell;
        SCBrepBody edited(body.Vertices(), body.Edges(), std::move(shells));
        if (!edited.IsValid(tolerance))
        {
            return {false, BrepTopologyEditIssue3d::InvalidResult, {}};
        }

        return {true, BrepTopologyEditIssue3d::None, std::move(edited)};
    }
}  // namespace Geometry


