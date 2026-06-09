#pragma once
#include "Brep/SCBrepBody.h"
#include "Brep/SCBrepFace.h"
#include "Brep/SCBrepLoop.h"
#include "Brep/SCBrepShell.h"
#include "Export/GeometryExport.h"

namespace Geometry
{
    enum class BrepLoopEditIssue3d
    {
        None,
        InvalidLoop,
        InvalidIndex,
        InvalidResult
    };

    enum class BrepTopologyEditIssue3d
    {
        None,
        InvalidFace,
        InvalidShell,
        InvalidBody,
        InvalidIndex,
        InvalidResult
    };

    struct GEOMETRY_API BrepLoopEdit3d
    {
        bool success{false};
        BrepLoopEditIssue3d issue{BrepLoopEditIssue3d::None};
        SCBrepLoop loop{};

        [[nodiscard]] bool IsValid() const
        {
            return !success || loop.IsValid();
        }
    };

    struct GEOMETRY_API BrepFaceEdit3d
    {
        bool success{false};
        BrepTopologyEditIssue3d issue{BrepTopologyEditIssue3d::None};
        SCBrepFace face{};

        [[nodiscard]] bool IsValid(const SCGeometryTolerance3d& tolerance = {}) const
        {
            return !success || face.IsValid(tolerance);
        }
    };

    struct GEOMETRY_API BrepShellEdit3d
    {
        bool success{false};
        BrepTopologyEditIssue3d issue{BrepTopologyEditIssue3d::None};
        SCBrepShell shell{};

        [[nodiscard]] bool IsValid(const SCGeometryTolerance3d& tolerance = {}) const
        {
            return !success || shell.IsValid(tolerance);
        }
    };

    struct GEOMETRY_API BrepBodyEdit3d
    {
        bool success{false};
        BrepTopologyEditIssue3d issue{BrepTopologyEditIssue3d::None};
        SCBrepBody body{};

        [[nodiscard]] bool IsValid(const SCGeometryTolerance3d& tolerance = {}) const
        {
            return !success || body.IsValid(tolerance);
        }
    };

    [[nodiscard]] GEOMETRY_API BrepLoopEdit3d InsertCoedge(const SCBrepLoop& loop,
                                                           std::size_t index,
                                                           const SCBrepCoedge& coedge);

    [[nodiscard]] GEOMETRY_API BrepLoopEdit3d RemoveCoedge(const SCBrepLoop& loop, std::size_t index);

    [[nodiscard]] GEOMETRY_API BrepLoopEdit3d FlipCoedgeDirection(const SCBrepLoop& loop, std::size_t index);

    [[nodiscard]] GEOMETRY_API BrepFaceEdit3d ReplaceOuterLoop(const SCBrepFace& face,
                                                               const SCBrepLoop& outerLoop,
                                                               SCCurveOnSurface outerTrim = {},
                                                               const SCGeometryTolerance3d& tolerance = {});

    [[nodiscard]] GEOMETRY_API BrepShellEdit3d ReplaceFace(const SCBrepShell& shell,
                                                           std::size_t faceIndex,
                                                           const SCBrepFace& face,
                                                           const SCGeometryTolerance3d& tolerance = {});

    [[nodiscard]] GEOMETRY_API BrepBodyEdit3d ReplaceShell(const SCBrepBody& body,
                                                           std::size_t shellIndex,
                                                           const SCBrepShell& shell,
                                                           const SCGeometryTolerance3d& tolerance = {});
}  // namespace Geometry


