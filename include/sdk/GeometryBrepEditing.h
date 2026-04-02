#pragma once

#include <cstddef>

#include "export/GeometryExport.h"
#include "sdk/BrepBody.h"
#include "sdk/BrepFace.h"
#include "sdk/BrepLoop.h"
#include "sdk/BrepShell.h"

namespace geometry::sdk
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
    BrepLoop loop{};

    [[nodiscard]] bool IsValid() const
    {
        return !success || loop.IsValid();
    }
};

struct GEOMETRY_API BrepFaceEdit3d
{
    bool success{false};
    BrepTopologyEditIssue3d issue{BrepTopologyEditIssue3d::None};
    BrepFace face{};

    [[nodiscard]] bool IsValid(const GeometryTolerance3d& tolerance = {}) const
    {
        return !success || face.IsValid(tolerance);
    }
};

struct GEOMETRY_API BrepShellEdit3d
{
    bool success{false};
    BrepTopologyEditIssue3d issue{BrepTopologyEditIssue3d::None};
    BrepShell shell{};

    [[nodiscard]] bool IsValid(const GeometryTolerance3d& tolerance = {}) const
    {
        return !success || shell.IsValid(tolerance);
    }
};

struct GEOMETRY_API BrepBodyEdit3d
{
    bool success{false};
    BrepTopologyEditIssue3d issue{BrepTopologyEditIssue3d::None};
    BrepBody body{};

    [[nodiscard]] bool IsValid(const GeometryTolerance3d& tolerance = {}) const
    {
        return !success || body.IsValid(tolerance);
    }
};

[[nodiscard]] GEOMETRY_API BrepLoopEdit3d InsertCoedge(
    const BrepLoop& loop,
    std::size_t index,
    const BrepCoedge& coedge);

[[nodiscard]] GEOMETRY_API BrepLoopEdit3d RemoveCoedge(
    const BrepLoop& loop,
    std::size_t index);

[[nodiscard]] GEOMETRY_API BrepLoopEdit3d FlipCoedgeDirection(
    const BrepLoop& loop,
    std::size_t index);

[[nodiscard]] GEOMETRY_API BrepFaceEdit3d ReplaceOuterLoop(
    const BrepFace& face,
    const BrepLoop& outerLoop,
    CurveOnSurface outerTrim = {},
    const GeometryTolerance3d& tolerance = {});

[[nodiscard]] GEOMETRY_API BrepShellEdit3d ReplaceFace(
    const BrepShell& shell,
    std::size_t faceIndex,
    const BrepFace& face,
    const GeometryTolerance3d& tolerance = {});

[[nodiscard]] GEOMETRY_API BrepBodyEdit3d ReplaceShell(
    const BrepBody& body,
    std::size_t shellIndex,
    const BrepShell& shell,
    const GeometryTolerance3d& tolerance = {});
} // namespace geometry::sdk
