#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "Brep/SCBrepBody.h"
#include "Brep/SCBrepEdge.h"
#include "Brep/SCBrepLoop.h"
#include "../Detail/AffinePlaneGeometry.h"
#include "Geometry3d/SCPlaneSurface.h"
#include "Support/Epsilon.h"

namespace Geometry::Detail
{
    enum class PlaneUvPurpose { VerifyOriginalVertex, ApproximateRepair };

    [[nodiscard]] inline bool TryGetCoedgeVertexIndices(const std::vector<SCBrepEdge>& edges,
                                                        std::size_t vertexCount,
                                                        const SCBrepCoedge& coedge,
                                                        std::size_t& startVertexIndex,
                                                        std::size_t& endVertexIndex)
    {
        if (coedge.EdgeIndex() >= edges.size())
        {
            return false;
        }
        const SCBrepEdge& edge = edges[coedge.EdgeIndex()];
        startVertexIndex = coedge.Reversed() ? edge.EndVertexIndex() : edge.StartVertexIndex();
        endVertexIndex = coedge.Reversed() ? edge.StartVertexIndex() : edge.EndVertexIndex();
        return startVertexIndex < vertexCount && endVertexIndex < vertexCount;
    }

    [[nodiscard]] inline bool AppendRawLoopVertices(const std::vector<SCBrepVertex>& vertices,
                                                    const std::vector<SCBrepEdge>& edges,
                                                    const SCBrepLoop& loop,
                                                    std::vector<SCPoint3d>& loopVertices)
    {
        loopVertices.clear();
        if (!loop.IsValid())
        {
            return false;
        }
        loopVertices.reserve(loop.CoedgeCount());
        for (const SCBrepCoedge& coedge : loop.Coedges())
        {
            std::size_t startVertexIndex = 0;
            std::size_t endVertexIndex = 0;
            if (!TryGetCoedgeVertexIndices(edges, vertices.size(), coedge,
                                           startVertexIndex, endVertexIndex))
            {
                return false;
            }
            loopVertices.push_back(vertices[startVertexIndex].Point());
        }
        return loopVertices.size() >= 3;
    }

    [[nodiscard]] inline bool AppendLoopVertices(const SCBrepBody& body,
                                                 const SCBrepLoop& loop,
                                                 std::vector<SCPoint3d>& vertices,
                                                 double epsilon)
    {
        vertices.clear();
        if (!loop.IsValid())
        {
            return false;
        }

        vertices.reserve(loop.CoedgeCount());
        for (std::size_t i = 0; i < loop.CoedgeCount(); ++i)
        {
            const SCBrepCoedge& coedge = loop.CoedgeAtRef(i);
            std::size_t vertexIndex = 0;
            std::size_t endVertexIndex = 0;
            if (!TryGetCoedgeVertexIndices(body.Edges(), body.VertexCount(), coedge,
                                           vertexIndex, endVertexIndex))
            {
                return false;
            }

            const SCPoint3d point = body.VertexAtRef(vertexIndex).Point();
            if (vertices.empty() || !vertices.back().AlmostEquals(point, epsilon))
            {
                vertices.push_back(point);
            }
        }

        while (vertices.size() >= 2 && vertices.front().AlmostEquals(vertices.back(), epsilon))
        {
            vertices.pop_back();
        }

        return vertices.size() >= 3;
    }

    [[nodiscard]] inline bool ProjectPointToPlaneUv(const SCPoint3d& point,
                                                   const SCPlaneSurface& planeSurface,
                                                   SCPoint2d& uv,
                                                   double epsilon,
                                                   PlaneUvPurpose purpose = PlaneUvPurpose::VerifyOriginalVertex)
    {
        SCGeometryTolerance3d tolerance{};
        tolerance.distanceEpsilon = epsilon;
        if (!tolerance.IsValid() || !TryAffinePlaneUv(planeSurface, point, tolerance, uv))
        {
            return false;
        }
        const SCPoint3d reconstructed = planeSurface.PointAt(uv.x, uv.y);
        if (!reconstructed.IsValid())
        {
            return false;
        }
        if (purpose == PlaneUvPurpose::ApproximateRepair) return uv.IsValid();
        const double residual = std::hypot(point.x-reconstructed.x, point.y-reconstructed.y,
                                           point.z-reconstructed.z);
        return std::isfinite(residual) && residual <= epsilon;
    }
}  // namespace Geometry::Detail
