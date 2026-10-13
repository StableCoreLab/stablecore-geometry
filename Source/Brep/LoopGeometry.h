#pragma once

#include <algorithm>
#include <vector>

#include "Brep/SCBrepBody.h"
#include "Brep/SCBrepLoop.h"
#include "Geometry3d/SCPlaneSurface.h"
#include "Support/Epsilon.h"

namespace Geometry::Detail
{
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
            const SCBrepCoedge coedge = loop.CoedgeAt(i);
            if (coedge.EdgeIndex() >= body.EdgeCount())
            {
                return false;
            }

            const SCBrepEdge edge = body.EdgeAt(coedge.EdgeIndex());
            const std::size_t vertexIndex = coedge.Reversed() ? edge.EndVertexIndex() : edge.StartVertexIndex();
            if (vertexIndex >= body.VertexCount())
            {
                return false;
            }

            const SCPoint3d point = body.VertexAt(vertexIndex).Point();
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

    [[nodiscard]] inline SCPoint2d ProjectPointToPlaneUv(const SCPoint3d& point, const SCPlaneSurface& planeSurface)
    {
        const SCPlane plane = planeSurface.SupportPlane();
        const SCVector3d delta = point - plane.origin;
        const SCVector3d uAxis = planeSurface.UAxis();
        const SCVector3d vAxis = planeSurface.VAxis();
        const double uDenom = std::max(uAxis.LengthSquared(), Geometry::kPlaneProjectionEpsilon);
        const double vDenom = std::max(vAxis.LengthSquared(), Geometry::kPlaneProjectionEpsilon);
        return SCPoint2d{Dot(delta, uAxis) / uDenom, Dot(delta, vAxis) / vDenom};
    }
}  // namespace Geometry::Detail
