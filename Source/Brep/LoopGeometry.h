#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "Brep/SCBrepBody.h"
#include "Brep/SCBrepLoop.h"
#include "../Detail/AffinePlaneGeometry.h"
#include "Geometry3d/SCPlaneSurface.h"
#include "Support/Epsilon.h"

namespace Geometry::Detail
{
    enum class PlaneUvPurpose { VerifyOriginalVertex, ApproximateRepair };
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
