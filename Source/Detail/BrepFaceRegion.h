#pragma once

#include <cmath>
#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

#include "Brep/SCBrepBody.h"
#include "Core/Relation.h"
#include "Geometry2d/SCPolygon2d.h"
#include "Geometry3d/SCLineCurve3d.h"
#include "Geometry3d/SCPlaneSurface.h"
#include "AffinePlaneGeometry.h"
#include "CurveOnSurfaceNumerics.h"

namespace Geometry::Detail
{
    [[nodiscard]] inline bool TryBuildPolygonFromBrepFaceTrims(
        const SCBrepFace& face, const SCGeometryTolerance3d& tolerance, SCPolygon2d& polygon)
    {
        if (face.SupportSurface() == nullptr || !face.OuterTrim().IsValid(tolerance) ||
            face.HoleTrims().size() != face.HoleCount() ||
            !SameSurfaceParameterization(*face.SupportSurface(), *face.OuterTrim().SupportSurface()) ||
            !CurveOnSurfaceUvInsideSupportDomain(face.OuterTrim()))
        {
            return false;
        }
        std::vector<SCPolyline2d> holes;
        holes.reserve(face.HoleTrims().size());
        for (const SCCurveOnSurface& trim : face.HoleTrims())
        {
            if (!trim.IsValid(tolerance) ||
                !SameSurfaceParameterization(*face.SupportSurface(), *trim.SupportSurface()) ||
                !CurveOnSurfaceUvInsideSupportDomain(trim))
            {
                return false;
            }
            holes.push_back(trim.UvCurve());
        }
        polygon = SCPolygon2d(face.OuterTrim().UvCurve(), std::move(holes));
        return polygon.IsValid();
    }

    [[nodiscard]] inline bool TryBuildPlanarLoopTrim(
        const SCBrepBody& body, const SCBrepLoop& loop,
        const std::shared_ptr<ISCSurface>& support, const SCPlaneSurface& planeSurface,
        const SCGeometryTolerance3d& tolerance, SCCurveOnSurface& trim)
    {
        if (!loop.IsValid() || loop.CoedgeCount() < 3)
        {
            return false;
        }
        std::vector<SCPoint2d> uvPoints;
        uvPoints.reserve(loop.CoedgeCount());
        std::size_t firstVertex = 0;
        std::size_t previousEnd = 0;
        const auto closeEnough = [&](const SCPoint3d& lhs, const SCPoint3d& rhs) {
            if (!lhs.IsValid() || !rhs.IsValid())
            {
                return false;
            }
            const double distance = std::hypot(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
            return std::isfinite(distance) && distance <= tolerance.distanceEpsilon;
        };
        for (std::size_t index = 0; index < loop.CoedgeCount(); ++index)
        {
            const SCBrepCoedge& coedge = loop.Coedges()[index];
            if (coedge.EdgeIndex() >= body.EdgeCount())
            {
                return false;
            }
            const SCBrepEdge& edge = body.Edges()[coedge.EdgeIndex()];
            const auto* lineCurve = dynamic_cast<const SCLineCurve3d*>(edge.Curve());
            if (lineCurve == nullptr || edge.StartVertexIndex() >= body.VertexCount() ||
                edge.EndVertexIndex() >= body.VertexCount())
            {
                return false;
            }
            const SCIntervald range = lineCurve->ParameterRange();
            if (!range.IsValid())
            {
                return false;
            }
            const SCPoint3d vertexStart = body.Vertices()[edge.StartVertexIndex()].Point();
            const SCPoint3d vertexEnd = body.Vertices()[edge.EndVertexIndex()].Point();
            if (!closeEnough(lineCurve->PointAt(range.min), vertexStart) ||
                !closeEnough(lineCurve->PointAt(range.max), vertexEnd))
            {
                return false;
            }
            const std::size_t start = coedge.Reversed() ? edge.EndVertexIndex() : edge.StartVertexIndex();
            const std::size_t end = coedge.Reversed() ? edge.StartVertexIndex() : edge.EndVertexIndex();
            if (index == 0)
            {
                firstVertex = start;
            }
            else if (previousEnd != start)
            {
                return false;
            }
            previousEnd = end;
            const SCPoint3d point = body.Vertices()[start].Point();
            SCPoint2d uv;
            if (!TryAffinePlaneUv(planeSurface, point, tolerance, uv) ||
                !closeEnough(point, planeSurface.PointAt(uv.x, uv.y)))
            {
                return false;
            }
            uvPoints.push_back(uv);
        }
        if (previousEnd != firstVertex)
        {
            return false;
        }
        trim = SCCurveOnSurface(support, SCPolyline2d(std::move(uvPoints), SCPolylineClosure::Closed));
        return trim.IsValid(tolerance);
    }

    // 已有 trim 也必须与 body 的固有 edge 参数方向和顶点对应；
    // 几何上可用的独立 trim 不能替代拓扑对应关系的证明。
    [[nodiscard]] inline bool ValidateTrimCorrespondence(
        const SCBrepBody& body, const SCBrepLoop& loop, const SCCurveOnSurface& actual,
        const std::shared_ptr<ISCSurface>& support, const SCPlaneSurface& plane,
        const SCGeometryTolerance3d& tolerance)
    {
        SCCurveOnSurface expected;
        if (!actual.IsValid(tolerance) ||
            !SameSurfaceParameterization(*actual.SupportSurface(), plane) ||
            !TryBuildPlanarLoopTrim(body, loop, support, plane, tolerance, expected) ||
            actual.PointCount() != expected.PointCount()) return false;
        for (std::size_t shift = 0; shift < actual.PointCount(); ++shift)
        {
            bool match = true;
            for (std::size_t i = 0; i < actual.PointCount(); ++i)
            {
                const auto a = actual.PointAt((i + shift) % actual.PointCount());
                const auto b = expected.PointAt(i);
                match &= a.IsValid() && b.IsValid() &&
                    std::hypot(a.x-b.x, a.y-b.y, a.z-b.z) <= tolerance.distanceEpsilon;
            }
            if (match) return true;
        }
        return false;
    }

    [[nodiscard]] inline bool TryPrepareFaceInBody(
        const SCBrepBody& body, const SCBrepFace& face,
        const SCGeometryTolerance3d& tolerance, SCBrepFace& prepared)
    {
        if (face.OuterTrim().SupportSurface() != nullptr)
        {
            SCPolygon2d polygon;
            if (!TryBuildPolygonFromBrepFaceTrims(face, tolerance, polygon))
            {
                return false;
            }
            // 此路径仅使用 trim 几何，不将其与 3D edge 拼接为拓扑证明。
            // 闭壳分类另行调用 ValidateTrimCorrespondence。
            prepared = face;
            return true;
        }
        const auto* planeSurface = dynamic_cast<const SCPlaneSurface*>(face.SupportSurface());
        if (planeSurface == nullptr)
        {
            return false;
        }
        std::shared_ptr<ISCSurface> support(face.SupportSurface()->Clone().release());
        if (support == nullptr)
        {
            return false;
        }
        SCCurveOnSurface outerTrim;
        if (!TryBuildPlanarLoopTrim(body, face.OuterLoopRef(), support, *planeSurface, tolerance, outerTrim))
        {
            return false;
        }
        std::vector<SCCurveOnSurface> holeTrims;
        holeTrims.reserve(face.HoleCount());
        for (const SCBrepLoop& hole : face.HoleLoops())
        {
            SCCurveOnSurface holeTrim;
            if (!TryBuildPlanarLoopTrim(body, hole, support, *planeSurface, tolerance, holeTrim))
            {
                return false;
            }
            holeTrims.push_back(std::move(holeTrim));
        }
        prepared = SCBrepFace(support, face.OuterLoop(), face.HoleLoops(),
                              std::move(outerTrim), std::move(holeTrims));
        SCPolygon2d polygon;
        return prepared.IsValid(tolerance) &&
               TryBuildPolygonFromBrepFaceTrims(prepared, tolerance, polygon);
    }
}
