#include "sdk/GeometryIntersection.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <set>

#include "sdk/GeometryProjection.h"
#include "sdk/GeometryRelation.h"
#include "sdk/LineCurve3d.h"
#include "sdk/NurbsSurface.h"
#include "sdk/PlaneSurface.h"

namespace geometry::sdk
{
namespace
{
[[nodiscard]] double SurfaceResidualSquared(
    const Line3d& line,
    double t,
    const Surface& surface,
    double u,
    double v)
{
    return (line.PointAt(t) - surface.PointAt(u, v)).LengthSquared();
}

[[nodiscard]] bool TryBuildSupportPlane(
    const NurbsSurface& surface,
    const GeometryTolerance3d& tolerance,
    Plane& plane)
{
    if (!surface.IsValid(tolerance))
    {
        return false;
    }

    const Intervald uRange = surface.URange();
    const Intervald vRange = surface.VRange();
    const double u = 0.5 * (uRange.min + uRange.max);
    const double v = 0.5 * (vRange.min + vRange.max);
    const SurfaceEval3d eval = surface.Evaluate(u, v, 1);
    if (!eval.IsValid() || eval.normal.Length() <= tolerance.distanceEpsilon)
    {
        return false;
    }

    plane = Plane::FromPointAndNormal(eval.point, eval.normal);
    if (!plane.IsValid(tolerance.distanceEpsilon))
    {
        return false;
    }

    for (const Point3d& controlPoint : surface.ControlPoints())
    {
        if (std::abs(plane.SignedDistanceTo(controlPoint, tolerance.distanceEpsilon)) >
            tolerance.distanceEpsilon)
        {
            return false;
        }
    }

    return true;
}

[[nodiscard]] bool TryProjectPointToAffinePlanarNurbs(
    const Point3d& point,
    const NurbsSurface& surface,
    const GeometryTolerance3d& tolerance,
    double& u,
    double& v,
    Point3d& surfacePoint)
{
    if (surface.DegreeU() != 1 || surface.DegreeV() != 1 ||
        surface.ControlPointCountU() != 2 || surface.ControlPointCountV() != 2)
    {
        return false;
    }

    const auto& controlPoints = surface.ControlPoints();
    if (controlPoints.size() != 4)
    {
        return false;
    }

    const Point3d& p00 = controlPoints[0];
    const Point3d& p10 = controlPoints[1];
    const Point3d& p01 = controlPoints[2];
    const Point3d& p11 = controlPoints[3];
    const Point3d affineCorner = p10 + (p01 - p00);
    if (!affineCorner.AlmostEquals(p11, tolerance.distanceEpsilon))
    {
        return false;
    }

    const Vector3d uDirection = p10 - p00;
    const Vector3d vDirection = p01 - p00;
    const Vector3d delta = point - p00;
    const double uu = Dot(uDirection, uDirection);
    const double uv = Dot(uDirection, vDirection);
    const double vv = Dot(vDirection, vDirection);
    const double du = Dot(delta, uDirection);
    const double dv = Dot(delta, vDirection);
    const double denominator = uu * vv - uv * uv;
    if (std::abs(denominator) <= tolerance.parameterEpsilon)
    {
        return false;
    }

    const double normalizedU = (du * vv - dv * uv) / denominator;
    const double normalizedV = (dv * uu - du * uv) / denominator;
    if (normalizedU < -tolerance.parameterEpsilon || normalizedU > 1.0 + tolerance.parameterEpsilon ||
        normalizedV < -tolerance.parameterEpsilon || normalizedV > 1.0 + tolerance.parameterEpsilon)
    {
        return false;
    }

    const Intervald uRange = surface.URange();
    const Intervald vRange = surface.VRange();
    u = uRange.min + uRange.Length() * std::clamp(normalizedU, 0.0, 1.0);
    v = vRange.min + vRange.Length() * std::clamp(normalizedV, 0.0, 1.0);
    surfacePoint = surface.PointAt(u, v);
    return surfacePoint.AlmostEquals(point, tolerance.distanceEpsilon);
}

[[nodiscard]] LineSurfaceIntersection3d RefineLineSurfaceIntersection(
    const Line3d& line,
    const Surface& surface,
    double initialT,
    double initialU,
    double initialV,
    const GeometryTolerance3d& tolerance)
{
    const Intervald uRange = surface.URange();
    const Intervald vRange = surface.VRange();
    double bestT = initialT;
    double bestU = std::clamp(initialU, uRange.min, uRange.max);
    double bestV = std::clamp(initialV, vRange.min, vRange.max);
    double bestResidualSquared = SurfaceResidualSquared(line, bestT, surface, bestU, bestV);

    double stepT = std::max(1.0, tolerance.parameterEpsilon * 8.0);
    double stepU = std::max(uRange.Length() * 0.25, tolerance.parameterEpsilon);
    double stepV = std::max(vRange.Length() * 0.25, tolerance.parameterEpsilon);

    for (int iteration = 0; iteration < 20; ++iteration)
    {
        bool improved = false;
        for (const double dt : std::array<double, 3>{-stepT, 0.0, stepT})
        {
            for (const double du : std::array<double, 3>{-stepU, 0.0, stepU})
            {
                for (const double dv : std::array<double, 3>{-stepV, 0.0, stepV})
                {
                    const double candidateT = bestT + dt;
                    const double candidateU = std::clamp(bestU + du, uRange.min, uRange.max);
                    const double candidateV = std::clamp(bestV + dv, vRange.min, vRange.max);
                    const double candidateResidualSquared =
                        SurfaceResidualSquared(line, candidateT, surface, candidateU, candidateV);
                    if (candidateResidualSquared +
                            tolerance.distanceEpsilon * tolerance.distanceEpsilon <
                        bestResidualSquared)
                    {
                        bestT = candidateT;
                        bestU = candidateU;
                        bestV = candidateV;
                        bestResidualSquared = candidateResidualSquared;
                        improved = true;
                    }
                }
            }
        }

        if (!improved)
        {
            stepT *= 0.5;
            stepU *= 0.5;
            stepV *= 0.5;
        }

        if (stepT <= tolerance.parameterEpsilon &&
            stepU <= tolerance.parameterEpsilon &&
            stepV <= tolerance.parameterEpsilon)
        {
            break;
        }
    }

    const Point3d pointOnLine = line.PointAt(bestT);
    const Point3d pointOnSurface = surface.PointAt(bestU, bestV);
    if ((pointOnLine - pointOnSurface).LengthSquared() >
        tolerance.distanceEpsilon * tolerance.distanceEpsilon)
    {
        return {};
    }

    return {true, false, false, bestT, bestU, bestV, pointOnSurface};
}

[[nodiscard]] bool IsTrimPointInsideFace(
    const BrepFace& face,
    double u,
    double v,
    double eps,
    bool& onBoundary)
{
    onBoundary = false;
    if (!face.OuterTrim().IsValid())
    {
        return true;
    }

    std::vector<Polyline2d> holes;
    holes.reserve(face.HoleTrims().size());
    for (const CurveOnSurface& trim : face.HoleTrims())
    {
        if (!trim.IsValid())
        {
            return false;
        }
        holes.push_back(trim.UvCurve());
    }

    const Polygon2d polygon(face.OuterTrim().UvCurve(), std::move(holes));
    if (!polygon.IsValid())
    {
        return false;
    }

    const PointContainment2d containment = LocatePoint(Point2d{u, v}, polygon, eps);
    onBoundary = containment == PointContainment2d::OnBoundary;
    return containment == PointContainment2d::Inside || onBoundary;
}

[[nodiscard]] bool PointInTriangle3d(
    const Point3d& point,
    const Triangle3d& triangle,
    double eps)
{
    const Vector3d v0 = triangle.c - triangle.a;
    const Vector3d v1 = triangle.b - triangle.a;
    const Vector3d v2 = point - triangle.a;

    const double dot00 = Dot(v0, v0);
    const double dot01 = Dot(v0, v1);
    const double dot02 = Dot(v0, v2);
    const double dot11 = Dot(v1, v1);
    const double dot12 = Dot(v1, v2);
    const double denominator = dot00 * dot11 - dot01 * dot01;
    if (std::abs(denominator) <= eps)
    {
        return false;
    }

    const double inverse = 1.0 / denominator;
    const double u = (dot11 * dot02 - dot01 * dot12) * inverse;
    const double v = (dot00 * dot12 - dot01 * dot02) * inverse;
    return u >= -eps && v >= -eps && u + v <= 1.0 + eps;
}
} // namespace

LinePlaneIntersection3d Intersect(
    const Line3d& line,
    const Plane& plane,
    const GeometryTolerance3d& tolerance)
{
    LinePlaneIntersection3d result{};
    if (!line.IsValid(tolerance.distanceEpsilon) || !plane.IsValid(tolerance.distanceEpsilon))
    {
        return result;
    }

    const Vector3d unitNormal = plane.UnitNormal(tolerance.distanceEpsilon);
    const double denominator = Dot(unitNormal, line.direction);
    const double numerator = Dot(unitNormal, plane.origin - line.origin);

    if (std::abs(denominator) <= tolerance.distanceEpsilon)
    {
        result.isParallel = true;
        result.liesOnPlane = std::abs(numerator) <= tolerance.distanceEpsilon;
        result.intersects = result.liesOnPlane;
        result.point = line.origin;
        return result;
    }

    result.intersects = true;
    result.parameter = numerator / denominator;
    result.point = line.PointAt(result.parameter);
    return result;
}

LineCurveIntersection3d Intersect(
    const Line3d& line,
    const Curve3d& curve,
    const GeometryTolerance3d& tolerance)
{
    if (!line.IsValid(tolerance.distanceEpsilon) || !curve.IsValid(tolerance))
    {
        return {};
    }

    if (const auto* lineCurve = dynamic_cast<const LineCurve3d*>(&curve))
    {
        const Line3d curveLine = lineCurve->Line();
        const Vector3d cross = Cross(line.direction, curveLine.direction);
        if (cross.LengthSquared() <= tolerance.distanceEpsilon * tolerance.distanceEpsilon)
        {
            const double separationSquared =
                Cross(curveLine.origin - line.origin, line.direction).LengthSquared() /
                std::max(line.direction.LengthSquared(), tolerance.distanceEpsilon);
            if (separationSquared > tolerance.distanceEpsilon * tolerance.distanceEpsilon)
            {
                return {};
            }

            const Intervald curveRange = lineCurve->ParameterRange();
            const double curveParameter = curveRange.min;
            const Point3d point = lineCurve->PointAt(curveParameter);
            const double lineParameter =
                Dot(point - line.origin, line.direction) /
                std::max(line.direction.LengthSquared(), tolerance.parameterEpsilon);
            return {true, lineParameter, curveParameter, point};
        }

        const Vector3d delta = curveLine.origin - line.origin;
        const double a = line.direction.LengthSquared();
        const double b = Dot(line.direction, curveLine.direction);
        const double c = curveLine.direction.LengthSquared();
        const double d = Dot(line.direction, delta);
        const double e = Dot(curveLine.direction, delta);
        const double denominator = a * c - b * b;
        if (std::abs(denominator) <= tolerance.distanceEpsilon)
        {
            return {};
        }

        const double lineParameter = (d * c - b * e) / denominator;
        const double curveParameter = (b * d - a * e) / denominator;
        const Intervald curveRange = lineCurve->ParameterRange();
        if (!curveRange.Contains(curveParameter, tolerance.parameterEpsilon))
        {
            return {};
        }

        const Point3d pointOnLine = line.PointAt(lineParameter);
        const Point3d pointOnCurve = lineCurve->PointAt(curveParameter);
        if (!pointOnLine.AlmostEquals(pointOnCurve, tolerance.distanceEpsilon))
        {
            return {};
        }

        return {true, lineParameter, curveParameter, pointOnCurve};
    }

    const Intervald range = curve.ParameterRange();
    if (!range.IsValid())
    {
        return {};
    }

    constexpr std::size_t sampleCount = 33;
    double bestLineParameter = 0.0;
    double bestCurveParameter = range.min;
    double bestDistanceSquared = (line.origin - curve.PointAt(bestCurveParameter)).LengthSquared();
    for (std::size_t i = 0; i < sampleCount; ++i)
    {
        const double curveParameter = i + 1 == sampleCount
                                          ? range.max
                                          : range.min + range.Length() * static_cast<double>(i) /
                                                             static_cast<double>(sampleCount - 1);
        const Point3d pointOnCurve = curve.PointAt(curveParameter);
        const double lineParameter =
            Dot(pointOnCurve - line.origin, line.direction) /
            std::max(line.direction.LengthSquared(), tolerance.parameterEpsilon);
        const Point3d pointOnLine = line.PointAt(lineParameter);
        const double distanceSquared = (pointOnLine - pointOnCurve).LengthSquared();
        if (distanceSquared < bestDistanceSquared)
        {
            bestLineParameter = lineParameter;
            bestCurveParameter = curveParameter;
            bestDistanceSquared = distanceSquared;
        }
    }

    if (bestDistanceSquared > tolerance.distanceEpsilon * tolerance.distanceEpsilon)
    {
        return {};
    }

    return {true, bestLineParameter, bestCurveParameter, curve.PointAt(bestCurveParameter)};
}

LineCurveOnSurfaceIntersection3d Intersect(
    const Line3d& line,
    const CurveOnSurface& curveOnSurface,
    const GeometryTolerance3d& tolerance)
{
    if (!line.IsValid(tolerance.distanceEpsilon) || !curveOnSurface.IsValid(tolerance) || curveOnSurface.PointCount() < 2)
    {
        return {};
    }

    const std::size_t pointCount = curveOnSurface.PointCount();
    const std::size_t segmentCount = curveOnSurface.UvCurve().IsClosed() ? pointCount : pointCount - 1;
    for (std::size_t i = 0; i < segmentCount; ++i)
    {
        const std::size_t j = (i + 1) % pointCount;
        const Point3d start = curveOnSurface.PointAt(i);
        const Point3d end = curveOnSurface.PointAt(j);
        const Vector3d segmentDirection = end - start;
        const double segmentLengthSquared = segmentDirection.LengthSquared();
        if (segmentLengthSquared <= tolerance.distanceEpsilon * tolerance.distanceEpsilon)
        {
            continue;
        }

        const Line3d segmentLine = Line3d::FromOriginAndDirection(start, segmentDirection);
        const Vector3d delta = segmentLine.origin - line.origin;
        const double a = line.direction.LengthSquared();
        const double b = Dot(line.direction, segmentLine.direction);
        const double c = segmentLine.direction.LengthSquared();
        const double d = Dot(line.direction, delta);
        const double e = Dot(segmentLine.direction, delta);
        const double denominator = a * c - b * b;
        if (std::abs(denominator) <= tolerance.distanceEpsilon)
        {
            continue;
        }

        const double lineParameter = (d * c - b * e) / denominator;
        const double segmentParameter = (a * e - b * d) / denominator;
        if (segmentParameter < -tolerance.parameterEpsilon || segmentParameter > 1.0 + tolerance.parameterEpsilon)
        {
            continue;
        }

        const Point3d pointOnLine = line.PointAt(lineParameter);
        const Point3d pointOnSegment = start + segmentDirection * segmentParameter;
        if (!pointOnLine.AlmostEquals(pointOnSegment, tolerance.distanceEpsilon))
        {
            continue;
        }

        const Point2d uv0 = curveOnSurface.UvPointAt(i);
        const Point2d uv1 = curveOnSurface.UvPointAt(j);
        const Point2d uv = uv0 + (uv1 - uv0) * segmentParameter;
        return {true, i, segmentParameter, lineParameter, uv, pointOnSegment};
    }

    return {};
}

LineSurfaceIntersection3d Intersect(
    const Line3d& line,
    const Surface& surface,
    const GeometryTolerance3d& tolerance)
{
    if (!line.IsValid(tolerance.distanceEpsilon) || !surface.IsValid(tolerance))
    {
        return {};
    }

    if (const auto* planeSurface = dynamic_cast<const PlaneSurface*>(&surface))
    {
        const LinePlaneIntersection3d planeIntersection =
            Intersect(line, planeSurface->SupportPlane(), tolerance);
        if (!planeIntersection.intersects)
        {
            return {false, planeIntersection.isParallel, planeIntersection.liesOnPlane, 0.0, 0.0, 0.0, {}};
        }

        const Vector3d delta = planeIntersection.point - planeSurface->SupportPlane().origin;
        const Vector3d uAxis = planeSurface->UAxis();
        const Vector3d vAxis = planeSurface->VAxis();
        const double uDenominator = std::max(uAxis.LengthSquared(), tolerance.parameterEpsilon);
        const double vDenominator = std::max(vAxis.LengthSquared(), tolerance.parameterEpsilon);
        const double u = Dot(delta, uAxis) / uDenominator;
        const double v = Dot(delta, vAxis) / vDenominator;
        const bool insideDomain = planeSurface->URange().Contains(u, tolerance.parameterEpsilon) &&
                                  planeSurface->VRange().Contains(v, tolerance.parameterEpsilon);
        if (!insideDomain)
        {
            return {};
        }

        return {true, false, planeIntersection.liesOnPlane, planeIntersection.parameter, u, v, planeIntersection.point};
    }

    if (const auto* nurbsSurface = dynamic_cast<const NurbsSurface*>(&surface))
    {
        Plane supportPlane{};
        if (TryBuildSupportPlane(*nurbsSurface, tolerance, supportPlane))
        {
            const LinePlaneIntersection3d planeIntersection = Intersect(line, supportPlane, tolerance);
            if (!planeIntersection.intersects)
            {
                return {false, planeIntersection.isParallel, planeIntersection.liesOnPlane, 0.0, 0.0, 0.0, {}};
            }

            double u = 0.0;
            double v = 0.0;
            Point3d surfacePoint{};
            if (TryProjectPointToAffinePlanarNurbs(
                    planeIntersection.point,
                    *nurbsSurface,
                    tolerance,
                    u,
                    v,
                    surfacePoint))
            {
                return {
                    true,
                    false,
                    planeIntersection.liesOnPlane,
                    planeIntersection.parameter,
                    u,
                    v,
                    surfacePoint};
            }

            const SurfaceProjection3d surfaceProjection =
                ProjectPointToSurface(planeIntersection.point, surface, tolerance);
            if (!surfaceProjection.success ||
                surfaceProjection.distanceSquared > tolerance.distanceEpsilon * tolerance.distanceEpsilon)
            {
                return {};
            }

            return {
                true,
                false,
                planeIntersection.liesOnPlane,
                planeIntersection.parameter,
                surfaceProjection.u,
                surfaceProjection.v,
                surfaceProjection.point};
        }
    }

    const Intervald uRange = surface.URange();
    const Intervald vRange = surface.VRange();
    if (!uRange.IsValid() || !vRange.IsValid())
    {
        return {};
    }

    constexpr std::size_t sampleCountT = 17;
    constexpr std::size_t sampleCountU = 9;
    constexpr std::size_t sampleCountV = 9;
    const double lineStep = 8.0;

    double bestT = -lineStep * static_cast<double>(sampleCountT / 2);
    double bestU = uRange.min;
    double bestV = vRange.min;
    double bestResidualSquared = SurfaceResidualSquared(line, bestT, surface, bestU, bestV);

    const SurfaceProjection3d originProjection = ProjectPointToSurface(line.origin, surface, tolerance);
    if (originProjection.success)
    {
        const double projectedLineParameter =
            Dot(originProjection.point - line.origin, line.direction) /
            std::max(line.direction.LengthSquared(), tolerance.parameterEpsilon);
        const double projectedResidualSquared = SurfaceResidualSquared(
            line,
            projectedLineParameter,
            surface,
            originProjection.u,
            originProjection.v);
        if (projectedResidualSquared < bestResidualSquared)
        {
            bestT = projectedLineParameter;
            bestU = originProjection.u;
            bestV = originProjection.v;
            bestResidualSquared = projectedResidualSquared;
        }
    }

    for (std::size_t ti = 0; ti < sampleCountT; ++ti)
    {
        const double t = (static_cast<double>(ti) - static_cast<double>(sampleCountT / 2)) * lineStep;
        for (std::size_t ui = 0; ui < sampleCountU; ++ui)
        {
            const double u = ui + 1 == sampleCountU
                                 ? uRange.max
                                 : uRange.min + uRange.Length() * static_cast<double>(ui) /
                                                    static_cast<double>(sampleCountU - 1);
            for (std::size_t vi = 0; vi < sampleCountV; ++vi)
            {
                const double v = vi + 1 == sampleCountV
                                     ? vRange.max
                                     : vRange.min + vRange.Length() * static_cast<double>(vi) /
                                                        static_cast<double>(sampleCountV - 1);
                const double residualSquared = SurfaceResidualSquared(line, t, surface, u, v);
                if (residualSquared < bestResidualSquared)
                {
                    bestT = t;
                    bestU = u;
                    bestV = v;
                    bestResidualSquared = residualSquared;
                }
            }
        }
    }

    return RefineLineSurfaceIntersection(line, surface, bestT, bestU, bestV, tolerance);
}

LineBrepEdgeIntersection3d Intersect(
    const Line3d& line,
    const BrepEdge& edge,
    const GeometryTolerance3d& tolerance)
{
    if (!line.IsValid(tolerance.distanceEpsilon) || !edge.IsValid(tolerance) || edge.Curve() == nullptr)
    {
        return {};
    }

    if (const auto* lineCurve = dynamic_cast<const LineCurve3d*>(edge.Curve()))
    {
        const Line3d edgeLine = lineCurve->Line();
        const Vector3d cross = Cross(line.direction, edgeLine.direction);
        if (cross.LengthSquared() <= tolerance.distanceEpsilon * tolerance.distanceEpsilon)
        {
            const double separationSquared =
                Cross(edgeLine.origin - line.origin, line.direction).LengthSquared() /
                std::max(line.direction.LengthSquared(), tolerance.distanceEpsilon);
            if (separationSquared > tolerance.distanceEpsilon * tolerance.distanceEpsilon)
            {
                return {};
            }

            const Intervald edgeRange = lineCurve->ParameterRange();
            const double edgeParameter = edgeRange.min;
            const Point3d point = lineCurve->PointAt(edgeParameter);
            const double lineParameter =
                Dot(point - line.origin, line.direction) / std::max(line.direction.LengthSquared(), tolerance.parameterEpsilon);
            return {true, lineParameter, edgeParameter, point};
        }

        const Vector3d delta = edgeLine.origin - line.origin;
        const double a = line.direction.LengthSquared();
        const double b = Dot(line.direction, edgeLine.direction);
        const double c = edgeLine.direction.LengthSquared();
        const double d = Dot(line.direction, delta);
        const double e = Dot(edgeLine.direction, delta);
        const double denominator = a * c - b * b;
        if (std::abs(denominator) <= tolerance.distanceEpsilon)
        {
            return {};
        }

        const double lineParameter = (d * c - b * e) / denominator;
        const double edgeParameter = (b * d - a * e) / denominator;
        const Intervald edgeRange = lineCurve->ParameterRange();
        if (!edgeRange.Contains(edgeParameter, tolerance.parameterEpsilon))
        {
            return {};
        }

        const Point3d pointOnLine = line.PointAt(lineParameter);
        const Point3d pointOnEdge = lineCurve->PointAt(edgeParameter);
        if (!pointOnLine.AlmostEquals(pointOnEdge, tolerance.distanceEpsilon))
        {
            return {};
        }

        return {true, lineParameter, edgeParameter, pointOnEdge};
    }

    const Curve3d& curve = *edge.Curve();
    const Intervald range = curve.ParameterRange();
    if (!range.IsValid())
    {
        return {};
    }

    constexpr std::size_t sampleCount = 33;
    double bestLineParameter = 0.0;
    double bestEdgeParameter = range.min;
    double bestDistanceSquared = (line.origin - curve.PointAt(bestEdgeParameter)).LengthSquared();
    for (std::size_t i = 0; i < sampleCount; ++i)
    {
        const double edgeParameter = i + 1 == sampleCount
                                         ? range.max
                                         : range.min + range.Length() * static_cast<double>(i) /
                                                            static_cast<double>(sampleCount - 1);
        const Point3d pointOnCurve = curve.PointAt(edgeParameter);
        const double lineParameter =
            Dot(pointOnCurve - line.origin, line.direction) /
            std::max(line.direction.LengthSquared(), tolerance.parameterEpsilon);
        const Point3d pointOnLine = line.PointAt(lineParameter);
        const double distanceSquared = (pointOnLine - pointOnCurve).LengthSquared();
        if (distanceSquared < bestDistanceSquared)
        {
            bestLineParameter = lineParameter;
            bestEdgeParameter = edgeParameter;
            bestDistanceSquared = distanceSquared;
        }
    }

    if (bestDistanceSquared > tolerance.distanceEpsilon * tolerance.distanceEpsilon)
    {
        return {};
    }

    return {true, bestLineParameter, bestEdgeParameter, curve.PointAt(bestEdgeParameter)};
}

LineBrepVertexIntersection3d Intersect(
    const Line3d& line,
    const BrepVertex& vertex,
    const GeometryTolerance3d& tolerance)
{
    if (!line.IsValid(tolerance.distanceEpsilon) || !vertex.IsValid())
    {
        return {};
    }

    const LineProjection3d projection = ProjectPointToLine(vertex.Point(), line, tolerance);
    if (!projection.isOnLine ||
        projection.distanceSquared > tolerance.distanceEpsilon * tolerance.distanceEpsilon)
    {
        return {};
    }

    return {true, projection.parameter, vertex.Point()};
}

LineBrepFaceIntersection3d Intersect(
    const Line3d& line,
    const BrepFace& face,
    const GeometryTolerance3d& tolerance)
{
    if (!line.IsValid(tolerance.distanceEpsilon) || !face.IsValid(tolerance) || face.SupportSurface() == nullptr)
    {
        return {};
    }

    const LineSurfaceIntersection3d surfaceIntersection =
        Intersect(line, *face.SupportSurface(), tolerance);
    if (!surfaceIntersection.intersects)
    {
        return {};
    }

    bool onBoundary = false;
    if (!IsTrimPointInsideFace(face, surfaceIntersection.u, surfaceIntersection.v, tolerance.distanceEpsilon, onBoundary))
    {
        return {};
    }

    return {
        true,
        onBoundary,
        surfaceIntersection.lineParameter,
        surfaceIntersection.u,
        surfaceIntersection.v,
        surfaceIntersection.point};
}

LineBrepBodyIntersection3d Intersect(
    const Line3d& line,
    const BrepBody& body,
    const GeometryTolerance3d& tolerance)
{
    LineBrepBodyIntersection3d result{};
    if (!line.IsValid(tolerance.distanceEpsilon) || !body.IsValid(tolerance))
    {
        return result;
    }

    std::vector<std::pair<std::size_t, LineBrepFaceIntersection3d>> collected;
    std::set<std::pair<long long, std::size_t>> dedup;
    std::size_t faceIndex = 0;
    for (std::size_t shellIndex = 0; shellIndex < body.ShellCount(); ++shellIndex)
    {
        const BrepShell shell = body.ShellAt(shellIndex);
        for (std::size_t localFaceIndex = 0; localFaceIndex < shell.FaceCount(); ++localFaceIndex, ++faceIndex)
        {
            const LineBrepFaceIntersection3d hit =
                Intersect(line, shell.FaceAt(localFaceIndex), tolerance);
            if (!hit.intersects)
            {
                continue;
            }

            const long long bucket = static_cast<long long>(
                std::llround(hit.lineParameter / std::max(tolerance.parameterEpsilon, geometry::kDefaultEpsilon)));
            if (!dedup.emplace(bucket, faceIndex).second)
            {
                continue;
            }

            collected.emplace_back(faceIndex, hit);
        }
    }

    std::sort(
        collected.begin(),
        collected.end(),
        [](const std::pair<std::size_t, LineBrepFaceIntersection3d>& lhs,
           const std::pair<std::size_t, LineBrepFaceIntersection3d>& rhs)
        {
            return lhs.second.lineParameter < rhs.second.lineParameter;
        });

    for (const auto& [hitFaceIndex, hit] : collected)
    {
        result.faceIndices.push_back(hitFaceIndex);
        result.hits.push_back(hit);
    }

    result.intersects = !result.hits.empty();
    return result;
}

LinePolyhedronFaceIntersection3d Intersect(
    const Line3d& line,
    const PolyhedronFace3d& face,
    const GeometryTolerance3d& tolerance)
{
    if (!line.IsValid(tolerance.distanceEpsilon) || !face.IsValid(tolerance.distanceEpsilon))
    {
        return {};
    }

    const LinePlaneIntersection3d planeIntersection =
        Intersect(line, face.SupportPlane(), tolerance);
    if (!planeIntersection.intersects)
    {
        return {};
    }

    const FaceProjection3d faceProjection = ProjectFaceToPolygon2d(face, tolerance);
    if (!faceProjection.success)
    {
        return {};
    }

    const Vector3d delta = planeIntersection.point - faceProjection.origin;
    const double u = Dot(delta, faceProjection.uAxis);
    const double v = Dot(delta, faceProjection.vAxis);
    const PointContainment2d containment =
        LocatePoint(Point2d{u, v}, faceProjection.polygon, tolerance.distanceEpsilon);
    if (containment == PointContainment2d::Outside)
    {
        return {};
    }

    return {
        true,
        containment == PointContainment2d::OnBoundary,
        planeIntersection.parameter,
        u,
        v,
        planeIntersection.point};
}

LinePolyhedronBodyIntersection3d Intersect(
    const Line3d& line,
    const PolyhedronBody& body,
    const GeometryTolerance3d& tolerance)
{
    LinePolyhedronBodyIntersection3d result{};
    if (!line.IsValid(tolerance.distanceEpsilon) || !body.IsValid(tolerance.distanceEpsilon))
    {
        return result;
    }

    std::vector<std::pair<std::size_t, LinePolyhedronFaceIntersection3d>> collected;
    std::set<std::pair<long long, std::size_t>> dedup;
    for (std::size_t faceIndex = 0; faceIndex < body.FaceCount(); ++faceIndex)
    {
        const LinePolyhedronFaceIntersection3d hit = Intersect(line, body.FaceAt(faceIndex), tolerance);
        if (!hit.intersects)
        {
            continue;
        }

        const long long bucket = static_cast<long long>(
            std::llround(hit.lineParameter / std::max(tolerance.parameterEpsilon, geometry::kDefaultEpsilon)));
        if (!dedup.emplace(bucket, faceIndex).second)
        {
            continue;
        }

        collected.emplace_back(faceIndex, hit);
    }

    std::sort(
        collected.begin(),
        collected.end(),
        [](const std::pair<std::size_t, LinePolyhedronFaceIntersection3d>& lhs,
           const std::pair<std::size_t, LinePolyhedronFaceIntersection3d>& rhs)
        {
            return lhs.second.lineParameter < rhs.second.lineParameter;
        });

    for (const auto& [hitFaceIndex, hit] : collected)
    {
        result.faceIndices.push_back(hitFaceIndex);
        result.hits.push_back(hit);
    }

    result.intersects = !result.hits.empty();
    return result;
}

LineTriangleMeshIntersection3d Intersect(
    const Line3d& line,
    const TriangleMesh& mesh,
    const GeometryTolerance3d& tolerance)
{
    LineTriangleMeshIntersection3d result{};
    if (!line.IsValid(tolerance.distanceEpsilon) || !mesh.IsValid(tolerance.distanceEpsilon))
    {
        return result;
    }

    std::set<std::pair<long long, std::size_t>> dedup;
    struct Hit
    {
        std::size_t triangleIndex;
        double lineParameter;
        Point3d point;
    };
    std::vector<Hit> hits;
    for (std::size_t triangleIndex = 0; triangleIndex < mesh.TriangleCount(); ++triangleIndex)
    {
        const Triangle3d triangle = mesh.TriangleAt(triangleIndex);
        const Plane plane = Plane::FromPointAndNormal(triangle.a, triangle.Normal());
        const LinePlaneIntersection3d planeHit = Intersect(line, plane, tolerance);
        if (!planeHit.intersects)
        {
            continue;
        }

        if (!PointInTriangle3d(planeHit.point, triangle, tolerance.distanceEpsilon))
        {
            continue;
        }

        const long long bucket = static_cast<long long>(
            std::llround(planeHit.parameter / std::max(tolerance.parameterEpsilon, geometry::kDefaultEpsilon)));
        if (!dedup.emplace(bucket, triangleIndex).second)
        {
            continue;
        }
        hits.push_back({triangleIndex, planeHit.parameter, planeHit.point});
    }

    std::sort(
        hits.begin(),
        hits.end(),
        [](const Hit& lhs, const Hit& rhs)
        {
            return lhs.lineParameter < rhs.lineParameter;
        });

    for (const Hit& hit : hits)
    {
        result.triangleIndices.push_back(hit.triangleIndex);
        result.lineParameters.push_back(hit.lineParameter);
        result.points.push_back(hit.point);
    }

    result.intersects = !result.points.empty();
    return result;
}

PlaneCurveIntersection3d Intersect(
    const Plane& plane,
    const Curve3d& curve,
    const GeometryTolerance3d& tolerance)
{
    if (!plane.IsValid(tolerance.distanceEpsilon) || !curve.IsValid(tolerance))
    {
        return {};
    }

    if (const auto* lineCurve = dynamic_cast<const LineCurve3d*>(&curve))
    {
        const LinePlaneIntersection3d hit = Intersect(lineCurve->Line(), plane, tolerance);
        if (!hit.intersects)
        {
            return {};
        }
        const Intervald range = lineCurve->ParameterRange();
        if (!range.Contains(hit.parameter, tolerance.parameterEpsilon))
        {
            return {};
        }
        return {true, hit.parameter, hit.point};
    }

    const Intervald range = curve.ParameterRange();
    if (!range.IsValid())
    {
        return {};
    }

    constexpr std::size_t sampleCount = 33;
    for (std::size_t i = 1; i < sampleCount; ++i)
    {
        const double t0 = range.min + range.Length() * static_cast<double>(i - 1) / static_cast<double>(sampleCount - 1);
        const double t1 = i + 1 == sampleCount
                              ? range.max
                              : range.min + range.Length() * static_cast<double>(i) / static_cast<double>(sampleCount - 1);
        const Point3d p0 = curve.PointAt(t0);
        const Point3d p1 = curve.PointAt(t1);
        const double d0 = plane.SignedDistanceTo(p0, tolerance.distanceEpsilon);
        const double d1 = plane.SignedDistanceTo(p1, tolerance.distanceEpsilon);
        if (std::abs(d0) <= tolerance.distanceEpsilon)
        {
            return {true, t0, p0};
        }
        if (d0 * d1 > 0.0)
        {
            continue;
        }

        const double weight = d0 / (d0 - d1);
        const double parameter = t0 + (t1 - t0) * weight;
        return {true, parameter, curve.PointAt(parameter)};
    }

    return {};
}

PlaneCurveOnSurfaceIntersection3d Intersect(
    const Plane& plane,
    const CurveOnSurface& curveOnSurface,
    const GeometryTolerance3d& tolerance)
{
    if (!plane.IsValid(tolerance.distanceEpsilon) || !curveOnSurface.IsValid(tolerance) || curveOnSurface.PointCount() < 2)
    {
        return {};
    }

    const std::size_t pointCount = curveOnSurface.PointCount();
    const std::size_t segmentCount = curveOnSurface.UvCurve().IsClosed() ? pointCount : pointCount - 1;
    for (std::size_t i = 0; i < segmentCount; ++i)
    {
        const std::size_t j = (i + 1) % pointCount;
        const Point3d p0 = curveOnSurface.PointAt(i);
        const Point3d p1 = curveOnSurface.PointAt(j);
        const double d0 = plane.SignedDistanceTo(p0, tolerance.distanceEpsilon);
        const double d1 = plane.SignedDistanceTo(p1, tolerance.distanceEpsilon);
        if (std::abs(d0) <= tolerance.distanceEpsilon)
        {
            return {true, i, 0.0, curveOnSurface.UvPointAt(i), p0};
        }
        if (d0 * d1 > 0.0)
        {
            continue;
        }

        const double weight = d0 / (d0 - d1);
        const Point2d uv0 = curveOnSurface.UvPointAt(i);
        const Point2d uv1 = curveOnSurface.UvPointAt(j);
        const Point2d uv = uv0 + (uv1 - uv0) * weight;
        return {true, i, weight, uv, p0 + (p1 - p0) * weight};
    }

    return {};
}

PlaneBrepEdgeIntersection3d Intersect(
    const Plane& plane,
    const BrepEdge& edge,
    const GeometryTolerance3d& tolerance)
{
    if (!plane.IsValid(tolerance.distanceEpsilon) || !edge.IsValid(tolerance) || edge.Curve() == nullptr)
    {
        return {};
    }

    const PlaneCurveIntersection3d hit = Intersect(plane, *edge.Curve(), tolerance);
    if (!hit.intersects)
    {
        return {};
    }
    return {true, hit.curveParameter, hit.point};
}

PlaneBrepVertexIntersection3d Intersect(
    const Plane& plane,
    const BrepVertex& vertex,
    const GeometryTolerance3d& tolerance)
{
    if (!plane.IsValid(tolerance.distanceEpsilon) || !vertex.IsValid())
    {
        return {};
    }

    if (std::abs(plane.SignedDistanceTo(vertex.Point(), tolerance.distanceEpsilon)) >
        tolerance.distanceEpsilon)
    {
        return {};
    }

    return {true, vertex.Point()};
}

PlanePlaneIntersection3d Intersect(
    const Plane& first,
    const Plane& second,
    const GeometryTolerance3d& tolerance)
{
    PlanePlaneIntersection3d result{};
    if (!first.IsValid(tolerance.distanceEpsilon) || !second.IsValid(tolerance.distanceEpsilon))
    {
        return result;
    }

    const Vector3d n1 = first.UnitNormal(tolerance.distanceEpsilon);
    const Vector3d n2 = second.UnitNormal(tolerance.distanceEpsilon);
    const Vector3d direction = Cross(n1, n2);
    const double directionLengthSquared = direction.LengthSquared();

    if (directionLengthSquared <= tolerance.distanceEpsilon * tolerance.distanceEpsilon)
    {
        result.isParallel = true;
        result.isCoincident = std::abs(second.SignedDistanceTo(first.origin, tolerance.distanceEpsilon)) <=
                              tolerance.distanceEpsilon;
        result.intersects = result.isCoincident;
        return result;
    }

    const double d1 = Dot(n1, first.origin - Point3d{});
    const double d2 = Dot(n2, second.origin - Point3d{});
    const Vector3d pointVector = Cross((d1 * n2) - (d2 * n1), direction) / directionLengthSquared;

    result.intersects = true;
    result.line = Line3d::FromOriginAndDirection(Point3d{pointVector.x, pointVector.y, pointVector.z}, direction);
    return result;
}
} // namespace geometry::sdk
