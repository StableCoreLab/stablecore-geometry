#include "sdk/GeometryProjection.h"

#include <algorithm>
#include <array>

#include "sdk/GeometryRelation.h"
#include "sdk/LineCurve3d.h"
#include "sdk/PlaneSurface.h"

namespace geometry::sdk
{
namespace
{
struct PlaneProjectionBasis
{
    Vector3d u{};
    Vector3d v{};
};

[[nodiscard]] PlaneProjectionBasis BuildPlaneProjectionBasis(const Plane& plane, double eps)
{
    const Vector3d normal = plane.UnitNormal(eps);
    const Vector3d axis = std::abs(normal.x) <= std::abs(normal.y) &&
                                  std::abs(normal.x) <= std::abs(normal.z)
                              ? Vector3d{1.0, 0.0, 0.0}
                              : (std::abs(normal.y) <= std::abs(normal.z)
                                     ? Vector3d{0.0, 1.0, 0.0}
                                     : Vector3d{0.0, 0.0, 1.0});
    const Vector3d u = Cross(normal, axis).Normalized(eps);
    const Vector3d v = Cross(normal, u).Normalized(eps);
    return {u, v};
}

[[nodiscard]] Point2d ProjectToLocalPlaneCoordinates(
    const Point3d& point,
    const Plane& plane,
    const PlaneProjectionBasis& basis)
{
    const Vector3d delta = point - plane.origin;
    return Point2d{Dot(delta, basis.u), Dot(delta, basis.v)};
}

[[nodiscard]] double DistanceSquaredToSurfacePoint(
    const Point3d& point,
    const Surface& surface,
    double u,
    double v)
{
    return (point - surface.PointAt(u, v)).LengthSquared();
}

[[nodiscard]] SurfaceProjection3d RefineSurfaceProjection(
    const Point3d& point,
    const Surface& surface,
    double u,
    double v,
    const GeometryTolerance3d& tolerance)
{
    const Intervald uRange = surface.URange();
    const Intervald vRange = surface.VRange();
    double bestU = std::clamp(u, uRange.min, uRange.max);
    double bestV = std::clamp(v, vRange.min, vRange.max);
    double bestDistanceSquared = DistanceSquaredToSurfacePoint(point, surface, bestU, bestV);

    double stepU = std::max(uRange.Length() * 0.25, tolerance.parameterEpsilon);
    double stepV = std::max(vRange.Length() * 0.25, tolerance.parameterEpsilon);

    for (int iteration = 0; iteration < 16; ++iteration)
    {
        bool improved = false;
        for (const double du : std::array<double, 3>{-stepU, 0.0, stepU})
        {
            for (const double dv : std::array<double, 3>{-stepV, 0.0, stepV})
            {
                const double candidateU = std::clamp(bestU + du, uRange.min, uRange.max);
                const double candidateV = std::clamp(bestV + dv, vRange.min, vRange.max);
                const double candidateDistanceSquared =
                    DistanceSquaredToSurfacePoint(point, surface, candidateU, candidateV);
                if (candidateDistanceSquared + tolerance.distanceEpsilon * tolerance.distanceEpsilon <
                    bestDistanceSquared)
                {
                    bestU = candidateU;
                    bestV = candidateV;
                    bestDistanceSquared = candidateDistanceSquared;
                    improved = true;
                }
            }
        }

        if (!improved)
        {
            stepU *= 0.5;
            stepV *= 0.5;
        }

        if (stepU <= tolerance.parameterEpsilon && stepV <= tolerance.parameterEpsilon)
        {
            break;
        }
    }

    return {true, surface.PointAt(bestU, bestV), bestU, bestV, bestDistanceSquared};
}

[[nodiscard]] bool BuildPolygonFromBrepFaceTrims(
    const BrepFace& face,
    Polygon2d& polygon)
{
    if (face.OuterTrim().SupportSurface() == nullptr || !face.OuterTrim().IsValid())
    {
        return false;
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

    polygon = Polygon2d(face.OuterTrim().UvCurve(), std::move(holes));
    return polygon.IsValid();
}

[[nodiscard]] LineProjection3d ProjectPointToSegment3d(
    const Point3d& point,
    const Point3d& start,
    const Point3d& end,
    double eps)
{
    const Vector3d direction = end - start;
    const double lengthSquared = direction.LengthSquared();
    if (lengthSquared <= eps * eps)
    {
        return {start, 0.0, (point - start).LengthSquared(), true};
    }

    const double parameter = std::clamp(Dot(point - start, direction) / lengthSquared, 0.0, 1.0);
    const Point3d projectedPoint = start + direction * parameter;
    return {projectedPoint, parameter, (point - projectedPoint).LengthSquared(), true};
}

[[nodiscard]] bool UpdateTrimClosestProjection(
    const Point3d& point,
    const CurveOnSurface& trim,
    BrepFaceProjection3d& best,
    bool onBoundary)
{
    if (!trim.IsValid() || trim.PointCount() < 2)
    {
        return false;
    }

    bool improved = false;
    const std::size_t pointCount = trim.PointCount();
    const std::size_t segmentCount = trim.UvCurve().IsClosed() ? pointCount : pointCount - 1;
    for (std::size_t i = 0; i < segmentCount; ++i)
    {
        const std::size_t j = (i + 1) % pointCount;
        const LineProjection3d projected =
            ProjectPointToSegment3d(point, trim.PointAt(i), trim.PointAt(j), geometry::kDefaultEpsilon);
        if (!best.success || projected.distanceSquared < best.distanceSquared)
        {
            const Point2d uv0 = trim.UvPointAt(i);
            const Point2d uv1 = trim.UvPointAt(j);
            const Point2d uv = uv0 + (uv1 - uv0) * projected.parameter;
            best.success = true;
            best.onTrimmedFace = false;
            best.onBoundary = onBoundary;
            best.point = projected.point;
            best.u = uv.x;
            best.v = uv.y;
            best.distanceSquared = projected.distanceSquared;
            improved = true;
        }
    }

    return improved;
}

[[nodiscard]] BrepEdgeProjection3d RefineCurveProjection(
    const Point3d& point,
    const Curve3d& curve,
    double initialParameter,
    const GeometryTolerance3d& tolerance)
{
    const Intervald range = curve.ParameterRange();
    if (!range.IsValid())
    {
        return {};
    }

    double bestParameter = std::clamp(initialParameter, range.min, range.max);
    Point3d bestPoint = curve.PointAt(bestParameter);
    double bestDistanceSquared = (point - bestPoint).LengthSquared();
    double step = std::max(range.Length() * 0.25, tolerance.parameterEpsilon);

    for (int iteration = 0; iteration < 16; ++iteration)
    {
        bool improved = false;
        for (const double dt : std::array<double, 3>{-step, 0.0, step})
        {
            const double candidateParameter = std::clamp(bestParameter + dt, range.min, range.max);
            const Point3d candidatePoint = curve.PointAt(candidateParameter);
            const double candidateDistanceSquared = (point - candidatePoint).LengthSquared();
            if (candidateDistanceSquared + tolerance.distanceEpsilon * tolerance.distanceEpsilon < bestDistanceSquared)
            {
                bestParameter = candidateParameter;
                bestPoint = candidatePoint;
                bestDistanceSquared = candidateDistanceSquared;
                improved = true;
            }
        }

        if (!improved)
        {
            step *= 0.5;
        }

        if (step <= tolerance.parameterEpsilon)
        {
            break;
        }
    }

    return {true, bestParameter, bestPoint, bestDistanceSquared};
}
} // namespace

LineProjection3d ProjectPointToLine(
    const Point3d& point,
    const Line3d& line,
    const GeometryTolerance3d& tolerance)
{
    if (!line.IsValid(tolerance.distanceEpsilon))
    {
        return LineProjection3d{line.origin, 0.0, (point - line.origin).LengthSquared(), false};
    }

    const double directionLengthSquared = line.direction.LengthSquared();
    const double parameter = Dot(point - line.origin, line.direction) / directionLengthSquared;
    const Point3d projectedPoint = line.PointAt(parameter);
    return LineProjection3d{projectedPoint, parameter, (point - projectedPoint).LengthSquared(), true};
}

PlaneProjection3d ProjectPointToPlane(
    const Point3d& point,
    const Plane& plane,
    const GeometryTolerance3d& tolerance)
{
    if (!plane.IsValid(tolerance.distanceEpsilon))
    {
        return PlaneProjection3d{plane.origin, 0.0, (point - plane.origin).LengthSquared()};
    }

    const Vector3d unitNormal = plane.UnitNormal(tolerance.distanceEpsilon);
    const double signedDistance = Dot(point - plane.origin, unitNormal);
    const Point3d projectedPoint = point - unitNormal * signedDistance;
    return PlaneProjection3d{projectedPoint, signedDistance, signedDistance * signedDistance};
}

SurfaceProjection3d ProjectPointToSurface(
    const Point3d& point,
    const Surface& surface,
    const GeometryTolerance3d& tolerance)
{
    if (!surface.IsValid(tolerance))
    {
        return {};
    }

    const Intervald uRange = surface.URange();
    const Intervald vRange = surface.VRange();
    if (!uRange.IsValid() || !vRange.IsValid())
    {
        return {};
    }

    if (const auto* planeSurface = dynamic_cast<const PlaneSurface*>(&surface))
    {
        const PlaneProjection3d planeProjection =
            ProjectPointToPlane(point, planeSurface->SupportPlane(), tolerance);
        const Vector3d delta = planeProjection.point - planeSurface->SupportPlane().origin;
        const Vector3d uAxis = planeSurface->UAxis();
        const Vector3d vAxis = planeSurface->VAxis();
        const double uDenominator = std::max(uAxis.LengthSquared(), tolerance.parameterEpsilon);
        const double vDenominator = std::max(vAxis.LengthSquared(), tolerance.parameterEpsilon);
        const double u = std::clamp(Dot(delta, uAxis) / uDenominator, uRange.min, uRange.max);
        const double v = std::clamp(Dot(delta, vAxis) / vDenominator, vRange.min, vRange.max);
        const Point3d surfacePoint = planeSurface->PointAt(u, v);
        return {true, surfacePoint, u, v, (point - surfacePoint).LengthSquared()};
    }

    const std::size_t sampleCountU = 9;
    const std::size_t sampleCountV = 9;
    double bestU = uRange.min;
    double bestV = vRange.min;
    double bestDistanceSquared = DistanceSquaredToSurfacePoint(point, surface, bestU, bestV);
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
            const double distanceSquared = DistanceSquaredToSurfacePoint(point, surface, u, v);
            if (distanceSquared < bestDistanceSquared)
            {
                bestU = u;
                bestV = v;
                bestDistanceSquared = distanceSquared;
            }
        }
    }

    return RefineSurfaceProjection(point, surface, bestU, bestV, tolerance);
}

BrepFaceProjection3d ProjectPointToBrepFace(
    const Point3d& point,
    const BrepFace& face,
    const GeometryTolerance3d& tolerance)
{
    BrepFaceProjection3d best{};
    if (!face.IsValid(tolerance) || face.SupportSurface() == nullptr)
    {
        return best;
    }

    const SurfaceProjection3d surfaceProjection =
        ProjectPointToSurface(point, *face.SupportSurface(), tolerance);
    if (surfaceProjection.success)
    {
        Polygon2d polygon{};
        if (BuildPolygonFromBrepFaceTrims(face, polygon))
        {
            const PointContainment2d containment =
                LocatePoint(Point2d{surfaceProjection.u, surfaceProjection.v}, polygon, tolerance.distanceEpsilon);
            if (containment == PointContainment2d::Inside || containment == PointContainment2d::OnBoundary)
            {
                best.success = true;
                best.onTrimmedFace = true;
                best.onBoundary = containment == PointContainment2d::OnBoundary;
                best.point = surfaceProjection.point;
                best.u = surfaceProjection.u;
                best.v = surfaceProjection.v;
                best.distanceSquared = surfaceProjection.distanceSquared;
                return best;
            }
        }
    }

    if (face.OuterTrim().IsValid())
    {
        UpdateTrimClosestProjection(point, face.OuterTrim(), best, true);
    }
    for (const CurveOnSurface& holeTrim : face.HoleTrims())
    {
        UpdateTrimClosestProjection(point, holeTrim, best, true);
    }
    return best;
}

BrepEdgeProjection3d ProjectPointToBrepEdge(
    const Point3d& point,
    const BrepEdge& edge,
    const GeometryTolerance3d& tolerance)
{
    if (!edge.IsValid(tolerance) || edge.Curve() == nullptr)
    {
        return {};
    }

    if (const auto* lineCurve = dynamic_cast<const LineCurve3d*>(edge.Curve()))
    {
        const LineProjection3d lineProjection = ProjectPointToLine(point, lineCurve->SupportLine(), tolerance);
        const Intervald range = lineCurve->ParameterRange();
        const double parameter = std::clamp(lineProjection.parameter, range.min, range.max);
        const Point3d projectedPoint = lineCurve->PointAt(parameter);
        return {true, parameter, projectedPoint, (point - projectedPoint).LengthSquared()};
    }

    const Curve3d& curve = *edge.Curve();
    const Intervald range = curve.ParameterRange();
    if (!range.IsValid())
    {
        return {};
    }

    constexpr std::size_t sampleCount = 17;
    double bestParameter = range.min;
    Point3d bestPoint = curve.PointAt(bestParameter);
    double bestDistanceSquared = (point - bestPoint).LengthSquared();
    for (std::size_t i = 0; i < sampleCount; ++i)
    {
        const double parameter = i + 1 == sampleCount
                                     ? range.max
                                     : range.min + range.Length() * static_cast<double>(i) /
                                                        static_cast<double>(sampleCount - 1);
        const Point3d candidatePoint = curve.PointAt(parameter);
        const double candidateDistanceSquared = (point - candidatePoint).LengthSquared();
        if (candidateDistanceSquared < bestDistanceSquared)
        {
            bestParameter = parameter;
            bestPoint = candidatePoint;
            bestDistanceSquared = candidateDistanceSquared;
        }
    }

    return RefineCurveProjection(point, curve, bestParameter, tolerance);
}

BrepBodyProjection3d ProjectPointToBrepBody(
    const Point3d& point,
    const BrepBody& body,
    const GeometryTolerance3d& tolerance)
{
    BrepBodyProjection3d best{};
    if (!body.IsValid(tolerance))
    {
        return best;
    }

    std::size_t faceIndex = 0;
    for (std::size_t shellIndex = 0; shellIndex < body.ShellCount(); ++shellIndex)
    {
        const BrepShell shell = body.ShellAt(shellIndex);
        for (std::size_t localFaceIndex = 0; localFaceIndex < shell.FaceCount(); ++localFaceIndex, ++faceIndex)
        {
            const BrepFaceProjection3d projected = ProjectPointToBrepFace(point, shell.FaceAt(localFaceIndex), tolerance);
            if (!projected.success)
            {
                continue;
            }

            if (!best.success || projected.distanceSquared < best.projection.distanceSquared)
            {
                best.success = true;
                best.faceIndex = faceIndex;
                best.projection = projected;
            }
        }
    }

    return best;
}

FaceProjection3d ProjectFaceToPolygon2d(const PolyhedronFace3d& face, const GeometryTolerance3d& tolerance)
{
    if (!face.IsValid(tolerance.distanceEpsilon))
    {
        return {};
    }

    const Plane plane = face.SupportPlane();
    const PlaneProjectionBasis basis = BuildPlaneProjectionBasis(plane, tolerance.distanceEpsilon);
    std::vector<Point2d> outerPoints;
    outerPoints.reserve(face.OuterLoop().VertexCount());
    for (const Point3d& vertex : face.OuterLoop().Vertices())
    {
        outerPoints.push_back(ProjectToLocalPlaneCoordinates(vertex, plane, basis));
    }

    std::vector<Polyline2d> holeRings;
    holeRings.reserve(face.HoleCount());
    for (std::size_t i = 0; i < face.HoleCount(); ++i)
    {
        std::vector<Point2d> holePoints;
        const PolyhedronLoop3d hole = face.HoleAt(i);
        holePoints.reserve(hole.VertexCount());
        for (const Point3d& vertex : hole.Vertices())
        {
            holePoints.push_back(ProjectToLocalPlaneCoordinates(vertex, plane, basis));
        }
        holeRings.emplace_back(std::move(holePoints), PolylineClosure::Closed);
    }

    Polygon2d polygon(Polyline2d(std::move(outerPoints), PolylineClosure::Closed), std::move(holeRings));
    if (!polygon.IsValid())
    {
        return {};
    }

    return FaceProjection3d{true, std::move(polygon), plane.origin, basis.u, basis.v};
}
} // namespace geometry::sdk
