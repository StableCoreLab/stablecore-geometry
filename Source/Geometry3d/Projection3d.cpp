#include "Core/Projection.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "Core/Relation.h"
#include "Core/Triangulation.h"
#include "../Detail/ParametricSeeds.h"
#include "../Detail/PolylineRing2d.h"
#include "Brep/PolyhedronBody.h"
#include "Brep/PolyhedronFace3d.h"
#include "Brep/SCBrepBody.h"
#include "Brep/SCBrepEdge.h"
#include "Brep/SCBrepFace.h"
#include "Brep/SCBrepVertex.h"
#include "Geometry3d/SCLineCurve3d.h"
#include "Geometry3d/SCNurbsCurve3d.h"
#include "Geometry3d/SCNurbsSurface.h"
#include "Geometry3d/SCOffsetSurface.h"
#include "Geometry3d/SCPlaneSurface.h"
#include "Geometry3d/TriangleMesh.h"
#include "Support/Epsilon.h"

namespace Geometry
{
    namespace
    {
        struct PlaneProjectionBasis
        {
            SCVector3d u{};
            SCVector3d v{};
        };

        PlaneProjectionBasis BuildPlaneProjectionBasis(const SCPlane& plane, double eps)
        {
            const SCVector3d normal = plane.UnitNormal(eps);
            const SCVector3d axis =
                std::abs(normal.x) <= std::abs(normal.y) && std::abs(normal.x) <= std::abs(normal.z)
                    ? SCVector3d{1.0, 0.0, 0.0}
                    : (std::abs(normal.y) <= std::abs(normal.z) ? SCVector3d{0.0, 1.0, 0.0} : SCVector3d{0.0, 0.0, 1.0});
            const SCVector3d u = Cross(normal, axis).Normalized(eps);
            const SCVector3d v = Cross(normal, u).Normalized(eps);
            return {u, v};
        }

        SCPoint2d ProjectToLocalPlaneCoordinates(const SCPoint3d& point,
                                               const SCPlane& plane,
                                               const PlaneProjectionBasis& basis)
        {
            const SCVector3d delta = point - plane.origin;
            return SCPoint2d{Dot(delta, basis.u), Dot(delta, basis.v)};
        }

        double SignedArea2d(const std::vector<SCPoint2d>& points)
        {
            if (points.size() < 3)
            {
                return 0.0;
            }

            double area = 0.0;
            for (std::size_t i = 0; i < points.size(); ++i)
            {
                const SCPoint2d& current = points[i];
                const SCPoint2d& next = points[(i + 1) % points.size()];
                area += current.x * next.y - next.x * current.y;
            }

            return 0.5 * area;
        }

        void EnsureRingOrientation(std::vector<SCPoint2d>& points, bool counterClockwise)
        {
            if (points.size() < 3)
            {
                return;
            }

            const double signedArea = SignedArea2d(points);
            if ((counterClockwise && signedArea < 0.0) || (!counterClockwise && signedArea > 0.0))
            {
                std::reverse(points.begin(), points.end());
            }
        }

        double DistanceSquaredToSurfacePoint(const SCPoint3d& point, const ISCSurface& surface, double u, double v)
        {
            return (point - surface.PointAt(u, v)).LengthSquared();
        }

        SCSurfaceProjection3d RefineSurfaceProjection(
            const SCPoint3d& point, const ISCSurface& surface, double u, double v, const SCGeometryTolerance3d& tolerance)
        {
            const SCIntervald uRange = surface.URange();
            const SCIntervald vRange = surface.VRange();
            double bestU = std::clamp(u, uRange.min, uRange.max);
            double bestV = std::clamp(v, vRange.min, vRange.max);
            double bestDistanceSquared = DistanceSquaredToSurfacePoint(point, surface, bestU, bestV);
            bool usePointOnlyRefinement = dynamic_cast<const SCOffsetSurface*>(&surface) != nullptr;

            for (int iteration = 0; iteration < 32 && !usePointOnlyRefinement; ++iteration)
            {
                const SCSurfaceEval3d eval = surface.Evaluate(bestU, bestV, 1);
                if (eval.derivativeOrder < 1 || !eval.point.IsValid() ||
                    !eval.derivativeU.IsValid() || !eval.derivativeV.IsValid() ||
                    eval.normal.Length() <= tolerance.distanceEpsilon)
                {
                    usePointOnlyRefinement = true;
                    break;
                }

                const SCVector3d residual = eval.point - point;
                const double uu = Dot(eval.derivativeU, eval.derivativeU);
                const double uv = Dot(eval.derivativeU, eval.derivativeV);
                const double vv = Dot(eval.derivativeV, eval.derivativeV);
                const double uGradient = Dot(eval.derivativeU, residual);
                const double vGradient = Dot(eval.derivativeV, residual);
                const double determinantScale = uu * vv;
                const double determinant = determinantScale - uv * uv;
                if (!std::isfinite(determinant) ||
                    determinant <= std::max(tolerance.parameterEpsilon, std::numeric_limits<double>::epsilon()) *
                                       determinantScale)
                {
                    usePointOnlyRefinement = true;
                    break;
                }

                const double deltaU = (-uGradient * vv + vGradient * uv) / determinant;
                const double deltaV = (-vGradient * uu + uGradient * uv) / determinant;
                if (!std::isfinite(deltaU) || !std::isfinite(deltaV))
                {
                    usePointOnlyRefinement = true;
                    break;
                }

                bool improved = false;
                for (double scale = 1.0; scale >= 1.0 / 128.0; scale *= 0.5)
                {
                    const double candidateU = std::clamp(bestU + scale * deltaU, uRange.min, uRange.max);
                    const double candidateV = std::clamp(bestV + scale * deltaV, vRange.min, vRange.max);
                    const double candidateDistanceSquared =
                        DistanceSquaredToSurfacePoint(point, surface, candidateU, candidateV);
                    if (candidateDistanceSquared < bestDistanceSquared)
                    {
                        bestU = candidateU;
                        bestV = candidateV;
                        bestDistanceSquared = candidateDistanceSquared;
                        improved = true;
                        break;
                    }
                }
                if (!improved)
                {
                    usePointOnlyRefinement = true;
                    break;
                }
            }

            if (usePointOnlyRefinement)
            {
                double stepU = std::max(uRange.Length() * 0.25, tolerance.parameterEpsilon);
                double stepV = std::max(vRange.Length() * 0.25, tolerance.parameterEpsilon);
                for (int iteration = 0; iteration < 64; ++iteration)
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
                            if (candidateDistanceSquared < bestDistanceSquared)
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
            }

            return {true, surface.PointAt(bestU, bestV), bestU, bestV, bestDistanceSquared};
        }

        bool TryBuildPolygonFromBrepFaceTrims(const SCBrepFace& face, SCPolygon2d& polygon)
        {
            if (face.OuterTrim().SupportSurface() == nullptr || !face.OuterTrim().IsValid())
            {
                return false;
            }

            std::vector<SCPolyline2d> holes;
            holes.reserve(face.HoleTrims().size());
            for (const SCCurveOnSurface& trim : face.HoleTrims())
            {
                if (!trim.IsValid())
                {
                    return false;
                }
                holes.push_back(trim.UvCurve());
            }

            polygon = SCPolygon2d(face.OuterTrim().UvCurve(), std::move(holes));
            return polygon.IsValid();
        }

        SCLineProjection3d ProjectPointToSegment3d(const SCPoint3d& point,
                                                   const SCPoint3d& start,
                                                   const SCPoint3d& end,
                                                   double eps)
        {
            const SCVector3d direction = end - start;
            const double lengthSquared = direction.LengthSquared();
            if (lengthSquared <= eps * eps)
            {
                return {start, 0.0, (point - start).LengthSquared(), true};
            }

            const double parameter = std::clamp(Dot(point - start, direction) / lengthSquared, 0.0, 1.0);
            const SCPoint3d projectedPoint = start + direction * parameter;
            return {projectedPoint, parameter, (point - projectedPoint).LengthSquared(), true};
        }

        bool UpdateClosestTrimProjection(const SCPoint3d& point,
                                         const SCCurveOnSurface& trim,
                                         SCBrepFaceProjection3d& best,
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
                const SCLineProjection3d projected = ProjectPointToSegment3d(
                    point, trim.PointAt(i), trim.PointAt(j), Geometry::kProjectionDefaultEpsilon);
                if (!best.success || projected.distanceSquared < best.distanceSquared)
                {
                    const SCPoint2d uv0 = trim.UvPointAt(i);
                    const SCPoint2d uv1 = trim.UvPointAt(j);
                    const SCPoint2d uv = uv0 + (uv1 - uv0) * projected.parameter;
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

        SCBrepEdgeProjection3d RefineCurveProjection(const SCPoint3d& point,
                                                   const ISCCurve3d& curve,
                                                   double initialParameter,
                                                   const SCGeometryTolerance3d& tolerance)
        {
            const SCIntervald range = curve.ParameterRange();
            if (!range.IsValid())
            {
                return {};
            }

            double bestParameter = std::clamp(initialParameter, range.min, range.max);
            SCPoint3d bestPoint = curve.PointAt(bestParameter);
            double bestDistanceSquared = (point - bestPoint).LengthSquared();
            bool usePointOnlyRefinement = false;

            for (int iteration = 0; iteration < 32; ++iteration)
            {
                const SCCurveEval3d eval = curve.Evaluate(bestParameter, 2);
                if (!eval.IsValid() || eval.derivativeOrder < 1)
                {
                    usePointOnlyRefinement = true;
                    break;
                }

                const SCVector3d residual = eval.point - point;
                const double gradient = Dot(residual, eval.firstDerivative);
                const double tangentSquared = Dot(eval.firstDerivative, eval.firstDerivative);
                const double curvatureTerm = eval.derivativeOrder >= 2
                                                 ? Dot(residual, eval.secondDerivative)
                                                 : 0.0;
                const double hessian = tangentSquared + curvatureTerm;
                const double hessianScale = tangentSquared + std::abs(curvatureTerm);
                const double denominator = hessian > std::numeric_limits<double>::epsilon() * hessianScale
                                               ? hessian
                                               : tangentSquared;
                if (!std::isfinite(denominator) || denominator <= 0.0)
                {
                    usePointOnlyRefinement = true;
                    break;
                }

                const double delta = -gradient / denominator;
                if (!std::isfinite(delta))
                {
                    usePointOnlyRefinement = true;
                    break;
                }

                bool improved = false;
                for (double scale = 1.0; scale >= 1.0 / 128.0; scale *= 0.5)
                {
                    const double candidateParameter =
                        std::clamp(bestParameter + scale * delta, range.min, range.max);
                    const SCPoint3d candidatePoint = curve.PointAt(candidateParameter);
                    const double candidateDistanceSquared = (point - candidatePoint).LengthSquared();
                    if (candidateDistanceSquared < bestDistanceSquared)
                    {
                        bestParameter = candidateParameter;
                        bestPoint = candidatePoint;
                        bestDistanceSquared = candidateDistanceSquared;
                        improved = true;
                        break;
                    }
                }
                if (!improved)
                {
                    usePointOnlyRefinement = true;
                    break;
                }
            }

            if (usePointOnlyRefinement)
            {
                double step = std::max(range.Length() * 0.25, tolerance.parameterEpsilon);
                for (int iteration = 0; iteration < 64; ++iteration)
                {
                    bool improved = false;
                    for (const double dt : std::array<double, 2>{-step, step})
                    {
                        const double candidateParameter = std::clamp(bestParameter + dt, range.min, range.max);
                        const SCPoint3d candidatePoint = curve.PointAt(candidateParameter);
                        const double candidateDistanceSquared = (point - candidatePoint).LengthSquared();
                        if (candidateDistanceSquared < bestDistanceSquared)
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
            }

            return {true, bestParameter, bestPoint, bestDistanceSquared};
        }

        SCPoint3d ClosestPointOnTriangle(const SCPoint3d& point, const SCTriangle3d& triangle, double eps)
        {
            const SCVector3d ab = triangle.b - triangle.a;
            const SCVector3d ac = triangle.c - triangle.a;
            const SCVector3d ap = point - triangle.a;

            const double d1 = Dot(ab, ap);
            const double d2 = Dot(ac, ap);
            if (d1 <= 0.0 && d2 <= 0.0)
            {
                return triangle.a;
            }

            const SCVector3d bp = point - triangle.b;
            const double d3 = Dot(ab, bp);
            const double d4 = Dot(ac, bp);
            if (d3 >= 0.0 && d4 <= d3)
            {
                return triangle.b;
            }

            const double vc = d1 * d4 - d3 * d2;
            if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
            {
                const double v = d1 / (d1 - d3);
                return triangle.a + ab * v;
            }

            const SCVector3d cp = point - triangle.c;
            const double d5 = Dot(ab, cp);
            const double d6 = Dot(ac, cp);
            if (d6 >= 0.0 && d5 <= d6)
            {
                return triangle.c;
            }

            const double vb = d5 * d2 - d1 * d6;
            if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
            {
                const double w = d2 / (d2 - d6);
                return triangle.a + ac * w;
            }

            const double va = d3 * d6 - d5 * d4;
            if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0)
            {
                const SCVector3d bc = triangle.c - triangle.b;
                const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                return triangle.b + bc * w;
            }

            const SCVector3d normal = Cross(ab, ac);
            const double normalLengthSquared = normal.LengthSquared();
            if (normalLengthSquared <= eps * eps)
            {
                return triangle.a;
            }

            const double distance = Dot(point - triangle.a, normal) / normalLengthSquared;
            return point - normal * distance;
        }

        template <typename BodyProjection, typename VisitFaces>
        [[nodiscard]] BodyProjection FindNearestBodyFaceProjection(VisitFaces&& visitFaces)
        {
            BodyProjection best{};
            visitFaces([&](std::size_t faceIndex, auto projection) {
                if (!projection.success ||
                    (best.success && projection.distanceSquared >= best.projection.distanceSquared))
                {
                    return;
                }
                best.success = true;
                best.faceIndex = faceIndex;
                best.projection = std::move(projection);
            });
            return best;
        }
    }  // namespace

    SCLineProjection3d ProjectPointToLine(const SCPoint3d& point, const SCLine3d& line, const SCGeometryTolerance3d& tolerance)
    {
        if (!line.IsValid(tolerance.distanceEpsilon))
        {
            return SCLineProjection3d{line.origin, 0.0, (point - line.origin).LengthSquared(), false};
        }

        const double directionLengthSquared = line.direction.LengthSquared();
        const double parameter = Dot(point - line.origin, line.direction) / directionLengthSquared;
        const SCPoint3d projectedPoint = line.PointAt(parameter);
        return SCLineProjection3d{projectedPoint, parameter, (point - projectedPoint).LengthSquared(), true};
    }

    SCCurveProjection3d ProjectPointToCurve(const SCPoint3d& point,
                                          const ISCCurve3d& curve,
                                          const SCGeometryTolerance3d& tolerance)
    {
        if (!curve.IsValid(tolerance))
        {
            return {};
        }

        if (const auto* lineCurve = dynamic_cast<const SCLineCurve3d*>(&curve))
        {
            const SCLineProjection3d lineProjection = ProjectPointToLine(point, lineCurve->Line(), tolerance);
            const SCIntervald range = lineCurve->ParameterRange();
            const double parameter = std::clamp(lineProjection.parameter, range.min, range.max);
            const SCPoint3d projectedPoint = lineCurve->PointAt(parameter);
            return {true, projectedPoint, parameter, (point - projectedPoint).LengthSquared()};
        }

        const SCIntervald range = curve.ParameterRange();
        if (!range.IsValid())
        {
            return {};
        }

        std::vector<double> parameterBoundaries{range.min, range.max};
        if (const auto* nurbsCurve = dynamic_cast<const SCNurbsCurve3d*>(&curve))
        {
            auto nurbsBoundaries = Detail::KnotSpanBoundaries(
                range, nurbsCurve->Knots(), nurbsCurve->Degree(), nurbsCurve->ControlPoints().size());
            if (nurbsBoundaries.size() >= 2)
            {
                parameterBoundaries = std::move(nurbsBoundaries);
            }
        }
        const SCBox3d curveBounds = curve.Bounds();
        const double curveScale = curveBounds.IsValid()
                                      ? (curveBounds.MaxPoint() - curveBounds.MinPoint()).Length()
                                      : tolerance.distanceEpsilon;
        const std::vector<double> parameterSeeds = Detail::AdaptiveCurveParameterSeeds(
            curve, std::move(parameterBoundaries), std::max(tolerance.distanceEpsilon * 4.0, curveScale * 1e-3));

        SCBrepEdgeProjection3d best{};
        for (const double parameter : parameterSeeds)
        {
            const SCBrepEdgeProjection3d refined = RefineCurveProjection(point, curve, parameter, tolerance);
            if (refined.success && (!best.success || refined.distanceSquared < best.distanceSquared))
            {
                best = refined;
            }
        }

        return {best.success, best.point, best.parameter, best.distanceSquared};
    }

    SCCurveOnSurfaceProjection3d ProjectPointToCurveOnSurface(const SCPoint3d& point,
                                                            const SCCurveOnSurface& curveOnSurface,
                                                            const SCGeometryTolerance3d& tolerance)
    {
        SCCurveOnSurfaceProjection3d best{};
        if (!curveOnSurface.IsValid(tolerance) || curveOnSurface.PointCount() < 2)
        {
            return best;
        }

        const std::size_t pointCount = curveOnSurface.PointCount();
        const std::size_t segmentCount = curveOnSurface.UvCurve().IsClosed() ? pointCount : pointCount - 1;
        for (std::size_t i = 0; i < segmentCount; ++i)
        {
            const std::size_t j = (i + 1) % pointCount;
            const SCLineProjection3d projected = ProjectPointToSegment3d(
                point, curveOnSurface.PointAt(i), curveOnSurface.PointAt(j), tolerance.distanceEpsilon);
            if (!best.success || projected.distanceSquared < best.distanceSquared)
            {
                const SCPoint2d uv0 = curveOnSurface.UvPointAt(i);
                const SCPoint2d uv1 = curveOnSurface.UvPointAt(j);
                const SCPoint2d uv = uv0 + (uv1 - uv0) * projected.parameter;
                best.success = true;
                best.point = projected.point;
                best.segmentIndex = i;
                best.segmentParameter = projected.parameter;
                best.uv = uv;
                best.distanceSquared = projected.distanceSquared;
            }
        }

        return best;
    }

    SCPlaneProjection3d ProjectPointToPlane(const SCPoint3d& point,
                                          const SCPlane& plane,
                                          const SCGeometryTolerance3d& tolerance)
    {
        if (!plane.IsValid(tolerance.distanceEpsilon))
        {
            return SCPlaneProjection3d{plane.origin, 0.0, (point - plane.origin).LengthSquared()};
        }

        const SCVector3d unitNormal = plane.UnitNormal(tolerance.distanceEpsilon);
        const double signedDistance = Dot(point - plane.origin, unitNormal);
        const SCPoint3d projectedPoint = point - unitNormal * signedDistance;
        return SCPlaneProjection3d{projectedPoint, signedDistance, signedDistance * signedDistance};
    }

    SCSurfaceProjection3d ProjectPointToSurface(const SCPoint3d& point,
                                              const ISCSurface& surface,
                                              const SCGeometryTolerance3d& tolerance)
    {
        if (!surface.IsValid(tolerance))
        {
            return {};
        }

        const SCIntervald uRange = surface.URange();
        const SCIntervald vRange = surface.VRange();
        if (!uRange.IsValid() || !vRange.IsValid())
        {
            return {};
        }

        if (const auto* planeSurface = dynamic_cast<const SCPlaneSurface*>(&surface))
        {
            const SCPlaneProjection3d planeProjection =
                ProjectPointToPlane(point, planeSurface->SupportPlane(), tolerance);
            const SCVector3d delta = planeProjection.point - planeSurface->SupportPlane().origin;
            const SCVector3d uAxis = planeSurface->UAxis();
            const SCVector3d vAxis = planeSurface->VAxis();
            const double uDenominator = std::max(uAxis.LengthSquared(), tolerance.parameterEpsilon);
            const double vDenominator = std::max(vAxis.LengthSquared(), tolerance.parameterEpsilon);
            const double u = std::clamp(Dot(delta, uAxis) / uDenominator, uRange.min, uRange.max);
            const double v = std::clamp(Dot(delta, vAxis) / vDenominator, vRange.min, vRange.max);
            const SCPoint3d surfacePoint = planeSurface->PointAt(u, v);
            return {true, surfacePoint, u, v, (point - surfacePoint).LengthSquared()};
        }

        std::vector<double> uBoundaries{uRange.min, uRange.max};
        std::vector<double> vBoundaries{vRange.min, vRange.max};
        if (const auto* nurbsSurface = dynamic_cast<const SCNurbsSurface*>(&surface))
        {
            auto nurbsUBoundaries = Detail::KnotSpanBoundaries(
                uRange, nurbsSurface->KnotsU(), nurbsSurface->DegreeU(), nurbsSurface->ControlPointCountU());
            auto nurbsVBoundaries = Detail::KnotSpanBoundaries(
                vRange, nurbsSurface->KnotsV(), nurbsSurface->DegreeV(), nurbsSurface->ControlPointCountV());
            if (nurbsUBoundaries.size() >= 2 && nurbsVBoundaries.size() >= 2)
            {
                uBoundaries = std::move(nurbsUBoundaries);
                vBoundaries = std::move(nurbsVBoundaries);
            }
        }
        const SCBox3d surfaceBounds = surface.Bounds();
        const double surfaceScale = surfaceBounds.IsValid()
                                        ? (surfaceBounds.MaxPoint() - surfaceBounds.MinPoint()).Length()
                                        : tolerance.distanceEpsilon;
        const std::vector<Detail::SurfaceParameterSeed> surfaceSeeds = Detail::AdaptiveSurfaceParameterSeeds(
            surface, std::move(uBoundaries), std::move(vBoundaries),
            std::max(tolerance.distanceEpsilon * 4.0, surfaceScale * 1e-2));

        SCSurfaceProjection3d best{};
        if (surfaceSeeds.empty())
        {
            return best;
        }

        bool pointOnlyRefinement = dynamic_cast<const SCOffsetSurface*>(&surface) != nullptr;
        if (!pointOnlyRefinement)
        {
            const SCSurfaceEval3d initialEval = surface.Evaluate(surfaceSeeds.front().u, surfaceSeeds.front().v, 1);
            pointOnlyRefinement = initialEval.derivativeOrder < 1 || !initialEval.point.IsValid() ||
                                  !initialEval.derivativeU.IsValid() || !initialEval.derivativeV.IsValid() ||
                                  initialEval.normal.Length() <= tolerance.distanceEpsilon;
        }
        const auto considerSeed = [&](const Detail::SurfaceParameterSeed& seed) {
            const SCSurfaceProjection3d refined = RefineSurfaceProjection(point, surface, seed.u, seed.v, tolerance);
            if (refined.success && (!best.success || refined.distanceSquared < best.distanceSquared))
            {
                best = refined;
            }
        };

        if (pointOnlyRefinement)
        {
            std::vector<std::pair<double, std::size_t>> rankedSeeds;
            rankedSeeds.reserve(surfaceSeeds.size());
            for (std::size_t index = 0; index < surfaceSeeds.size(); ++index)
            {
                const auto& seed = surfaceSeeds[index];
                const double distanceSquared = DistanceSquaredToSurfacePoint(point, surface, seed.u, seed.v);
                if (std::isfinite(distanceSquared))
                {
                    rankedSeeds.emplace_back(distanceSquared, index);
                }
            }
            constexpr std::size_t kMaxPointOnlyRefinements = 16;
            const std::size_t refinementCount = std::min(kMaxPointOnlyRefinements, rankedSeeds.size());
            std::partial_sort(rankedSeeds.begin(), rankedSeeds.begin() + refinementCount, rankedSeeds.end());
            for (std::size_t i = 0; i < refinementCount; ++i)
            {
                considerSeed(surfaceSeeds[rankedSeeds[i].second]);
            }
        }
        else
        {
            for (const Detail::SurfaceParameterSeed& seed : surfaceSeeds)
            {
                considerSeed(seed);
            }
        }

        return best;
    }

    SCBrepFaceProjection3d ProjectPointToBrepFace(const SCPoint3d& point,
                                                const SCBrepFace& face,
                                                const SCGeometryTolerance3d& tolerance)
    {
        SCBrepFaceProjection3d best{};
        if (!face.IsValid(tolerance) || face.SupportSurface() == nullptr)
        {
            return best;
        }

        const SCSurfaceProjection3d surfaceProjection = ProjectPointToSurface(point, *face.SupportSurface(), tolerance);
        if (surfaceProjection.success)
        {
            SCPolygon2d polygon{};
            if (TryBuildPolygonFromBrepFaceTrims(face, polygon))
            {
                const SCPointContainment2d containment =
                    LocatePoint(SCPoint2d{surfaceProjection.u, surfaceProjection.v}, polygon, tolerance.distanceEpsilon);
                if (containment == SCPointContainment2d::Inside || containment == SCPointContainment2d::OnBoundary)
                {
                    best.success = true;
                    best.onTrimmedFace = true;
                    best.onBoundary = containment == SCPointContainment2d::OnBoundary;
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
            UpdateClosestTrimProjection(point, face.OuterTrim(), best, true);
        }
        for (const SCCurveOnSurface& holeTrim : face.HoleTrims())
        {
            UpdateClosestTrimProjection(point, holeTrim, best, true);
        }
        return best;
    }

    SCBrepEdgeProjection3d ProjectPointToBrepEdge(const SCPoint3d& point,
                                                const SCBrepEdge& edge,
                                                const SCGeometryTolerance3d& tolerance)
    {
        if (!edge.IsValid(tolerance) || edge.Curve() == nullptr)
        {
            return {};
        }

        const SCCurveProjection3d curveProjection = ProjectPointToCurve(point, *edge.Curve(), tolerance);
        return {curveProjection.success,
                curveProjection.parameter,
                curveProjection.point,
                curveProjection.distanceSquared};
    }

    SCBrepVertexProjection3d ProjectPointToBrepVertex(const SCPoint3d& point,
                                                    const SCBrepVertex& vertex,
                                                    const SCGeometryTolerance3d&)
    {
        if (!vertex.IsValid())
        {
            return {};
        }

        const SCPoint3d vertexPoint = vertex.Point();
        return {true, vertexPoint, (point - vertexPoint).LengthSquared()};
    }

    SCBrepBodyProjection3d ProjectPointToBrepBody(const SCPoint3d& point,
                                                const SCBrepBody& body,
                                                const SCGeometryTolerance3d& tolerance)
    {
        if (!body.IsValid(tolerance))
        {
            return {};
        }
        return FindNearestBodyFaceProjection<SCBrepBodyProjection3d>([&](auto&& consider) {
            std::size_t faceIndex = 0;
            for (std::size_t shellIndex = 0; shellIndex < body.ShellCount(); ++shellIndex)
            {
                const SCBrepShell& shell = body.ShellAtRef(shellIndex);
                for (std::size_t localFaceIndex = 0; localFaceIndex < shell.FaceCount(); ++localFaceIndex, ++faceIndex)
                {
                    consider(faceIndex, ProjectPointToBrepFace(point, shell.FaceAtRef(localFaceIndex), tolerance));
                }
            }
        });
    }

    SCPolyhedronFaceProjection3d ProjectPointToPolyhedronFace(const SCPoint3d& point,
                                                            const PolyhedronFace3d& face,
                                                            const SCGeometryTolerance3d& tolerance)
    {
        SCPolyhedronFaceProjection3d best{};
        if (!face.IsValid(tolerance.distanceEpsilon))
        {
            return best;
        }

        const SCFaceProjection3d faceProjection = ProjectFaceToPolygon2d(face, tolerance);
        if (!faceProjection.success)
        {
            return best;
        }

        const SCPlaneProjection3d planeProjection = ProjectPointToPlane(point, face.SupportPlane(), tolerance);
        const SCPoint2d uv = ProjectToLocalPlaneCoordinates(
            planeProjection.point, face.SupportPlane(), {faceProjection.uAxis, faceProjection.vAxis});
        const SCPointContainment2d containment = LocatePoint(uv, faceProjection.polygon, tolerance.distanceEpsilon);
        if (containment == SCPointContainment2d::Inside || containment == SCPointContainment2d::OnBoundary)
        {
            best.success = true;
            best.onFace = true;
            best.onBoundary = containment == SCPointContainment2d::OnBoundary;
            best.point = planeProjection.point;
            best.u = uv.x;
            best.v = uv.y;
            best.distanceSquared = planeProjection.distanceSquared;
            return best;
        }

        auto updateLoop = [&](const PolyhedronLoop3d& loop) {
            const auto& vertices = loop.Vertices();
            for (std::size_t i = 0; i < vertices.size(); ++i)
            {
                const SCPoint3d& start = vertices[i];
                const SCPoint3d& end = vertices[(i + 1) % vertices.size()];
                const SCLineProjection3d projected =
                    ProjectPointToSegment3d(point, start, end, tolerance.distanceEpsilon);
                if (!best.success || projected.distanceSquared < best.distanceSquared)
                {
                    best.success = true;
                    best.onFace = false;
                    best.onBoundary = true;
                    best.point = projected.point;
                    best.distanceSquared = projected.distanceSquared;
                    const SCPoint2d uv0 = ProjectToLocalPlaneCoordinates(
                        start, face.SupportPlane(), {faceProjection.uAxis, faceProjection.vAxis});
                    const SCPoint2d uv1 = ProjectToLocalPlaneCoordinates(
                        end, face.SupportPlane(), {faceProjection.uAxis, faceProjection.vAxis});
                    const SCPoint2d edgeUv = uv0 + (uv1 - uv0) * projected.parameter;
                    best.u = edgeUv.x;
                    best.v = edgeUv.y;
                }
            }
        };

        updateLoop(face.OuterLoopRef());
        for (std::size_t i = 0; i < face.HoleCount(); ++i)
        {
            updateLoop(face.HoleAtRef(i));
        }
        return best;
    }

    SCPolyhedronBodyProjection3d ProjectPointToPolyhedronBody(const SCPoint3d& point,
                                                            const PolyhedronBody& body,
                                                            const SCGeometryTolerance3d& tolerance)
    {
        if (!body.IsValid(tolerance))
        {
            return {};
        }
        return FindNearestBodyFaceProjection<SCPolyhedronBodyProjection3d>([&](auto&& consider) {
            for (std::size_t faceIndex = 0; faceIndex < body.FaceCount(); ++faceIndex)
            {
                consider(faceIndex, ProjectPointToPolyhedronFace(point, body.FaceAtRef(faceIndex), tolerance));
            }
        });
    }

    SCTriangleMeshProjection3d ProjectPointToTriangleMesh(const SCPoint3d& point,
                                                        const TriangleMesh& mesh,
                                                        const SCGeometryTolerance3d& tolerance)
    {
        SCTriangleMeshProjection3d best{};
        if (!mesh.IsValid(tolerance.distanceEpsilon))
        {
            return best;
        }

        for (std::size_t triangleIndex = 0; triangleIndex < mesh.TriangleCount(); ++triangleIndex)
        {
            const SCTriangle3d triangle = mesh.TriangleAt(triangleIndex);
            const SCPoint3d projectedPoint = ClosestPointOnTriangle(point, triangle, tolerance.distanceEpsilon);
            const double distanceSquared = (point - projectedPoint).LengthSquared();
            if (!best.success || distanceSquared < best.distanceSquared)
            {
                best.success = true;
                best.triangleIndex = triangleIndex;
                best.point = projectedPoint;
                best.distanceSquared = distanceSquared;
            }
        }

        return best;
    }

    SCFaceProjection3d ProjectFaceToPolygon2d(const PolyhedronFace3d& face, const SCGeometryTolerance3d& tolerance)
    {
        if (!face.IsValid(tolerance.distanceEpsilon))
        {
            return {};
        }

        const SCPlane plane = face.SupportPlane();
        const PlaneProjectionBasis basis = BuildPlaneProjectionBasis(plane, tolerance.distanceEpsilon);
        const PolyhedronLoop3d& outerLoop = face.OuterLoopRef();
        std::vector<SCPoint2d> outerPoints;
        outerPoints.reserve(outerLoop.VertexCount());
        for (const SCPoint3d& vertex : outerLoop.Vertices())
        {
            outerPoints.push_back(ProjectToLocalPlaneCoordinates(vertex, plane, basis));
        }
        EnsureRingOrientation(outerPoints, true);

        std::vector<SCPolyline2d> holeRings;
        holeRings.reserve(face.HoleCount());
        for (std::size_t i = 0; i < face.HoleCount(); ++i)
        {
            std::vector<SCPoint2d> holePoints;
            const PolyhedronLoop3d& hole = face.HoleAtRef(i);
            holePoints.reserve(hole.VertexCount());
            for (const SCPoint3d& vertex : hole.Vertices())
            {
                holePoints.push_back(ProjectToLocalPlaneCoordinates(vertex, plane, basis));
            }
            EnsureRingOrientation(holePoints, false);
            holeRings.push_back(Detail::MakeVertexPreservingRing(holePoints));
        }

        SCPolygon2d polygon(Detail::MakeVertexPreservingRing(outerPoints), std::move(holeRings));
        if (!Triangulate(polygon, tolerance.distanceEpsilon))
        {
            return {};
        }

        return SCFaceProjection3d{true, std::move(polygon), plane.origin, basis.u, basis.v};
    }
}  // namespace Geometry
