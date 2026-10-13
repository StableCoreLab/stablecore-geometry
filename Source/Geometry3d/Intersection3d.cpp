#include "Core/Intersection.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <set>
#include <type_traits>
#include <utility>
#include <vector>

#include "Core/Projection.h"
#include "Core/Relation.h"
#include "Brep/PolyhedronBody.h"
#include "Brep/PolyhedronFace3d.h"
#include "Brep/SCBrepBody.h"
#include "Brep/SCBrepEdge.h"
#include "Brep/SCBrepFace.h"
#include "Brep/SCBrepVertex.h"
#include "Geometry3d/SCLineCurve3d.h"
#include "Geometry3d/SCNurbsCurve3d.h"
#include "Geometry3d/SCNurbsSurface.h"
#include "Geometry3d/SCPlaneSurface.h"
#include "Geometry3d/TriangleMesh.h"
#include "Support/Epsilon.h"
#include "../Detail/ParametricSeeds.h"

namespace Geometry
{
    namespace
    {
        [[nodiscard]] double SurfaceResidualSquared(
            const SCLine3d& line, double t, const ISCSurface& surface, double u, double v)
        {
            return (line.PointAt(t) - surface.PointAt(u, v)).LengthSquared();
        }

        [[nodiscard]] bool TryBuildSupportPlane(const SCNurbsSurface& surface,
                                                const SCGeometryTolerance3d& tolerance,
                                                SCPlane& plane)
        {
            if (!surface.IsValid(tolerance))
            {
                return false;
            }

            const SCIntervald uRange = surface.URange();
            const SCIntervald vRange = surface.VRange();
            const double u = 0.5 * (uRange.min + uRange.max);
            const double v = 0.5 * (vRange.min + vRange.max);
            const SCSurfaceEval3d eval = surface.Evaluate(u, v, 1);
            if (!eval.IsValid() || eval.normal.Length() <= tolerance.distanceEpsilon)
            {
                return false;
            }

            plane = SCPlane::FromPointAndNormal(eval.point, eval.normal);
            if (!plane.IsValid(tolerance.distanceEpsilon))
            {
                return false;
            }

            for (const SCPoint3d& controlPoint : surface.ControlPoints())
            {
                if (std::abs(plane.SignedDistanceTo(controlPoint, tolerance.distanceEpsilon)) >
                    tolerance.distanceEpsilon)
                {
                    return false;
                }
            }

            return true;
        }

        [[nodiscard]] bool TryProjectPointToAffinePlanarNurbs(const SCPoint3d& point,
                                                              const SCNurbsSurface& surface,
                                                              const SCGeometryTolerance3d& tolerance,
                                                              double& u,
                                                              double& v,
                                                              SCPoint3d& surfacePoint)
        {
            if (surface.DegreeU() != 1 || surface.DegreeV() != 1 || surface.ControlPointCountU() != 2 ||
                surface.ControlPointCountV() != 2)
            {
                return false;
            }

            const auto& controlPoints = surface.ControlPoints();
            if (controlPoints.size() != 4)
            {
                return false;
            }

            const SCPoint3d& p00 = controlPoints[0];
            const SCPoint3d& p10 = controlPoints[1];
            const SCPoint3d& p01 = controlPoints[2];
            const SCPoint3d& p11 = controlPoints[3];
            const SCPoint3d affineCorner = p10 + (p01 - p00);
            if (!affineCorner.AlmostEquals(p11, tolerance.distanceEpsilon))
            {
                return false;
            }

            const SCVector3d uDirection = p10 - p00;
            const SCVector3d vDirection = p01 - p00;
            const SCVector3d delta = point - p00;
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

            const SCIntervald uRange = surface.URange();
            const SCIntervald vRange = surface.VRange();
            u = uRange.min + uRange.Length() * std::clamp(normalizedU, 0.0, 1.0);
            v = vRange.min + vRange.Length() * std::clamp(normalizedV, 0.0, 1.0);
            surfacePoint = surface.PointAt(u, v);
            return surfacePoint.AlmostEquals(point, tolerance.distanceEpsilon);
        }

        [[nodiscard]] SCLineSurfaceIntersection3d TryRefineLineSurfaceNewton(const SCLine3d& line,
                                                                             const ISCSurface& surface,
                                                                             double initialT,
                                                                             double initialU,
                                                                             double initialV,
                                                                             const SCGeometryTolerance3d& tolerance)
        {
            const SCIntervald uRange = surface.URange();
            const SCIntervald vRange = surface.VRange();
            double t = initialT;
            double u = std::clamp(initialU, uRange.min, uRange.max);
            double v = std::clamp(initialV, vRange.min, vRange.max);
            const double distanceEpsilonSquared = tolerance.distanceEpsilon * tolerance.distanceEpsilon;

            for (int iteration = 0; iteration < 32; ++iteration)
            {
                const SCPoint3d linePoint = line.PointAt(t);
                const SCSurfaceEval3d surfaceEval = surface.Evaluate(u, v, 1);
                if (surfaceEval.derivativeOrder < 1 || !surfaceEval.point.IsValid() ||
                    !surfaceEval.derivativeU.IsValid() || !surfaceEval.derivativeV.IsValid())
                {
                    return {};
                }

                const SCVector3d residual = linePoint - surfaceEval.point;
                if (residual.LengthSquared() <= distanceEpsilonSquared)
                {
                    return {true, false, false, t, u, v, surfaceEval.point};
                }

                const SCVector3d columnT = line.direction;
                const SCVector3d columnU = -surfaceEval.derivativeU;
                const SCVector3d columnV = -surfaceEval.derivativeV;
                const double determinant = Dot(columnT, Cross(columnU, columnV));
                const double determinantScale = columnT.Length() * columnU.Length() * columnV.Length();
                if (!std::isfinite(determinant) || !std::isfinite(determinantScale) ||
                    std::abs(determinant) <=
                        std::max(tolerance.parameterEpsilon, std::numeric_limits<double>::epsilon()) *
                            determinantScale)
                {
                    return {};
                }

                const SCVector3d rhs = -residual;
                const double deltaT = Dot(rhs, Cross(columnU, columnV)) / determinant;
                const double deltaU = Dot(columnT, Cross(rhs, columnV)) / determinant;
                const double deltaV = Dot(columnT, Cross(columnU, rhs)) / determinant;
                if (!std::isfinite(deltaT) || !std::isfinite(deltaU) || !std::isfinite(deltaV))
                {
                    return {};
                }

                bool accepted = false;
                for (double scale = 1.0; scale >= 1.0 / 128.0; scale *= 0.5)
                {
                    const double candidateT = t + scale * deltaT;
                    const double candidateU = std::clamp(u + scale * deltaU, uRange.min, uRange.max);
                    const double candidateV = std::clamp(v + scale * deltaV, vRange.min, vRange.max);
                    const double candidateResidualSquared =
                        SurfaceResidualSquared(line, candidateT, surface, candidateU, candidateV);
                    if (candidateResidualSquared < residual.LengthSquared())
                    {
                        t = candidateT;
                        u = candidateU;
                        v = candidateV;
                        accepted = true;
                        break;
                    }
                }

                if (!accepted)
                {
                    return {};
                }
            }

            const SCPoint3d linePoint = line.PointAt(t);
            const SCPoint3d surfacePoint = surface.PointAt(u, v);
            if ((linePoint - surfacePoint).LengthSquared() <= distanceEpsilonSquared)
            {
                return {true, false, false, t, u, v, surfacePoint};
            }
            return {};
        }

        [[nodiscard]] SCLineCurveIntersection3d TryRefineLineCurveNewton(const SCLine3d& line,
                                                                          const ISCCurve3d& curve,
                                                                          double initialParameter,
                                                                          const SCGeometryTolerance3d& tolerance)
        {
            const SCIntervald range = curve.ParameterRange();
            if (!range.IsValid())
            {
                return {};
            }

            const double directionLengthSquared = line.direction.LengthSquared();
            if (directionLengthSquared <= tolerance.distanceEpsilon * tolerance.distanceEpsilon)
            {
                return {};
            }

            const auto removeLineComponent = [&](const SCVector3d& vector) {
                return vector - line.direction * (Dot(vector, line.direction) / directionLengthSquared);
            };
            double parameter = std::clamp(initialParameter, range.min, range.max);
            const double distanceEpsilonSquared = tolerance.distanceEpsilon * tolerance.distanceEpsilon;
            for (int iteration = 0; iteration < 32; ++iteration)
            {
                const SCCurveEval3d eval = curve.Evaluate(parameter, 2);
                if (!eval.IsValid())
                {
                    return {};
                }
                if (eval.derivativeOrder < 1)
                {
                    double bestResidualSquared =
                        removeLineComponent(curve.PointAt(parameter) - line.origin).LengthSquared();
                    double step = std::max(range.Length() * 0.25, tolerance.parameterEpsilon);
                    for (int refinement = 0; refinement < 64 && bestResidualSquared > distanceEpsilonSquared;
                         ++refinement)
                    {
                        bool improved = false;
                        for (const double dt : std::array<double, 2>{-step, step})
                        {
                            const double candidate = std::clamp(parameter + dt, range.min, range.max);
                            const double candidateResidualSquared =
                                removeLineComponent(curve.PointAt(candidate) - line.origin).LengthSquared();
                            if (candidateResidualSquared < bestResidualSquared)
                            {
                                parameter = candidate;
                                bestResidualSquared = candidateResidualSquared;
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
                    if (bestResidualSquared > distanceEpsilonSquared)
                    {
                        return {};
                    }
                    const SCPoint3d point = curve.PointAt(parameter);
                    const double lineParameter = Dot(point - line.origin, line.direction) / directionLengthSquared;
                    return {true, lineParameter, parameter, point};
                }

                const SCVector3d residual = removeLineComponent(eval.point - line.origin);
                const double residualSquared = residual.LengthSquared();
                if (residualSquared <= distanceEpsilonSquared)
                {
                    const double lineParameter = Dot(eval.point - line.origin, line.direction) / directionLengthSquared;
                    return {true, lineParameter, parameter, eval.point};
                }

                const SCVector3d firstDerivative = removeLineComponent(eval.firstDerivative);
                const double gradient = Dot(residual, firstDerivative);
                const double tangentSquared = firstDerivative.LengthSquared();
                const double curvatureTerm = eval.derivativeOrder >= 2
                                                 ? Dot(residual, removeLineComponent(eval.secondDerivative))
                                                 : 0.0;
                const double hessian = tangentSquared + curvatureTerm;
                const double hessianScale = tangentSquared + std::abs(curvatureTerm);
                if (!std::isfinite(hessian) || !std::isfinite(hessianScale) ||
                    std::abs(hessian) <= std::numeric_limits<double>::epsilon() * hessianScale)
                {
                    return {};
                }

                const double delta = -gradient / hessian;
                if (!std::isfinite(delta))
                {
                    return {};
                }

                bool accepted = false;
                for (double scale = 1.0; scale >= 1.0 / 128.0; scale *= 0.5)
                {
                    const double candidateParameter = std::clamp(parameter + scale * delta, range.min, range.max);
                    const SCPoint3d candidatePoint = curve.PointAt(candidateParameter);
                    const double candidateResidualSquared =
                        removeLineComponent(candidatePoint - line.origin).LengthSquared();
                    if (candidateResidualSquared < residualSquared)
                    {
                        parameter = candidateParameter;
                        accepted = true;
                        break;
                    }
                }

                if (!accepted)
                {
                    return {};
                }
            }

            const SCPoint3d point = curve.PointAt(parameter);
            const SCVector3d residual = removeLineComponent(point - line.origin);
            if (residual.LengthSquared() > distanceEpsilonSquared)
            {
                return {};
            }
            const double lineParameter = Dot(point - line.origin, line.direction) / directionLengthSquared;
            return {true, lineParameter, parameter, point};
        }

        [[nodiscard]] SCLineSurfaceIntersection3d RefineLineSurfaceIntersection(const SCLine3d& line,
                                                                              const ISCSurface& surface,
                                                                              double initialT,
                                                                              double initialU,
                                                                              double initialV,
                                                                              const SCGeometryTolerance3d& tolerance)
        {
            const SCIntervald uRange = surface.URange();
            const SCIntervald vRange = surface.VRange();
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
                            if (candidateResidualSquared + tolerance.distanceEpsilon * tolerance.distanceEpsilon <
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

                if (stepT <= tolerance.parameterEpsilon && stepU <= tolerance.parameterEpsilon &&
                    stepV <= tolerance.parameterEpsilon)
                {
                    break;
                }
            }

            const SCPoint3d pointOnLine = line.PointAt(bestT);
            const SCPoint3d pointOnSurface = surface.PointAt(bestU, bestV);
            if ((pointOnLine - pointOnSurface).LengthSquared() > tolerance.distanceEpsilon * tolerance.distanceEpsilon)
            {
                return {};
            }

            return {true, false, false, bestT, bestU, bestV, pointOnSurface};
        }

        [[nodiscard]] bool IsTrimPointInsideFace(const SCBrepFace& face, double u, double v, double eps, bool& onBoundary)
        {
            onBoundary = false;
            if (!face.OuterTrim().IsValid())
            {
                return true;
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

            const SCPolygon2d polygon(face.OuterTrim().UvCurve(), std::move(holes));
            if (!polygon.IsValid())
            {
                return false;
            }

            const SCPointContainment2d containment = LocatePoint(SCPoint2d{u, v}, polygon, eps);
            onBoundary = containment == SCPointContainment2d::OnBoundary;
            return containment == SCPointContainment2d::Inside || onBoundary;
        }

        template <typename BodyIntersection, typename VisitFaces>
        [[nodiscard]] BodyIntersection IntersectLineWithBodyFaces(const SCGeometryTolerance3d& tolerance,
                                                                  VisitFaces&& visitFaces)
        {
            BodyIntersection result{};
            using Hit = typename std::decay_t<decltype(result.hits)>::value_type;
            std::vector<std::pair<std::size_t, Hit>> collected;
            std::set<std::pair<long long, std::size_t>> dedup;
            const auto collect = [&](std::size_t faceIndex, Hit hit) {
                if (!hit.intersects)
                {
                    return;
                }
                const long long bucket = static_cast<long long>(std::llround(
                    hit.lineParameter / std::max(tolerance.parameterEpsilon, Geometry::kIntersectionDefaultEpsilon)));
                if (dedup.emplace(bucket, faceIndex).second)
                {
                    collected.emplace_back(faceIndex, std::move(hit));
                }
            };
            visitFaces(collect);
            std::sort(collected.begin(), collected.end(), [](const auto& lhs, const auto& rhs) {
                return lhs.second.lineParameter < rhs.second.lineParameter;
            });
            for (auto& [faceIndex, hit] : collected)
            {
                result.faceIndices.push_back(faceIndex);
                result.hits.push_back(std::move(hit));
            }
            result.intersects = !result.hits.empty();
            return result;
        }

        [[nodiscard]] bool PointInTriangle3d(const SCPoint3d& point, const SCTriangle3d& triangle, double eps)
        {
            const SCVector3d v0 = triangle.c - triangle.a;
            const SCVector3d v1 = triangle.b - triangle.a;
            const SCVector3d v2 = point - triangle.a;

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
    }  // namespace

    SCLinePlaneIntersection3d Intersect(const SCLine3d& line, const SCPlane& plane, const SCGeometryTolerance3d& tolerance)
    {
        SCLinePlaneIntersection3d result{};
        if (!line.IsValid(tolerance.distanceEpsilon) || !plane.IsValid(tolerance.distanceEpsilon))
        {
            return result;
        }

        const SCVector3d unitNormal = plane.UnitNormal(tolerance.distanceEpsilon);
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

    SCLineCurveIntersection3d Intersect(const SCLine3d& line, const ISCCurve3d& curve, const SCGeometryTolerance3d& tolerance)
    {
        if (!line.IsValid(tolerance.distanceEpsilon) || !curve.IsValid(tolerance))
        {
            return {};
        }

        if (const auto* lineCurve = dynamic_cast<const SCLineCurve3d*>(&curve))
        {
            const SCLine3d curveLine = lineCurve->Line();
            const SCVector3d cross = Cross(line.direction, curveLine.direction);
            if (cross.LengthSquared() <= tolerance.distanceEpsilon * tolerance.distanceEpsilon)
            {
                const double separationSquared = Cross(curveLine.origin - line.origin, line.direction).LengthSquared() /
                                                 line.direction.LengthSquared();
                if (separationSquared > tolerance.distanceEpsilon * tolerance.distanceEpsilon)
                {
                    return {};
                }

                const SCIntervald curveRange = lineCurve->ParameterRange();
                const double curveParameter = curveRange.min;
                const SCPoint3d point = lineCurve->PointAt(curveParameter);
                const double lineParameter = Dot(point - line.origin, line.direction) /
                                             line.direction.LengthSquared();
                return {true, lineParameter, curveParameter, point};
            }

            const SCVector3d delta = curveLine.origin - line.origin;
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
            const SCIntervald curveRange = lineCurve->ParameterRange();
            if (!curveRange.Contains(curveParameter, tolerance.parameterEpsilon))
            {
                return {};
            }

            const SCPoint3d pointOnLine = line.PointAt(lineParameter);
            const SCPoint3d pointOnCurve = lineCurve->PointAt(curveParameter);
            if (!pointOnLine.AlmostEquals(pointOnCurve, tolerance.distanceEpsilon))
            {
                return {};
            }

            return {true, lineParameter, curveParameter, pointOnCurve};
        }

        const SCIntervald range = curve.ParameterRange();
        if (!range.IsValid())
        {
            return {};
        }

        std::vector<double> curveBoundaries{range.min, range.max};
        if (const auto* nurbsCurve = dynamic_cast<const SCNurbsCurve3d*>(&curve))
        {
            auto nurbsBoundaries = Detail::KnotSpanBoundaries(
                range, nurbsCurve->Knots(), nurbsCurve->Degree(), nurbsCurve->ControlPoints().size());
            if (nurbsBoundaries.size() >= 2)
            {
                curveBoundaries = std::move(nurbsBoundaries);
            }
        }
        const SCBox3d curveBounds = curve.Bounds();
        const double curveScale = curveBounds.IsValid()
                                      ? (curveBounds.MaxPoint() - curveBounds.MinPoint()).Length()
                                      : tolerance.distanceEpsilon;
        const std::vector<double> curveSeeds = Detail::AdaptiveCurveParameterSeeds(
            curve, std::move(curveBoundaries), std::max(tolerance.distanceEpsilon * 4.0, curveScale * 1e-3));

        double bestLineParameter = 0.0;
        double bestCurveParameter = range.min;
        double bestDistanceSquared = (line.origin - curve.PointAt(bestCurveParameter)).LengthSquared();
        for (const double curveParameter : curveSeeds)
        {
            const SCPoint3d pointOnCurve = curve.PointAt(curveParameter);
            const double lineParameter = Dot(pointOnCurve - line.origin, line.direction) /
                                         line.direction.LengthSquared();
            const SCPoint3d pointOnLine = line.PointAt(lineParameter);
            const double distanceSquared = (pointOnLine - pointOnCurve).LengthSquared();
            if (distanceSquared < bestDistanceSquared)
            {
                bestLineParameter = lineParameter;
                bestCurveParameter = curveParameter;
                bestDistanceSquared = distanceSquared;
            }
            const SCLineCurveIntersection3d refined =
                TryRefineLineCurveNewton(line, curve, curveParameter, tolerance);
            if (refined.intersects)
            {
                return refined;
            }
        }

        const SCLineCurveIntersection3d newtonResult =
            TryRefineLineCurveNewton(line, curve, bestCurveParameter, tolerance);
        if (newtonResult.intersects)
        {
            return newtonResult;
        }

        if (bestDistanceSquared > tolerance.distanceEpsilon * tolerance.distanceEpsilon)
        {
            return {};
        }

        return {true, bestLineParameter, bestCurveParameter, curve.PointAt(bestCurveParameter)};
    }

    SCLineCurveOnSurfaceIntersection3d Intersect(const SCLine3d& line,
                                               const SCCurveOnSurface& curveOnSurface,
                                               const SCGeometryTolerance3d& tolerance)
    {
        if (!line.IsValid(tolerance.distanceEpsilon) || !curveOnSurface.IsValid(tolerance) ||
            curveOnSurface.PointCount() < 2)
        {
            return {};
        }

        const std::size_t pointCount = curveOnSurface.PointCount();
        const std::size_t segmentCount = curveOnSurface.UvCurve().IsClosed() ? pointCount : pointCount - 1;
        for (std::size_t i = 0; i < segmentCount; ++i)
        {
            const std::size_t j = (i + 1) % pointCount;
            const SCPoint3d start = curveOnSurface.PointAt(i);
            const SCPoint3d end = curveOnSurface.PointAt(j);
            const SCVector3d segmentDirection = end - start;
            const double segmentLengthSquared = segmentDirection.LengthSquared();
            if (segmentLengthSquared <= tolerance.distanceEpsilon * tolerance.distanceEpsilon)
            {
                continue;
            }

            const SCLine3d segmentLine = SCLine3d::FromOriginAndDirection(start, segmentDirection);
            const SCVector3d delta = segmentLine.origin - line.origin;
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

            const SCPoint3d pointOnLine = line.PointAt(lineParameter);
            const SCPoint3d pointOnSegment = start + segmentDirection * segmentParameter;
            if (!pointOnLine.AlmostEquals(pointOnSegment, tolerance.distanceEpsilon))
            {
                continue;
            }

            const SCPoint2d uv0 = curveOnSurface.UvPointAt(i);
            const SCPoint2d uv1 = curveOnSurface.UvPointAt(j);
            const SCPoint2d uv = uv0 + (uv1 - uv0) * segmentParameter;
            return {true, i, segmentParameter, lineParameter, uv, pointOnSegment};
        }

        return {};
    }

    SCLineSurfaceIntersection3d Intersect(const SCLine3d& line,
                                        const ISCSurface& surface,
                                        const SCGeometryTolerance3d& tolerance)
    {
        if (!line.IsValid(tolerance.distanceEpsilon) || !surface.IsValid(tolerance))
        {
            return {};
        }

        if (const auto* planeSurface = dynamic_cast<const SCPlaneSurface*>(&surface))
        {
            const SCLinePlaneIntersection3d planeIntersection = Intersect(line, planeSurface->SupportPlane(), tolerance);
            if (!planeIntersection.intersects)
            {
                return {false, planeIntersection.isParallel, planeIntersection.liesOnPlane, 0.0, 0.0, 0.0, {}};
            }

            const SCVector3d delta = planeIntersection.point - planeSurface->SupportPlane().origin;
            const SCVector3d uAxis = planeSurface->UAxis();
            const SCVector3d vAxis = planeSurface->VAxis();
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

            return {
                true, false, planeIntersection.liesOnPlane, planeIntersection.parameter, u, v, planeIntersection.point};
        }

        if (const auto* nurbsSurface = dynamic_cast<const SCNurbsSurface*>(&surface))
        {
            SCPlane supportPlane{};
            if (TryBuildSupportPlane(*nurbsSurface, tolerance, supportPlane))
            {
                const SCLinePlaneIntersection3d planeIntersection = Intersect(line, supportPlane, tolerance);
                if (!planeIntersection.intersects)
                {
                    return {false, planeIntersection.isParallel, planeIntersection.liesOnPlane, 0.0, 0.0, 0.0, {}};
                }

                double u = 0.0;
                double v = 0.0;
                SCPoint3d surfacePoint{};
                if (TryProjectPointToAffinePlanarNurbs(
                        planeIntersection.point, *nurbsSurface, tolerance, u, v, surfacePoint))
                {
                    return {
                        true, false, planeIntersection.liesOnPlane, planeIntersection.parameter, u, v, surfacePoint};
                }

                const SCSurfaceProjection3d surfaceProjection =
                    ProjectPointToSurface(planeIntersection.point, surface, tolerance);
                if (!surfaceProjection.success ||
                    surfaceProjection.distanceSquared > tolerance.distanceEpsilon * tolerance.distanceEpsilon)
                {
                    return {};
                }

                return {true,
                        false,
                        planeIntersection.liesOnPlane,
                        planeIntersection.parameter,
                        surfaceProjection.u,
                        surfaceProjection.v,
                        surfaceProjection.point};
            }
        }

        const SCIntervald uRange = surface.URange();
        const SCIntervald vRange = surface.VRange();
        if (!uRange.IsValid() || !vRange.IsValid())
        {
            return {};
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
        double bestU = 0.5 * (uRange.min + uRange.max);
        double bestV = 0.5 * (vRange.min + vRange.max);
        const SCPoint3d centerSurfacePoint = surface.PointAt(bestU, bestV);
        double bestT = Dot(centerSurfacePoint - line.origin, line.direction) /
                       std::max(line.direction.LengthSquared(), tolerance.parameterEpsilon);
        double bestResidualSquared = SurfaceResidualSquared(line, bestT, surface, bestU, bestV);
        const SCLineSurfaceIntersection3d centerNewton =
            TryRefineLineSurfaceNewton(line, surface, bestT, bestU, bestV, tolerance);
        if (centerNewton.intersects)
        {
            return centerNewton;
        }

        const SCSurfaceProjection3d originProjection = ProjectPointToSurface(line.origin, surface, tolerance);
        if (originProjection.success)
        {
            const double projectedLineParameter = Dot(originProjection.point - line.origin, line.direction) /
                                                  std::max(line.direction.LengthSquared(), tolerance.parameterEpsilon);
            const double projectedResidualSquared =
                SurfaceResidualSquared(line, projectedLineParameter, surface, originProjection.u, originProjection.v);
            if (projectedResidualSquared < bestResidualSquared)
            {
                bestT = projectedLineParameter;
                bestU = originProjection.u;
                bestV = originProjection.v;
                bestResidualSquared = projectedResidualSquared;
            }
            const SCLineSurfaceIntersection3d projectedNewton = TryRefineLineSurfaceNewton(
                line, surface, projectedLineParameter, originProjection.u, originProjection.v, tolerance);
            if (projectedNewton.intersects)
            {
                return projectedNewton;
            }
        }

        for (const Detail::SurfaceParameterSeed& seed : surfaceSeeds)
        {
            const double u = seed.u;
            const double v = seed.v;
            const SCPoint3d surfacePoint = surface.PointAt(u, v);
            const double t = Dot(surfacePoint - line.origin, line.direction) /
                             std::max(line.direction.LengthSquared(), tolerance.parameterEpsilon);
            const double residualSquared = SurfaceResidualSquared(line, t, surface, u, v);
            const SCLineSurfaceIntersection3d seedNewton =
                TryRefineLineSurfaceNewton(line, surface, t, u, v, tolerance);
            if (seedNewton.intersects)
            {
                return seedNewton;
            }
            if (residualSquared < bestResidualSquared)
            {
                bestT = t;
                bestU = u;
                bestV = v;
                bestResidualSquared = residualSquared;
            }
        }

        const SCLineSurfaceIntersection3d newtonResult =
            TryRefineLineSurfaceNewton(line, surface, bestT, bestU, bestV, tolerance);
        if (newtonResult.intersects)
        {
            return newtonResult;
        }
        return RefineLineSurfaceIntersection(line, surface, bestT, bestU, bestV, tolerance);
    }

    SCLineBrepEdgeIntersection3d Intersect(const SCLine3d& line, const SCBrepEdge& edge, const SCGeometryTolerance3d& tolerance)
    {
        if (!line.IsValid(tolerance.distanceEpsilon) || !edge.IsValid(tolerance) || edge.Curve() == nullptr)
        {
            return {};
        }

        const SCLineCurveIntersection3d curveIntersection = Intersect(line, *edge.Curve(), tolerance);
        return {curveIntersection.intersects,
                curveIntersection.lineParameter,
                curveIntersection.curveParameter,
                curveIntersection.point};
    }

    SCLineBrepVertexIntersection3d Intersect(const SCLine3d& line,
                                           const SCBrepVertex& vertex,
                                           const SCGeometryTolerance3d& tolerance)
    {
        if (!line.IsValid(tolerance.distanceEpsilon) || !vertex.IsValid())
        {
            return {};
        }

        const SCLineProjection3d projection = ProjectPointToLine(vertex.Point(), line, tolerance);
        if (!projection.isOnLine || projection.distanceSquared > tolerance.distanceEpsilon * tolerance.distanceEpsilon)
        {
            return {};
        }

        return {true, projection.parameter, vertex.Point()};
    }

    SCLineBrepFaceIntersection3d Intersect(const SCLine3d& line, const SCBrepFace& face, const SCGeometryTolerance3d& tolerance)
    {
        if (!line.IsValid(tolerance.distanceEpsilon) || !face.IsValid(tolerance) || face.SupportSurface() == nullptr)
        {
            return {};
        }

        const SCLineSurfaceIntersection3d surfaceIntersection = Intersect(line, *face.SupportSurface(), tolerance);
        if (!surfaceIntersection.intersects)
        {
            return {};
        }

        bool onBoundary = false;
        if (!IsTrimPointInsideFace(
                face, surfaceIntersection.u, surfaceIntersection.v, tolerance.distanceEpsilon, onBoundary))
        {
            return {};
        }

        return {true,
                onBoundary,
                surfaceIntersection.lineParameter,
                surfaceIntersection.u,
                surfaceIntersection.v,
                surfaceIntersection.point};
    }

    SCLineBrepBodyIntersection3d Intersect(const SCLine3d& line, const SCBrepBody& body, const SCGeometryTolerance3d& tolerance)
    {
        if (!line.IsValid(tolerance.distanceEpsilon) || !body.IsValid(tolerance))
        {
            return {};
        }
        return IntersectLineWithBodyFaces<SCLineBrepBodyIntersection3d>(
            tolerance, [&](auto&& collect) {
                std::size_t faceIndex = 0;
                for (std::size_t shellIndex = 0; shellIndex < body.ShellCount(); ++shellIndex)
                {
                    const SCBrepShell& shell = body.ShellAtRef(shellIndex);
                    for (std::size_t localFaceIndex = 0; localFaceIndex < shell.FaceCount(); ++localFaceIndex, ++faceIndex)
                    {
                        collect(faceIndex, Intersect(line, shell.FaceAtRef(localFaceIndex), tolerance));
                    }
                }
            });
    }

    SCLinePolyhedronFaceIntersection3d Intersect(const SCLine3d& line,
                                               const PolyhedronFace3d& face,
                                               const SCGeometryTolerance3d& tolerance)
    {
        if (!line.IsValid(tolerance.distanceEpsilon) || !face.IsValid(tolerance))
        {
            return {};
        }

        const SCLinePlaneIntersection3d planeIntersection = Intersect(line, face.SupportPlane(), tolerance);
        if (!planeIntersection.intersects)
        {
            return {};
        }

        const SCFaceProjection3d faceProjection = ProjectFaceToPolygon2d(face, tolerance);
        if (!faceProjection.success)
        {
            return {};
        }

        const SCVector3d delta = planeIntersection.point - faceProjection.origin;
        const double u = Dot(delta, faceProjection.uAxis);
        const double v = Dot(delta, faceProjection.vAxis);
        const SCPointContainment2d containment =
            LocatePoint(SCPoint2d{u, v}, faceProjection.polygon, tolerance.distanceEpsilon);
        if (containment == SCPointContainment2d::Outside)
        {
            return {};
        }

        return {true,
                containment == SCPointContainment2d::OnBoundary,
                planeIntersection.parameter,
                u,
                v,
                planeIntersection.point};
    }

    SCLinePolyhedronBodyIntersection3d Intersect(const SCLine3d& line,
                                               const PolyhedronBody& body,
                                               const SCGeometryTolerance3d& tolerance)
    {
        if (!line.IsValid(tolerance.distanceEpsilon) || !body.IsValid(tolerance))
        {
            return {};
        }
        return IntersectLineWithBodyFaces<SCLinePolyhedronBodyIntersection3d>(
            tolerance, [&](auto&& collect) {
                for (std::size_t faceIndex = 0; faceIndex < body.FaceCount(); ++faceIndex)
                {
                    collect(faceIndex, Intersect(line, body.FaceAtRef(faceIndex), tolerance));
                }
            });
    }

    SCLineTriangleMeshIntersection3d Intersect(const SCLine3d& line,
                                             const TriangleMesh& mesh,
                                             const SCGeometryTolerance3d& tolerance)
    {
        SCLineTriangleMeshIntersection3d result{};
        if (!line.IsValid(tolerance.distanceEpsilon) || !mesh.IsValid(tolerance.distanceEpsilon))
        {
            return result;
        }

        std::set<std::pair<long long, std::size_t>> dedup;
        struct Hit
        {
            std::size_t triangleIndex;
            double lineParameter;
            SCPoint3d point;
        };
        std::vector<Hit> hits;
        for (std::size_t triangleIndex = 0; triangleIndex < mesh.TriangleCount(); ++triangleIndex)
        {
            const SCTriangle3d triangle = mesh.TriangleAt(triangleIndex);
            const SCPlane plane = SCPlane::FromPointAndNormal(triangle.a, triangle.Normal());
            const SCLinePlaneIntersection3d planeHit = Intersect(line, plane, tolerance);
            if (!planeHit.intersects)
            {
                continue;
            }

            if (!PointInTriangle3d(planeHit.point, triangle, tolerance.distanceEpsilon))
            {
                continue;
            }

            const long long bucket = static_cast<long long>(std::llround(
                planeHit.parameter / std::max(tolerance.parameterEpsilon, Geometry::kIntersectionDefaultEpsilon)));
            if (!dedup.emplace(bucket, triangleIndex).second)
            {
                continue;
            }
            hits.push_back({triangleIndex, planeHit.parameter, planeHit.point});
        }

        std::sort(hits.begin(), hits.end(), [](const Hit& lhs, const Hit& rhs) {
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

    SCPlaneCurveIntersection3d Intersect(const SCPlane& plane, const ISCCurve3d& curve, const SCGeometryTolerance3d& tolerance)
    {
        if (!plane.IsValid(tolerance.distanceEpsilon) || !curve.IsValid(tolerance))
        {
            return {};
        }

        if (const auto* lineCurve = dynamic_cast<const SCLineCurve3d*>(&curve))
        {
            const SCLinePlaneIntersection3d hit = Intersect(lineCurve->Line(), plane, tolerance);
            if (!hit.intersects)
            {
                return {};
            }
            const SCIntervald range = lineCurve->ParameterRange();
            if (!range.Contains(hit.parameter, tolerance.parameterEpsilon))
            {
                return {};
            }
            return {true, hit.parameter, hit.point};
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
        const SCVector3d unitNormal = plane.UnitNormal(tolerance.distanceEpsilon);

        const auto refineTangentRoot = [&](double initialParameter) -> std::optional<double> {
            double parameter = std::clamp(initialParameter, range.min, range.max);
            for (int iteration = 0; iteration < 32; ++iteration)
            {
                const SCCurveEval3d eval = curve.Evaluate(parameter, 1);
                if (!eval.IsValid())
                {
                    return std::nullopt;
                }
                const double distance = plane.SignedDistanceTo(eval.point, tolerance.distanceEpsilon);
                if (std::abs(distance) <= tolerance.distanceEpsilon)
                {
                    return parameter;
                }
                const double derivative = Dot(unitNormal, eval.firstDerivative);
                const double derivativeScale = eval.firstDerivative.Length();
                if (!std::isfinite(derivative) || !std::isfinite(derivativeScale) ||
                    std::abs(derivative) <= std::numeric_limits<double>::epsilon() * derivativeScale)
                {
                    return std::nullopt;
                }
                bool accepted = false;
                for (double scale = 1.0; scale >= 1.0 / 128.0; scale *= 0.5)
                {
                    const double candidate = std::clamp(parameter - scale * distance / derivative, range.min, range.max);
                    const double candidateDistance =
                        plane.SignedDistanceTo(curve.PointAt(candidate), tolerance.distanceEpsilon);
                    if (std::abs(candidateDistance) < std::abs(distance))
                    {
                        parameter = candidate;
                        accepted = true;
                        break;
                    }
                }
                if (!accepted)
                {
                    return std::nullopt;
                }
            }

            return std::nullopt;
        };

        for (std::size_t i = 0; i < parameterSeeds.size(); ++i)
        {
            const double t0 = parameterSeeds[i];
            const SCPoint3d p0 = curve.PointAt(t0);
            const double d0 = plane.SignedDistanceTo(p0, tolerance.distanceEpsilon);
            if (std::abs(d0) <= tolerance.distanceEpsilon)
            {
                return {true, t0, p0};
            }

            if (i + 1 < parameterSeeds.size())
            {
                double low = t0;
                double high = parameterSeeds[i + 1];
                const double d1 = plane.SignedDistanceTo(curve.PointAt(high), tolerance.distanceEpsilon);
                if (d0 * d1 < 0.0)
                {
                    for (int iteration = 0; iteration < 64; ++iteration)
                    {
                        const double middle = 0.5 * (low + high);
                        const SCPoint3d middlePoint = curve.PointAt(middle);
                        const double middleDistance = plane.SignedDistanceTo(middlePoint, tolerance.distanceEpsilon);
                        if (std::abs(middleDistance) <= tolerance.distanceEpsilon)
                        {
                            return {true, middle, middlePoint};
                        }
                        if ((d0 < 0.0) == (middleDistance < 0.0))
                        {
                            low = middle;
                        }
                        else
                        {
                            high = middle;
                        }
                    }
                    const double parameter = 0.5 * (low + high);
                    const SCPoint3d point = curve.PointAt(parameter);
                    if (std::abs(plane.SignedDistanceTo(point, tolerance.distanceEpsilon)) <=
                        tolerance.distanceEpsilon)
                    {
                        return {true, parameter, point};
                    }
                    continue;
                }
            }

            if (const std::optional<double> tangentRoot = refineTangentRoot(t0))
            {
                return {true, *tangentRoot, curve.PointAt(*tangentRoot)};
            }
        }

        return {};
    }

    SCPlaneCurveOnSurfaceIntersection3d Intersect(const SCPlane& plane,
                                                const SCCurveOnSurface& curveOnSurface,
                                                const SCGeometryTolerance3d& tolerance)
    {
        if (!plane.IsValid(tolerance.distanceEpsilon) || !curveOnSurface.IsValid(tolerance) ||
            curveOnSurface.PointCount() < 2)
        {
            return {};
        }

        const std::size_t pointCount = curveOnSurface.PointCount();
        const std::size_t segmentCount = curveOnSurface.UvCurve().IsClosed() ? pointCount : pointCount - 1;
        for (std::size_t i = 0; i < segmentCount; ++i)
        {
            const std::size_t j = (i + 1) % pointCount;
            const SCPoint3d p0 = curveOnSurface.PointAt(i);
            const SCPoint3d p1 = curveOnSurface.PointAt(j);
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
            const SCPoint2d uv0 = curveOnSurface.UvPointAt(i);
            const SCPoint2d uv1 = curveOnSurface.UvPointAt(j);
            const SCPoint2d uv = uv0 + (uv1 - uv0) * weight;
            return {true, i, weight, uv, p0 + (p1 - p0) * weight};
        }

        return {};
    }

    SCPlaneBrepEdgeIntersection3d Intersect(const SCPlane& plane,
                                          const SCBrepEdge& edge,
                                          const SCGeometryTolerance3d& tolerance)
    {
        if (!plane.IsValid(tolerance.distanceEpsilon) || !edge.IsValid(tolerance) || edge.Curve() == nullptr)
        {
            return {};
        }

        const SCPlaneCurveIntersection3d hit = Intersect(plane, *edge.Curve(), tolerance);
        if (!hit.intersects)
        {
            return {};
        }
        return {true, hit.curveParameter, hit.point};
    }

    SCPlaneBrepVertexIntersection3d Intersect(const SCPlane& plane,
                                            const SCBrepVertex& vertex,
                                            const SCGeometryTolerance3d& tolerance)
    {
        if (!plane.IsValid(tolerance.distanceEpsilon) || !vertex.IsValid())
        {
            return {};
        }

        if (std::abs(plane.SignedDistanceTo(vertex.Point(), tolerance.distanceEpsilon)) > tolerance.distanceEpsilon)
        {
            return {};
        }

        return {true, vertex.Point()};
    }

    SCPlanePlaneIntersection3d Intersect(const SCPlane& first, const SCPlane& second, const SCGeometryTolerance3d& tolerance)
    {
        SCPlanePlaneIntersection3d result{};
        if (!first.IsValid(tolerance.distanceEpsilon) || !second.IsValid(tolerance.distanceEpsilon))
        {
            return result;
        }

        const SCVector3d n1 = first.UnitNormal(tolerance.distanceEpsilon);
        const SCVector3d n2 = second.UnitNormal(tolerance.distanceEpsilon);
        const SCVector3d direction = Cross(n1, n2);
        const double directionLengthSquared = direction.LengthSquared();

        if (directionLengthSquared <= tolerance.distanceEpsilon * tolerance.distanceEpsilon)
        {
            result.isParallel = true;
            result.isCoincident =
                std::abs(second.SignedDistanceTo(first.origin, tolerance.distanceEpsilon)) <= tolerance.distanceEpsilon;
            result.intersects = result.isCoincident;
            return result;
        }

        const double d1 = Dot(n1, first.origin - SCPoint3d{});
        const double d2 = Dot(n2, second.origin - SCPoint3d{});
        const SCVector3d pointVector = Cross((d1 * n2) - (d2 * n1), direction) / directionLengthSquared;

        result.intersects = true;
        result.line = SCLine3d::FromOriginAndDirection(SCPoint3d{pointVector.x, pointVector.y, pointVector.z}, direction);
        return result;
    }
}  // namespace Geometry

