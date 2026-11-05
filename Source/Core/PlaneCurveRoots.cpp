#include "Core/Intersection.h"
#include "Core/Projection.h"
#include "Support/StableNumerics.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

#include "Geometry3d/SCLineCurve3d.h"
#include "Geometry3d/SCNurbsCurve3d.h"
#include "Geometry3d/Detail/SurfaceNumerics.h"
#include "Detail/CurveOnSurfaceNumerics.h"

namespace Geometry
{
    namespace
    {
        double Norm(const SCVector3d& vector)
        {
            return std::hypot(vector.x, vector.y, vector.z);
        }

        [[nodiscard]] SCGeometryTolerance3d UnitPieceTolerance(
            const SCIntervald& range, const SCGeometryTolerance3d& tolerance)
        {
            auto local = tolerance;
            const double width = range.max - range.min;
            // 原参数容差换算到 [0,1]；异号端点的跨度可能不可表示。
            if (tolerance.parameterEpsilon == 0.0)
            {
                // 即使分片宽度相减下溢为零，精确容差也必须保持为零。
                local.parameterEpsilon = 0.0;
            }
            else if (std::isfinite(width) && width > 0.0)
                local.parameterEpsilon = tolerance.parameterEpsilon / width;
            else
            {
                const double scale = std::max(std::abs(range.min), std::abs(range.max));
                local.parameterEpsilon = (tolerance.parameterEpsilon / scale) /
                    (range.max / scale - range.min / scale);
            }
            // 极窄分片的比值可溢出，但结构容差必须保持有限。
            local.parameterEpsilon = std::min(local.parameterEpsilon, std::numeric_limits<double>::max());
            return local;
        }
    }

    namespace
    {
        struct PlaneCurveControl
        {
            double numerator{0.0};
            double weight{1.0};
        };

        struct PlaneCurveSpan
        {
            double start{0.0};
            double end{0.0};
            std::vector<PlaneCurveControl> controls{};
            int depth{0};
            bool coplanarControlHull{false};
        };

        [[nodiscard]] bool BuildPlaneCurveBezierSpans(const SCNurbsCurve3d& curve,
                                                       const SCPlane& plane,
                                                       double eps,
                                                       std::vector<PlaneCurveSpan>& spans)
        {
            const int degree = curve.Degree();
            if (degree < 1) return false;
            std::vector<double> knots = curve.Knots();
            std::vector<PlaneCurveControl> controls;
            controls.reserve(curve.ControlPoints().size());
            const double weightScale = *std::max_element(curve.Weights().begin(), curve.Weights().end());
            if (!(weightScale > 0.0) || !std::isfinite(weightScale)) return false;
            const auto buildControls = [&](double scale) {
                controls.clear();
                for (std::size_t i = 0; i < curve.ControlPoints().size(); ++i)
                {
                    const double signedDistance = plane.SignedDistanceTo(curve.ControlPoints()[i], eps);
                    const double weight = curve.Weights()[i] / scale;
                    const double numerator = signedDistance * weight;
                    if (!std::isfinite(signedDistance) || !std::isfinite(weight) || !(weight > 0.0) ||
                        !std::isfinite(numerator) || (signedDistance != 0.0 && numerator == 0.0))
                        return false;
                    controls.push_back({numerator, weight});
                }
                return true;
            };
            if (!buildControls(weightScale) && !buildControls(1.0)) return false;
            const auto originalControls = controls;
            const auto range = curve.ParameterRange();
            const auto multiplicity = [&](double knot) {
                return static_cast<int>(std::count(knots.begin(), knots.end(), knot));
            };
            if (multiplicity(range.min) < degree + 1 || multiplicity(range.max) < degree + 1)
                return false;

            std::vector<std::pair<double, int>> internalKnots;
            for (const double knot : knots)
            {
                if (!(knot > range.min && knot < range.max)) continue;
                if (internalKnots.empty() || internalKnots.back().first != knot)
                    internalKnots.emplace_back(knot, 1);
                else
                    ++internalKnots.back().second;
            }
            for (const auto& [knot, initialMultiplicity] : internalKnots)
            {
                int count = initialMultiplicity;
                while (count < degree)
                {
                    const std::size_t span = static_cast<std::size_t>(
                        std::upper_bound(knots.begin(), knots.end(), knot) - knots.begin() - 1);
                    std::vector<PlaneCurveControl> refined(controls.size() + 1);
                    for (std::size_t i = 0; i <= span - static_cast<std::size_t>(degree); ++i)
                        refined[i] = controls[i];
                    for (std::size_t i = span - static_cast<std::size_t>(count); i < controls.size(); ++i)
                        refined[i + 1] = controls[i];
                    for (std::size_t i = span - static_cast<std::size_t>(degree) + 1;
                         i <= span - static_cast<std::size_t>(count); ++i)
                    {
                        const double denominator = knots[i + static_cast<std::size_t>(degree)] - knots[i];
                        if (!(denominator > 0.0)) return false;
                        const double alpha = (knot - knots[i]) / denominator;
                        refined[i].numerator = std::lerp(controls[i - 1].numerator, controls[i].numerator, alpha);
                        refined[i].weight = std::lerp(controls[i - 1].weight, controls[i].weight, alpha);
                    }
                    knots.insert(knots.begin() + static_cast<std::ptrdiff_t>(span + 1), knot);
                    controls = std::move(refined);
                    ++count;
                }
            }

            for (std::size_t span = static_cast<std::size_t>(degree);
                 span < controls.size(); ++span)
            {
                if (!(knots[span + 1] > knots[span])) continue;
                PlaneCurveSpan cell;
                cell.start = knots[span];
                cell.end = knots[span + 1];
                cell.controls.assign(controls.begin() + static_cast<std::ptrdiff_t>(span - degree),
                                     controls.begin() + static_cast<std::ptrdiff_t>(span + 1));
                const std::size_t originalSpan = static_cast<std::size_t>(
                    std::upper_bound(curve.Knots().begin(), curve.Knots().end(), cell.start) -
                    curve.Knots().begin() - 1);
                cell.coplanarControlHull = std::all_of(
                    originalControls.begin() + static_cast<std::ptrdiff_t>(originalSpan - degree),
                    originalControls.begin() + static_cast<std::ptrdiff_t>(originalSpan + 1),
                    [](const auto& control) { return control.numerator == 0.0; });
                spans.push_back(std::move(cell));
            }
            return !spans.empty();
        }

        void BisectPlaneCurveSpan(const PlaneCurveSpan& cell,
                                  PlaneCurveSpan& left,
                                  PlaneCurveSpan& right)
        {
            left.start = cell.start;
            left.end = std::midpoint(cell.start, cell.end);
            right.start = left.end;
            right.end = cell.end;
            left.depth = right.depth = cell.depth + 1;
            left.coplanarControlHull = right.coplanarControlHull = cell.coplanarControlHull;
            const std::size_t count = cell.controls.size();
            left.controls.resize(count);
            right.controls.resize(count);
            std::vector<PlaneCurveControl> work = cell.controls;
            left.controls.front() = work.front();
            right.controls.back() = work.back();
            for (std::size_t level = 1; level < count; ++level)
            {
                for (std::size_t i = 0; i < count - level; ++i)
                {
                    work[i].numerator = std::midpoint(work[i].numerator, work[i + 1].numerator);
                    work[i].weight = std::midpoint(work[i].weight, work[i + 1].weight);
                }
                left.controls[level] = work.front();
                right.controls[count - level - 1] = work[count - level - 1];
            }
        }
        [[nodiscard]] bool SurfaceControlHullSeparatedFromPlane(
            const SCNurbsSurface& surface, const SCPlane& plane, double eps)
        {
            // 调用方已校验正权重 NURBS；其参数域内曲面包含于原控制点凸包。
            int side = 0;
            if (surface.ControlPoints().empty()) return false;
            for (const auto& control : surface.ControlPoints())
            {
                const double distance = plane.SignedDistanceTo(control, eps);
                if (!std::isfinite(distance) || std::abs(distance) <= eps) return false;
                const int controlSide = distance > 0.0 ? 1 : -1;
                if (side != 0 && side != controlSide) return false;
                side = controlSide;
            }
            return true;
        }
        [[nodiscard]] bool SurfaceControlHullSeparatedFromLine(
            const SCNurbsSurface& surface, const SCLine3d& line, double eps)
        {
            const double components[] = {std::abs(line.direction.x), std::abs(line.direction.y),
                                         std::abs(line.direction.z)};
            std::size_t dominant = 0;
            for (std::size_t axis = 1; axis < 3; ++axis)
                if (components[axis] > components[dominant]) dominant = axis;
            // 先尝试法向较稳定的两个平面，再检查其余坐标轴对应的分离平面。
            const std::size_t axes[] = {(dominant + 1) % 3, (dominant + 2) % 3, dominant};
            for (const std::size_t axis : axes)
            {
                const SCVector3d basis{axis == 0 ? 1.0 : 0.0, axis == 1 ? 1.0 : 0.0,
                                       axis == 2 ? 1.0 : 0.0};
                // 用原方向与坐标轴叉乘，避免平方中间量；平面经过整条目标直线。
                const SCPlane plane = SCPlane::FromPointAndNormal(line.origin, Cross(line.direction, basis));
                if (plane.IsValid(eps) && SurfaceControlHullSeparatedFromPlane(surface, plane, eps)) return true;
            }
            return false;
        }
    }  // namespace

    SCPlaneCurveIntersections3dWithStatus IntersectAllPlaneCurveWithStatus(
        const SCPlane& plane, const ISCCurve3d& curve, const SCGeometryTolerance3d& tolerance)
    {
        SCPlaneCurveIntersections3dWithStatus result;
        if (!tolerance.IsValid() || !plane.IsValid(tolerance.distanceEpsilon) || !curve.IsValid(tolerance))
            return result;

        result.state = SCGeometryOperationState::Unknown;
        result.evidence.hit = SCGeometryEvidenceState::Unknown;
        result.evidence.negativeCompleteness = SCGeometryEvidenceState::Unknown;
        result.evidence.reasons = SCGeometryFailureReason::SearchIncomplete;
        const SCIntervald range = curve.ParameterRange();
        if (!range.IsValid()) return result;

        if (const auto* line = dynamic_cast<const SCLineCurve3d*>(&curve))
        {
            const auto relation = IntersectLinePlaneWithStatus(line->Line(), plane, tolerance);
            if (relation.state == SCGeometryOperationState::InvalidInput ||
                relation.state == SCGeometryOperationState::Unknown)
                return result;
            if (relation.state == SCGeometryOperationState::NonUnique && range.max > range.min)
            {
                result.state = SCGeometryOperationState::NonUnique;
                result.evidence.hit = SCGeometryEvidenceState::Verified;
                result.evidence.negativeCompleteness = SCGeometryEvidenceState::NotApplicable;
                result.evidence.reasons = SCGeometryFailureReason::NonUnique;
                return result;
            }
            if (relation.state == SCGeometryOperationState::NonUnique)
            {
                const SCPoint3d point = line->PointAt(range.min);
                if (!point.IsValid()) return result;
                const double distance = plane.SignedDistanceTo(point, tolerance.distanceEpsilon);
                if (!std::isfinite(distance) || std::abs(distance) > tolerance.distanceEpsilon)
                    return result;
                result.hits.push_back({true, range.min, point});
            }
            else
            {
                const auto hit = Intersect(plane, *line, tolerance);
                if (hit.intersects && hit.IsValid()) result.hits.push_back(hit);
            }
            result.state = result.hits.empty() ? SCGeometryOperationState::NoResult
                                                : SCGeometryOperationState::HasResult;
            result.evidence.hit = result.hits.empty() ? SCGeometryEvidenceState::NotApplicable
                                                       : SCGeometryEvidenceState::Verified;
            result.evidence.negativeCompleteness = SCGeometryEvidenceState::Verified;
            result.evidence.reasons = SCGeometryFailureReason::None;
            return result;
        }

        if (const auto* surfaceCurve = dynamic_cast<const SCCurveOnSurface*>(&curve))
        {
            if (const auto* surface = dynamic_cast<const SCNurbsSurface*>(surfaceCurve->SupportSurface());
                surface != nullptr && SurfaceControlHullSeparatedFromPlane(*surface, plane, tolerance.distanceEpsilon))
            {
                result.state = SCGeometryOperationState::NoResult;
                result.evidence.hit = SCGeometryEvidenceState::NotApplicable;
                result.evidence.negativeCompleteness = SCGeometryEvidenceState::Verified;
                result.evidence.reasons = SCGeometryFailureReason::None;
                return result;
            }
            const bool planar = dynamic_cast<const SCPlaneSurface*>(surfaceCurve->SupportSurface()) != nullptr;
            bool complete = planar;
            const std::size_t segmentCount = surfaceCurve->UvPieceCount();
            const auto appendHit = [&](const SCPlaneCurveIntersection3d& hit) {
                if (!result.hits.empty())
                {
                    const auto& previous = result.hits.back();
                    if (std::abs(previous.curveParameter - hit.curveParameter) <= tolerance.parameterEpsilon &&
                        previous.point.AlmostEquals(hit.point, tolerance.distanceEpsilon))
                    {
                        return;
                    }
                }
                result.hits.push_back(hit);
            };
            for (std::size_t segment = 0; segment < segmentCount; ++segment)
            {
                if (planar && surfaceCurve->UvGeometry().IsLinearPiece(segment))
                {
                    const auto start = Detail::EvaluateCurveOnSurfaceSegment(*surfaceCurve, segment, 0.0);
                    const auto end = Detail::EvaluateCurveOnSurfaceSegment(*surfaceCurve, segment, 1.0);
                    if (!start.valid || !end.valid)
                    {
                        complete = false;
                        continue;
                    }
                    const SCLineCurve3d localCurve(
                        SCLine3d::FromOriginAndDirection(start.point, end.point - start.point), {0.0, 1.0});
                    if (!localCurve.IsValid(tolerance))
                    {
                        complete = false;
                        continue;
                    }
                    const auto localTolerance = UnitPieceTolerance(
                        surfaceCurve->UvGeometry().PieceRange(segment), tolerance);
                    const auto local = IntersectAllPlaneCurveWithStatus(plane, localCurve, localTolerance);
                    if (local.state == SCGeometryOperationState::NonUnique)
                    {
                        result.hits.clear();
                        result.state = SCGeometryOperationState::NonUnique;
                        result.evidence.hit = SCGeometryEvidenceState::Verified;
                        result.evidence.negativeCompleteness = SCGeometryEvidenceState::NotApplicable;
                        result.evidence.reasons = SCGeometryFailureReason::NonUnique;
                        return result;
                    }
                    complete &= local.evidence.negativeCompleteness == SCGeometryEvidenceState::Verified;
                    for (const auto& hit : local.hits)
                    {
                        appendHit({true, surfaceCurve->UvParameterAt(segment, hit.curveParameter), hit.point});
                    }
                }
                else
                {
                    complete = false;
                    bool nonUnique = false;
                    const auto hits = Detail::FindPlaneCurveOnSurfaceSegmentHits(
                        plane, *surfaceCurve, segment, tolerance, &nonUnique);
                    if (nonUnique)
                    {
                        result.hits.clear();
                        result.state = SCGeometryOperationState::NonUnique;
                        result.evidence.hit = SCGeometryEvidenceState::Verified;
                        result.evidence.negativeCompleteness = SCGeometryEvidenceState::NotApplicable;
                        result.evidence.reasons = SCGeometryFailureReason::NonUnique;
                        return result;
                    }
                    for (const auto& hit : hits)
                    {
                        appendHit({true, surfaceCurve->UvParameterAt(segment, hit.parameter), hit.point});
                    }
                }
            }
            result.state = result.hits.empty() && !complete ? SCGeometryOperationState::Unknown :
                           result.hits.empty() ? SCGeometryOperationState::NoResult :
                                                 SCGeometryOperationState::HasResult;
            result.evidence.hit = result.hits.empty() ?
                (complete ? SCGeometryEvidenceState::NotApplicable : SCGeometryEvidenceState::Unknown) :
                SCGeometryEvidenceState::Verified;
            result.evidence.negativeCompleteness = complete ? SCGeometryEvidenceState::Verified :
                                                              SCGeometryEvidenceState::Unknown;
            result.evidence.reasons = complete ? SCGeometryFailureReason::None :
                                                 SCGeometryFailureReason::SearchIncomplete |
                                                     SCGeometryFailureReason::NoProof;
            return result;
        }

        const auto* nurbs = dynamic_cast<const SCNurbsCurve3d*>(&curve);
        if (nurbs == nullptr)
        {
            const auto hit = Intersect(plane, curve, tolerance);
            if (hit.intersects && hit.IsValid())
            {
                result.hits.push_back(hit);
                result.state = SCGeometryOperationState::HasResult;
                result.evidence.hit = SCGeometryEvidenceState::Verified;
            }
            return result;
        }

        if (nurbs->Degree() < 1 ||
            std::count(nurbs->Knots().begin(), nurbs->Knots().end(), range.min) < nurbs->Degree() + 1 ||
            std::count(nurbs->Knots().begin(), nurbs->Knots().end(), range.max) < nurbs->Degree() + 1)
        {
            const auto hit = Intersect(plane, curve, tolerance);
            if (hit.intersects && hit.IsValid())
            {
                result.hits.push_back(hit);
                result.state = SCGeometryOperationState::HasResult;
                result.evidence.hit = SCGeometryEvidenceState::Verified;
            }
            return result;
        }

        std::vector<PlaneCurveSpan> cells;
        if (!BuildPlaneCurveBezierSpans(*nurbs, plane, tolerance.distanceEpsilon, cells))
            return result;

        const auto signedDistanceAt = [&](double parameter) {
            return plane.SignedDistanceTo(curve.PointAt(parameter), tolerance.distanceEpsilon);
        };
        const SCVector3d planeNormal = plane.UnitNormal(tolerance.distanceEpsilon);
        const auto addHit = [&](double parameter, double cellStart, double cellEnd) {
            for (int iteration = 0; iteration < 48; ++iteration)
            {
                const double distance = signedDistanceAt(parameter);
                const SCCurveEval3d eval = curve.Evaluate(parameter, 2);
                if (!std::isfinite(distance) || !eval.IsValid() || eval.derivativeOrder < 1) break;
                const double derivative = Dot(planeNormal, eval.firstDerivative);
                if (!std::isfinite(derivative)) break;
                if (eval.derivativeOrder >= 2 &&
                    std::abs(distance) <= tolerance.distanceEpsilon)
                {
                    const double second = Dot(planeNormal, eval.secondDerivative);
                    if (std::isfinite(second) && second != 0.0 &&
                        std::abs(derivative) <=
                            4.0 * std::sqrt(tolerance.distanceEpsilon * std::abs(second)))
                    {
                        const double stationary = std::clamp(parameter - derivative / second,
                                                             cellStart, cellEnd);
                        if (std::isfinite(stationary) &&
                            std::abs(signedDistanceAt(stationary)) <= std::abs(distance))
                        {
                            parameter = stationary;
                            break;
                        }
                    }
                }
                if (derivative == 0.0) break;
                const double candidate = std::clamp(parameter - distance / derivative,
                                                    cellStart, cellEnd);
                if (!std::isfinite(candidate) || candidate == parameter ||
                    std::abs(signedDistanceAt(candidate)) >= std::abs(distance)) break;
                parameter = candidate;
            }
            const SCPoint3d point = curve.PointAt(parameter);
            if (!point.IsValid()) return false;
            const double distance = plane.SignedDistanceTo(point, tolerance.distanceEpsilon);
            if (!std::isfinite(distance) || std::abs(distance) > tolerance.distanceEpsilon)
                return false;
            result.hits.push_back({true, parameter, point});
            return true;
        };

        bool complete = true;
        std::size_t visited = 0;
        constexpr std::size_t maxCells = 32768;
        while (!cells.empty())
        {
            if (++visited > maxCells)
            {
                complete = false;
                break;
            }
            PlaneCurveSpan cell = std::move(cells.back());
            cells.pop_back();
            bool positive = true;
            bool negative = true;
            bool identicallyZero = true;
            bool invalidControl = false;
            int variations = 0;
            int previousSign = 0;
            for (const PlaneCurveControl& control : cell.controls)
            {
                if (!(control.weight > 0.0) || !std::isfinite(control.numerator) ||
                    !std::isfinite(control.weight))
                {
                    complete = false;
                    invalidControl = true;
                    break;
                }
                positive &= control.numerator > 0.0;
                negative &= control.numerator < 0.0;
                identicallyZero &= control.numerator == 0.0;
                const int sign = (control.numerator > 0.0) - (control.numerator < 0.0);
                if (sign != 0)
                {
                    if (previousSign != 0 && previousSign != sign) ++variations;
                    previousSign = sign;
                }
            }
            if (invalidControl) continue;
            if (identicallyZero)
            {
                if (!cell.coplanarControlHull)
                {
                    // knot 插入或细分后的全零可能来自下溢，不能替代原控制包的共面证明。
                    complete = false;
                    (void)addHit(std::midpoint(cell.start, cell.end), cell.start, cell.end);
                    continue;
                }
                result.state = SCGeometryOperationState::NonUnique;
                result.hits.clear();
                result.evidence.hit = SCGeometryEvidenceState::Verified;
                result.evidence.negativeCompleteness = SCGeometryEvidenceState::NotApplicable;
                result.evidence.reasons = SCGeometryFailureReason::NonUnique;
                return result;
            }
            if (positive || negative)
            {
                bool nearPlane = false;
                for (const PlaneCurveControl& control : cell.controls)
                    nearPlane |= std::abs(control.numerator) <=
                                 tolerance.distanceEpsilon * control.weight;
                if (nearPlane)
                {
                    // 同号控制点只能排除精确根，不能证明容差带内没有其他接触。
                    complete = false;
                    const auto derivativeAt = [&](double parameter) {
                        const SCCurveEval3d eval = curve.Evaluate(parameter, 1);
                        return eval.IsValid() && eval.derivativeOrder >= 1
                            ? Dot(planeNormal, eval.firstDerivative)
                            : std::numeric_limits<double>::quiet_NaN();
                    };
                    double low = cell.start;
                    double high = cell.end;
                    double firstDerivative = derivativeAt(low);
                    const double lastDerivative = derivativeAt(high);
                    if (std::isfinite(firstDerivative) && std::isfinite(lastDerivative))
                    {
                        if (firstDerivative == 0.0) addHit(low, cell.start, cell.end);
                        else if (lastDerivative == 0.0) addHit(high, cell.start, cell.end);
                        else if ((firstDerivative < 0.0) != (lastDerivative < 0.0))
                        {
                            for (int iteration = 0; iteration < 64; ++iteration)
                            {
                                const double middle = std::midpoint(low, high);
                                if (middle <= low || middle >= high) break;
                                const double derivative = derivativeAt(middle);
                                if (!std::isfinite(derivative)) { complete = false; break; }
                                if (derivative == 0.0) { low = high = middle; break; }
                                if ((derivative < 0.0) == (firstDerivative < 0.0))
                                {
                                    low = middle;
                                    firstDerivative = derivative;
                                }
                                else high = middle;
                            }
                            addHit(std::midpoint(low, high), cell.start, cell.end);
                        }
                    }
                    else complete = false;
                }
                continue;
            }

            const double middle = std::midpoint(cell.start, cell.end);
            const double width = cell.end - cell.start;
            const double parameterLimit = std::max(tolerance.parameterEpsilon,
                16.0 * std::numeric_limits<double>::epsilon() *
                    std::max({1.0, std::abs(cell.start), std::abs(cell.end)}));
            if (cell.depth >= 48 || width <= parameterLimit ||
                middle <= cell.start || middle >= cell.end)
            {
                bool accepted = addHit(cell.start, cell.start, cell.end) ||
                                addHit(middle, cell.start, cell.end) ||
                                addHit(cell.end, cell.start, cell.end);
                const double startDistance = signedDistanceAt(cell.start);
                const double endDistance = signedDistanceAt(cell.end);
                if (!accepted && std::isfinite(startDistance) && std::isfinite(endDistance) &&
                    ((startDistance < 0.0 && endDistance > 0.0) ||
                     (startDistance > 0.0 && endDistance < 0.0)))
                {
                    double low = cell.start;
                    double high = cell.end;
                    for (int iteration = 0; iteration < 64; ++iteration)
                    {
                        const double candidate = std::midpoint(low, high);
                        if (candidate <= low || candidate >= high) break;
                        const double distance = signedDistanceAt(candidate);
                        if (!std::isfinite(distance)) break;
                        if (addHit(candidate, cell.start, cell.end)) { accepted = true; break; }
                        if ((distance < 0.0) == (startDistance < 0.0)) low = candidate;
                        else high = candidate;
                    }
                }
                if (!accepted)
                {
                    double parameter = middle;
                    for (int iteration = 0; iteration < 48; ++iteration)
                    {
                        const double distance = signedDistanceAt(parameter);
                        if (!std::isfinite(distance)) break;
                        if (addHit(parameter, cell.start, cell.end)) { accepted = true; break; }
                        const SCCurveEval3d eval = curve.Evaluate(parameter, 1);
                        if (!eval.IsValid() || eval.derivativeOrder < 1) break;
                        const double derivative = Dot(planeNormal, eval.firstDerivative);
                        if (!std::isfinite(derivative) || derivative == 0.0) break;
                        const double candidate = std::clamp(parameter - distance / derivative,
                                                            cell.start, cell.end);
                        if (candidate == parameter ||
                            std::abs(signedDistanceAt(candidate)) >= std::abs(distance)) break;
                        parameter = candidate;
                    }
                }
                if (!accepted || variations > 1) complete = false;
                continue;
            }
            PlaneCurveSpan left;
            PlaneCurveSpan right;
            BisectPlaneCurveSpan(cell, left, right);
            cells.push_back(std::move(right));
            cells.push_back(std::move(left));
        }

        std::sort(result.hits.begin(), result.hits.end(), [](const auto& first, const auto& second) {
            return first.curveParameter < second.curveParameter;
        });
        const double mergeTolerance = std::max(tolerance.parameterEpsilon,
            16.0 * std::numeric_limits<double>::epsilon() *
                std::max({1.0, std::abs(range.min), std::abs(range.max)}));
        const auto rootResolution = [&](double parameter) {
            const auto& knots = nurbs->Knots();
            std::size_t span = static_cast<std::size_t>(
                std::upper_bound(knots.begin(), knots.end(), parameter) - knots.begin());
            if (span > 0) --span;
            span = std::clamp(span, static_cast<std::size_t>(nurbs->Degree()),
                              nurbs->ControlPoints().size() - 1);
            const double spanWidth = knots[span + 1] - knots[span];
            double scale = 0.0;
            for (std::size_t i = span - static_cast<std::size_t>(nurbs->Degree()); i <= span; ++i)
                scale = std::max(scale, std::abs(plane.SignedDistanceTo(
                    nurbs->ControlPoints()[i], tolerance.distanceEpsilon)));
            if (!(spanWidth > 0.0) || !(scale > 0.0)) return mergeTolerance;
            return std::max(mergeTolerance, spanWidth * std::pow(
                std::min(1.0, tolerance.distanceEpsilon / scale),
                1.0 / static_cast<double>(nurbs->Degree())));
        };
        std::vector<SCPlaneCurveIntersection3d> distinct;
        for (const auto& hit : result.hits)
        {
            const double separation = distinct.empty() ? std::numeric_limits<double>::infinity() :
                hit.curveParameter - distinct.back().curveParameter;
            const double midpointDistance = distinct.empty() ? std::numeric_limits<double>::infinity() :
                std::abs(signedDistanceAt(std::midpoint(hit.curveParameter,
                                                        distinct.back().curveParameter)));
            if (!distinct.empty() && !std::isfinite(midpointDistance)) complete = false;
            if (distinct.empty() ||
                separation > std::max(rootResolution(hit.curveParameter),
                                      rootResolution(distinct.back().curveParameter)) ||
                !std::isfinite(midpointDistance) || midpointDistance > tolerance.distanceEpsilon)
            {
                distinct.push_back(hit);
            }
            else
            {
                if (hit.curveParameter - distinct.back().curveParameter > mergeTolerance)
                    complete = false;
                if (std::abs(plane.SignedDistanceTo(hit.point, tolerance.distanceEpsilon)) <
                    std::abs(plane.SignedDistanceTo(distinct.back().point, tolerance.distanceEpsilon)))
                    distinct.back() = hit;
            }
        }
        result.hits = std::move(distinct);
        result.state = result.hits.empty() ? (complete ? SCGeometryOperationState::NoResult
                                                       : SCGeometryOperationState::Unknown)
                                           : SCGeometryOperationState::HasResult;
        result.evidence.hit = result.hits.empty()
            ? (complete ? SCGeometryEvidenceState::NotApplicable : SCGeometryEvidenceState::Unknown)
            : SCGeometryEvidenceState::Verified;
        result.evidence.negativeCompleteness = complete ? SCGeometryEvidenceState::Verified
                                                         : SCGeometryEvidenceState::Unknown;
        result.evidence.reasons = complete ? SCGeometryFailureReason::None
                                           : SCGeometryFailureReason::SearchIncomplete;
        return result;
    }

    SCLineCurveIntersections3dWithStatus IntersectAllLineCurveWithStatus(
        const SCLine3d& line, const ISCCurve3d& curve, const SCGeometryTolerance3d& tolerance)
    {
        SCLineCurveIntersections3dWithStatus result;
        if (!tolerance.IsValid() || !line.IsValid(tolerance.distanceEpsilon) || !curve.IsValid(tolerance))
            return result;

        result.state = SCGeometryOperationState::Unknown;
        result.evidence.hit = SCGeometryEvidenceState::Unknown;
        result.evidence.negativeCompleteness = SCGeometryEvidenceState::Unknown;
        result.evidence.reasons = SCGeometryFailureReason::SearchIncomplete;
        if (dynamic_cast<const SCNurbsCurve3d*>(&curve) == nullptr)
        {
            if (dynamic_cast<const SCLineCurve3d*>(&curve) != nullptr)
            {
                const auto analytic = IntersectLineCurveWithStatus(line, curve, tolerance);
                result.state = analytic.state;
                result.evidence = analytic.evidence;
                if (analytic.payloadValid) result.hits.push_back(analytic.payload);
                return result;
            }
            if (const auto* surfaceCurve = dynamic_cast<const SCCurveOnSurface*>(&curve))
            {
                if (const auto* surface = dynamic_cast<const SCNurbsSurface*>(surfaceCurve->SupportSurface());
                    surface != nullptr && SurfaceControlHullSeparatedFromLine(*surface, line, tolerance.distanceEpsilon))
                {
                    result.state = SCGeometryOperationState::NoResult;
                    result.evidence.hit = SCGeometryEvidenceState::NotApplicable;
                    result.evidence.negativeCompleteness = SCGeometryEvidenceState::Verified;
                    result.evidence.reasons = SCGeometryFailureReason::None;
                    return result;
                }
                const bool planar = dynamic_cast<const SCPlaneSurface*>(surfaceCurve->SupportSurface()) != nullptr;
                bool complete = planar;
                const std::size_t segmentCount = surfaceCurve->UvPieceCount();
                const auto appendHit = [&](const SCLineCurveIntersection3d& hit) {
                    if (!result.hits.empty())
                    {
                        const auto& previous = result.hits.back();
                        if (std::abs(previous.curveParameter - hit.curveParameter) <= tolerance.parameterEpsilon &&
                            previous.point.AlmostEquals(hit.point, tolerance.distanceEpsilon))
                        {
                            return;
                        }
                    }
                    result.hits.push_back(hit);
                };
                for (std::size_t segment = 0; segment < segmentCount; ++segment)
                {
                    if (planar && surfaceCurve->UvGeometry().IsLinearPiece(segment))
                    {
                        const auto start = Detail::EvaluateCurveOnSurfaceSegment(*surfaceCurve, segment, 0.0);
                        const auto end = Detail::EvaluateCurveOnSurfaceSegment(*surfaceCurve, segment, 1.0);
                        if (!start.valid || !end.valid)
                        {
                            complete = false;
                            continue;
                        }
                        const SCLineCurve3d localCurve(
                            SCLine3d::FromOriginAndDirection(start.point, end.point - start.point), {0.0, 1.0});
                        if (!localCurve.IsValid(tolerance))
                        {
                            complete = false;
                            continue;
                        }
                        const auto localTolerance = UnitPieceTolerance(
                            surfaceCurve->UvGeometry().PieceRange(segment), tolerance);
                        const auto local = IntersectAllLineCurveWithStatus(line, localCurve, localTolerance);
                        if (local.state == SCGeometryOperationState::NonUnique)
                        {
                            result.hits.clear();
                            result.state = SCGeometryOperationState::NonUnique;
                            result.evidence.hit = SCGeometryEvidenceState::Verified;
                            result.evidence.negativeCompleteness = SCGeometryEvidenceState::NotApplicable;
                            result.evidence.reasons = SCGeometryFailureReason::NonUnique;
                            return result;
                        }
                        complete &= local.evidence.negativeCompleteness == SCGeometryEvidenceState::Verified;
                        for (const auto& hit : local.hits)
                        {
                            appendHit({true, hit.lineParameter,
                                       surfaceCurve->UvParameterAt(segment, hit.curveParameter), hit.point});
                        }
                    }
                    else
                    {
                        complete = false;
                        bool nonUnique = false;
                        const auto candidates = Detail::FindLineCurveOnSurfaceSegmentHits(
                            line, *surfaceCurve, segment, tolerance, &nonUnique);
                        if (nonUnique)
                        {
                            result.hits.clear();
                            result.state = SCGeometryOperationState::NonUnique;
                            result.evidence.hit = SCGeometryEvidenceState::Verified;
                            result.evidence.negativeCompleteness = SCGeometryEvidenceState::NotApplicable;
                            result.evidence.reasons = SCGeometryFailureReason::NonUnique;
                            return result;
                        }
                        for (const auto& candidate : candidates)
                        {
                            appendHit({true, candidate.lineParameter,
                                       surfaceCurve->UvParameterAt(segment, candidate.sample.parameter),
                                       candidate.sample.point});
                        }
                    }
                }
                result.state = result.hits.empty() && !complete ? SCGeometryOperationState::Unknown :
                               result.hits.empty() ? SCGeometryOperationState::NoResult :
                                                     SCGeometryOperationState::HasResult;
                result.evidence.hit = result.hits.empty() ?
                    (complete ? SCGeometryEvidenceState::NotApplicable : SCGeometryEvidenceState::Unknown) :
                    SCGeometryEvidenceState::Verified;
                result.evidence.negativeCompleteness = complete ? SCGeometryEvidenceState::Verified :
                                                                  SCGeometryEvidenceState::Unknown;
                result.evidence.reasons = complete ? SCGeometryFailureReason::None :
                                                     SCGeometryFailureReason::SearchIncomplete |
                                                         SCGeometryFailureReason::NoProof;
                return result;
            }
            const auto hit = Intersect(line, curve, tolerance);
            if (hit.intersects && hit.IsValid())
            {
                result.hits.push_back(hit);
                result.state = SCGeometryOperationState::HasResult;
                result.evidence.hit = SCGeometryEvidenceState::Verified;
            }
            return result;
        }

        SCVector3d direction;
        if (!Detail::TryUnitVector(line.direction, direction)) return result;
        const SCVector3d axis = std::abs(direction.x) < 0.9
            ? SCVector3d{1.0, 0.0, 0.0} : SCVector3d{0.0, 1.0, 0.0};
        const SCVector3d firstNormal = Cross(direction, axis);
        const SCVector3d secondNormal = Cross(direction, firstNormal);
        const SCPlane firstPlane = SCPlane::FromPointAndNormal(line.origin, firstNormal);
        const SCPlane secondPlane = SCPlane::FromPointAndNormal(line.origin, secondNormal);
        if (!firstPlane.IsValid(tolerance.distanceEpsilon) ||
            !secondPlane.IsValid(tolerance.distanceEpsilon)) return result;

        auto planeResult = IntersectAllPlaneCurveWithStatus(firstPlane, curve, tolerance);
        if (planeResult.state == SCGeometryOperationState::NonUnique)
            planeResult = IntersectAllPlaneCurveWithStatus(secondPlane, curve, tolerance);
        if (planeResult.state == SCGeometryOperationState::NonUnique)
        {
            std::vector<PlaneCurveSpan> firstSpans;
            std::vector<PlaneCurveSpan> secondSpans;
            const auto& nurbs = static_cast<const SCNurbsCurve3d&>(curve);
            const bool spansValid =
                BuildPlaneCurveBezierSpans(nurbs, firstPlane, tolerance.distanceEpsilon, firstSpans) &&
                BuildPlaneCurveBezierSpans(nurbs, secondPlane, tolerance.distanceEpsilon, secondSpans) &&
                firstSpans.size() == secondSpans.size();
            bool onLine = false;
            for (std::size_t span = 0; spansValid && span < firstSpans.size(); ++span)
            {
                if (firstSpans[span].start != secondSpans[span].start ||
                    firstSpans[span].end != secondSpans[span].end ||
                    firstSpans[span].controls.size() != secondSpans[span].controls.size())
                    continue;
                bool coincident = firstSpans[span].coplanarControlHull &&
                                  secondSpans[span].coplanarControlHull;
                for (std::size_t control = 0; control < firstSpans[span].controls.size(); ++control)
                    coincident &= firstSpans[span].controls[control].numerator == 0.0 &&
                                  secondSpans[span].controls[control].numerator == 0.0;
                if (coincident) { onLine = true; break; }
            }
            if (!onLine && spansValid)
            {
                std::vector<double> boundaries;
                boundaries.reserve(firstSpans.size() + 1);
                for (const PlaneCurveSpan& span : firstSpans)
                    boundaries.push_back(span.start);
                boundaries.push_back(firstSpans.back().end);
                std::sort(boundaries.begin(), boundaries.end());
                boundaries.erase(std::unique(boundaries.begin(), boundaries.end()), boundaries.end());
                for (double parameter : boundaries)
                {
                    const SCPoint3d point = curve.PointAt(parameter);
                    if (!point.IsValid()) continue;
                    const SCLineProjection3d projection = ProjectPointToLine(point, line, tolerance);
                    if (!projection.IsValid() || !projection.isOnLine) continue;
                    const SCPoint3d linePoint = line.PointAt(projection.parameter);
                    if (!linePoint.IsValid() ||
                        Norm(linePoint - point) > tolerance.distanceEpsilon) continue;
                    result.hits.push_back({true, projection.parameter, parameter, point});
                }
            }
            result.state = onLine ? SCGeometryOperationState::NonUnique
                                  : (result.hits.empty() ? SCGeometryOperationState::Unknown
                                                         : SCGeometryOperationState::HasResult);
            result.evidence.hit = onLine ? SCGeometryEvidenceState::Verified
                                         : (result.hits.empty() ? SCGeometryEvidenceState::Unknown
                                                                : SCGeometryEvidenceState::Verified);
            result.evidence.negativeCompleteness = onLine ? SCGeometryEvidenceState::NotApplicable
                                                          : SCGeometryEvidenceState::Unknown;
            result.evidence.reasons = onLine ? SCGeometryFailureReason::NonUnique
                                             : SCGeometryFailureReason::SearchIncomplete;
            return result;
        }

        bool verifiedCandidates = true;
        for (const auto& planeHit : planeResult.hits)
        {
            const SCLineProjection3d projection = ProjectPointToLine(planeHit.point, line, tolerance);
            if (!projection.IsValid())
            {
                verifiedCandidates = false;
                continue;
            }
            if (!projection.isOnLine) continue;
            const SCPoint3d linePoint = line.PointAt(projection.parameter);
            if (!linePoint.IsValid())
            {
                verifiedCandidates = false;
                continue;
            }
            const double separation = Norm(linePoint - planeHit.point);
            if (!std::isfinite(separation))
            {
                verifiedCandidates = false;
                continue;
            }
            if (separation > tolerance.distanceEpsilon) continue;
            result.hits.push_back({true, projection.parameter, planeHit.curveParameter, planeHit.point});
        }
        std::sort(result.hits.begin(), result.hits.end(), [](const auto& first, const auto& second) {
            return first.curveParameter < second.curveParameter;
        });
        result.evidence = planeResult.evidence;
        if (!verifiedCandidates)
        {
            result.evidence.negativeCompleteness = SCGeometryEvidenceState::Unknown;
            result.evidence.reasons |= SCGeometryFailureReason::Unrepresentable |
                                       SCGeometryFailureReason::SearchIncomplete;
        }
        result.evidence.hit = result.hits.empty()
            ? (result.evidence.negativeCompleteness == SCGeometryEvidenceState::Verified
                ? SCGeometryEvidenceState::NotApplicable : SCGeometryEvidenceState::Unknown)
            : SCGeometryEvidenceState::Verified;
        result.state = result.hits.empty()
            ? (result.evidence.negativeCompleteness == SCGeometryEvidenceState::Verified
                ? SCGeometryOperationState::NoResult : SCGeometryOperationState::Unknown)
            : SCGeometryOperationState::HasResult;
        return result;
    }

}  // namespace Geometry
