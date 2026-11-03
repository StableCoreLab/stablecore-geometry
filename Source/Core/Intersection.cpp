#include "Core/Intersection.h"

#include "Detail/SegmentKernel2d.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <utility>

#include "Support/Epsilon.h"
#include "Support/Geometry2d/Predicate2.h"

namespace Geometry
{
    namespace
    {
        constexpr double kPi = std::numbers::pi_v<double>;
        constexpr double kTwoPi = 2.0 * kPi;

        struct ArcProjectionCandidate
        {
            SCPoint2d point{};
            double parameter{0.0};
        };

        [[nodiscard]] SCSegmentIntersection2d IntersectLineLineInternal(const SCLineSegment2d& first,
                                                                      const SCLineSegment2d& second,
                                                                      double eps);

        [[nodiscard]] SCSegmentIntersection2d IntersectLineArcInternal(const SCLineSegment2d& line,
                                                                     const SCArcSegment2d& arc,
                                                                     double eps);

        [[nodiscard]] SCSegmentIntersection2d IntersectArcArcInternal(const SCArcSegment2d& first,
                                                                    const SCArcSegment2d& second,
                                                                    double eps);

        [[nodiscard]] SCExtendedIntersection2d IntersectArcArcExtendedInternal(const SCArcSegment2d& first,
                                                                                const SCArcSegment2d& second,
                                                                                bool extendFirst,
                                                                                bool extendSecond,
                                                                                double eps);

        [[nodiscard]] ArcProjectionCandidate ProjectPointToArcSegmentLocal(const SCPoint2d& point,
                                                                           const SCArcSegment2d& arc,
                                                                           double eps);

        [[nodiscard]] double Clamp(double value, double low, double high)
        {
            return std::max(low, std::min(value, high));
        }

        [[nodiscard]] double NormalizeAngle(double angle)
        {
            angle = std::fmod(angle, kTwoPi);
            if (angle < 0.0)
            {
                angle += kTwoPi;
            }
            return angle;
        }

        [[nodiscard]] bool AlmostEqual(double lhs, double rhs, double eps)
        {
            return std::abs(lhs - rhs) <= eps;
        }

        [[nodiscard]] bool AlmostEqualPoint(const SCPoint2d& lhs, const SCPoint2d& rhs, double eps)
        {
            return lhs.AlmostEquals(rhs, eps);
        }

        [[nodiscard]] double DistanceSquaredPoints(const SCPoint2d& lhs, const SCPoint2d& rhs)
        {
            return (rhs - lhs).LengthSquared();
        }

        [[nodiscard]] SCPoint2d MakePoint(double x, double y)
        {
            return SCPoint2d{x, y};
        }

        [[nodiscard]] SCPoint2d PointAtAngle(const SCArcSegment2d& arc, double angle)
        {
            return MakePoint(arc.center.x + arc.radius * std::cos(angle), arc.center.y + arc.radius * std::sin(angle));
        }

        [[nodiscard]] bool IsAngleOnArc(const SCArcSegment2d& arc, double candidateAngle, double eps)
        {
            if (arc.sweepAngle >= 0.0)
            {
                const double delta = NormalizeAngle(candidateAngle - arc.startAngle);
                return delta <= arc.sweepAngle + eps;
            }

            const double delta = NormalizeAngle(arc.startAngle - candidateAngle);
            return delta <= (-arc.sweepAngle) + eps;
        }

        [[nodiscard]] double ArcParameterAtAngle(const SCArcSegment2d& arc, double angle)
        {
            if (arc.sweepAngle == 0.0)
            {
                return 0.0;
            }

            if (arc.sweepAngle > 0.0)
            {
                return NormalizeAngle(angle - arc.startAngle) / arc.sweepAngle;
            }

            return -NormalizeAngle(arc.startAngle - angle) / arc.sweepAngle;
        }

        [[nodiscard]] bool SegmentValidAtTolerance(const ISCSegment2d& segment, double eps)
        {
            if (!std::isfinite(eps) || eps <= 0.0)
            {
                return false;
            }
            if (const auto* line = dynamic_cast<const SCLineSegment2d*>(&segment))
            {
                return line->IsValid(eps);
            }
            if (const auto* arc = dynamic_cast<const SCArcSegment2d*>(&segment))
            {
                return arc->IsValid(eps);
            }
            return segment.IsValid();
        }

        [[nodiscard]] double SegmentLengthAtTolerance(const ISCSegment2d& segment, double eps)
        {
            if (const auto* arc = dynamic_cast<const SCArcSegment2d*>(&segment))
            {
                return arc->IsValid(eps) ? arc->Length(eps) : arc->Length();
            }
            return segment.Length();
        }

        void SortIntersectionPointsByFirstParameter(std::array<SCIntersectionPoint2d, 2>& points, std::size_t pointCount)
        {
            if (pointCount == 2 && points[1].parameterOnFirst < points[0].parameterOnFirst)
            {
                std::swap(points[0], points[1]);
            }
        }

        [[nodiscard]] SCExtendedIntersection2d MakeExtendedOverlapIntersection(const SCIntersectionPoint2d& firstPoint,
                                                                                const SCIntersectionPoint2d& secondPoint,
                                                                                bool onFirstSegment,
                                                                                bool onSecondSegment)
        {
            SCExtendedIntersection2d result;
            result.kind = SCIntersectionKind2d::Overlap;
            result.pointCount = 2;
            result.points[0] = firstPoint;
            result.points[1] = secondPoint;
            SortIntersectionPointsByFirstParameter(result.points, result.pointCount);
            result.onFirstSegment = onFirstSegment;
            result.onSecondSegment = onSecondSegment;
            return result;
        }

        [[nodiscard]] SCSegmentProjection2d ProjectPointToLineSegmentLocal(const SCPoint2d& point,
                                                                         const SCLineSegment2d& segment,
                                                                         bool clampToSegment = true)
        {
            const SCVector2d segmentVector = segment.endPoint - segment.startPoint;
            const double segmentLengthSquared = segmentVector.LengthSquared();
            if (segmentLengthSquared <= Geometry::kDefaultEpsilon * Geometry::kDefaultEpsilon)
            {
                return SCSegmentProjection2d{
                    segment.startPoint, 0.0, DistanceSquaredPoints(point, segment.startPoint), true};
            }

            const SCVector2d startToPoint = point - segment.startPoint;
            const double rawParameter = Dot(startToPoint, segmentVector) / segmentLengthSquared;
            const double parameter = clampToSegment ? Clamp(rawParameter, 0.0, 1.0) : rawParameter;
            const SCPoint2d projectedPoint =
                MakePoint(segment.startPoint.x + (segment.endPoint.x - segment.startPoint.x) * parameter,
                          segment.startPoint.y + (segment.endPoint.y - segment.startPoint.y) * parameter);

            return SCSegmentProjection2d{projectedPoint,
                                       parameter,
                                       DistanceSquaredPoints(point, projectedPoint),
                                       clampToSegment || (rawParameter >= -Geometry::kDefaultParameterEpsilon &&
                                                          rawParameter <= 1.0 + Geometry::kDefaultParameterEpsilon)};
        }

        [[nodiscard]] ArcProjectionCandidate ProjectPointToArcSegmentLocal(const SCPoint2d& point,
                                                                           const SCArcSegment2d& arc,
                                                                           double eps)
        {
            if (!arc.IsValid())
            {
                return ArcProjectionCandidate{point, 0.0};
            }

            const SCVector2d fromCenter = point - arc.center;
            if (std::hypot(fromCenter.x,fromCenter.y) <= eps)
            {
                return ArcProjectionCandidate{arc.StartPoint(), 0.0};
            }

            const double angle = std::atan2(fromCenter.y, fromCenter.x);
            if (IsAngleOnArc(arc, angle, eps))
            {
                return ArcProjectionCandidate{PointAtAngle(arc, angle), ArcParameterAtAngle(arc, angle)};
            }

            const SCPoint2d startPoint = arc.StartPoint();
            const SCPoint2d endPoint = arc.EndPoint();
            const double startDistanceSquared = DistanceSquaredPoints(point, startPoint);
            const double endDistanceSquared = DistanceSquaredPoints(point, endPoint);

            if (startDistanceSquared <= endDistanceSquared)
            {
                return ArcProjectionCandidate{startPoint, 0.0};
            }

            return ArcProjectionCandidate{endPoint, 1.0};
        }

        [[nodiscard]] SCClosestPoints2d MakeClosestPoints(const SCPoint2d& firstPoint,
                                                        const SCPoint2d& secondPoint,
                                                        double firstParameter,
                                                        double secondParameter)
        {
            return SCClosestPoints2d{firstPoint,
                                   secondPoint,
                                   firstParameter,
                                   secondParameter,
                                   DistanceSquaredPoints(firstPoint, secondPoint)};
        }

        [[nodiscard]] SCClosestPoints2d MakeClosestPoints(const SCPoint2d& firstPoint,
                                                        const SCPoint2d& secondPoint,
                                                        double firstParameter,
                                                        double secondParameter,
                                                        double distanceSquared)
        {
            return SCClosestPoints2d{firstPoint, secondPoint, firstParameter, secondParameter, distanceSquared};
        }

        void UpdateClosest(SCClosestPoints2d& best,
                           bool& hasBest,
                           const SCPoint2d& firstPoint,
                           const SCPoint2d& secondPoint,
                           double firstParameter,
                           double secondParameter)
        {
            const double distanceSquared = DistanceSquaredPoints(firstPoint, secondPoint);
            if (!hasBest || distanceSquared < best.distanceSquared)
            {
                best = MakeClosestPoints(firstPoint, secondPoint, firstParameter, secondParameter, distanceSquared);
                hasBest = true;
            }
        }

        [[nodiscard]] SCSegmentIntersection2d MakeNoIntersection()
        {
            return SCSegmentIntersection2d{};
        }

        void AddIntersectionPoint(SCSegmentIntersection2d& result,
                                  SCIntersectionKind2d kind,
                                  const SCPoint2d& point,
                                  double parameterOnFirst,
                                  double parameterOnSecond,
                                  double eps)
        {
            const SCIntersectionPoint2d candidate{point, parameterOnFirst, parameterOnSecond};

            if (result.pointCount == 0)
            {
                result.points[0] = candidate;
                result.pointCount = 1;
                result.kind = kind;
                return;
            }

            if (AlmostEqualPoint(result.points[0].point, candidate.point, eps))
            {
                result.points[0] = candidate;
                return;
            }

            if (result.pointCount == 1)
            {
                result.points[1] = candidate;
                result.pointCount = 2;
                return;
            }

            if (!AlmostEqualPoint(result.points[1].point, candidate.point, eps))
            {
                result.points[1] = candidate;
            }
        }

        [[nodiscard]] SCSegmentIntersection2d MakePointIntersection(const SCPoint2d& point,
                                                                  double firstParameter,
                                                                  double secondParameter,
                                                                  SCIntersectionKind2d kind = SCIntersectionKind2d::Point)
        {
            SCSegmentIntersection2d result;
            result.kind = kind;
            result.pointCount = 1;
            result.points[0] = SCIntersectionPoint2d{point, firstParameter, secondParameter};
            return result;
        }

        [[nodiscard]] SCSegmentIntersection2d MakeOverlapIntersection(const SCIntersectionPoint2d& firstPoint,
                                                                    const SCIntersectionPoint2d& secondPoint)
        {
            SCSegmentIntersection2d result;
            result.kind = SCIntersectionKind2d::Overlap;
            result.pointCount = 2;
            result.points[0] = firstPoint;
            result.points[1] = secondPoint;
            return result;
        }

        [[nodiscard]] SCSegmentIntersection2d IntersectLineLineInternal(const SCLineSegment2d& first,
                                                                      const SCLineSegment2d& second,
                                                                      double eps)
        {
            if (!std::isfinite(eps) || eps < 0.0 || !first.startPoint.IsValid() ||
                !first.endPoint.IsValid() || !second.startPoint.IsValid() || !second.endPoint.IsValid())
            {
                return MakeNoIntersection();
            }
            const SCPoint2d p = first.startPoint;
            const SCPoint2d q = second.startPoint;
            const SCVector2d r = first.endPoint - first.startPoint;
            const SCVector2d s = second.endPoint - second.startPoint;
            const SCVector2d qp = q - p;
            const double firstLength = std::hypot(r.x, r.y);
            const double secondLength = std::hypot(s.x, s.y);
            if (!std::isfinite(firstLength) || !std::isfinite(secondLength) ||
                firstLength <= eps || secondLength <= eps)
            {
                return MakeNoIntersection();
            }
            const double firstParameterTolerance = eps / firstLength;
            const double secondParameterTolerance = eps / secondLength;
            const auto pointOnFirst = [&](double parameter) {
                return SCPoint2d{p.x + r.x * parameter, p.y + r.y * parameter};
            };

            const double rxs = Cross(r, s);
            const double qpxr = Cross(qp, r);
            if (!std::isfinite(rxs) || !std::isfinite(qpxr))
            {
                return MakeNoIntersection();
            }

            if (std::abs(rxs) <= eps * std::max(firstLength, secondLength))
            {
                if (std::abs(qpxr) > eps * firstLength)
                {
                    return MakeNoIntersection();
                }

                const double rr = Dot(r, r);
                const double ss = Dot(s, s);
                if (!(rr > 0.0) || !(ss > 0.0) || !std::isfinite(rr) || !std::isfinite(ss))
                {
                    return MakeNoIntersection();
                }

                double t0 = Dot(q - p, r) / rr;
                double t1 = Dot(second.endPoint - p, r) / rr;
                if (!std::isfinite(t0) || !std::isfinite(t1))
                {
                    return MakeNoIntersection();
                }
                if (t0 > t1)
                {
                    std::swap(t0, t1);
                }

                const double overlapStart = std::max(0.0, t0);
                const double overlapEnd = std::min(1.0, t1);
                if (overlapEnd + firstParameterTolerance < overlapStart)
                {
                    return MakeNoIntersection();
                }

                const SCPoint2d firstPoint = pointOnFirst(overlapStart);
                const SCPoint2d secondPoint = pointOnFirst(overlapEnd);
                const double firstParameterOnSecond = Dot(firstPoint - second.startPoint, s) / ss;
                const double secondParameterOnSecond = Dot(secondPoint - second.startPoint, s) / ss;
                if (!std::isfinite(firstParameterOnSecond) || !std::isfinite(secondParameterOnSecond))
                {
                    return MakeNoIntersection();
                }

                if (std::abs(overlapStart - overlapEnd) <= firstParameterTolerance)
                {
                    return MakePointIntersection(
                        firstPoint, overlapStart, firstParameterOnSecond, SCIntersectionKind2d::Point);
                }

                return MakeOverlapIntersection(SCIntersectionPoint2d{firstPoint, overlapStart, firstParameterOnSecond},
                                               SCIntersectionPoint2d{secondPoint, overlapEnd, secondParameterOnSecond});
            }

            const double t = Cross(qp, s) / rxs;
            const double u = Cross(qp, r) / rxs;
            if (!std::isfinite(t) || !std::isfinite(u))
            {
                return MakeNoIntersection();
            }
            if (t < -firstParameterTolerance || t > 1.0 + firstParameterTolerance ||
                u < -secondParameterTolerance || u > 1.0 + secondParameterTolerance)
            {
                return MakeNoIntersection();
            }

            const double clampedT = Clamp(t, 0.0, 1.0);
            const double clampedU = Clamp(u, 0.0, 1.0);
            const SCPoint2d intersectionPoint = pointOnFirst(clampedT);
            return MakePointIntersection(intersectionPoint, clampedT, clampedU);
        }

        [[nodiscard]] bool LineCircleParameters(const SCPoint2d& origin,
                                                const SCVector2d& direction,
                                                const SCArcSegment2d& arc,
                                                double eps,
                                                std::array<double, 2>& parameters,
                                                double& lineDistance)
        {
            const double lineLength = direction.Length();
            const SCVector2d fromCenter = origin - arc.center;
            if (!std::isfinite(lineLength) || !(lineLength > 0.0) || !fromCenter.IsValid())
            {
                return false;
            }

            const SCVector2d unitDirection = direction / lineLength;
            double signedDistance = Cross(fromCenter, unitDirection);
            double alongLine = Dot(fromCenter, unitDirection);
            double offsetScale = 1.0;
            if (!std::isfinite(signedDistance) || !std::isfinite(alongLine))
            {
                const SCVector2d halfOffset{fromCenter.x * 0.5, fromCenter.y * 0.5};
                signedDistance = Cross(halfOffset, unitDirection);
                alongLine = Dot(halfOffset, unitDirection);
                offsetScale = 2.0;
            }
            lineDistance = std::abs(signedDistance) * offsetScale;
            if (!std::isfinite(lineDistance) || !std::isfinite(alongLine) || lineDistance - arc.radius > eps)
            {
                return false;
            }

            const double halfChord = std::sqrt(std::max(0.0, arc.radius - lineDistance))
                                   * std::sqrt(arc.radius) * std::sqrt(1.0 + lineDistance / arc.radius);
            if (!std::isfinite(halfChord))
            {
                return false;
            }
            parameters = {((-alongLine - halfChord / offsetScale) / lineLength) * offsetScale,
                          ((-alongLine + halfChord / offsetScale) / lineLength) * offsetScale};
            return true;
        }

        [[nodiscard]] SCSegmentIntersection2d IntersectLineArcInternal(const SCLineSegment2d& line,
                                                                     const SCArcSegment2d& arc,
                                                                     double eps)
        {
            if (!std::isfinite(eps) || eps <= 0.0 || !line.IsValid(eps) || !arc.IsValid(eps))
            {
                return MakeNoIntersection();
            }

            const SCVector2d direction = line.endPoint - line.startPoint;
            std::array<double, 2> roots{};
            double lineDistance = 0.0;
            if (!LineCircleParameters(line.startPoint, direction, arc, eps, roots, lineDistance))
            {
                return MakeNoIntersection();
            }
            const double lineLength = direction.Length();
            const double parameterEps = eps / lineLength;
            const double angleEps = eps / arc.radius;

            SCSegmentIntersection2d result;
            bool hasPoint = false;
            for (double root : roots)
            {
                if (!std::isfinite(root) || root < -parameterEps || root > 1.0 + parameterEps)
                {
                    continue;
                }

                const double parameter = Clamp(root, 0.0, 1.0);
                const SCPoint2d point = line.PointAt(parameter, eps);
                const double angle = std::atan2(point.y - arc.center.y, point.x - arc.center.x);
                if (!IsAngleOnArc(arc, angle, angleEps))
                {
                    continue;
                }

                const double arcParameter = ArcParameterAtAngle(arc, angle);
                if (!hasPoint)
                {
                    result = MakePointIntersection(
                        point,
                        parameter,
                        arcParameter,
                        std::abs(lineDistance - arc.radius) <= eps
                            ? SCIntersectionKind2d::Tangent : SCIntersectionKind2d::Point);
                    hasPoint = true;
                    continue;
                }

                AddIntersectionPoint(result, SCIntersectionKind2d::Point, point, parameter, arcParameter, eps);
                if (result.pointCount == 2)
                {
                    result.kind = SCIntersectionKind2d::Point;
                }
            }

            return result;
        }

        [[nodiscard]] bool SameCircle(const SCArcSegment2d& first, const SCArcSegment2d& second, double eps)
        {
            return first.center.AlmostEquals(second.center, eps) && std::abs(first.radius - second.radius) <= eps;
        }

        [[nodiscard]] std::vector<SCSegmentIntersection2d> IntersectAllArcsSameCircle(
            const SCArcSegment2d& first, const SCArcSegment2d& second, double eps)
        {
            // 将第二弧在第一弧方向的展开角区间平移整周期，再逐个与 [0, firstLength] 相交。
            // 不跨第二弧参数接缝合并区间，避免端点参数线性插值穿过错误的半圆。
            const double direction = first.sweepAngle > 0.0 ? 1.0 : -1.0;
            const double firstLength = std::abs(first.sweepAngle);
            const double secondSweep = direction * second.sweepAngle;
            const double secondStart = NormalizeAngle(direction *
                (NormalizeAngle(second.startAngle) - NormalizeAngle(first.startAngle)));
            const double angleEps = std::min(eps / first.radius, eps / second.radius);
            const double parameterEps = angleEps / firstLength;
            std::vector<SCSegmentIntersection2d> candidates;
            for (int period = -2; period <= 2; ++period)
            {
                const double start = secondStart + period * kTwoPi;
                const double end = start + secondSweep;
                const double low = std::max(0.0, std::min(start, end));
                const double high = std::min(firstLength, std::max(start, end));
                if (low - high > angleEps)
                {
                    continue;
                }
                const auto pointAt = [&](double angle) {
                    const double firstParameter = Clamp(angle / firstLength, 0.0, 1.0);
                    const double secondParameter = Clamp((angle - start) / secondSweep, 0.0, 1.0);
                    return SCIntersectionPoint2d{
                        first.PointAt(firstParameter, eps), firstParameter, secondParameter};
                };
                if (high - low <= angleEps)
                {
                    const auto point = pointAt(0.5 * (low + high));
                    candidates.push_back(MakePointIntersection(point.point, point.parameterOnFirst,
                        point.parameterOnSecond, SCIntersectionKind2d::Point));
                }
                else
                {
                    candidates.push_back(MakeOverlapIntersection(pointAt(low), pointAt(high)));
                }
            }
            std::sort(candidates.begin(), candidates.end(), [](const auto& lhs, const auto& rhs) {
                return lhs.points[0].parameterOnFirst < rhs.points[0].parameterOnFirst;
            });
            std::vector<SCSegmentIntersection2d> result;
            for (const auto& candidate : candidates)
            {
                if (candidate.kind == SCIntersectionKind2d::Point)
                {
                    const double parameter = candidate.points[0].parameterOnFirst;
                    const bool covered = std::any_of(candidates.begin(), candidates.end(), [&](const auto& other) {
                        return other.kind == SCIntersectionKind2d::Overlap &&
                            parameter >= other.points[0].parameterOnFirst - parameterEps &&
                            parameter <= other.points[1].parameterOnFirst + parameterEps;
                    });
                    const bool duplicate = std::any_of(result.begin(), result.end(), [&](const auto& other) {
                        return other.kind == SCIntersectionKind2d::Point &&
                            std::abs(parameter - other.points[0].parameterOnFirst) <= parameterEps;
                    });
                    if (covered || duplicate)
                    {
                        continue;
                    }
                }
                result.push_back(candidate);
            }
            return result;
        }

        [[nodiscard]] SCSegmentIntersection2d IntersectArcArcSameCircle(const SCArcSegment2d& first,
                                                                      const SCArcSegment2d& second,
                                                                      double eps)
        {
            const double firstAngleEps = eps / first.radius;
            const double secondAngleEps = eps / second.radius;
            const bool firstFullCircle = AlmostEqual(std::abs(first.sweepAngle), kTwoPi, firstAngleEps);
            const bool secondFullCircle = AlmostEqual(std::abs(second.sweepAngle), kTwoPi, secondAngleEps);
            if (firstFullCircle && secondFullCircle)
            {
                const double middleAngle = first.startAngle + first.sweepAngle * 0.5;
                return MakeOverlapIntersection(
                    SCIntersectionPoint2d{first.StartPoint(), 0.0,
                                          ArcParameterAtAngle(second, first.startAngle)},
                    SCIntersectionPoint2d{first.PointAt(0.5), 0.5,
                                          ArcParameterAtAngle(second, middleAngle)});
            }

            const SCPoint2d firstStart = first.StartPoint();
            const SCPoint2d firstEnd = first.EndPoint();
            const SCPoint2d secondStart = second.StartPoint();
            const SCPoint2d secondEnd = second.EndPoint();

            const bool firstStartOnSecond = IsAngleOnArc(second, first.startAngle, secondAngleEps);
            const bool firstEndOnSecond = IsAngleOnArc(second, first.EndAngle(), secondAngleEps);
            const bool secondStartOnFirst = IsAngleOnArc(first, second.startAngle, firstAngleEps);
            const bool secondEndOnFirst = IsAngleOnArc(first, second.EndAngle(), firstAngleEps);

            std::array<SCIntersectionPoint2d, 4> candidates{};
            std::size_t candidateCount = 0;

            auto addCandidate = [&](const SCPoint2d& point, double parameterOnFirst, double parameterOnSecond) {
                for (std::size_t index = 0; index < candidateCount; ++index)
                {
                    if (AlmostEqualPoint(candidates[index].point, point, eps))
                    {
                        candidates[index].parameterOnFirst = parameterOnFirst;
                        candidates[index].parameterOnSecond = parameterOnSecond;
                        return;
                    }
                }

                if (candidateCount < candidates.size())
                {
                    candidates[candidateCount++] = SCIntersectionPoint2d{point, parameterOnFirst, parameterOnSecond};
                }
            };

            if (firstStartOnSecond)
            {
                addCandidate(firstStart, 0.0, ArcParameterAtAngle(second, first.startAngle));
            }

            if (firstEndOnSecond)
            {
                addCandidate(firstEnd, 1.0, ArcParameterAtAngle(second, first.EndAngle()));
            }

            if (secondStartOnFirst)
            {
                addCandidate(secondStart, ArcParameterAtAngle(first, second.startAngle), 0.0);
            }

            if (secondEndOnFirst)
            {
                addCandidate(secondEnd, ArcParameterAtAngle(first, second.EndAngle()), 1.0);
            }

            if (candidateCount == 0)
            {
                return MakeNoIntersection();
            }

            if (candidateCount == 1)
            {
                return MakePointIntersection(candidates[0].point,
                                             candidates[0].parameterOnFirst,
                                             candidates[0].parameterOnSecond,
                                             SCIntersectionKind2d::Point);
            }

            if (candidateCount == 2)
            {
                if (AlmostEqualPoint(candidates[0].point, candidates[1].point, eps))
                {
                    return MakePointIntersection(candidates[0].point,
                                                 candidates[0].parameterOnFirst,
                                                 candidates[0].parameterOnSecond,
                                                 SCIntersectionKind2d::Point);
                }

                if (candidates[0].parameterOnFirst > candidates[1].parameterOnFirst)
                {
                    std::swap(candidates[0], candidates[1]);
                }

                SCSegmentIntersection2d result = MakeOverlapIntersection(candidates[0], candidates[1]);
                const double midpointParameter =
                    0.5 * (candidates[0].parameterOnFirst + candidates[1].parameterOnFirst);
                const double midpointAngle = first.startAngle + first.sweepAngle * midpointParameter;
                if (!IsAngleOnArc(second, midpointAngle, secondAngleEps))
                {
                    result.kind = SCIntersectionKind2d::Point;
                }
                return result;
            }

            std::sort(candidates.begin(),
                      candidates.begin() + static_cast<std::ptrdiff_t>(candidateCount),
                      [](const SCIntersectionPoint2d& lhs, const SCIntersectionPoint2d& rhs) {
                          return lhs.parameterOnFirst < rhs.parameterOnFirst;
                      });

            for (std::size_t index = 1; index < candidateCount; ++index)
            {
                const double midpointParameter =
                    0.5 * (candidates[index - 1].parameterOnFirst + candidates[index].parameterOnFirst);
                const double midpointAngle = first.startAngle + first.sweepAngle * midpointParameter;
                if (IsAngleOnArc(second, midpointAngle, secondAngleEps))
                {
                    return MakeOverlapIntersection(candidates[index - 1], candidates[index]);
                }
            }

            SCSegmentIntersection2d result = MakeOverlapIntersection(candidates[0], candidates[1]);
            result.kind = SCIntersectionKind2d::Point;
            return result;
        }

        [[nodiscard]] SCClosestPoints2d ClosestPointsLineLineInternal(const SCLineSegment2d& first,
                                                                    const SCLineSegment2d& second)
        {
            const SCSegmentIntersection2d intersection =
                IntersectLineLineInternal(first, second, Geometry::kDefaultEpsilon);
            if (intersection.HasIntersection())
            {
                const SCIntersectionPoint2d& point = intersection.points[0];
                return SCClosestPoints2d{point.point, point.point, point.parameterOnFirst, point.parameterOnSecond, 0.0};
            }

            SCClosestPoints2d best{};
            bool hasBest = false;

            const SCSegmentProjection2d firstStartProjection =
                ProjectPointToLineSegmentLocal(first.startPoint, second, true);
            UpdateClosest(
                best, hasBest, first.startPoint, firstStartProjection.point, 0.0, firstStartProjection.parameter);

            const SCSegmentProjection2d firstEndProjection = ProjectPointToLineSegmentLocal(first.endPoint, second, true);
            UpdateClosest(best, hasBest, first.endPoint, firstEndProjection.point, 1.0, firstEndProjection.parameter);

            const SCSegmentProjection2d secondStartProjection =
                ProjectPointToLineSegmentLocal(second.startPoint, first, true);
            UpdateClosest(
                best, hasBest, secondStartProjection.point, second.startPoint, secondStartProjection.parameter, 0.0);

            const SCSegmentProjection2d secondEndProjection =
                ProjectPointToLineSegmentLocal(second.endPoint, first, true);
            UpdateClosest(
                best, hasBest, secondEndProjection.point, second.endPoint, secondEndProjection.parameter, 1.0);

            return best;
        }

        [[nodiscard]] SCClosestPoints2d ClosestPointsLineArcInternal(const SCLineSegment2d& line, const SCArcSegment2d& arc)
        {
            const SCSegmentIntersection2d intersection =
                IntersectLineArcInternal(line, arc, Geometry::kDefaultEpsilon);
            if (intersection.HasIntersection())
            {
                const SCIntersectionPoint2d& point = intersection.points[0];
                return SCClosestPoints2d{point.point, point.point, point.parameterOnFirst, point.parameterOnSecond, 0.0};
            }

            SCClosestPoints2d best{};
            bool hasBest = false;

            const auto updateFromLineEndpoint = [&](const SCPoint2d& endpoint, double parameterOnLine) {
                const ArcProjectionCandidate projected =
                    ProjectPointToArcSegmentLocal(endpoint, arc, Geometry::kDefaultEpsilon);
                UpdateClosest(best, hasBest, endpoint, projected.point, parameterOnLine, projected.parameter);
            };

            const auto updateFromArcEndpoint = [&](const SCPoint2d& endpoint, double parameterOnArc) {
                const SCSegmentProjection2d projection = ProjectPointToLineSegmentLocal(endpoint, line, true);
                UpdateClosest(best, hasBest, projection.point, endpoint, projection.parameter, parameterOnArc);
            };

            updateFromLineEndpoint(line.startPoint, 0.0);
            updateFromLineEndpoint(line.endPoint, 1.0);

            updateFromArcEndpoint(arc.StartPoint(), 0.0);
            updateFromArcEndpoint(arc.EndPoint(), 1.0);

            const SCSegmentProjection2d centerProjection = ProjectPointToLineSegmentLocal(arc.center, line, true);
            const SCVector2d fromCenter = centerProjection.point - arc.center;
            if (fromCenter.LengthSquared() >
                Geometry::kDefaultEpsilon * Geometry::kDefaultEpsilon)
            {
                const double angle = std::atan2(fromCenter.y, fromCenter.x);
                if (IsAngleOnArc(arc, angle, Geometry::kDefaultAngleEpsilon))
                {
                    const SCPoint2d arcPoint = PointAtAngle(arc, angle);
                    UpdateClosest(best,
                                  hasBest,
                                  centerProjection.point,
                                  arcPoint,
                                  centerProjection.parameter,
                                  ArcParameterAtAngle(arc, angle));
                }
            }

            return best;
        }

        [[nodiscard]] SCClosestPoints2d ClosestPointsArcArcInternal(const SCArcSegment2d& first, const SCArcSegment2d& second)
        {
            const SCSegmentIntersection2d intersection =
                IntersectArcArcInternal(first, second, Geometry::kDefaultEpsilon);
            if (intersection.HasIntersection())
            {
                const SCIntersectionPoint2d& point = intersection.points[0];
                return SCClosestPoints2d{point.point, point.point, point.parameterOnFirst, point.parameterOnSecond, 0.0};
            }

            SCClosestPoints2d best{};
            bool hasBest = false;

            const auto updateFromFirstEndpoint = [&](const SCPoint2d& endpoint, double parameterOnFirst) {
                const ArcProjectionCandidate projected =
                    ProjectPointToArcSegmentLocal(endpoint, second, Geometry::kDefaultEpsilon);
                UpdateClosest(best, hasBest, endpoint, projected.point, parameterOnFirst, projected.parameter);
            };

            const auto updateFromSecondEndpoint = [&](const SCPoint2d& endpoint, double parameterOnSecond) {
                const ArcProjectionCandidate projected =
                    ProjectPointToArcSegmentLocal(endpoint, first, Geometry::kDefaultEpsilon);
                UpdateClosest(best, hasBest, projected.point, endpoint, projected.parameter, parameterOnSecond);
            };

            updateFromFirstEndpoint(first.StartPoint(), 0.0);
            updateFromFirstEndpoint(first.EndPoint(), 1.0);
            updateFromSecondEndpoint(second.StartPoint(), 0.0);
            updateFromSecondEndpoint(second.EndPoint(), 1.0);

            const SCVector2d centerDelta = second.center - first.center;
            const double centerDistanceSquared = Dot(centerDelta, centerDelta);
            if (centerDistanceSquared > Geometry::kDefaultEpsilon * Geometry::kDefaultEpsilon)
            {
                const double centerDistance = std::sqrt(centerDistanceSquared);
                const SCVector2d unit = centerDelta / centerDistance;
                const SCPoint2d firstCandidate =
                    MakePoint(first.center.x + first.radius * unit.x, first.center.y + first.radius * unit.y);
                const SCPoint2d secondCandidate =
                    MakePoint(second.center.x - second.radius * unit.x, second.center.y - second.radius * unit.y);
                const double firstAngle =
                    std::atan2(firstCandidate.y - first.center.y, firstCandidate.x - first.center.x);
                const double secondAngle =
                    std::atan2(secondCandidate.y - second.center.y, secondCandidate.x - second.center.x);
                if (IsAngleOnArc(first, firstAngle, Geometry::kDefaultAngleEpsilon) &&
                    IsAngleOnArc(second, secondAngle, Geometry::kDefaultAngleEpsilon))
                {
                    UpdateClosest(best,
                                  hasBest,
                                  firstCandidate,
                                  secondCandidate,
                                  ArcParameterAtAngle(first, firstAngle),
                                  ArcParameterAtAngle(second, secondAngle));
                }
            }

            return best;
        }

        [[nodiscard]] SCSegmentIntersection2d SwapIntersectionResult(SCSegmentIntersection2d result)
        {
            if (result.pointCount >= 1)
            {
                std::swap(result.points[0].parameterOnFirst, result.points[0].parameterOnSecond);
            }

            if (result.pointCount == 2)
            {
                std::swap(result.points[1].parameterOnFirst, result.points[1].parameterOnSecond);
                std::swap(result.points[0], result.points[1]);
            }

            return result;
        }

        [[nodiscard]] SCClosestPoints2d SwapClosestPointsResult(SCClosestPoints2d result)
        {
            std::swap(result.firstPoint, result.secondPoint);
            std::swap(result.parameterOnFirst, result.parameterOnSecond);
            return result;
        }

        [[nodiscard]] SCSegmentIntersection2d IntersectArcArcInternal(const SCArcSegment2d& first,
                                                                    const SCArcSegment2d& second,
                                                                    double eps)
        {
            if (!std::isfinite(eps) || eps <= 0.0 || !first.IsValid(eps) || !second.IsValid(eps))
            {
                return MakeNoIntersection();
            }

            if (SameCircle(first, second, eps))
            {
                return IntersectArcArcSameCircle(first, second, eps);
            }

            const SCVector2d centerDelta = second.center - first.center;
            const double distance = std::hypot(centerDelta.x, centerDelta.y);
            if (!std::isfinite(distance) || distance <= eps)
            {
                return MakeNoIntersection();
            }

            if (distance > first.radius + second.radius + eps ||
                distance < std::abs(first.radius - second.radius) - eps)
            {
                return MakeNoIntersection();
            }

            const double a =
                0.5 * (distance + (first.radius-second.radius) * (first.radius/distance+second.radius/distance));
            const double ratio = a/first.radius;
            const double hSquared = (1.0-ratio)*(1.0+ratio);
            if (!std::isfinite(a) || !std::isfinite(hSquared) ||
                (hSquared < 0 && std::abs(a)-first.radius > eps))
            {
                return MakeNoIntersection();
            }

            const double h = hSquared <= 0.0 ? 0.0 : first.radius*std::sqrt(hSquared);
            const SCVector2d unit = centerDelta / distance;
            const SCPoint2d basePoint = MakePoint(first.center.x + a * unit.x, first.center.y + a * unit.y);
            const SCVector2d perpendicular{-unit.y, unit.x};

            const SCPoint2d candidateOne =
                MakePoint(basePoint.x + h * perpendicular.x, basePoint.y + h * perpendicular.y);
            const SCPoint2d candidateTwo =
                MakePoint(basePoint.x - h * perpendicular.x, basePoint.y - h * perpendicular.y);

            SCSegmentIntersection2d result;
            bool hasPoint = false;
            for (const SCPoint2d& point : std::array<SCPoint2d, 2>{candidateOne, candidateTwo})
            {
                const double angleFirst = std::atan2(point.y - first.center.y, point.x - first.center.x);
                const double angleSecond = std::atan2(point.y - second.center.y, point.x - second.center.x);
                if (!IsAngleOnArc(first, angleFirst, eps / first.radius) ||
                    !IsAngleOnArc(second, angleSecond, eps / second.radius))
                {
                    continue;
                }

                const double firstParameter = ArcParameterAtAngle(first, angleFirst);
                const double secondParameter = ArcParameterAtAngle(second, angleSecond);
                if (!hasPoint)
                {
                    result = MakePointIntersection(
                        point,
                        firstParameter,
                        secondParameter,
                        AlmostEqual(h, 0.0, eps) ? SCIntersectionKind2d::Tangent : SCIntersectionKind2d::Point);
                    hasPoint = true;
                    continue;
                }

                AddIntersectionPoint(result, SCIntersectionKind2d::Point, point, firstParameter, secondParameter, eps);
                if (result.pointCount == 2)
                {
                    result.kind = SCIntersectionKind2d::Point;
                }
            }

            return result;
        }

        [[nodiscard]] SCExtendedIntersection2d IntersectArcArcExtendedInternal(const SCArcSegment2d& first,
                                                                                const SCArcSegment2d& second,
                                                                                bool extendFirst,
                                                                                bool extendSecond,
                                                                                double eps)
        {
            SCExtendedIntersection2d result;
            if (!std::isfinite(eps) || eps <= 0.0 || !first.IsValid(eps) || !second.IsValid(eps))
            {
                return result;
            }

            if (SameCircle(first, second, eps))
            {
                const bool firstActsAsFullCircle =
                    extendFirst || AlmostEqual(std::abs(first.sweepAngle), kTwoPi, eps / first.radius);
                const bool secondActsAsFullCircle =
                    extendSecond || AlmostEqual(std::abs(second.sweepAngle), kTwoPi, eps / second.radius);

                if (firstActsAsFullCircle && secondActsAsFullCircle)
                {
                    result.kind = SCIntersectionKind2d::Overlap;
                    result.infiniteOverlap = true;
                    return result;
                }

                if (firstActsAsFullCircle)
                {
                    return MakeExtendedOverlapIntersection(
                        SCIntersectionPoint2d{second.StartPoint(), ArcParameterAtAngle(first, second.startAngle), 0.0},
                        SCIntersectionPoint2d{second.EndPoint(), ArcParameterAtAngle(first, second.EndAngle()), 1.0},
                        !extendFirst,
                        true);
                }

                if (secondActsAsFullCircle)
                {
                    return MakeExtendedOverlapIntersection(
                        SCIntersectionPoint2d{first.StartPoint(), 0.0, ArcParameterAtAngle(second, first.startAngle)},
                        SCIntersectionPoint2d{first.EndPoint(), 1.0, ArcParameterAtAngle(second, first.EndAngle())},
                        true,
                        !extendSecond);
                }

                const SCSegmentIntersection2d intersection = IntersectArcArcSameCircle(first, second, eps);
                if (!intersection.HasIntersection())
                {
                    return result;
                }

                result.kind = intersection.kind;
                result.pointCount = intersection.pointCount;
                for (std::size_t index = 0; index < intersection.pointCount; ++index)
                {
                    result.points[index] = intersection.points[index];
                }
                result.onFirstSegment = true;
                result.onSecondSegment = true;
                return result;
            }

            const SCVector2d centerDelta = second.center - first.center;
            const double distance = std::hypot(centerDelta.x, centerDelta.y);
            if (!std::isfinite(distance) || distance <= eps)
            {
                return result;
            }

            if (distance > first.radius + second.radius + eps ||
                distance < std::abs(first.radius - second.radius) - eps)
            {
                return result;
            }

            const double a =
                0.5 * (distance + (first.radius-second.radius) * (first.radius/distance+second.radius/distance));
            const double ratio = a/first.radius;
            const double hSquared = (1.0-ratio)*(1.0+ratio);
            if (!std::isfinite(a) || !std::isfinite(hSquared) ||
                (hSquared < 0 && std::abs(a)-first.radius > eps))
            {
                return result;
            }

            const double h = hSquared <= 0.0 ? 0.0 : first.radius*std::sqrt(hSquared);
            const SCVector2d unit = centerDelta / distance;
            const SCPoint2d basePoint = MakePoint(first.center.x + a * unit.x, first.center.y + a * unit.y);
            const SCVector2d perpendicular{-unit.y, unit.x};

            const SCPoint2d candidateOne =
                MakePoint(basePoint.x + h * perpendicular.x, basePoint.y + h * perpendicular.y);
            const SCPoint2d candidateTwo =
                MakePoint(basePoint.x - h * perpendicular.x, basePoint.y - h * perpendicular.y);

            SCSegmentIntersection2d intersection;
            bool hasPoint = false;
            for (const SCPoint2d& point : std::array<SCPoint2d, 2>{candidateOne, candidateTwo})
            {
                const double angleFirst = std::atan2(point.y - first.center.y, point.x - first.center.x);
                const double angleSecond = std::atan2(point.y - second.center.y, point.x - second.center.x);
                if ((!extendFirst && !IsAngleOnArc(first, angleFirst, eps / first.radius)) ||
                    (!extendSecond && !IsAngleOnArc(second, angleSecond, eps / second.radius)))
                {
                    continue;
                }

                const double firstParameter = ArcParameterAtAngle(first, angleFirst);
                const double secondParameter = ArcParameterAtAngle(second, angleSecond);
                if (!hasPoint)
                {
                    intersection = MakePointIntersection(
                        point,
                        firstParameter,
                        secondParameter,
                        AlmostEqual(h, 0.0, eps) ? SCIntersectionKind2d::Tangent : SCIntersectionKind2d::Point);
                    hasPoint = true;
                    continue;
                }

                AddIntersectionPoint(intersection,
                                     SCIntersectionKind2d::Point,
                                     point,
                                     firstParameter,
                                     secondParameter,
                                     eps);
                if (intersection.pointCount == 2)
                {
                    intersection.kind = SCIntersectionKind2d::Point;
                }
            }

            if (!intersection.HasIntersection())
            {
                return result;
            }

            result.kind = intersection.kind;
            result.pointCount = intersection.pointCount;
            for (std::size_t index = 0; index < intersection.pointCount; ++index)
            {
                result.points[index] = intersection.points[index];
            }
            result.onFirstSegment = true;
            result.onSecondSegment = true;
            for (std::size_t index = 0; index < intersection.pointCount; ++index)
            {
                const double firstParameter = result.points[index].parameterOnFirst;
                const double secondParameter = result.points[index].parameterOnSecond;
                const double firstParameterEps = eps / first.Length();
                const double secondParameterEps = eps / second.Length();
                result.onFirstSegment = result.onFirstSegment && firstParameter >= -firstParameterEps &&
                                        firstParameter <= 1.0 + firstParameterEps;
                result.onSecondSegment =
                    result.onSecondSegment && secondParameter >= -secondParameterEps &&
                    secondParameter <= 1.0 + secondParameterEps;
            }
            return result;
        }

        [[nodiscard]] SCClosestPoints2d ClosestPointsLineArcInternal(const SCLineSegment2d& line,
                                                                   const SCArcSegment2d& arc,
                                                                   double eps)
        {
            const SCSegmentIntersection2d intersection = IntersectLineArcInternal(line, arc, eps);
            if (intersection.HasIntersection())
            {
                const SCIntersectionPoint2d& point = intersection.points[0];
                return SCClosestPoints2d{point.point, point.point, point.parameterOnFirst, point.parameterOnSecond, 0.0};
            }

            SCClosestPoints2d best{};
            bool hasBest = false;

            const auto updateFromLineEndpoint = [&](const SCPoint2d& endpoint, double parameterOnLine) {
                const ArcProjectionCandidate projected = ProjectPointToArcSegmentLocal(endpoint, arc, eps);
                UpdateClosest(best, hasBest, endpoint, projected.point, parameterOnLine, projected.parameter);
            };

            const auto updateFromArcEndpoint = [&](const SCPoint2d& endpoint, double parameterOnArc) {
                const SCSegmentProjection2d projection = ProjectPointToLineSegmentLocal(endpoint, line, true);
                UpdateClosest(best, hasBest, projection.point, endpoint, projection.parameter, parameterOnArc);
            };

            updateFromLineEndpoint(line.startPoint, 0.0);
            updateFromLineEndpoint(line.endPoint, 1.0);
            updateFromArcEndpoint(arc.StartPoint(), 0.0);
            updateFromArcEndpoint(arc.EndPoint(), 1.0);

            const SCSegmentProjection2d centerProjection = ProjectPointToLineSegmentLocal(arc.center, line, true);
            const SCVector2d fromCenter = centerProjection.point - arc.center;
            if (std::hypot(fromCenter.x,fromCenter.y) > eps)
            {
                const double angle = std::atan2(fromCenter.y, fromCenter.x);
                if (IsAngleOnArc(arc, angle, eps))
                {
                    const SCPoint2d arcPoint = PointAtAngle(arc, angle);
                    UpdateClosest(best,
                                  hasBest,
                                  centerProjection.point,
                                  arcPoint,
                                  centerProjection.parameter,
                                  ArcParameterAtAngle(arc, angle));
                }
            }

            return best;
        }
    }  // namespace

    SCSegmentIntersection2d Intersect(const SCLineSegment2d& first, const SCLineSegment2d& second, double eps)
    {
        return IntersectLineLineInternal(first, second, eps);
    }

    SCSegmentIntersection2d Intersect(const SCLineSegment2d& first, const SCArcSegment2d& second, double eps)
    {
        return IntersectLineArcInternal(first, second, eps);
    }

    SCSegmentIntersection2d Intersect(const SCArcSegment2d& first, const SCArcSegment2d& second, double eps)
    {
        return IntersectArcArcInternal(first, second, eps);
    }

    SCSegmentIntersection2d Intersect(const ISCSegment2d& first, const ISCSegment2d& second, double eps)
    {
        return Detail::IntersectKernelSegments(first, second, eps);
    }

    std::vector<SCSegmentIntersection2d> IntersectAll(
        const ISCSegment2d& first, const ISCSegment2d& second, double eps)
    {
        if (!SegmentValidAtTolerance(first, eps) || !SegmentValidAtTolerance(second, eps))
        {
            return {};
        }
        const auto* firstArc = dynamic_cast<const SCArcSegment2d*>(&first);
        const auto* secondArc = dynamic_cast<const SCArcSegment2d*>(&second);
        if (firstArc && secondArc && SameCircle(*firstArc, *secondArc, eps))
        {
            return IntersectAllArcsSameCircle(*firstArc, *secondArc, eps);
        }
        const auto intersection = Intersect(first, second, eps);
        if (!intersection.HasIntersection())
        {
            return {};
        }
        if (intersection.kind == SCIntersectionKind2d::Overlap)
        {
            return {intersection};
        }
        std::vector<SCSegmentIntersection2d> result;
        for (std::size_t i = 0; i < intersection.pointCount; ++i)
        {
            const auto& point = intersection.points[i];
            result.push_back(MakePointIntersection(point.point, point.parameterOnFirst,
                point.parameterOnSecond, intersection.kind));
        }
        std::sort(result.begin(), result.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.points[0].parameterOnFirst < rhs.points[0].parameterOnFirst;
        });
        return result;
    }

    bool HasIntersection(const ISCSegment2d& first, const ISCSegment2d& second, double eps)
    {
        return Intersect(first, second, eps).HasIntersection();
    }

    SCClosestPoints2d ClosestPoints(const SCLineSegment2d& first, const SCLineSegment2d& second)
    {
        return ClosestPointsLineLineInternal(first, second);
    }

    SCClosestPoints2d ClosestPoints(const SCLineSegment2d& first, const SCArcSegment2d& second)
    {
        return ClosestPointsLineArcInternal(first, second, Geometry::kDefaultEpsilon);
    }

    SCClosestPoints2d ClosestPoints(const SCArcSegment2d& first, const SCArcSegment2d& second)
    {
        return ClosestPointsArcArcInternal(first, second);
    }

    SCClosestPoints2d ClosestPoints(const ISCSegment2d& first, const ISCSegment2d& second)
    {
        return Detail::ClosestKernelSegments(first, second);
    }

        [[nodiscard]] double LineParameterAtPoint(const SCLine2d& line, const SCPoint2d& point, double eps)
        {
            const SCVector2d delta = point - line.origin;
            const double length = std::hypot(line.direction.x,line.direction.y);
            if (!std::isfinite(length) || length <= eps)
            {
                return 0.0;
            }
            return Dot(delta, line.direction/length) / length;
        }

        [[nodiscard]] SCLineIntersection2d MakeNoLineIntersection()
        {
            return {};
        }

        [[nodiscard]] SCLineIntersection2d MakeLinePointIntersection(const SCPoint2d& point,
                                                                    double firstParameter,
                                                                    double secondParameter)
        {
            SCLineIntersection2d result;
            result.kind = SCIntersectionKind2d::Point;
            result.pointCount = 1;
            result.points[0] = SCIntersectionPoint2d{point, firstParameter, secondParameter};
            return result;
        }

        [[nodiscard]] SCLineIntersection2d SwapLineIntersection(SCLineIntersection2d result)
        {
            if (result.pointCount >= 1)
            {
                std::swap(result.points[0].parameterOnFirst, result.points[0].parameterOnSecond);
            }
            if (result.pointCount == 2)
            {
                std::swap(result.points[1].parameterOnFirst, result.points[1].parameterOnSecond);
                std::swap(result.points[0], result.points[1]);
            }
            return result;
        }

        [[nodiscard]] SCExtendedIntersection2d MakeNoExtendedIntersection()
        {
            return {};
        }

        [[nodiscard]] SCExtendedIntersection2d MakeExtendedIntersectionFromIntersection(const SCSegmentIntersection2d& intersection)
        {
            SCExtendedIntersection2d result;
            if (!intersection.HasIntersection())
            {
                return result;
            }

            result.kind = intersection.kind;
            result.pointCount = intersection.pointCount;
            result.infiniteOverlap = false;
            for (std::size_t index = 0; index < intersection.pointCount; ++index)
            {
                result.points[index] = intersection.points[index];
            }
            if (intersection.pointCount >= 1)
            {
                result.onFirstSegment = true;
                result.onSecondSegment = true;
            }
            return result;
        }

        [[nodiscard]] SCExtensionPolicy SwapExtendedIntersectionPolicy(SCExtensionPolicy policy)
        {
            switch (policy)
            {
                case SCExtensionPolicy::ExtendFirst:
                    return SCExtensionPolicy::ExtendSecond;
                case SCExtensionPolicy::ExtendSecond:
                    return SCExtensionPolicy::ExtendFirst;
                default:
                    return policy;
            }
        }

        [[nodiscard]] double PolylineSegmentGlobalParameter(const SCPolyline2d& polyline,
                                                            std::size_t segmentIndex,
                                                            double localParameter,
                                                            const std::vector<double>& segmentLengths,
                                                            double totalLength)
        {
            if (!(totalLength > 0.0) || !std::isfinite(totalLength))
            {
                return 0.0;
            }

            double lengthBefore = 0.0;
            for (std::size_t i = 0; i < segmentIndex && i < segmentLengths.size(); ++i)
            {
                lengthBefore += segmentLengths[i];
            }

            return (lengthBefore + segmentLengths[segmentIndex] * localParameter) / totalLength;
        }

        void AppendPolylineIntersection(std::vector<SCPolylineIntersectionPoint2d>& result,
                                        const SCPolylineIntersectionPoint2d& candidate,
                                        double eps,
                                        double firstLength,
                                        double secondLength,
                                        bool firstClosed,
                                        bool secondClosed)
        {
            const double firstParameterEps = firstLength > 0.0 ? eps / firstLength : 0.0;
            const double secondParameterEps = secondLength > 0.0 ? eps / secondLength : 0.0;
            auto samePosition = [](double lhs, double rhs, double parameterEps, bool closed) {
                const double separation = std::abs(lhs - rhs);
                return separation <= parameterEps || (closed && std::abs(1.0 - separation) <= parameterEps);
            };
            for (auto& existing : result)
            {
                if (existing.point.AlmostEquals(candidate.point, eps) &&
                    samePosition(existing.globalParameterOnFirst, candidate.globalParameterOnFirst,
                                 firstParameterEps, firstClosed) &&
                    samePosition(existing.globalParameterOnSecond, candidate.globalParameterOnSecond,
                                 secondParameterEps, secondClosed))
                {
                    if (candidate.globalParameterOnFirst < existing.globalParameterOnFirst)
                    {
                        existing = candidate;
                    }
                    return;
                }
            }

            result.push_back(candidate);
        }

        [[nodiscard]] SCLineIntersection2d IntersectLineLineInternal(const SCLine2d& first,
                                                                     const SCLine2d& second,
                                                                     double eps)
        {
            SCLineIntersection2d result;
            if (!std::isfinite(eps) || eps < 0.0 || !first.IsValid(eps) || !second.IsValid(eps))
            {
                return result;
            }
            const SCVector2d r = first.direction;
            const SCVector2d s = second.direction;
            SCVector2d qp = second.origin - first.origin;
            double offsetScale = 1.0;
            if (!qp.IsValid())
            {
                qp = SCVector2d{0.5 * second.origin.x - 0.5 * first.origin.x,
                                0.5 * second.origin.y - 0.5 * first.origin.y};
                offsetScale = 2.0;
            }
            const double firstLength = r.Length();
            const double secondLength = s.Length();
            if (!qp.IsValid() || !std::isfinite(firstLength) || !std::isfinite(secondLength))
            {
                return result;
            }
            const SCVector2d unitFirst = r / firstLength;
            const SCVector2d unitSecond = s / secondLength;
            double det = Cross(unitFirst, unitSecond);
            double offsetCross = Cross(qp, unitFirst);
            double parallelThreshold = eps / std::min(firstLength, secondLength);
            double collinearThreshold = eps / offsetScale;
            double firstParameterScale = firstLength;
            double secondParameterScale = secondLength;
            bool useRawDirections = false;
            // 单位方向可能丢失可表示的极小斜率，此时复用仍可表示的原始叉积。
            if (det == 0.0)
            {
                const double rawDet = Cross(r, s);
                const double rawOffsetCross = Cross(qp, r);
                if (std::isfinite(rawDet) && rawDet != 0.0 && std::isfinite(rawOffsetCross))
                {
                    det = rawDet;
                    offsetCross = rawOffsetCross;
                    parallelThreshold = eps * std::max(firstLength, secondLength);
                    collinearThreshold = (eps / offsetScale) * firstLength;
                    firstParameterScale = 1.0;
                    secondParameterScale = 1.0;
                    useRawDirections = true;
                }
            }
            if (!std::isfinite(offsetCross))
            {
                return result;
            }
            if (std::abs(det) <= parallelThreshold)
            {
                result.parallel = true;
                if (std::abs(offsetCross) <= collinearThreshold)
                {
                    result.kind = SCIntersectionKind2d::Overlap;
                    result.collinear = true;
                    result.infiniteOverlap = true;
                }
                return result;
            }

            const double numeratorCross = Cross(qp, useRawDirections ? s : unitSecond);
            const double t = ((numeratorCross / det) / firstParameterScale) * offsetScale;
            const double u = ((offsetCross / det) / secondParameterScale) * offsetScale;
            if (!std::isfinite(t) || !std::isfinite(u))
            {
                return result;
            }
            const SCPoint2d point = first.PointAt(t);
            if (!point.IsValid())
            {
                return result;
            }
            result = MakeLinePointIntersection(point, t, u);
            return result;
        }

        [[nodiscard]] SCLineIntersection2d IntersectLineSegmentInternal(const SCLine2d& line,
                                                                        const SCLineSegment2d& segment,
                                                                        double eps)
        {
            const SCLine2d segmentLine = SCLine2d::FromTwoPoints(segment.startPoint, segment.endPoint);
            const SCLineIntersection2d lineLine = IntersectLineLineInternal(line, segmentLine, eps);
            if (lineLine.kind == SCIntersectionKind2d::Point)
            {
                const double segmentParameter = lineLine.points[0].parameterOnSecond;
                const double parameterEps = eps / segment.Length();
                if (segmentParameter < -parameterEps || segmentParameter > 1.0 + parameterEps)
                {
                    return {};
                }
                SCLineIntersection2d result = lineLine;
                result.points[0].parameterOnSecond = std::clamp(segmentParameter, 0.0, 1.0);
                return result;
            }

            if (lineLine.collinear)
            {
                const double firstParameter = LineParameterAtPoint(line, segment.startPoint, eps);
                const double secondParameter = LineParameterAtPoint(line, segment.endPoint, eps);
                SCLineIntersection2d result;
                result.kind = SCIntersectionKind2d::Overlap;
                result.parallel = true;
                result.collinear = true;
                result.infiniteOverlap = false;
                result.pointCount = 2;
                result.points[0] = SCIntersectionPoint2d{segment.startPoint, firstParameter, 0.0};
                result.points[1] = SCIntersectionPoint2d{segment.endPoint, secondParameter, 1.0};
                if (result.points[0].parameterOnFirst > result.points[1].parameterOnFirst)
                {
                    std::swap(result.points[0], result.points[1]);
                }
                return result;
            }

            return {};
        }

        [[nodiscard]] SCSegmentIntersection2d IntersectLineArcExtended(const SCLine2d& line,
                                                                       const SCArcSegment2d& arc,
                                                                       bool extendLine,
                                                                       bool extendArc,
                                                                       double eps)
        {
            if (!line.IsValid(eps) || !arc.IsValid(eps))
            {
                return {};
            }

            const SCVector2d direction = line.direction;
            std::array<double, 2> roots{};
            double lineDistance = 0.0;
            if (!LineCircleParameters(line.origin, direction, arc, eps, roots, lineDistance))
            {
                return {};
            }
            const double lineLength = direction.Length();
            const double parameterEps = eps / lineLength;
            const double angleEps = eps / arc.radius;

            SCSegmentIntersection2d result;
            bool hasPoint = false;
            for (double t : roots)
            {
                if (!std::isfinite(t))
                {
                    continue;
                }
                const SCPoint2d point = line.PointAt(t);
                if (!point.IsValid())
                {
                    continue;
                }
                const double angle = std::atan2(point.y - arc.center.y, point.x - arc.center.x);
                const double arcParameter = arc.sweepAngle >= 0.0 ? (angle - arc.startAngle) / arc.sweepAngle
                                                                   : (arc.startAngle - angle) / (-arc.sweepAngle);
                const bool onLine = extendLine || (t >= -parameterEps && t <= 1.0 + parameterEps);
                const bool onArc = extendArc || IsAngleOnArc(arc, angle, angleEps);
                if (!onLine || !onArc)
                {
                    continue;
                }

                double clampedT = t;
                if (!extendLine)
                {
                    clampedT = std::clamp(t, 0.0, 1.0);
                }

                double clampedArcParameter = arcParameter;
                if (!extendArc)
                {
                    clampedArcParameter = std::clamp(arcParameter, 0.0, 1.0);
                }
                if (!hasPoint)
                {
                    result.kind = std::abs(lineDistance - arc.radius) <= eps
                                      ? SCIntersectionKind2d::Tangent : SCIntersectionKind2d::Point;
                    result.pointCount = 1;
                    result.points[0] = SCIntersectionPoint2d{point, clampedT, clampedArcParameter};
                    hasPoint = true;
                }
                else
                {
                    AddIntersectionPoint(result,
                                         SCIntersectionKind2d::Point,
                                         point,
                                         clampedT,
                                         clampedArcParameter,
                                         eps);
                    if (result.pointCount == 2)
                    {
                        result.kind = SCIntersectionKind2d::Point;
                    }
                }
            }

            return result;
        }

        [[nodiscard]] std::vector<SCPolylineIntersectionPoint2d> IntersectPolylinePolylineInternal(const SCPolyline2d& first,
                                                                                                 const SCPolyline2d& second,
                                                                                                 double eps)
        {
            std::vector<SCPolylineIntersectionPoint2d> result;
            if (!first.IsValid(eps) || !second.IsValid(eps))
            {
                return result;
            }

            std::vector<double> firstSegmentLengths;
            std::vector<double> secondSegmentLengths;
            firstSegmentLengths.reserve(first.SegmentCount());
            secondSegmentLengths.reserve(second.SegmentCount());
            double firstTotalLength = 0.0;
            double secondTotalLength = 0.0;
            for (std::size_t i = 0; i < first.SegmentCount(); ++i)
            {
                const auto segment = first.SegmentAt(i);
                const double length = segment == nullptr ? 0.0 : SegmentLengthAtTolerance(*segment, eps);
                firstSegmentLengths.push_back(length);
                firstTotalLength += length;
            }
            for (std::size_t i = 0; i < second.SegmentCount(); ++i)
            {
                const auto segment = second.SegmentAt(i);
                const double length = segment == nullptr ? 0.0 : SegmentLengthAtTolerance(*segment, eps);
                secondSegmentLengths.push_back(length);
                secondTotalLength += length;
            }

            for (std::size_t i = 0; i < first.SegmentCount(); ++i)
            {
                const auto firstSegment = first.SegmentAt(i);
                if (firstSegment == nullptr)
                {
                    continue;
                }
                for (std::size_t j = 0; j < second.SegmentCount(); ++j)
                {
                    const auto secondSegment = second.SegmentAt(j);
                    if (secondSegment == nullptr)
                    {
                        continue;
                    }

                    for (const auto& intersection : IntersectAll(*firstSegment, *secondSegment, eps))
                    {
                        for (std::size_t index = 0; index < intersection.pointCount; ++index)
                        {
                            const SCIntersectionPoint2d& point = intersection.points[index];
                            const SCPolylineIntersectionPoint2d polyPoint{intersection.kind,
                                                                           point.point,
                                                                           i,
                                                                           j,
                                                                           point.parameterOnFirst,
                                                                           point.parameterOnSecond,
                                                                           PolylineSegmentGlobalParameter(first, i, point.parameterOnFirst, firstSegmentLengths, firstTotalLength),
                                                                           PolylineSegmentGlobalParameter(second, j, point.parameterOnSecond, secondSegmentLengths, secondTotalLength)};
                            AppendPolylineIntersection(result, polyPoint, eps, firstTotalLength, secondTotalLength,
                                                       first.IsClosed(), second.IsClosed());
                        }
                    }
                }
            }

            std::sort(result.begin(), result.end(), [](const SCPolylineIntersectionPoint2d& lhs, const SCPolylineIntersectionPoint2d& rhs) {
                if (lhs.segmentIndexOnFirst != rhs.segmentIndexOnFirst)
                {
                    return lhs.segmentIndexOnFirst < rhs.segmentIndexOnFirst;
                }
                if (lhs.parameterOnFirstSegment != rhs.parameterOnFirstSegment)
                {
                    return lhs.parameterOnFirstSegment < rhs.parameterOnFirstSegment;
                }
                return lhs.segmentIndexOnSecond < rhs.segmentIndexOnSecond;
            });
            return result;
        }

    SCLineIntersection2d Intersect(const SCLine2d& first, const SCLine2d& second, double eps)
    {
        return IntersectLineLineInternal(first, second, eps);
    }

    SCLineIntersection2d Intersect(const SCLine2d& first, const SCLineSegment2d& second, double eps)
    {
        return IntersectLineSegmentInternal(first, second, eps);
    }

    SCLineIntersection2d Intersect(const SCLineSegment2d& first, const SCLine2d& second, double eps)
    {
        return SwapLineIntersection(IntersectLineSegmentInternal(second, first, eps));
    }

    std::vector<SCPolylineIntersectionPoint2d> Intersect(const SCPolyline2d& first,
                                                         const SCPolyline2d& second,
                                                         double eps)
    {
        return IntersectPolylinePolylineInternal(first, second, eps);
    }

    std::vector<SCPolylineIntersectionPoint2d> Intersect(const SCPolyline2d& polyline,
                                                         const ISCSegment2d& segment,
                                                         double eps)
    {
        std::vector<SCPolylineIntersectionPoint2d> result;
        if (!polyline.IsValid(eps) || !SegmentValidAtTolerance(segment, eps))
        {
            return result;
        }

        std::vector<double> segmentLengths;
        segmentLengths.reserve(polyline.SegmentCount());
        double totalLength = 0.0;
        for (std::size_t i = 0; i < polyline.SegmentCount(); ++i)
        {
            const auto polySegment = polyline.SegmentAt(i);
            const double length = polySegment == nullptr ? 0.0 : SegmentLengthAtTolerance(*polySegment, eps);
            segmentLengths.push_back(length);
            totalLength += length;
        }
        const double otherLength = SegmentLengthAtTolerance(segment, eps);

        for (std::size_t i = 0; i < polyline.SegmentCount(); ++i)
        {
            const auto polySegment = polyline.SegmentAt(i);
            if (polySegment == nullptr)
            {
                continue;
            }

            for (const auto& intersection : IntersectAll(*polySegment, segment, eps))
            {
                for (std::size_t index = 0; index < intersection.pointCount; ++index)
                {
                    const SCIntersectionPoint2d& point = intersection.points[index];
                    const SCPolylineIntersectionPoint2d polyPoint{intersection.kind,
                                                                   point.point,
                                                                   i,
                                                                   0,
                                                                   point.parameterOnFirst,
                                                                   point.parameterOnSecond,
                                                                   PolylineSegmentGlobalParameter(polyline, i, point.parameterOnFirst, segmentLengths, totalLength),
                                                                   point.parameterOnSecond};
                    AppendPolylineIntersection(result, polyPoint, eps, totalLength, otherLength,
                                               polyline.IsClosed(), false);
                }
            }
        }

        std::sort(result.begin(), result.end(), [](const SCPolylineIntersectionPoint2d& lhs, const SCPolylineIntersectionPoint2d& rhs) {
            if (lhs.segmentIndexOnFirst != rhs.segmentIndexOnFirst)
            {
                return lhs.segmentIndexOnFirst < rhs.segmentIndexOnFirst;
            }
            return lhs.parameterOnFirstSegment < rhs.parameterOnFirstSegment;
        });
        return result;
    }

    std::vector<SCPolylineIntersectionPoint2d> Intersect(const ISCSegment2d& segment,
                                                         const SCPolyline2d& polyline,
                                                         double eps)
    {
        std::vector<SCPolylineIntersectionPoint2d> result = Intersect(polyline, segment, eps);
        for (auto& point : result)
        {
            std::swap(point.segmentIndexOnFirst, point.segmentIndexOnSecond);
            std::swap(point.parameterOnFirstSegment, point.parameterOnSecondSegment);
            std::swap(point.globalParameterOnFirst, point.globalParameterOnSecond);
        }
        std::sort(result.begin(), result.end(), [](const SCPolylineIntersectionPoint2d& lhs,
                                                   const SCPolylineIntersectionPoint2d& rhs) {
            if (lhs.parameterOnFirstSegment != rhs.parameterOnFirstSegment)
            {
                return lhs.parameterOnFirstSegment < rhs.parameterOnFirstSegment;
            }
            if (lhs.segmentIndexOnSecond != rhs.segmentIndexOnSecond)
            {
                return lhs.segmentIndexOnSecond < rhs.segmentIndexOnSecond;
            }
            return lhs.parameterOnSecondSegment < rhs.parameterOnSecondSegment;
        });
        return result;
    }

    SCExtendedIntersection2d IntersectExtended(const ISCSegment2d& first,
                                               const ISCSegment2d& second,
                                               SCExtensionPolicy policy,
                                               double eps)
    {
        if (!SegmentValidAtTolerance(first, eps) || !SegmentValidAtTolerance(second, eps))
        {
            return {};
        }

        const bool extendFirst = policy == SCExtensionPolicy::ExtendFirst || policy == SCExtensionPolicy::ExtendBoth;
        const bool extendSecond = policy == SCExtensionPolicy::ExtendSecond || policy == SCExtensionPolicy::ExtendBoth;

        if (policy == SCExtensionPolicy::None)
        {
            return MakeExtendedIntersectionFromIntersection(Intersect(first, second, eps));
        }

        if (first.Kind() == SCSegmentKind2d::Line && second.Kind() == SCSegmentKind2d::Line)
        {
            const SCLineSegment2d& firstLine = static_cast<const SCLineSegment2d&>(first);
            const SCLineSegment2d& secondLine = static_cast<const SCLineSegment2d&>(second);
            const double firstParameterEps = eps / firstLine.Length();
            const double secondParameterEps = eps / secondLine.Length();
            const SCLine2d lineA = SCLine2d::FromTwoPoints(firstLine.startPoint, firstLine.endPoint);
            const SCLine2d lineB = SCLine2d::FromTwoPoints(secondLine.startPoint, secondLine.endPoint);
            const SCLineIntersection2d lineIntersection = Intersect(lineA, lineB, eps);
            SCExtendedIntersection2d result;
            if (lineIntersection.kind == SCIntersectionKind2d::None)
            {
                return result;
            }

            if (lineIntersection.collinear)
            {
                if (extendFirst && extendSecond)
                {
                    result.kind = SCIntersectionKind2d::Overlap;
                    result.infiniteOverlap = true;
                    return result;
                }

                const SCLineSegment2d& overlapSegment = extendFirst ? secondLine : firstLine;
                const double startParameterOnFirst = LineParameterAtPoint(lineA, overlapSegment.startPoint, eps);
                const double endParameterOnFirst = LineParameterAtPoint(lineA, overlapSegment.endPoint, eps);
                const double startParameterOnSecond = LineParameterAtPoint(lineB, overlapSegment.startPoint, eps);
                const double endParameterOnSecond = LineParameterAtPoint(lineB, overlapSegment.endPoint, eps);
                result.kind = SCIntersectionKind2d::Overlap;
                result.pointCount = 2;
                result.points[0] = SCIntersectionPoint2d{overlapSegment.startPoint,
                                                         startParameterOnFirst,
                                                         startParameterOnSecond};
                result.points[1] = SCIntersectionPoint2d{overlapSegment.endPoint,
                                                         endParameterOnFirst,
                                                         endParameterOnSecond};
                if (result.points[0].parameterOnFirst > result.points[1].parameterOnFirst)
                {
                    std::swap(result.points[0], result.points[1]);
                }
                result.onFirstSegment = result.points[0].parameterOnFirst >= -firstParameterEps &&
                                       result.points[0].parameterOnFirst <= 1.0 + firstParameterEps &&
                                       result.points[1].parameterOnFirst >= -firstParameterEps &&
                                       result.points[1].parameterOnFirst <= 1.0 + firstParameterEps;
                result.onSecondSegment = result.points[0].parameterOnSecond >= -secondParameterEps &&
                                        result.points[0].parameterOnSecond <= 1.0 + secondParameterEps &&
                                        result.points[1].parameterOnSecond >= -secondParameterEps &&
                                        result.points[1].parameterOnSecond <= 1.0 + secondParameterEps;
                return result;
            }

            const double firstParameter = lineIntersection.points[0].parameterOnFirst;
            const double secondParameter = lineIntersection.points[0].parameterOnSecond;
            const bool onFirst = extendFirst ||
                                 (firstParameter >= -firstParameterEps && firstParameter <= 1.0 + firstParameterEps);
            const bool onSecond = extendSecond ||
                                  (secondParameter >= -secondParameterEps && secondParameter <= 1.0 + secondParameterEps);
            if (!onFirst || !onSecond)
            {
                return result;
            }

            result.kind = SCIntersectionKind2d::Point;
            result.pointCount = 1;
            result.points[0] = lineIntersection.points[0];
            result.onFirstSegment = firstParameter >= -firstParameterEps && firstParameter <= 1.0 + firstParameterEps;
            result.onSecondSegment = secondParameter >= -secondParameterEps && secondParameter <= 1.0 + secondParameterEps;
            return result;
        }

        if (first.Kind() == SCSegmentKind2d::Line && second.Kind() == SCSegmentKind2d::Arc)
        {
            const SCLineSegment2d& lineSegment = static_cast<const SCLineSegment2d&>(first);
            const SCArcSegment2d& arc = static_cast<const SCArcSegment2d&>(second);
            const SCLine2d line = SCLine2d::FromTwoPoints(lineSegment.startPoint, lineSegment.endPoint);
            const SCSegmentIntersection2d intersection = IntersectLineArcExtended(line, arc, extendFirst, extendSecond, eps);
            SCExtendedIntersection2d result;
            if (!intersection.HasIntersection())
            {
                return result;
            }

            result.kind = intersection.kind;
            result.pointCount = intersection.pointCount;
            for (std::size_t index = 0; index < intersection.pointCount; ++index)
            {
                result.points[index] = intersection.points[index];
            }
            if (intersection.pointCount >= 1)
            {
                const double lineParameterEps = eps / lineSegment.Length();
                const double angleEps = eps / arc.radius;
                result.onFirstSegment = true;
                result.onSecondSegment = true;
                for (std::size_t index = 0; index < intersection.pointCount; ++index)
                {
                    const SCIntersectionPoint2d& point = result.points[index];
                    result.onFirstSegment = result.onFirstSegment &&
                                            point.parameterOnFirst >= -lineParameterEps &&
                                            point.parameterOnFirst <= 1.0 + lineParameterEps;
                    result.onSecondSegment = result.onSecondSegment &&
                                             IsAngleOnArc(arc,
                                                          arc.startAngle + arc.sweepAngle * point.parameterOnSecond,
                                                          angleEps);
                }
            }
            return result;
        }

        if (first.Kind() == SCSegmentKind2d::Arc && second.Kind() == SCSegmentKind2d::Line)
        {
            const SCExtendedIntersection2d swapped = IntersectExtended(second, first, SwapExtendedIntersectionPolicy(policy), eps);
            SCExtendedIntersection2d result = swapped;
            for (std::size_t index = 0; index < result.pointCount; ++index)
            {
                std::swap(result.points[index].parameterOnFirst, result.points[index].parameterOnSecond);
            }
            std::swap(result.onFirstSegment, result.onSecondSegment);
            return result;
        }

        if (first.Kind() == SCSegmentKind2d::Arc && second.Kind() == SCSegmentKind2d::Arc)
        {
            const SCArcSegment2d& firstArc = static_cast<const SCArcSegment2d&>(first);
            const SCArcSegment2d& secondArc = static_cast<const SCArcSegment2d&>(second);
            return IntersectArcArcExtendedInternal(firstArc, secondArc, extendFirst, extendSecond, eps);
        }

        return MakeNoExtendedIntersection();
    }
}  // namespace Geometry
