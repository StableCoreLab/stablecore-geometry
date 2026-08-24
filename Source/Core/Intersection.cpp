#include "Core/Intersection.h"

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
            if (AlmostEqual(arc.sweepAngle, 0.0, Geometry::kIntersectionDefaultEpsilon))
            {
                return 0.0;
            }

            if (arc.sweepAngle > 0.0)
            {
                return NormalizeAngle(angle - arc.startAngle) / arc.sweepAngle;
            }

            return -NormalizeAngle(arc.startAngle - angle) / arc.sweepAngle;
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
            if (segmentLengthSquared <= Geometry::kIntersectionDefaultEpsilon * Geometry::kIntersectionDefaultEpsilon)
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
                                       clampToSegment || (rawParameter >= -Geometry::kIntersectionDefaultEpsilon &&
                                                          rawParameter <= 1.0 + Geometry::kIntersectionDefaultEpsilon)};
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
            if (fromCenter.LengthSquared() <= eps * eps)
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
            if (!hasBest || distanceSquared + Geometry::kIntersectionDefaultEpsilon < best.distanceSquared)
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
            const SCPoint2d p = first.startPoint;
            const SCPoint2d q = second.startPoint;
            const SCVector2d r = first.endPoint - first.startPoint;
            const SCVector2d s = second.endPoint - second.startPoint;
            const SCVector2d qp = q - p;

            const double rxs = Cross(r, s);
            const double qpxr = Cross(qp, r);

            if (AlmostEqual(rxs, 0.0, eps))
            {
                if (!AlmostEqual(qpxr, 0.0, eps))
                {
                    return MakeNoIntersection();
                }

                const double rr = Dot(r, r);
                const double ss = Dot(s, s);
                if (rr <= eps || ss <= eps)
                {
                    return MakeNoIntersection();
                }

                double t0 = Dot(q - p, r) / rr;
                double t1 = Dot(second.endPoint - p, r) / rr;
                if (t0 > t1)
                {
                    std::swap(t0, t1);
                }

                const double overlapStart = std::max(0.0, t0);
                const double overlapEnd = std::min(1.0, t1);
                if (overlapEnd + eps < overlapStart)
                {
                    return MakeNoIntersection();
                }

                const SCPoint2d firstPoint = first.PointAt(overlapStart);
                const SCPoint2d secondPoint = first.PointAt(overlapEnd);
                const double firstParameterOnSecond = Dot(firstPoint - second.startPoint, s) / ss;
                const double secondParameterOnSecond = Dot(secondPoint - second.startPoint, s) / ss;

                if (AlmostEqual(overlapStart, overlapEnd, eps))
                {
                    return MakePointIntersection(
                        firstPoint, overlapStart, firstParameterOnSecond, SCIntersectionKind2d::Point);
                }

                return MakeOverlapIntersection(SCIntersectionPoint2d{firstPoint, overlapStart, firstParameterOnSecond},
                                               SCIntersectionPoint2d{secondPoint, overlapEnd, secondParameterOnSecond});
            }

            const double t = Cross(qp, s) / rxs;
            const double u = Cross(qp, r) / rxs;
            if (t < -eps || t > 1.0 + eps || u < -eps || u > 1.0 + eps)
            {
                return MakeNoIntersection();
            }

            const double clampedT = Clamp(t, 0.0, 1.0);
            const double clampedU = Clamp(u, 0.0, 1.0);
            const SCPoint2d intersectionPoint = first.PointAt(clampedT);
            return MakePointIntersection(intersectionPoint, clampedT, clampedU);
        }

        [[nodiscard]] SCSegmentIntersection2d IntersectLineArcInternal(const SCLineSegment2d& line,
                                                                     const SCArcSegment2d& arc,
                                                                     double eps)
        {
            if (!line.IsValid() || !arc.IsValid())
            {
                return MakeNoIntersection();
            }

            const SCVector2d direction = line.endPoint - line.startPoint;
            const SCVector2d fromCenter = line.startPoint - arc.center;
            const double a = Dot(direction, direction);
            if (a <= eps)
            {
                return MakeNoIntersection();
            }

            const double b = 2.0 * Dot(fromCenter, direction);
            const double c = Dot(fromCenter, fromCenter) - arc.radius * arc.radius;
            const double discriminant = b * b - 4.0 * a * c;
            if (discriminant < -eps)
            {
                return MakeNoIntersection();
            }

            const double sqrtDiscriminant = discriminant <= 0.0 ? 0.0 : std::sqrt(discriminant);
            const double invDenominator = 0.5 / a;
            const double roots[2] = {(-b - sqrtDiscriminant) * invDenominator,
                                     (-b + sqrtDiscriminant) * invDenominator};

            SCSegmentIntersection2d result;
            bool hasPoint = false;
            for (double root : roots)
            {
                if (root < -eps || root > 1.0 + eps)
                {
                    continue;
                }

                const double parameter = Clamp(root, 0.0, 1.0);
                const SCPoint2d point = line.PointAt(parameter);
                const double angle = std::atan2(point.y - arc.center.y, point.x - arc.center.x);
                if (!IsAngleOnArc(arc, angle, eps))
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
                        AlmostEqual(discriminant, 0.0, eps) ? SCIntersectionKind2d::Tangent : SCIntersectionKind2d::Point);
                    hasPoint = true;
                    continue;
                }

                AddIntersectionPoint(result, SCIntersectionKind2d::Point, point, parameter, arcParameter, eps);
            }

            return result;
        }

        [[nodiscard]] bool SameCircle(const SCArcSegment2d& first, const SCArcSegment2d& second, double eps)
        {
            return first.center.AlmostEquals(second.center, eps) && std::abs(first.radius - second.radius) <= eps;
        }

        [[nodiscard]] SCSegmentIntersection2d IntersectArcArcSameCircle(const SCArcSegment2d& first,
                                                                      const SCArcSegment2d& second,
                                                                      double eps)
        {
            const bool firstFullCircle = AlmostEqual(std::abs(first.sweepAngle), kTwoPi, eps);
            const bool secondFullCircle = AlmostEqual(std::abs(second.sweepAngle), kTwoPi, eps);
            if (firstFullCircle && secondFullCircle)
            {
                return MakeOverlapIntersection(SCIntersectionPoint2d{first.StartPoint(), 0.0, 0.0},
                                               SCIntersectionPoint2d{first.PointAt(0.5), 0.5, 0.5});
            }

            const SCPoint2d firstStart = first.StartPoint();
            const SCPoint2d firstEnd = first.EndPoint();
            const SCPoint2d secondStart = second.StartPoint();
            const SCPoint2d secondEnd = second.EndPoint();

            const bool firstStartOnSecond = IsAngleOnArc(second, first.startAngle, eps);
            const bool firstEndOnSecond = IsAngleOnArc(second, first.EndAngle(), eps);
            const bool secondStartOnFirst = IsAngleOnArc(first, second.startAngle, eps);
            const bool secondEndOnFirst = IsAngleOnArc(first, second.EndAngle(), eps);

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

                return MakeOverlapIntersection(candidates[0], candidates[1]);
            }

            std::sort(candidates.begin(),
                      candidates.begin() + static_cast<std::ptrdiff_t>(candidateCount),
                      [](const SCIntersectionPoint2d& lhs, const SCIntersectionPoint2d& rhs) {
                          return lhs.parameterOnFirst < rhs.parameterOnFirst;
                      });

            return MakeOverlapIntersection(candidates[0], candidates[1]);
        }

        [[nodiscard]] SCClosestPoints2d ClosestPointsLineLineInternal(const SCLineSegment2d& first,
                                                                    const SCLineSegment2d& second)
        {
            const SCSegmentIntersection2d intersection =
                IntersectLineLineInternal(first, second, Geometry::kIntersectionDefaultEpsilon);
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
                IntersectLineArcInternal(line, arc, Geometry::kIntersectionDefaultEpsilon);
            if (intersection.HasIntersection())
            {
                const SCIntersectionPoint2d& point = intersection.points[0];
                return SCClosestPoints2d{point.point, point.point, point.parameterOnFirst, point.parameterOnSecond, 0.0};
            }

            SCClosestPoints2d best{};
            bool hasBest = false;

            const auto updateFromLineEndpoint = [&](const SCPoint2d& endpoint, double parameterOnLine) {
                const ArcProjectionCandidate projected =
                    ProjectPointToArcSegmentLocal(endpoint, arc, Geometry::kIntersectionDefaultEpsilon);
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
                Geometry::kIntersectionDefaultEpsilon * Geometry::kIntersectionDefaultEpsilon)
            {
                const double angle = std::atan2(fromCenter.y, fromCenter.x);
                if (IsAngleOnArc(arc, angle, Geometry::kIntersectionDefaultEpsilon))
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
                IntersectArcArcInternal(first, second, Geometry::kIntersectionDefaultEpsilon);
            if (intersection.HasIntersection())
            {
                const SCIntersectionPoint2d& point = intersection.points[0];
                return SCClosestPoints2d{point.point, point.point, point.parameterOnFirst, point.parameterOnSecond, 0.0};
            }

            SCClosestPoints2d best{};
            bool hasBest = false;

            const auto updateFromFirstEndpoint = [&](const SCPoint2d& endpoint, double parameterOnFirst) {
                const ArcProjectionCandidate projected =
                    ProjectPointToArcSegmentLocal(endpoint, second, Geometry::kIntersectionDefaultEpsilon);
                UpdateClosest(best, hasBest, endpoint, projected.point, parameterOnFirst, projected.parameter);
            };

            const auto updateFromSecondEndpoint = [&](const SCPoint2d& endpoint, double parameterOnSecond) {
                const ArcProjectionCandidate projected =
                    ProjectPointToArcSegmentLocal(endpoint, first, Geometry::kIntersectionDefaultEpsilon);
                UpdateClosest(best, hasBest, projected.point, endpoint, projected.parameter, parameterOnSecond);
            };

            updateFromFirstEndpoint(first.StartPoint(), 0.0);
            updateFromFirstEndpoint(first.EndPoint(), 1.0);
            updateFromSecondEndpoint(second.StartPoint(), 0.0);
            updateFromSecondEndpoint(second.EndPoint(), 1.0);

            const SCVector2d centerDelta = second.center - first.center;
            const double centerDistanceSquared = Dot(centerDelta, centerDelta);
            if (centerDistanceSquared > Geometry::kIntersectionDefaultEpsilon * Geometry::kIntersectionDefaultEpsilon)
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
                if (IsAngleOnArc(first, firstAngle, Geometry::kIntersectionDefaultEpsilon) &&
                    IsAngleOnArc(second, secondAngle, Geometry::kIntersectionDefaultEpsilon))
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
            if (!first.IsValid() || !second.IsValid())
            {
                return MakeNoIntersection();
            }

            if (SameCircle(first, second, eps))
            {
                return IntersectArcArcSameCircle(first, second, eps);
            }

            const SCVector2d centerDelta = second.center - first.center;
            const double distanceSquared = Dot(centerDelta, centerDelta);
            if (distanceSquared <= eps * eps)
            {
                return MakeNoIntersection();
            }

            const double distance = std::sqrt(distanceSquared);
            if (distance > first.radius + second.radius + eps ||
                distance < std::abs(first.radius - second.radius) - eps)
            {
                return MakeNoIntersection();
            }

            const double a =
                (first.radius * first.radius - second.radius * second.radius + distanceSquared) / (2.0 * distance);
            const double hSquared = first.radius * first.radius - a * a;
            if (hSquared < -eps)
            {
                return MakeNoIntersection();
            }

            const double h = hSquared <= 0.0 ? 0.0 : std::sqrt(hSquared);
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
                if (!IsAngleOnArc(first, angleFirst, eps) || !IsAngleOnArc(second, angleSecond, eps))
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
            if (!first.IsValid() || !second.IsValid())
            {
                return result;
            }

            if (SameCircle(first, second, eps))
            {
                const bool firstActsAsFullCircle = extendFirst || AlmostEqual(std::abs(first.sweepAngle), kTwoPi, eps);
                const bool secondActsAsFullCircle = extendSecond || AlmostEqual(std::abs(second.sweepAngle), kTwoPi, eps);

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
            const double distanceSquared = Dot(centerDelta, centerDelta);
            if (distanceSquared <= eps * eps)
            {
                return result;
            }

            const double distance = std::sqrt(distanceSquared);
            if (distance > first.radius + second.radius + eps ||
                distance < std::abs(first.radius - second.radius) - eps)
            {
                return result;
            }

            const double a =
                (first.radius * first.radius - second.radius * second.radius + distanceSquared) / (2.0 * distance);
            const double hSquared = first.radius * first.radius - a * a;
            if (hSquared < -eps)
            {
                return result;
            }

            const double h = hSquared <= 0.0 ? 0.0 : std::sqrt(hSquared);
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
                if ((!extendFirst && !IsAngleOnArc(first, angleFirst, eps)) ||
                    (!extendSecond && !IsAngleOnArc(second, angleSecond, eps)))
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
                result.onFirstSegment = result.onFirstSegment && firstParameter >= -eps && firstParameter <= 1.0 + eps;
                result.onSecondSegment =
                    result.onSecondSegment && secondParameter >= -eps && secondParameter <= 1.0 + eps;
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
            if (fromCenter.LengthSquared() > eps * eps)
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
        if (!first.IsValid() || !second.IsValid())
        {
            return SCSegmentIntersection2d{};
        }

        if (first.Kind() == SCSegmentKind2::Line && second.Kind() == SCSegmentKind2::Line)
        {
            return Intersect(static_cast<const SCLineSegment2d&>(first), static_cast<const SCLineSegment2d&>(second), eps);
        }

        if (first.Kind() == SCSegmentKind2::Line && second.Kind() == SCSegmentKind2::Arc)
        {
            return Intersect(static_cast<const SCLineSegment2d&>(first), static_cast<const SCArcSegment2d&>(second), eps);
        }

        if (first.Kind() == SCSegmentKind2::Arc && second.Kind() == SCSegmentKind2::Line)
        {
            return SwapIntersectionResult(
                Intersect(static_cast<const SCLineSegment2d&>(second), static_cast<const SCArcSegment2d&>(first), eps));
        }

        if (first.Kind() == SCSegmentKind2::Arc && second.Kind() == SCSegmentKind2::Arc)
        {
            return Intersect(static_cast<const SCArcSegment2d&>(first), static_cast<const SCArcSegment2d&>(second), eps);
        }

        return SCSegmentIntersection2d{};
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
        return ClosestPointsLineArcInternal(first, second, Geometry::kIntersectionDefaultEpsilon);
    }

    SCClosestPoints2d ClosestPoints(const SCArcSegment2d& first, const SCArcSegment2d& second)
    {
        return ClosestPointsArcArcInternal(first, second);
    }

    SCClosestPoints2d ClosestPoints(const ISCSegment2d& first, const ISCSegment2d& second)
    {
        if (!first.IsValid() || !second.IsValid())
        {
            return SCClosestPoints2d{};
        }

        if (first.Kind() == SCSegmentKind2::Line && second.Kind() == SCSegmentKind2::Line)
        {
            return ClosestPoints(static_cast<const SCLineSegment2d&>(first), static_cast<const SCLineSegment2d&>(second));
        }

        if (first.Kind() == SCSegmentKind2::Line && second.Kind() == SCSegmentKind2::Arc)
        {
            return ClosestPoints(static_cast<const SCLineSegment2d&>(first), static_cast<const SCArcSegment2d&>(second));
        }

        if (first.Kind() == SCSegmentKind2::Arc && second.Kind() == SCSegmentKind2::Line)
        {
            return SwapClosestPointsResult(
                ClosestPoints(static_cast<const SCLineSegment2d&>(second), static_cast<const SCArcSegment2d&>(first)));
        }

        if (first.Kind() == SCSegmentKind2::Arc && second.Kind() == SCSegmentKind2::Arc)
        {
            return ClosestPoints(static_cast<const SCArcSegment2d&>(first), static_cast<const SCArcSegment2d&>(second));
        }

        return SCClosestPoints2d{};
    }

        [[nodiscard]] double LineParameterAtPoint(const SCLine2d& line, const SCPoint2d& point, double eps)
        {
            const SCVector2d delta = point - line.origin;
            const double denominator = Dot(line.direction, line.direction);
            if (denominator <= eps * eps)
            {
                return 0.0;
            }
            return Dot(delta, line.direction) / denominator;
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
            if (totalLength <= Geometry::kIntersectionDefaultEpsilon)
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
                                        double eps)
        {
            for (auto& existing : result)
            {
                if (existing.point.AlmostEquals(candidate.point, eps))
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
            const SCVector2d p = first.origin - SCPoint2d{};
            const SCVector2d q = second.origin - SCPoint2d{};
            const SCVector2d r = first.direction;
            const SCVector2d s = second.direction;
            const SCVector2d qp = second.origin - first.origin;
            const double det = Cross(r, s);
            if (std::abs(det) <= eps)
            {
                result.parallel = true;
                if (std::abs(Cross(qp, r)) <= eps)
                {
                    result.kind = SCIntersectionKind2d::Overlap;
                    result.collinear = true;
                    result.infiniteOverlap = true;
                }
                return result;
            }

            const double t = Cross(qp, s) / det;
            const double u = Cross(qp, r) / det;
            const SCPoint2d point = first.PointAt(t);
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
                if (segmentParameter < -eps || segmentParameter > 1.0 + eps)
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
            if (!line.IsValid(eps) || !arc.IsValid())
            {
                return {};
            }

            const SCVector2d direction = line.direction;
            const SCVector2d fromCenter = line.origin - arc.center;
            const double a = Dot(direction, direction);
            if (a <= eps)
            {
                return {};
            }

            const double b = 2.0 * Dot(fromCenter, direction);
            const double c = Dot(fromCenter, fromCenter) - arc.radius * arc.radius;
            const double discriminant = b * b - 4.0 * a * c;
            if (discriminant < -eps)
            {
                return {};
            }

            const double sqrtDiscriminant = discriminant <= 0.0 ? 0.0 : std::sqrt(discriminant);
            const double invDenominator = 0.5 / a;
            const double roots[2] = {(-b - sqrtDiscriminant) * invDenominator, (-b + sqrtDiscriminant) * invDenominator};

            SCSegmentIntersection2d result;
            bool hasPoint = false;
            for (double t : roots)
            {
                const SCPoint2d point = line.PointAt(t);
                const double angle = std::atan2(point.y - arc.center.y, point.x - arc.center.x);
                const double arcParameter = arc.sweepAngle >= 0.0 ? (angle - arc.startAngle) / arc.sweepAngle
                                                                   : (arc.startAngle - angle) / (-arc.sweepAngle);
                const bool onLine = extendLine || (t >= -eps && t <= 1.0 + eps);
                const bool onArc = extendArc || IsAngleOnArc(arc, angle, eps);
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
                    result.kind = AlmostEqual(discriminant, 0.0, eps) ? SCIntersectionKind2d::Tangent
                                                                       : SCIntersectionKind2d::Point;
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
                }
            }

            return result;
        }

        [[nodiscard]] std::vector<SCPolylineIntersectionPoint2d> IntersectPolylinePolylineInternal(const SCPolyline2d& first,
                                                                                                 const SCPolyline2d& second,
                                                                                                 double eps)
        {
            std::vector<SCPolylineIntersectionPoint2d> result;
            if (!first.IsValid() || !second.IsValid())
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
                const double length = segment == nullptr ? 0.0 : segment->Length();
                firstSegmentLengths.push_back(length);
                firstTotalLength += length;
            }
            for (std::size_t i = 0; i < second.SegmentCount(); ++i)
            {
                const auto segment = second.SegmentAt(i);
                const double length = segment == nullptr ? 0.0 : segment->Length();
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

                    const SCSegmentIntersection2d intersection = Intersect(*firstSegment, *secondSegment, eps);
                    if (!intersection.HasIntersection())
                    {
                        continue;
                    }

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
                        AppendPolylineIntersection(result, polyPoint, eps);
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
        if (!polyline.IsValid() || !segment.IsValid())
        {
            return result;
        }

        std::vector<double> segmentLengths;
        segmentLengths.reserve(polyline.SegmentCount());
        double totalLength = 0.0;
        for (std::size_t i = 0; i < polyline.SegmentCount(); ++i)
        {
            const auto polySegment = polyline.SegmentAt(i);
            const double length = polySegment == nullptr ? 0.0 : polySegment->Length();
            segmentLengths.push_back(length);
            totalLength += length;
        }

        for (std::size_t i = 0; i < polyline.SegmentCount(); ++i)
        {
            const auto polySegment = polyline.SegmentAt(i);
            if (polySegment == nullptr)
            {
                continue;
            }

            const SCSegmentIntersection2d intersection = Intersect(*polySegment, segment, eps);
            if (!intersection.HasIntersection())
            {
                continue;
            }

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
                AppendPolylineIntersection(result, polyPoint, eps);
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
        return result;
    }

    SCExtendedIntersection2d IntersectExtended(const ISCSegment2d& first,
                                               const ISCSegment2d& second,
                                               SCExtensionPolicy policy,
                                               double eps)
    {
        if (!first.IsValid() || !second.IsValid())
        {
            return {};
        }

        const bool extendFirst = policy == SCExtensionPolicy::ExtendFirst || policy == SCExtensionPolicy::ExtendBoth;
        const bool extendSecond = policy == SCExtensionPolicy::ExtendSecond || policy == SCExtensionPolicy::ExtendBoth;

        if (policy == SCExtensionPolicy::None)
        {
            return MakeExtendedIntersectionFromIntersection(Intersect(first, second, eps));
        }

        if (first.Kind() == SCSegmentKind2::Line && second.Kind() == SCSegmentKind2::Line)
        {
            const SCLineSegment2d& firstLine = static_cast<const SCLineSegment2d&>(first);
            const SCLineSegment2d& secondLine = static_cast<const SCLineSegment2d&>(second);
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
                result.onFirstSegment = result.points[0].parameterOnFirst >= -eps &&
                                       result.points[0].parameterOnFirst <= 1.0 + eps &&
                                       result.points[1].parameterOnFirst >= -eps &&
                                       result.points[1].parameterOnFirst <= 1.0 + eps;
                result.onSecondSegment = result.points[0].parameterOnSecond >= -eps &&
                                        result.points[0].parameterOnSecond <= 1.0 + eps &&
                                        result.points[1].parameterOnSecond >= -eps &&
                                        result.points[1].parameterOnSecond <= 1.0 + eps;
                return result;
            }

            const double firstParameter = lineIntersection.points[0].parameterOnFirst;
            const double secondParameter = lineIntersection.points[0].parameterOnSecond;
            const bool onFirst = extendFirst || (firstParameter >= -eps && firstParameter <= 1.0 + eps);
            const bool onSecond = extendSecond || (secondParameter >= -eps && secondParameter <= 1.0 + eps);
            if (!onFirst || !onSecond)
            {
                return result;
            }

            result.kind = SCIntersectionKind2d::Point;
            result.pointCount = 1;
            result.points[0] = lineIntersection.points[0];
            result.onFirstSegment = firstParameter >= -eps && firstParameter <= 1.0 + eps;
            result.onSecondSegment = secondParameter >= -eps && secondParameter <= 1.0 + eps;
            return result;
        }

        if (first.Kind() == SCSegmentKind2::Line && second.Kind() == SCSegmentKind2::Arc)
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
                result.onFirstSegment = result.points[0].parameterOnFirst >= -eps && result.points[0].parameterOnFirst <= 1.0 + eps;
                result.onSecondSegment = IsAngleOnArc(arc,
                                                      arc.startAngle + arc.sweepAngle * result.points[0].parameterOnSecond,
                                                      eps);
            }
            return result;
        }

        if (first.Kind() == SCSegmentKind2::Arc && second.Kind() == SCSegmentKind2::Line)
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

        if (first.Kind() == SCSegmentKind2::Arc && second.Kind() == SCSegmentKind2::Arc)
        {
            const SCArcSegment2d& firstArc = static_cast<const SCArcSegment2d&>(first);
            const SCArcSegment2d& secondArc = static_cast<const SCArcSegment2d&>(second);
            return IntersectArcArcExtendedInternal(firstArc, secondArc, extendFirst, extendSecond, eps);
        }

        return MakeNoExtendedIntersection();
    }
}  // namespace Geometry
