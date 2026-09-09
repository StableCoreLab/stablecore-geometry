#include "PolygonPositiveAreaIntersection2d.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <limits>
#include <optional>
#include <iterator>
#include <vector>

#include "Core/Relation.h"
#include "Core/Projection.h"
#include "CurveArrangement2d.h"
#include "../Core/RingIntegral2d.h"
#include "SegmentKernel2d.h"

namespace Geometry::Detail
{
    namespace
    {
        [[nodiscard]] PositiveAreaStatus2d MapArrangementFailure(CurveArrangementFailure2d failure)
        {
            switch (failure)
            {
                case CurveArrangementFailure2d::NumericalIndeterminate:
                case CurveArrangementFailure2d::EventOrderingFailure:
                    return PositiveAreaStatus2d::NumericalIndeterminate;
                case CurveArrangementFailure2d::None:
                    return PositiveAreaStatus2d::ArrangementFailure;
                case CurveArrangementFailure2d::InvalidInput:
                case CurveArrangementFailure2d::EventInsertionFailure:
                case CurveArrangementFailure2d::PieceConstructionFailure:
                case CurveArrangementFailure2d::ReverseEdgeFailure:
                case CurveArrangementFailure2d::FaceClosureFailure:
                    return PositiveAreaStatus2d::ArrangementFailure;
            }
            return PositiveAreaStatus2d::ArrangementFailure;
        }

        [[nodiscard]] std::optional<bool> TryClassifyExternalCircularPolygons(const SCPolygon2d& first,
                                                                                const SCPolygon2d& second,
                                                                                double tolerance)
        {
            if (first.HoleCount() != 0 || second.HoleCount() != 0 || first.OuterRing().SegmentCount() == 0 ||
                second.OuterRing().SegmentCount() == 0)
            {
                return std::nullopt;
            }
            const auto getCircle = [tolerance](const SCPolyline2d& ring, SCPoint2d& center, double& radius) {
                const std::unique_ptr<ISCSegment2d> firstSegment = ring.SegmentAt(0);
                const auto* firstArc = firstSegment == nullptr
                                           ? nullptr
                                           : dynamic_cast<const SCArcSegment2d*>(firstSegment.get());
                if (firstArc == nullptr || !firstArc->center.IsValid() || !std::isfinite(firstArc->radius) ||
                    firstArc->radius <= 0.0)
                {
                    return false;
                }
                center = firstArc->center;
                radius = firstArc->radius;
                for (std::size_t i = 1; i < ring.SegmentCount(); ++i)
                {
                    const std::unique_ptr<ISCSegment2d> segment = ring.SegmentAt(i);
                    const auto* arc = segment == nullptr ? nullptr : dynamic_cast<const SCArcSegment2d*>(segment.get());
                    if (arc == nullptr || !arc->center.AlmostEquals(center, tolerance) ||
                        std::abs(arc->radius - radius) > tolerance)
                    {
                        return false;
                    }
                }
                return true;
            };

            SCPoint2d firstCenter{};
            SCPoint2d secondCenter{};
            double firstRadius = 0.0;
            double secondRadius = 0.0;
            if (!getCircle(first.OuterRing(), firstCenter, firstRadius) ||
                !getCircle(second.OuterRing(), secondCenter, secondRadius))
            {
                return std::nullopt;
            }
            const SCVector2d delta = secondCenter - firstCenter;
            const double distanceSquared = delta.x * delta.x + delta.y * delta.y;
            const double radiusSum = firstRadius + secondRadius;
            if (!std::isfinite(distanceSquared) || !std::isfinite(radiusSum))
            {
                return std::nullopt;
            }
            if (distanceSquared >= radiusSum * radiusSum)
            {
                return false;
            }
            return std::nullopt;
        }

        [[nodiscard]] bool IsFinite(double value)
        {
            return std::isfinite(value);
        }

        [[nodiscard]] bool WithinToleranceSquared(double distanceSquared, double tolerance)
        {
            if (!IsFinite(distanceSquared) || !IsFinite(tolerance) || tolerance < 0.0)
            {
                return false;
            }
            const double kSqrtMax = std::sqrt(std::numeric_limits<double>::max());
            if (tolerance >= kSqrtMax)
            {
                return true;
            }
            return distanceSquared <= tolerance * tolerance;
        }

        // 内核的 SCSegmentIntersection2d 没有失败状态，不能把 None 直接解释为
        // 无交。仅当包围盒严格分离时，才能独立证明无交；最近点结果没有可验证的
        // 全局最优性契约，不能单独作为 NoIntersection 的依据。
        [[nodiscard]] bool CanProveNoIntersection(const ISCSegment2d& first,
                                                   const ISCSegment2d& second,
                                                   double tolerance)
        {
            const SCBox2d firstBounds = first.Bounds();
            const SCBox2d secondBounds = second.Bounds();
            if (firstBounds.IsValid() && secondBounds.IsValid())
            {
                const SCPoint2d firstMin = firstBounds.MinPoint();
                const SCPoint2d firstMax = firstBounds.MaxPoint();
                const SCPoint2d secondMin = secondBounds.MinPoint();
                const SCPoint2d secondMax = secondBounds.MaxPoint();
                if (firstMax.x < secondMin.x || secondMax.x < firstMin.x ||
                    firstMax.y < secondMin.y || secondMax.y < firstMin.y)
                {
                    return true;
                }
            }

            // 两个圆周本身外离或一方严格包含另一方时，不存在圆周交点；
            // 这一步可处理圆弧多边形的严格包含，不能只依赖弧段包围盒。
            const auto* firstArc = dynamic_cast<const SCArcSegment2d*>(&first);
            const auto* secondArc = dynamic_cast<const SCArcSegment2d*>(&second);
            if (firstArc != nullptr && secondArc != nullptr)
            {
                const SCVector2d centerDelta = secondArc->center - firstArc->center;
                const double centerDistanceSquared = centerDelta.x * centerDelta.x + centerDelta.y * centerDelta.y;
                const double radiusSum = firstArc->radius + secondArc->radius;
                const double radiusDifference = std::abs(firstArc->radius - secondArc->radius);
                if (!IsFinite(centerDistanceSquared) || !IsFinite(radiusSum) || !IsFinite(radiusDifference))
                {
                    return false;
                }
                // 使用严格平方关系；等号和接近切点的情形交给 checked 求交。
                if (centerDistanceSquared > radiusSum * radiusSum ||
                    centerDistanceSquared < radiusDifference * radiusDifference)
                {
                    return true;
                }
                // Arc-Arc 的内核求解已经在有效参数域内检查了候选交点；当完整圆周相交但
                // 当前两段没有候选点时，不能再要求完整圆周外离/包含证明，否则会把有效的
                // 部分重叠圆弧误报为 NumericalIndeterminate。
                if (firstArc->center.IsValid() && secondArc->center.IsValid() &&
                    IsFinite(firstArc->radius) && IsFinite(secondArc->radius) && firstArc->radius > 0.0 &&
                    secondArc->radius > 0.0 && IsFinite(firstArc->startAngle) &&
                    IsFinite(firstArc->sweepAngle) && IsFinite(secondArc->startAngle) &&
                    IsFinite(secondArc->sweepAngle) && firstArc->sweepAngle != 0.0 &&
                    secondArc->sweepAngle != 0.0)
                {
                    return true;
                }
            }

            (void)tolerance;
            return false;
        }

        [[nodiscard]] std::vector<SCPolyline2d> CollectRings(const SCPolygon2d& polygon)
        {
            std::vector<SCPolyline2d> rings;
            rings.reserve(1 + polygon.HoleCount());
            rings.push_back(polygon.OuterRing());
            for (std::size_t i = 0; i < polygon.HoleCount(); ++i)
            {
                rings.push_back(polygon.HoleAt(i));
            }
            return rings;
        }

        [[nodiscard]] std::vector<std::vector<std::unique_ptr<ISCSegment2d>>> CollectRingSegments(
            const SCPolygon2d& polygon)
        {
            std::vector<std::vector<std::unique_ptr<ISCSegment2d>>> rings;
            rings.reserve(1 + polygon.HoleCount());
            auto collect = [&rings](const SCPolyline2d& ring) {
                std::vector<std::unique_ptr<ISCSegment2d>> segments;
                segments.reserve(ring.SegmentCount());
                for (std::size_t i = 0; i < ring.SegmentCount(); ++i)
                {
                    segments.push_back(ring.SegmentAt(i));
                }
                rings.push_back(std::move(segments));
            };
            collect(polygon.OuterRing());
            for (std::size_t i = 0; i < polygon.HoleCount(); ++i)
            {
                collect(polygon.HoleAt(i));
            }
            return rings;
        }

        // 构造面内部代表点：依据符号面积确定朝环内部偏移的方向，从段中点沿内向法线偏移。
        // 失败返回 nullopt；不调用公开 LocatePoint 作为失败判定依据。
        [[nodiscard]] std::optional<SCPoint2d> BuildInteriorSample(const SCPolyline2d& ring,
                                                                   double signedArea,
                                                                   double tolerance)
        {
            if (!ring.IsClosed() || !ring.IsValid() || ring.SegmentCount() == 0)
            {
                return std::nullopt;
            }
            const double side = signedArea > 0.0 ? 1.0 : -1.0;
            for (std::size_t i = 0; i < ring.SegmentCount(); ++i)
            {
                const std::unique_ptr<ISCSegment2d> segment = ring.SegmentAt(i);
                if (segment == nullptr || segment->Length() <= tolerance)
                {
                    continue;
                }
                const SCPoint2d before = segment->PointAt(0.49);
                const SCPoint2d after = segment->PointAt(0.51);
                const SCVector2d tangent = after - before;
                const double length = tangent.Length();
                if (!IsFinite(length) || length <= tolerance)
                {
                    continue;
                }
                const SCVector2d inward{-side * tangent.y / length, side * tangent.x / length};
                const double offset = std::max(tolerance * 8.0, std::min(segment->Length() * 1e-4, 1e-3));
                const SCPoint2d sample = SCPoint2d{segment->PointAt(0.5).x + inward.x * offset,
                                                   segment->PointAt(0.5).y + inward.y * offset};
                if (sample.IsValid())
                {
                    return sample;
                }
            }
            return std::nullopt;
        }

        // LocatePoint 的公开契约会把无效输入和部分数值异常统一回退为 Outside，
        // 这对正面积查询不可接受。这里复制其二维射线判定所需的最小逻辑，并把
        // 每个中间结果的失败显式传回调用方。
        [[nodiscard]] SCCheckedContainment2d CheckedLocateRing(const SCPoint2d& point,
                                                               const SCPolyline2d& ring,
                                                               double tolerance)
        {
            SCCheckedContainment2d result;
            if (!point.IsValid() || !ring.IsClosed() || !ring.IsValid() || ring.SegmentCount() == 0 ||
                !IsFinite(tolerance) || tolerance <= 0.0)
            {
                result.status = SCCheckedGeometryStatus2d::InvalidInput;
                return result;
            }

            std::size_t crossings = 0;
            for (std::size_t i = 0; i < ring.SegmentCount(); ++i)
            {
                const std::unique_ptr<ISCSegment2d> segment = ring.SegmentAt(i);
                if (segment == nullptr || !segment->IsValid() ||
                    !segment->StartPoint().IsValid() || !segment->EndPoint().IsValid())
                {
                    result.status = SCCheckedGeometryStatus2d::NumericalIndeterminate;
                    return result;
                }

                const SCSegmentProjection2d projection = ProjectPointToSegment(point, *segment, true);
                if (!projection.IsValid())
                {
                    result.status = SCCheckedGeometryStatus2d::NumericalIndeterminate;
                    return result;
                }
                if (WithinToleranceSquared(projection.distanceSquared, tolerance))
                {
                    result.status = SCCheckedGeometryStatus2d::Success;
                    result.containment = SCPointContainment2d::OnBoundary;
                    return result;
                }

                if (segment->Kind() == SCSegmentKind2::Line)
                {
                    const auto& line = static_cast<const SCLineSegment2d&>(*segment);
                    const double dy = line.endPoint.y - line.startPoint.y;
                    const bool straddles = (line.startPoint.y <= point.y && line.endPoint.y > point.y) ||
                                           (line.endPoint.y <= point.y && line.startPoint.y > point.y);
                    if (straddles)
                    {
                        const double x = line.startPoint.x +
                                         (line.endPoint.x - line.startPoint.x) * (point.y - line.startPoint.y) / dy;
                        if (!IsFinite(x))
                        {
                            result.status = SCCheckedGeometryStatus2d::NumericalIndeterminate;
                            return result;
                        }
                        if (x > point.x + tolerance)
                        {
                            ++crossings;
                        }
                    }
                    continue;
                }

                if (segment->Kind() != SCSegmentKind2::Arc)
                {
                    result.status = SCCheckedGeometryStatus2d::InvalidInput;
                    return result;
                }
                const auto& arc = static_cast<const SCArcSegment2d&>(*segment);
                const double dy = point.y - arc.center.y;
                const double radialSquared = arc.radius * arc.radius - dy * dy;
                if (!IsFinite(radialSquared))
                {
                    result.status = SCCheckedGeometryStatus2d::NumericalIndeterminate;
                    return result;
                }
                if (std::abs(dy) >= arc.radius - tolerance || radialSquared <= 0.0)
                {
                    continue;
                }
                const double dx = std::sqrt(radialSquared);
                for (const double x : {arc.center.x - dx, arc.center.x + dx})
                {
                    if (!IsFinite(x))
                    {
                        result.status = SCCheckedGeometryStatus2d::NumericalIndeterminate;
                        return result;
                    }
                    if (x <= point.x + tolerance)
                    {
                        continue;
                    }
                    const SCPoint2d candidate{x, point.y};
                    const SCSegmentProjection2d arcProjection = ProjectPointToArcSegment(candidate, arc, false);
                    if (!arcProjection.IsValid())
                    {
                        result.status = SCCheckedGeometryStatus2d::NumericalIndeterminate;
                        return result;
                    }
                    if (WithinToleranceSquared(arcProjection.distanceSquared, tolerance) && arcProjection.isOnSegment &&
                        arcProjection.parameter >= -tolerance && arcProjection.parameter < 1.0 - tolerance)
                    {
                        ++crossings;
                    }
                }
            }
            result.status = SCCheckedGeometryStatus2d::Success;
            result.containment = (crossings % 2U) == 1U ? SCPointContainment2d::Inside
                                                        : SCPointContainment2d::Outside;
            return result;
        }

        [[nodiscard]] SCCheckedContainment2d CheckedLocatePolygon(const SCPoint2d& point,
                                                                  const SCPolygon2d& polygon,
                                                                  double tolerance)
        {
            SCCheckedContainment2d result;
            if (!point.IsValid() || !polygon.IsValid())
            {
                result.status = SCCheckedGeometryStatus2d::InvalidInput;
                return result;
            }
            result = CheckedLocateRing(point, polygon.OuterRing(), tolerance);
            if (result.status != SCCheckedGeometryStatus2d::Success ||
                result.containment == SCPointContainment2d::OnBoundary)
            {
                return result;
            }
            if (result.containment == SCPointContainment2d::Outside)
            {
                return result;
            }
            for (std::size_t i = 0; i < polygon.HoleCount(); ++i)
            {
                const SCCheckedContainment2d hole = CheckedLocateRing(point, polygon.HoleAt(i), tolerance);
                if (hole.status != SCCheckedGeometryStatus2d::Success ||
                    hole.containment == SCPointContainment2d::OnBoundary)
                {
                    return hole;
                }
                if (hole.containment == SCPointContainment2d::Inside)
                {
                    result.containment = SCPointContainment2d::Outside;
                    return result;
                }
            }
            result.containment = SCPointContainment2d::Inside;
            return result;
        }
    }  // namespace

    SCCheckedIntersection2d CheckedIntersectSegments(const ISCSegment2d& first,
                                                     const ISCSegment2d& second,
                                                     double tolerance)
    {
        SCCheckedIntersection2d result;
        if (!IsFinite(tolerance) || tolerance <= 0.0 || !first.IsValid() || !second.IsValid() ||
            !IsKernelSegment(first) || !IsKernelSegment(second))
        {
            result.status = SCCheckedGeometryStatus2d::InvalidInput;
            return result;
        }
        const SCSegmentIntersection2d hit = IntersectKernelSegments(first, second, tolerance);
        if (hit.kind == SCIntersectionKind2d::None)
        {
            if (hit.pointCount != 0)
            {
                result.status = SCCheckedGeometryStatus2d::NumericalIndeterminate;
                return result;
            }
            if (!CanProveNoIntersection(first, second, tolerance))
            {
                result.status = SCCheckedGeometryStatus2d::NumericalIndeterminate;
                return result;
            }
            result.status = SCCheckedGeometryStatus2d::NoIntersection;
            result.intersection = hit;
            return result;
        }
        // 验证交点参数有限；Overlap 必须给出恰好两个端点，否则视为不确定。
        for (std::size_t i = 0; i < hit.pointCount; ++i)
        {
            if (!IsFinite(hit.points[i].parameterOnFirst) || !IsFinite(hit.points[i].parameterOnSecond) ||
                !hit.points[i].point.IsValid())
            {
                result.status = SCCheckedGeometryStatus2d::NumericalIndeterminate;
                return result;
            }
        }
        if (hit.kind == SCIntersectionKind2d::Overlap && hit.pointCount != 2)
        {
            result.status = SCCheckedGeometryStatus2d::NumericalIndeterminate;
            return result;
        }
        result.status = SCCheckedGeometryStatus2d::Success;
        result.intersection = hit;
        return result;
    }

    SCCheckedArea2d CheckedComputeSignedArea(const SCPolyline2d& ring, double tolerance)
    {
        (void)tolerance;
        SCCheckedArea2d result;
        if (!ring.IsClosed() || !ring.IsValid() || ring.SegmentCount() == 0)
        {
            result.status = SCCheckedGeometryStatus2d::InvalidInput;
            return result;
        }
        const double area = ComputeSignedArea(ring);
        if (!IsFinite(area))
        {
            result.status = SCCheckedGeometryStatus2d::NumericalIndeterminate;
            return result;
        }
        result.status = SCCheckedGeometryStatus2d::Success;
        result.signedArea = area;
        return result;
    }

    SCCheckedRepresentative2d CheckedFaceRepresentative(const SCPolyline2d& ring,
                                                         double signedArea,
                                                         double tolerance)
    {
        SCCheckedRepresentative2d result;
        const std::optional<SCPoint2d> sample = BuildInteriorSample(ring, signedArea, tolerance);
        if (!sample.has_value())
        {
            result.status = SCCheckedGeometryStatus2d::NumericalIndeterminate;
            return result;
        }
        if (!sample->IsValid())
        {
            result.status = SCCheckedGeometryStatus2d::NumericalIndeterminate;
            return result;
        }
        // 先通过 checked 定位确认代表点严格位于该面环内部。
        const SCCheckedContainment2d location = CheckedStrictlyInsideRing(*sample, ring, tolerance);
        if (location.status == SCCheckedGeometryStatus2d::NumericalIndeterminate)
        {
            result.status = SCCheckedGeometryStatus2d::NumericalIndeterminate;
            return result;
        }
        if (location.status != SCCheckedGeometryStatus2d::Success ||
            location.containment != SCPointContainment2d::Inside)
        {
            result.status = SCCheckedGeometryStatus2d::NumericalIndeterminate;
            return result;
        }
        result.status = SCCheckedGeometryStatus2d::Success;
        result.point = *sample;
        return result;
    }

    SCCheckedContainment2d CheckedStrictlyInsideRing(const SCPoint2d& point,
                                                      const SCPolyline2d& ring,
                                                      double tolerance)
    {
        SCCheckedContainment2d result;
        // 显式验证输入有效性：LocatePoint 对无效环会回退为 Outside，
        // 不得将该回退当作成功的 Outside 判定。
        if (!point.IsValid() || !ring.IsClosed() || !ring.IsValid() || ring.SegmentCount() == 0)
        {
            result.status = SCCheckedGeometryStatus2d::InvalidInput;
            return result;
        }
        return CheckedLocateRing(point, ring, tolerance);
    }

    SCCheckedContainment2d CheckedStrictlyInsidePolygon(const SCPoint2d& point,
                                                         const SCPolygon2d& polygon,
                                                         double tolerance)
    {
        SCCheckedContainment2d result;
        if (!point.IsValid() || !polygon.IsValid())
        {
            result.status = SCCheckedGeometryStatus2d::InvalidInput;
            return result;
        }
        return CheckedLocatePolygon(point, polygon, tolerance);
    }

    PositiveAreaResult2d BuildPositiveAreaArrangement2d(const SCPolygon2d& first,
                                                       const SCPolygon2d& second,
                                                       double tolerance)
    {
        PositiveAreaResult2d result;
        if (!first.IsValid() || !second.IsValid())
        {
            result.status = PositiveAreaStatus2d::ArrangementFailure;
            return result;
        }

        // checked 求交：检测非有限参数或畸形重叠等不确定情形，避免被 arrangement 静默吞掉。
        if (const std::optional<bool> externalCircularResult =
                TryClassifyExternalCircularPolygons(first, second, tolerance);
            externalCircularResult.has_value())
        {
            result.status = PositiveAreaStatus2d::Success;
            result.hasPositiveAreaIntersection = *externalCircularResult;
            return result;
        }

        const std::vector<SCPolyline2d> firstRings = CollectRings(first);
        const std::vector<SCPolyline2d> secondRings = CollectRings(second);
        for (const SCPolyline2d& firstRing : firstRings)
        {
            for (std::size_t i = 0; i < firstRing.SegmentCount(); ++i)
            {
                const std::unique_ptr<ISCSegment2d> a = firstRing.SegmentAt(i);
                if (a == nullptr)
                {
                    continue;
                }
                for (const SCPolyline2d& secondRing : secondRings)
                {
                    for (std::size_t j = 0; j < secondRing.SegmentCount(); ++j)
                    {
                        const std::unique_ptr<ISCSegment2d> b = secondRing.SegmentAt(j);
                        if (b == nullptr)
                        {
                            continue;
                        }
                        const SCCheckedIntersection2d checked = CheckedIntersectSegments(*a, *b, tolerance);
                        if (checked.status == SCCheckedGeometryStatus2d::InvalidInput ||
                            checked.status == SCCheckedGeometryStatus2d::NumericalIndeterminate)
                        {
                            result.status = PositiveAreaStatus2d::NumericalIndeterminate;
                            return result;
                        }
                    }
                }
            }
        }

        std::vector<std::vector<std::unique_ptr<ISCSegment2d>>> arrangementRings = CollectRingSegments(first);
        std::vector<std::vector<std::unique_ptr<ISCSegment2d>>> secondSegments = CollectRingSegments(second);
        arrangementRings.insert(arrangementRings.end(),
                               std::make_move_iterator(secondSegments.begin()),
                               std::make_move_iterator(secondSegments.end()));

        const CurveArrangementResult2d arrangement = BuildPositiveAreaCurveArrangement2d(arrangementRings, tolerance);
        if (!arrangement.success)
        {
            result.status = MapArrangementFailure(arrangement.failure);
            return result;
        }
        if (arrangement.hasAmbiguousEvent)
        {
            result.status = PositiveAreaStatus2d::NumericalIndeterminate;
            return result;
        }

        bool hasPositiveArea = false;
        for (const SCPolyline2d& face : arrangement.faceRings)
        {
            const SCCheckedArea2d areaResult = CheckedComputeSignedArea(face, tolerance);
            if (areaResult.status == SCCheckedGeometryStatus2d::NumericalIndeterminate)
            {
                result.status = PositiveAreaStatus2d::NonFiniteResult;
                return result;
            }
            if (areaResult.status != SCCheckedGeometryStatus2d::Success)
            {
                result.status = PositiveAreaStatus2d::FaceClassificationFailure;
                return result;
            }
            if (!IsFinite(areaResult.signedArea))
            {
                result.status = PositiveAreaStatus2d::NonFiniteResult;
                return result;
            }
            if (areaResult.signedArea <= 0.0)
            {
                // 零面积面不贡献正面积；不得用 tolerance² 过滤小的正面积面。
                continue;
            }

            const SCCheckedRepresentative2d representative =
                CheckedFaceRepresentative(face, areaResult.signedArea, tolerance);
            if (representative.status == SCCheckedGeometryStatus2d::NumericalIndeterminate)
            {
                result.status = PositiveAreaStatus2d::FaceClassificationFailure;
                return result;
            }
            if (representative.status != SCCheckedGeometryStatus2d::Success)
            {
                result.status = PositiveAreaStatus2d::FaceClassificationFailure;
                return result;
            }

            const SCCheckedContainment2d inFirst = CheckedStrictlyInsidePolygon(representative.point, first, tolerance);
            if (inFirst.status == SCCheckedGeometryStatus2d::NumericalIndeterminate)
            {
                result.status = PositiveAreaStatus2d::NumericalIndeterminate;
                return result;
            }
            if (inFirst.status != SCCheckedGeometryStatus2d::Success)
            {
                result.status = PositiveAreaStatus2d::FaceClassificationFailure;
                return result;
            }
            const SCCheckedContainment2d inSecond = CheckedStrictlyInsidePolygon(representative.point, second, tolerance);
            if (inSecond.status == SCCheckedGeometryStatus2d::NumericalIndeterminate)
            {
                result.status = PositiveAreaStatus2d::NumericalIndeterminate;
                return result;
            }
            if (inSecond.status != SCCheckedGeometryStatus2d::Success)
            {
                result.status = PositiveAreaStatus2d::FaceClassificationFailure;
                return result;
            }

            if (inFirst.containment == SCPointContainment2d::Inside &&
                inSecond.containment == SCPointContainment2d::Inside)
            {
                hasPositiveArea = true;
            }
        }

        result.status = PositiveAreaStatus2d::Success;
        result.hasPositiveAreaIntersection = hasPositiveArea;
        return result;
    }
}  // namespace Geometry::Detail
