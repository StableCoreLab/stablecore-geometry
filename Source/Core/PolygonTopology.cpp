#include "Core/PolygonTopology.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "Core/Intersection.h"
#include "Core/Relation.h"
#include "Core/Sampling.h"
#include "Core/ShapeOps.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "RingIntegral2d.h"
#include "../Detail/CurveArrangement2d.h"
#include "../Detail/PolygonPositiveAreaIntersection2d.h"
#include "../Detail/SegmentKernel2d.h"
#include "../Detail/TriangulateContours2d.h"

namespace Geometry
{
    namespace
    {
        struct RingInfo
        {
            bool valid{false};
            SCPolygonTopologyFailure failure{SCPolygonTopologyFailure::None};
            std::uint32_t segmentIndex{0};
            double signedArea{0.0};
        };

        [[nodiscard]] bool IsSupportedSegment(const ISCSegment2d& segment)
        {
            return Detail::IsKernelSegment(segment);
        }

        [[nodiscard]] bool IsFinite(double value)
        {
            return std::isfinite(value);
        }

        constexpr std::size_t kMaxTopologyElementCount = std::numeric_limits<std::uint32_t>::max();

        [[nodiscard]] bool ExceedsTopologyElementCount(std::size_t count)
        {
            return count > kMaxTopologyElementCount;
        }

        [[nodiscard]] bool IsSharedEndpoint(const SCPoint2d& point, const SCPoint2d& expected, double tolerance)
        {
            return point.AlmostEquals(expected, tolerance);
        }

        [[nodiscard]] bool IsAllowedAdjacentIntersection(const SCSegmentIntersection2d& intersection,
                                                         const SCPoint2d& sharedPoint,
                                                         double tolerance)
        {
            if (!intersection.HasIntersection() || intersection.kind == SCIntersectionKind2d::Tangent)
            {
                return intersection.kind == SCIntersectionKind2d::None ||
                       (intersection.pointCount == 1 &&
                        IsSharedEndpoint(intersection.points[0].point, sharedPoint, tolerance));
            }
            if (intersection.kind == SCIntersectionKind2d::Overlap || intersection.pointCount == 0)
            {
                return false;
            }
            for (std::size_t i = 0; i < intersection.pointCount; ++i)
            {
                if (!IsSharedEndpoint(intersection.points[i].point, sharedPoint, tolerance))
                {
                    return false;
                }
            }
            return true;
        }

        [[nodiscard]] RingInfo InspectRing(const SCPolyline2d& ring, double tolerance)
        {
            if (!ring.IsClosed() || ring.SegmentCount() < 2)
            {
                return {false, SCPolygonTopologyFailure::InvalidOuterRing, 0, 0.0};
            }
            if (ExceedsTopologyElementCount(ring.SegmentCount()))
            {
                return {false, SCPolygonTopologyFailure::IndexOverflow, 0, 0.0};
            }

            std::vector<std::unique_ptr<ISCSegment2d>> segments;
            segments.reserve(ring.SegmentCount());
            for (std::size_t i = 0; i < ring.SegmentCount(); ++i)
            {
                std::unique_ptr<ISCSegment2d> segment = ring.SegmentAt(i);
                if (segment == nullptr || !segment->IsValid() || !IsSupportedSegment(*segment))
                {
                    return {false,
                            segment != nullptr && !IsSupportedSegment(*segment)
                                ? SCPolygonTopologyFailure::UnsupportedSegmentType
                                : SCPolygonTopologyFailure::InvalidOuterRing,
                            static_cast<std::uint32_t>(i),
                            0.0};
                }
                if (!IsFinite(segment->Length()) || segment->Length() <= tolerance)
                {
                    return {false, SCPolygonTopologyFailure::DegenerateRing, static_cast<std::uint32_t>(i), 0.0};
                }
                if (i > 0 && !segment->StartPoint().AlmostEquals(segments.back()->EndPoint(), tolerance))
                {
                    return {false, SCPolygonTopologyFailure::InvalidOuterRing, static_cast<std::uint32_t>(i), 0.0};
                }
                segments.push_back(std::move(segment));
            }
            if (!segments.back()->EndPoint().AlmostEquals(segments.front()->StartPoint(), tolerance))
            {
                return {false, SCPolygonTopologyFailure::InvalidOuterRing, 0, 0.0};
            }

            const std::size_t count = segments.size();
            for (std::size_t i = 0; i < count; ++i)
            {
                for (std::size_t j = i + 1; j < count; ++j)
                {
                    const SCSegmentIntersection2d hit = Detail::IntersectKernelSegments(*segments[i], *segments[j], tolerance);
                    const bool adjacent = j == i + 1 || (i == 0 && j + 1 == count);
                    if (!adjacent)
                    {
                        if (hit.HasIntersection())
                        {
                            return {false, SCPolygonTopologyFailure::SelfIntersection, static_cast<std::uint32_t>(i), 0.0};
                        }
                    }
                    else
                    {
                        const SCPoint2d shared = j == i + 1 ? segments[i]->EndPoint() : segments[i]->StartPoint();
                        bool adjacentIntersectionAllowed = IsAllowedAdjacentIntersection(hit, shared, tolerance);
                        if (!adjacentIntersectionAllowed && count == 2)
                        {
                            adjacentIntersectionAllowed = true;
                            for (std::size_t pointIndex = 0; pointIndex < hit.pointCount; ++pointIndex)
                            {
                                const SCPoint2d point = hit.points[pointIndex].point;
                                const bool endpointOfFirst = point.AlmostEquals(segments[i]->StartPoint(), tolerance) ||
                                                             point.AlmostEquals(segments[i]->EndPoint(), tolerance);
                                const bool endpointOfSecond = point.AlmostEquals(segments[j]->StartPoint(), tolerance) ||
                                                              point.AlmostEquals(segments[j]->EndPoint(), tolerance);
                                adjacentIntersectionAllowed = adjacentIntersectionAllowed && endpointOfFirst && endpointOfSecond;
                            }
                        }
                        if (!adjacentIntersectionAllowed)
                        {
                            return {false,
                                    hit.kind == SCIntersectionKind2d::Overlap
                                        ? SCPolygonTopologyFailure::BoundaryOverlap
                                        : SCPolygonTopologyFailure::SelfIntersection,
                                    static_cast<std::uint32_t>(i),
                                    0.0};
                        }
                    }
                }
            }

            const double area = Detail::ComputeSignedArea(ring);
            if (!IsFinite(area) || std::abs(area) <= tolerance * tolerance)
            {
                return {false, SCPolygonTopologyFailure::DegenerateRing, 0, area};
            }
            return {true, SCPolygonTopologyFailure::None, 0, area};
        }

        [[nodiscard]] bool HasBoundaryIntersection(const SCPolyline2d& first,
                                                    const SCPolyline2d& second,
                                                    double tolerance,
                                                    bool& hasOverlap)
        {
            hasOverlap = false;
            for (std::size_t i = 0; i < first.SegmentCount(); ++i)
            {
                const std::unique_ptr<ISCSegment2d> lhs = first.SegmentAt(i);
                for (std::size_t j = 0; j < second.SegmentCount(); ++j)
                {
                    const std::unique_ptr<ISCSegment2d> rhs = second.SegmentAt(j);
                    const SCSegmentIntersection2d hit = Detail::IntersectKernelSegments(*lhs, *rhs, tolerance);
                    if (hit.kind == SCIntersectionKind2d::Overlap)
                    {
                        hasOverlap = true;
                        return true;
                    }
                    if (hit.HasIntersection())
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        [[nodiscard]] bool HasBoundaryCrossing(const SCPolygon2d& first,
                                                const SCPolygon2d& second,
                                                double tolerance,
                                                bool& hasOverlap,
                                                bool& hasTouch)
        {
            hasOverlap = false;
            hasTouch = false;
            std::vector<SCPolyline2d> firstRings{first.OuterRing()};
            std::vector<SCPolyline2d> secondRings{second.OuterRing()};
            for (std::size_t i = 0; i < first.HoleCount(); ++i)
            {
                firstRings.push_back(first.HoleAt(i));
            }
            for (std::size_t i = 0; i < second.HoleCount(); ++i)
            {
                secondRings.push_back(second.HoleAt(i));
            }
            for (const SCPolyline2d& firstRing : firstRings)
            {
                for (const SCPolyline2d& secondRing : secondRings)
                {
                    for (std::size_t i = 0; i < firstRing.SegmentCount(); ++i)
                    {
                        const std::unique_ptr<ISCSegment2d> firstSegment = firstRing.SegmentAt(i);
                        for (std::size_t j = 0; j < secondRing.SegmentCount(); ++j)
                        {
                            const std::unique_ptr<ISCSegment2d> secondSegment = secondRing.SegmentAt(j);
                            const SCSegmentIntersection2d hit = Detail::IntersectKernelSegments(*firstSegment, *secondSegment, tolerance);
                            if (hit.kind == SCIntersectionKind2d::Overlap)
                            {
                                hasOverlap = true;
                                continue;
                            }
                            if (!hit.HasIntersection())
                            {
                                continue;
                            }
                            hasTouch = true;
                            for (std::size_t k = 0; k < hit.pointCount; ++k)
                            {
                                const double firstParameter = hit.points[k].parameterOnFirst;
                                const double secondParameter = hit.points[k].parameterOnSecond;
                                const double firstDelta = std::min(0.25, std::max(1e-7, tolerance));
                                const double secondDelta = firstDelta;
                                const double firstBefore = std::clamp(firstParameter - firstDelta, 0.0, 1.0);
                                const double firstAfter = std::clamp(firstParameter + firstDelta, 0.0, 1.0);
                                const double secondBefore = std::clamp(secondParameter - secondDelta, 0.0, 1.0);
                                const double secondAfter = std::clamp(secondParameter + secondDelta, 0.0, 1.0);
                                const SCPointContainment2d firstBeforeLocation =
                                    LocatePoint(firstSegment->PointAt(firstBefore), second, tolerance);
                                const SCPointContainment2d firstAfterLocation =
                                    LocatePoint(firstSegment->PointAt(firstAfter), second, tolerance);
                                const SCPointContainment2d secondBeforeLocation =
                                    LocatePoint(secondSegment->PointAt(secondBefore), first, tolerance);
                                const SCPointContainment2d secondAfterLocation =
                                    LocatePoint(secondSegment->PointAt(secondAfter), first, tolerance);
                                const bool firstChangesSide =
                                    (firstBeforeLocation == SCPointContainment2d::Inside &&
                                     firstAfterLocation == SCPointContainment2d::Outside) ||
                                    (firstBeforeLocation == SCPointContainment2d::Outside &&
                                     firstAfterLocation == SCPointContainment2d::Inside);
                                const bool secondChangesSide =
                                    (secondBeforeLocation == SCPointContainment2d::Inside &&
                                     secondAfterLocation == SCPointContainment2d::Outside) ||
                                    (secondBeforeLocation == SCPointContainment2d::Outside &&
                                     secondAfterLocation == SCPointContainment2d::Inside);
                                if (firstChangesSide || secondChangesSide)
                                {
                                    return true;
                                }
                            }
                        }
                    }
                }
            }
            return false;
        }

        [[nodiscard]] SCPolygonNormalizeResult NormalizeInternal(const SCPolygon2d& polygon, double tolerance)
        {
            if (!IsFinite(tolerance) || tolerance <= 0.0)
            {
                return {false, {}, SCPolygonTopologyFailure::InvalidValidationTolerance, 0, 0};
            }
            if (ExceedsTopologyElementCount(polygon.HoleCount()))
            {
                return {false, {}, SCPolygonTopologyFailure::IndexOverflow, 0, 0};
            }
            const RingInfo outerInfo = InspectRing(polygon.OuterRing(), tolerance);
            if (!outerInfo.valid)
            {
                return {false, {}, outerInfo.failure, 0, outerInfo.segmentIndex};
            }

            SCPolyline2d outer = outerInfo.signedArea < 0.0 ? Reverse(polygon.OuterRing()) : polygon.OuterRing();
            std::vector<SCPolyline2d> holes;
            holes.reserve(polygon.HoleCount());
            for (std::size_t h = 0; h < polygon.HoleCount(); ++h)
            {
                const auto loopIndex = static_cast<std::uint32_t>(h + 1);
                const SCPolyline2d sourceHole = polygon.HoleAt(h);
                const RingInfo holeInfo = InspectRing(sourceHole, tolerance);
                if (!holeInfo.valid)
                {
                            return {false,
                                    {},
                                    holeInfo.failure == SCPolygonTopologyFailure::InvalidOuterRing
                                        ? SCPolygonTopologyFailure::InvalidHoleRing
                                        : holeInfo.failure,
                                    loopIndex,
                                    holeInfo.segmentIndex};
                }
                SCPolyline2d hole = holeInfo.signedArea > 0.0 ? Reverse(sourceHole) : sourceHole;
                const SCPointContainment2d location = LocatePoint(hole.StartPoint(), outer, tolerance);
                if (location != SCPointContainment2d::Inside)
                {
                    return {false,
                            {},
                            location == SCPointContainment2d::OnBoundary
                                ? SCPolygonTopologyFailure::BoundaryTouching
                                : SCPolygonTopologyFailure::HoleOutsideOuterRing,
                            loopIndex,
                            0};
                }
                bool overlap = false;
                if (HasBoundaryIntersection(outer, hole, tolerance, overlap))
                {
                    return {false,
                            {},
                            overlap ? SCPolygonTopologyFailure::BoundaryOverlap
                                    : SCPolygonTopologyFailure::BoundaryTouching,
                            loopIndex,
                            0};
                }
                for (std::size_t previous = 0; previous < holes.size(); ++previous)
                {
                    if (HasBoundaryIntersection(holes[previous], hole, tolerance, overlap))
                    {
                        return {false,
                                {},
                                overlap ? SCPolygonTopologyFailure::BoundaryOverlap
                                        : SCPolygonTopologyFailure::HoleIntersection,
                                loopIndex,
                                0};
                    }
                    if (LocatePoint(hole.StartPoint(), holes[previous], tolerance) == SCPointContainment2d::Inside ||
                        LocatePoint(holes[previous].StartPoint(), hole, tolerance) == SCPointContainment2d::Inside)
                    {
                        return {false, {}, SCPolygonTopologyFailure::HoleContainment, loopIndex, 0};
                    }
                }
                holes.push_back(std::move(hole));
            }
            SCPolygon2d normalized(std::move(outer), std::move(holes));
            return {true, std::move(normalized), SCPolygonTopologyFailure::None, 0, 0};
        }

        struct SampledRing
        {
            std::vector<SCPoint2d> points{};
            std::vector<SCTessellatedBoundaryVertex2d> vertices{};
        };

        enum class SampleRingFailure
        {
            None,
            TessellationFailure,
            IndexOverflow
        };

        [[nodiscard]] bool ArcPartCount(const SCArcSegment2d& arc, double tolerance, std::size_t& partCount)
        {
            const double radius = arc.radius;
            const double q = std::min(tolerance, radius) / radius;
            const double alpha = 2.0 * std::asin(std::sqrt(q * (2.0 - q)));
            if (!IsFinite(alpha) || alpha <= 0.0)
            {
                return false;
            }
            const double count = std::ceil(std::abs(arc.sweepAngle) / alpha);
            if (!IsFinite(count) || count < 1.0 || count > static_cast<double>(std::numeric_limits<std::uint32_t>::max()))
            {
                return false;
            }
            partCount = std::max<std::size_t>(1, static_cast<std::size_t>(count));
            return true;
        }

        [[nodiscard]] SampleRingFailure SampleRing(const SCPolyline2d& ring,
                                                   std::uint32_t loopIndex,
                                                   std::uint64_t stableVertexBase,
                                                   double tolerance,
                                                   SampledRing& output)
        {
            output = {};
            for (std::size_t i = 0; i < ring.SegmentCount(); ++i)
            {
                const std::unique_ptr<ISCSegment2d> segment = ring.SegmentAt(i);
                if (segment == nullptr)
                {
                    return SampleRingFailure::TessellationFailure;
                }
                std::size_t partCount = 1;
                if (segment->Kind() == SCSegmentKind2::Arc)
                {
                    const auto* arc = dynamic_cast<const SCArcSegment2d*>(segment.get());
                    if (arc == nullptr || !ArcPartCount(*arc, tolerance, partCount))
                    {
                        return SampleRingFailure::TessellationFailure;
                    }
                }
                std::vector<double> parameters;
                parameters.reserve(partCount + 1);
                for (std::size_t p = 0; p <= partCount; ++p)
                {
                    parameters.push_back(static_cast<double>(p) / static_cast<double>(partCount));
                }
                const Detail::SegmentSplitResult2d split =
                    Detail::SplitAtParameters(*segment, parameters, tolerance);
                if (!split.success || split.pieces.size() != partCount)
                {
                    return SampleRingFailure::TessellationFailure;
                }
                for (std::size_t p = 0; p < split.pieces.size(); ++p)
                {
                    const Detail::SegmentSplitPiece2d& piece = split.pieces[p];
                    const SCPoint2d point = piece.segment->StartPoint();
                    if (output.points.size() > 0 && p == 0 &&
                        output.points.back().AlmostEquals(point, tolerance))
                    {
                        continue;
                    }
                    if (!point.IsValid())
                    {
                        return SampleRingFailure::TessellationFailure;
                    }
                    if (output.points.size() >= kMaxTopologyElementCount)
                    {
                        return SampleRingFailure::IndexOverflow;
                    }
                    output.vertices.push_back({stableVertexBase + output.vertices.size(),
                                               loopIndex,
                                               static_cast<std::uint32_t>(i),
                                               piece.sourceStart,
                                               point});
                    output.points.push_back(point);
                }
            }
            return output.points.size() >= 3 ? SampleRingFailure::None : SampleRingFailure::TessellationFailure;
        }

        [[nodiscard]] bool AnyVertexInside(const SCPolyline2d& ring, const SCPolygon2d& polygon, double tolerance)
        {
            for (std::size_t i = 0; i < ring.VertexCount(); ++i)
            {
                if (LocatePoint(ring.VertexAt(i), polygon, tolerance) == SCPointContainment2d::Inside)
                {
                    return true;
                }
            }
            return false;
        }

        [[nodiscard]] bool AllVerticesStrictlyInside(const SCPolyline2d& ring,
                                                     const SCPolygon2d& polygon,
                                                     double tolerance)
        {
            for (std::size_t i = 0; i < ring.VertexCount(); ++i)
            {
                if (LocatePoint(ring.VertexAt(i), polygon, tolerance) != SCPointContainment2d::Inside)
                {
                    return false;
                }
            }
            return true;
        }

        [[nodiscard]] bool AllVerticesStrictlyInsideRing(const SCPolyline2d& ring,
                                                         const SCPolyline2d& boundary,
                                                         double tolerance)
        {
            for (std::size_t i = 0; i < ring.VertexCount(); ++i)
            {
                if (LocatePoint(ring.VertexAt(i), boundary, tolerance) != SCPointContainment2d::Inside)
                {
                    return false;
                }
            }
            return true;
        }

        [[nodiscard]] double TriangleSignedArea(const SCPoint2d& first,
                                                 const SCPoint2d& second,
                                                 const SCPoint2d& third)
        {
            const SCVector2d firstEdge = second - first;
            const SCVector2d secondEdge = third - first;
            return 0.5 * (firstEdge.x * secondEdge.y - firstEdge.y * secondEdge.x);
        }

        [[nodiscard]] std::optional<SCPoint2d> FindFaceInteriorSample(const SCPolyline2d& ring,
                                                                        double signedArea,
                                                                        double tolerance)
        {
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
                const SCPoint2d sample = segment->PointAt(0.5) + inward * offset;
                if (sample.IsValid())
                {
                    return sample;
                }
            }
            return std::nullopt;
        }
    }  // namespace

    SCPolygonNormalizeResult NormalizePolygon(const SCPolygon2d& polygon, double validationTolerance)
    {
        return NormalizeInternal(polygon, validationTolerance);
    }

    SCTessellatedPolygonResult TessellateAndTriangulatePolygon(const SCPolygon2d& polygon,
                                                               double validationTolerance,
                                                               double deterministicTolerance)
    {
        if (!IsFinite(deterministicTolerance) || deterministicTolerance <= 0.0)
        {
            return {false, {}, SCPolygonTopologyFailure::InvalidDeterministicTolerance, 0, 0};
        }
        const SCPolygonNormalizeResult normalized = NormalizeInternal(polygon, validationTolerance);
        if (!normalized.success)
        {
            return {false, {}, normalized.failure, normalized.loopIndex, normalized.segmentIndex};
        }

        SCTessellatedPolygon2d result;
        result.normalizedPolygon = normalized.polygon;
        result.deterministicTolerance = deterministicTolerance;
        std::vector<std::vector<SCPoint2d>> sampledLoops;
        sampledLoops.reserve(1 + result.normalizedPolygon.HoleCount());
        auto sampleLoop = [&](const SCPolyline2d& ring, std::uint32_t loopIndex) -> SampleRingFailure {
            SampledRing sampled;
            const SampleRingFailure sampleFailure =
                SampleRing(ring, loopIndex, result.vertices.size(), deterministicTolerance, sampled);
            if (sampleFailure != SampleRingFailure::None)
            {
                return sampleFailure;
            }
            if (sampled.vertices.size() > kMaxTopologyElementCount - result.vertices.size())
            {
                return SampleRingFailure::IndexOverflow;
            }
            result.loopStartIndices.push_back(static_cast<std::uint32_t>(result.vertices.size()));
            result.vertices.insert(result.vertices.end(), sampled.vertices.begin(), sampled.vertices.end());
            sampledLoops.push_back(std::move(sampled.points));
            return SampleRingFailure::None;
        };
        if (const SampleRingFailure failure = sampleLoop(result.normalizedPolygon.OuterRing(), 0);
            failure != SampleRingFailure::None)
        {
            return {false,
                    {},
                    failure == SampleRingFailure::IndexOverflow ? SCPolygonTopologyFailure::IndexOverflow
                                                                : SCPolygonTopologyFailure::TessellationFailure,
                    0,
                    0};
        }
        for (std::size_t h = 0; h < result.normalizedPolygon.HoleCount(); ++h)
        {
            const auto loopIndex = static_cast<std::uint32_t>(h + 1);
            if (const SampleRingFailure failure = sampleLoop(result.normalizedPolygon.HoleAt(h), loopIndex);
                failure != SampleRingFailure::None)
            {
                return {false,
                        {},
                        failure == SampleRingFailure::IndexOverflow ? SCPolygonTopologyFailure::IndexOverflow
                                                                    : SCPolygonTopologyFailure::TessellationFailure,
                        loopIndex,
                        0};
            }
        }

        std::vector<SCPolyline2d> sampledHoleRings;
        auto makeRing = [](const std::vector<SCPoint2d>& points) {
            return SCPolyline2d(points, SCPolylineClosure::Closed);
        };
        const SCPolyline2d sampledOuter = makeRing(sampledLoops[0]);
        for (std::size_t h = 1; h < sampledLoops.size(); ++h)
        {
            sampledHoleRings.push_back(makeRing(sampledLoops[h]));
        }
        const auto triangles = Detail::TriangulateContours2d(sampledLoops, validationTolerance);
        if (!triangles.has_value())
        {
            return {false, {}, SCPolygonTopologyFailure::TriangulationFailure, 0, 0};
        }
        const SCPolygon2d sampledPolygon(sampledOuter, sampledHoleRings);
        const double expectedArea = std::abs(Detail::ComputeSignedArea(sampledOuter)) -
                                    [&]() {
                                        double holeArea = 0.0;
                                        for (const SCPolyline2d& hole : sampledHoleRings)
                                        {
                                            holeArea += std::abs(Detail::ComputeSignedArea(hole));
                                        }
                                        return holeArea;
                                    }();
        double actualArea = 0.0;
        for (const SCTriangleIndex3& triangle : *triangles)
        {
            for (const std::size_t index : triangle.indices)
            {
                if (index >= result.vertices.size())
                {
                    return {false, {}, SCPolygonTopologyFailure::InvalidTriangulation, 0, 0};
                }
            }
            if (triangle.indices[0] == triangle.indices[1] || triangle.indices[1] == triangle.indices[2] ||
                triangle.indices[0] == triangle.indices[2])
            {
                return {false, {}, SCPolygonTopologyFailure::InvalidTriangulation, 0, 0};
            }
            const SCPoint2d& first = result.vertices[triangle.indices[0]].point;
            const SCPoint2d& second = result.vertices[triangle.indices[1]].point;
            const SCPoint2d& third = result.vertices[triangle.indices[2]].point;
            const double signedArea = TriangleSignedArea(first, second, third);
            if (!IsFinite(signedArea) || signedArea <= validationTolerance * validationTolerance)
            {
                return {false, {}, SCPolygonTopologyFailure::InvalidTriangulation, 0, 0};
            }
            const SCPoint2d center{(first.x + second.x + third.x) / 3.0,
                                   (first.y + second.y + third.y) / 3.0};
            if (LocatePoint(center, sampledPolygon, validationTolerance) == SCPointContainment2d::Outside)
            {
                return {false, {}, SCPolygonTopologyFailure::InvalidTriangulation, 0, 0};
            }
            actualArea += signedArea;
            result.triangles.push_back(triangle);
        }
        const double areaTolerance = std::max(validationTolerance * validationTolerance,
                                              expectedArea * 1e-9);
        if (!IsFinite(expectedArea) || !IsFinite(actualArea) || expectedArea <= 0.0 ||
            std::abs(actualArea - expectedArea) > areaTolerance)
        {
            return {false, {}, SCPolygonTopologyFailure::InvalidTriangulation, 0, 0};
        }
        return {true, std::move(result), SCPolygonTopologyFailure::None, 0, 0};
    }

    SCPolygonContainmentResult ClassifyContainment(const SCPolygon2d& container,
                                                    const SCPolygon2d& candidate,
                                                    double validationTolerance)
    {
        const SCPolygonNormalizeResult normalizedContainer = NormalizeInternal(container, validationTolerance);
        const SCPolygonNormalizeResult normalizedCandidate = NormalizeInternal(candidate, validationTolerance);
        if (!normalizedContainer.success || !normalizedCandidate.success)
        {
            return {false, SCPolygonContainment::Unknown,
                    !normalizedContainer.success ? normalizedContainer.failure : normalizedCandidate.failure};
        }
        const SCPolygon2d& c = normalizedContainer.polygon;
        const SCPolygon2d& p = normalizedCandidate.polygon;
        std::vector<std::vector<std::unique_ptr<ISCSegment2d>>> arrangementRings;
        const auto collectRings = [&arrangementRings](const SCPolygon2d& source) {
            std::vector<SCPolyline2d> rings{source.OuterRing()};
            for (std::size_t i = 0; i < source.HoleCount(); ++i)
            {
                rings.push_back(source.HoleAt(i));
            }
            for (const SCPolyline2d& ring : rings)
            {
                std::vector<std::unique_ptr<ISCSegment2d>> segments;
                for (std::size_t i = 0; i < ring.SegmentCount(); ++i)
                {
                    segments.push_back(ring.SegmentAt(i));
                }
                arrangementRings.push_back(std::move(segments));
            }
        };
        collectRings(c);
        collectRings(p);
        const Detail::CurveArrangementResult2d arrangement =
            Detail::BuildCurveArrangement2d(arrangementRings, validationTolerance);
        if (!arrangement.success)
        {
            return {false, SCPolygonContainment::Unknown, SCPolygonTopologyFailure::UnsupportedSegmentOperation};
        }
        if (arrangement.hasAmbiguousEvent)
        {
            return {false, SCPolygonContainment::Unknown, SCPolygonTopologyFailure::AmbiguousTopology};
        }
        bool overlap = false;
        bool touching = false;
        if (HasBoundaryCrossing(c, p, validationTolerance, overlap, touching))
        {
            return {true, SCPolygonContainment::Intersecting, SCPolygonTopologyFailure::None};
        }
        if (overlap || touching)
        {
            return {true,
                    overlap ? SCPolygonContainment::Intersecting : SCPolygonContainment::Touching,
                    SCPolygonTopologyFailure::None};
        }

        bool candidateHasFilledFace = false;
        bool candidateHasFaceOutsideContainer = false;
        bool candidateHasFaceInsideContainer = false;
        int candidateOutsideContainerHole{-2};
        bool containerHasFaceInsideCandidate = false;
        for (const SCPolyline2d& face : arrangement.faceRings)
        {
            const double faceArea = Detail::ComputeSignedArea(face);
            if (!IsFinite(faceArea) || faceArea <= validationTolerance * validationTolerance)
            {
                continue;
            }
            const std::optional<SCPoint2d> sample = FindFaceInteriorSample(face, faceArea, validationTolerance);
            if (!sample.has_value())
            {
                continue;
            }
            const bool inContainer = LocatePoint(*sample, c, validationTolerance) == SCPointContainment2d::Inside;
            const bool inCandidate = LocatePoint(*sample, p, validationTolerance) == SCPointContainment2d::Inside;
            if (inCandidate)
            {
                candidateHasFilledFace = true;
                candidateHasFaceInsideContainer = candidateHasFaceInsideContainer || inContainer;
                candidateHasFaceOutsideContainer = candidateHasFaceOutsideContainer || !inContainer;
                if (!inContainer)
                {
                    int holeIndex = -1;
                    for (std::size_t h = 0; h < c.HoleCount(); ++h)
                    {
                        if (LocatePoint(*sample, c.HoleAt(h), validationTolerance) == SCPointContainment2d::Inside)
                        {
                            holeIndex = static_cast<int>(h);
                            break;
                        }
                    }
                    if (candidateOutsideContainerHole == -2)
                    {
                        candidateOutsideContainerHole = holeIndex;
                    }
                    else if (candidateOutsideContainerHole != holeIndex)
                    {
                        candidateOutsideContainerHole = -1;
                    }
                }
            }
            if (inContainer && inCandidate)
            {
                containerHasFaceInsideCandidate = true;
            }
        }
        if (candidateHasFilledFace && !candidateHasFaceOutsideContainer && candidateHasFaceInsideContainer)
        {
            return {true, SCPolygonContainment::StrictInside, SCPolygonTopologyFailure::None};
        }
        if (candidateHasFilledFace && !candidateHasFaceInsideContainer && candidateOutsideContainerHole >= 0)
        {
            return {true, SCPolygonContainment::InsideHole, SCPolygonTopologyFailure::None};
        }
        if ((candidateHasFaceInsideContainer && candidateHasFaceOutsideContainer) || containerHasFaceInsideCandidate)
        {
            return {true, SCPolygonContainment::Intersecting, SCPolygonTopologyFailure::None};
        }
        if (!candidateHasFilledFace)
        {
            return {false, SCPolygonContainment::Unknown, SCPolygonTopologyFailure::AmbiguousTopology};
        }
        return {true, SCPolygonContainment::Disjoint, SCPolygonTopologyFailure::None};
    }

    SCPolygonAppendHoleResult AppendHole(const SCPolygon2d& polygon,
                                         const SCPolyline2d& hole,
                                         double validationTolerance)
    {
        const SCPolygonNormalizeResult normalized = NormalizeInternal(polygon, validationTolerance);
        if (!normalized.success)
        {
            return {false, {}, normalized.failure};
        }
        const SCPolygonNormalizeResult candidate = NormalizeInternal(SCPolygon2d(hole), validationTolerance);
        if (!candidate.success)
        {
            return {false, {}, candidate.failure};
        }
        const SCPolygonContainmentResult containment =
            ClassifyContainment(normalized.polygon, candidate.polygon, validationTolerance);
        if (!containment.success || containment.containment != SCPolygonContainment::StrictInside)
        {
            return {false,
                    {},
                    containment.success ? SCPolygonTopologyFailure::HoleOutsideOuterRing : containment.failure};
        }
        std::vector<SCPolyline2d> holes;
        holes.reserve(normalized.polygon.HoleCount() + 1);
        for (std::size_t i = 0; i < normalized.polygon.HoleCount(); ++i)
        {
            holes.push_back(normalized.polygon.HoleAt(i));
        }
        holes.push_back(candidate.polygon.OuterRing());
        const SCPolygonNormalizeResult result = NormalizeInternal(
            SCPolygon2d(normalized.polygon.OuterRing(), std::move(holes)), validationTolerance);
        if (!result.success)
        {
            return {false, {}, result.failure};
        }
        return {true, std::move(result.polygon), SCPolygonTopologyFailure::None};
    }

    SCPolygonPositiveAreaIntersectionResult2d QueryPolygonPositiveAreaIntersection(const SCPolygon2d& first,
                                                                                   const SCPolygon2d& second,
                                                                                   double eps)
    {
        SCPolygonPositiveAreaIntersectionResult2d result;
        // eps 仅用于输入规范化和普通浮点稳健控制，不是面积阈值；必须有限且为正。
        if (!IsFinite(eps) || eps <= 0.0)
        {
            result.success = false;
            result.hasPositiveAreaIntersection = false;
            result.failure = SCPolygonPositiveAreaIntersectionFailure2d::InvalidInput;
            return result;
        }
        if (!first.IsValid() || !second.IsValid())
        {
            result.success = false;
            result.hasPositiveAreaIntersection = false;
            result.failure = SCPolygonPositiveAreaIntersectionFailure2d::InvalidInput;
            return result;
        }

        // 复用 NormalizePolygon 的输入检查；规范化失败不得解释为无冲突。
        const SCPolygonNormalizeResult normalizedFirst = NormalizePolygon(first, eps);
        if (!normalizedFirst.success)
        {
            result.success = false;
            result.hasPositiveAreaIntersection = false;
            result.failure = SCPolygonPositiveAreaIntersectionFailure2d::NormalizationFailure;
            return result;
        }
        const SCPolygonNormalizeResult normalizedSecond = NormalizePolygon(second, eps);
        if (!normalizedSecond.success)
        {
            result.success = false;
            result.hasPositiveAreaIntersection = false;
            result.failure = SCPolygonPositiveAreaIntersectionFailure2d::NormalizationFailure;
            return result;
        }

        const Detail::PositiveAreaResult2d arrangement = Detail::BuildPositiveAreaArrangement2d(
            normalizedFirst.polygon, normalizedSecond.polygon, eps);
        switch (arrangement.status)
        {
            case Detail::PositiveAreaStatus2d::Success:
                result.success = true;
                result.hasPositiveAreaIntersection = arrangement.hasPositiveAreaIntersection;
                result.failure = SCPolygonPositiveAreaIntersectionFailure2d::None;
                break;
            case Detail::PositiveAreaStatus2d::ArrangementFailure:
                result.success = false;
                result.hasPositiveAreaIntersection = false;
                result.failure = SCPolygonPositiveAreaIntersectionFailure2d::ArrangementFailure;
                break;
            case Detail::PositiveAreaStatus2d::FaceClassificationFailure:
                result.success = false;
                result.hasPositiveAreaIntersection = false;
                result.failure = SCPolygonPositiveAreaIntersectionFailure2d::FaceClassificationFailure;
                break;
            case Detail::PositiveAreaStatus2d::NumericalIndeterminate:
                result.success = false;
                result.hasPositiveAreaIntersection = false;
                result.failure = SCPolygonPositiveAreaIntersectionFailure2d::NumericalIndeterminate;
                break;
            case Detail::PositiveAreaStatus2d::NonFiniteResult:
                result.success = false;
                result.hasPositiveAreaIntersection = false;
                result.failure = SCPolygonPositiveAreaIntersectionFailure2d::NonFiniteResult;
                break;
        }
        return result;
    }
}  // namespace Geometry
