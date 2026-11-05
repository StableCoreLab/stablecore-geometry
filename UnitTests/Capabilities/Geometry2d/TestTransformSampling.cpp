#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include "Core/Editing.h"
#include "Core/Sampling.h"
#include "Core/Transform.h"

using Geometry::SCArcSegment2d;
using Geometry::SCBox2d;
using Geometry::InsertPoint;
using Geometry::SCLineSegment2d;
using Geometry::Mirror;
using Geometry::Normalize;
using Geometry::SCPoint2d;
using Geometry::SCPolygon2d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;
using Geometry::Rotate;
using Geometry::SampleByMaxAngle;
using Geometry::Stretch;
using Geometry::Translate;
using Geometry::SCVector2d;

TEST(TransformSamplingTest, TranslatesRotatesMirrorsAndStretches)
{
    const SCLineSegment2d line(SCPoint2d{0.0, 0.0}, SCPoint2d{2.0, 0.0});
    const auto moved = Translate(line, SCVector2d{1.0, 2.0});
    EXPECT_TRUE(moved.startPoint.AlmostEquals(SCPoint2d{1.0, 2.0}, 1e-12));

    const auto rotated = Rotate(SCPoint2d{1.0, 0.0}, SCPoint2d{0.0, 0.0}, Geometry::kPi * 0.5);
    EXPECT_TRUE(rotated.AlmostEquals(SCPoint2d{0.0, 1.0}, 1e-12));

    const auto mirrored = Mirror(SCPoint2d{1.0, 2.0}, SCPoint2d{0.0, 0.0}, SCVector2d{1.0, 0.0});
    EXPECT_TRUE(mirrored.AlmostEquals(SCPoint2d{1.0, -2.0}, 1e-12));

    const auto stretched =
        Stretch(SCPoint2d{1.0, 1.0}, SCBox2d::FromMinMax(SCPoint2d{0.0, 0.0}, SCPoint2d{2.0, 2.0}), SCVector2d{3.0, 0.0});
    EXPECT_TRUE(stretched.AlmostEquals(SCPoint2d{4.0, 1.0}, 1e-12));
}

TEST(TransformSamplingTest, SamplesArcAndEditsPolyline)
{
    const SCArcSegment2d arc(SCPoint2d{0.0, 0.0}, 1.0, 0.0, Geometry::kPi * 0.5);
    const auto arcSamples = SampleByMaxAngle(arc, Geometry::kPi * 0.25);
    ASSERT_EQ(arcSamples.size(), 3);

    const SCPolyline2d path({SCPoint2d{0.0, 0.0}, SCPoint2d{2.0, 0.0}, SCPoint2d{2.0, 0.0}, SCPoint2d{2.0, 2.0}},
                          SCPolylineClosure::Open);
    const auto normalized = Normalize(path);
    ASSERT_EQ(normalized.PointCount(), 3);
    EXPECT_FALSE(path.HasNonLineSegments());

    const auto inserted =
        InsertPoint(SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}}, SCPolylineClosure::Open), SCPoint2d{2.0, 0.0});
    ASSERT_EQ(inserted.PointCount(), 3);
}

TEST(TransformSamplingTest, TransformsMixedPolylineAndPolygon)
{
    const SCPolyline2d mixedPath(
        {std::make_shared<SCLineSegment2d>(SCPoint2d{0.0, 0.0}, SCPoint2d{1.0, 0.0}),
         std::make_shared<SCArcSegment2d>(
             SCPoint2d{1.0, 1.0}, 1.0, -Geometry::kPi * 0.5, 0.0, Geometry::SCArcDirection2d::CounterClockwise)},
        SCPolylineClosure::Open);
    const SCPolyline2d movedMixedPath = Translate(mixedPath, SCVector2d{2.0, 3.0});
    ASSERT_TRUE(movedMixedPath.IsValid());
    ASSERT_EQ(movedMixedPath.SegmentAt(0)->Kind(), Geometry::SCSegmentKind2d::Line);
    ASSERT_EQ(movedMixedPath.SegmentAt(1)->Kind(), Geometry::SCSegmentKind2d::Arc);
    const SCPolyline2d rotatedMixedPath = Rotate(mixedPath, SCPoint2d{0.0, 0.0}, Geometry::kPi * 0.5);
    ASSERT_TRUE(rotatedMixedPath.IsValid());
    ASSERT_EQ(rotatedMixedPath.SegmentAt(0)->Kind(), Geometry::SCSegmentKind2d::Line);
    ASSERT_EQ(rotatedMixedPath.SegmentAt(1)->Kind(), Geometry::SCSegmentKind2d::Arc);

    const SCPolygon2d polygon(SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}},
                                       SCPolylineClosure::Closed));
    const auto movedPolygon = Translate(polygon, SCVector2d{1.0, 0.0});
    EXPECT_TRUE(movedPolygon.OuterRing().VertexAt(0).AlmostEquals(SCPoint2d{1.0, 0.0}, 1e-12));
}

TEST(TransformSamplingTest, NormalizePreservesArcWhileRemovingZeroLengthLine)
{
    const auto line = std::make_shared<SCLineSegment2d>(SCPoint2d{0.0, 0.0}, SCPoint2d{1.0, 0.0});
    const auto duplicate = std::make_shared<SCLineSegment2d>(SCPoint2d{1.0, 0.0}, SCPoint2d{1.0, 0.0});
    const auto arc = std::make_shared<SCArcSegment2d>(
        SCPoint2d{1.0, 1.0}, 1.0, -Geometry::kPi * 0.5, Geometry::kPi * 0.5);
    const SCPolyline2d path({line, duplicate, arc}, SCPolylineClosure::Open);
    EXPECT_TRUE(path.HasNonLineSegments());

    const SCPolyline2d normalized = Normalize(path, 1e-12);
    ASSERT_TRUE(normalized.IsValid(1e-12));
    ASSERT_EQ(normalized.SegmentCount(), 2U);
    EXPECT_EQ(normalized.SegmentAt(1)->Kind(), Geometry::SCSegmentKind2d::Arc);
    EXPECT_NEAR(normalized.Length(), 1.0 + Geometry::kPi * 0.5, 1e-12);
    EXPECT_TRUE(normalized.Bounds().MaxPoint().AlmostEquals({2.0, 1.0}, 1e-12));

    const auto closeLine = std::make_shared<SCLineSegment2d>(arc->EndPoint(), arc->StartPoint());
    const SCPolyline2d closed({arc, closeLine}, SCPolylineClosure::Closed);
    const SCPolyline2d normalizedClosed = Normalize(closed, 1e-12);
    ASSERT_TRUE(normalizedClosed.IsValid());
    EXPECT_TRUE(normalizedClosed.IsClosed());
    EXPECT_EQ(normalizedClosed.SegmentAt(0)->Kind(), Geometry::SCSegmentKind2d::Arc);
    EXPECT_NEAR(normalizedClosed.Length(), closed.Length(), 1e-12);
}

TEST(TransformSamplingTest, InsertPointPreservesMixedSegmentsAndPolygonHoles)
{
    const auto line = std::make_shared<SCLineSegment2d>(SCPoint2d{0.0, 0.0}, SCPoint2d{1.0, 0.0});
    const auto arc = std::make_shared<SCArcSegment2d>(
        SCPoint2d{1.0, 1.0}, 1.0, -Geometry::kPi * 0.5, Geometry::kPi * 0.5);
    const auto tail = std::make_shared<SCLineSegment2d>(arc->EndPoint(), SCPoint2d{3.0, 1.0});
    const SCPolyline2d path({line, arc, tail}, SCPolylineClosure::Open);
    const SCPolyline2d insertedLine = InsertPoint(path, {0.5, 0.0}, 1e-12);
    ASSERT_TRUE(insertedLine.IsValid(1e-12));
    ASSERT_EQ(insertedLine.SegmentCount(), 4U);
    EXPECT_EQ(insertedLine.SegmentAt(2)->Kind(), Geometry::SCSegmentKind2d::Arc);
    EXPECT_EQ(insertedLine.SegmentAt(3)->Kind(), Geometry::SCSegmentKind2d::Line);
    EXPECT_NEAR(insertedLine.Length(), path.Length(), 1e-12);

    const SCPolyline2d insertedArc = InsertPoint(path, arc->PointAt(0.5), 1e-12);
    ASSERT_TRUE(insertedArc.IsValid(1e-12));
    ASSERT_EQ(insertedArc.SegmentCount(), 4U);
    EXPECT_EQ(insertedArc.SegmentAt(1)->Kind(), Geometry::SCSegmentKind2d::Arc);
    EXPECT_EQ(insertedArc.SegmentAt(2)->Kind(), Geometry::SCSegmentKind2d::Arc);
    EXPECT_EQ(insertedArc.SegmentAt(3)->Kind(), Geometry::SCSegmentKind2d::Line);
    EXPECT_NEAR(insertedArc.Length(), path.Length(), 1e-12);

    const auto clockwiseArc = std::make_shared<SCArcSegment2d>(
        SCPoint2d{0.0, 0.0}, 1.0, Geometry::kPi * 0.5, -Geometry::kPi * 0.5);
    const auto closingLine = std::make_shared<SCLineSegment2d>(
        clockwiseArc->EndPoint(), clockwiseArc->StartPoint());
    const SCPolyline2d closed({clockwiseArc, closingLine}, SCPolylineClosure::Closed);
    const SCPolyline2d insertedClockwise = InsertPoint(closed, clockwiseArc->PointAt(0.5), 1e-12);
    ASSERT_TRUE(insertedClockwise.IsValid(1e-12));
    EXPECT_TRUE(insertedClockwise.IsClosed());
    ASSERT_EQ(insertedClockwise.SegmentCount(), 3U);
    EXPECT_EQ(insertedClockwise.SegmentAt(0)->Kind(), Geometry::SCSegmentKind2d::Arc);
    EXPECT_EQ(insertedClockwise.SegmentAt(1)->Kind(), Geometry::SCSegmentKind2d::Arc);
    EXPECT_NEAR(insertedClockwise.Length(), closed.Length(), 1e-12);

    const auto shortSweep = std::make_shared<SCArcSegment2d>(
        SCPoint2d{0.0, 0.0}, 1e6, 0.0, 1e-10);
    const SCPolyline2d shortSweepPath({shortSweep}, SCPolylineClosure::Open);
    const SCPolyline2d insertedShortSweep =
        InsertPoint(shortSweepPath, shortSweep->PointAt(0.5, 1e-8), 1e-8);
    ASSERT_TRUE(insertedShortSweep.IsValid(1e-8));
    ASSERT_EQ(insertedShortSweep.SegmentCount(), 2U);
    EXPECT_EQ(insertedShortSweep.SegmentAt(0)->Kind(), Geometry::SCSegmentKind2d::Arc);
    EXPECT_EQ(insertedShortSweep.SegmentAt(1)->Kind(), Geometry::SCSegmentKind2d::Arc);
    EXPECT_NEAR(insertedShortSweep.Length(1e-8), shortSweepPath.Length(1e-8), 1e-10);

    const SCPolyline2d outer({{0.0, 0.0}, {4.0, 0.0}, {4.0, 4.0}, {0.0, 4.0}},
                             SCPolylineClosure::Closed);
    const SCPolyline2d hole({{1.0, 1.0}, {1.0, 2.0}, {2.0, 2.0}, {2.0, 1.0}},
                            SCPolylineClosure::Closed);
    const SCPolygon2d polygon(outer, {hole});
    const SCPolygon2d insertedPolygon = InsertPoint(polygon, {2.0, 0.0}, 1e-12);
    ASSERT_TRUE(insertedPolygon.IsValid(1e-12));
    EXPECT_EQ(insertedPolygon.OuterRing().PointCount(), 5U);
    EXPECT_EQ(insertedPolygon.HoleCount(), 1U);
    EXPECT_EQ(insertedPolygon.HoleAt(0).PointCount(), 4U);

    const SCPolygon2d insertedHole = InsertPoint(polygon, {1.0, 1.5}, 1e-12);
    ASSERT_TRUE(insertedHole.IsValid(1e-12));
    EXPECT_EQ(insertedHole.OuterRing().PointCount(), 4U);
    ASSERT_EQ(insertedHole.HoleCount(), 1U);
    EXPECT_EQ(insertedHole.HoleAt(0).PointCount(), 5U);
}

TEST(TransformSamplingTest, EditingRejectsInvalidTolerance)
{
    const SCPolyline2d line({{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}}, SCPolylineClosure::Open);
    const SCPolygon2d polygon(SCPolyline2d(
        {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}}, SCPolylineClosure::Closed));
    ASSERT_TRUE(line.IsValid());
    ASSERT_TRUE(polygon.IsValid());

    for (double eps : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN()})
    {
        EXPECT_FALSE(Normalize(line, eps).IsValid());
        EXPECT_FALSE(Normalize(polygon, eps).IsValid());
        EXPECT_FALSE(InsertPoint(line, {1.0, 0.0}, eps).IsValid());
        EXPECT_FALSE(InsertPoint(polygon, {1.0, 0.0}, eps).IsValid());
    }
}

TEST(TransformSamplingTest, MirrorDirectionScalePreservesAxis)
{
    const SCPoint2d point{1.0, 2.0};
    const SCPoint2d origin{0.0, 0.0};
    const SCPoint2d expected{1.0, -2.0};
    EXPECT_TRUE(Mirror(point, origin, SCVector2d{1.0, 0.0}).AlmostEquals(expected, 1e-12));
    EXPECT_TRUE(Mirror(point, origin, SCVector2d{1e-6, 0.0}).AlmostEquals(expected, 1e-12));
    EXPECT_TRUE(Mirror(point, origin, SCVector2d{1e6, 0.0}).AlmostEquals(expected, 1e-12));
    EXPECT_TRUE(Mirror(point, origin, SCVector2d{1e-10, 0.0}).AlmostEquals(point, 1e-12));
}

TEST(TransformSamplingTest, InvalidMirrorDirectionPreservesAllGeometryKinds)
{
    const SCPoint2d origin{-2.0, 1.0};
    const SCPoint2d point{1.0, 2.0};
    const SCArcSegment2d arc({2.0, 3.0}, 1.5, 0.3, 0.7);
    const SCLineSegment2d line(point, arc.StartPoint());
    std::vector<std::shared_ptr<Geometry::ISCSegment2d>> segments;
    segments.push_back(std::make_shared<SCLineSegment2d>(line));
    segments.push_back(std::make_shared<SCArcSegment2d>(arc));
    const SCPolyline2d path(std::move(segments), SCPolylineClosure::Open);
    const SCPolyline2d outer({{-3.0, -3.0}, {3.0, -3.0}, {3.0, 3.0}, {-3.0, 3.0}},
                             SCPolylineClosure::Closed);
    const SCPolyline2d hole({{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}}, SCPolylineClosure::Closed);
    const SCPolygon2d polygon(outer, {hole});
    ASSERT_TRUE(path.IsValid());
    ASSERT_TRUE(polygon.IsValid());
    const auto expectSamePath = [](const SCPolyline2d& actual, const SCPolyline2d& expected)
    {
        ASSERT_EQ(actual.SegmentCount(), expected.SegmentCount());
        EXPECT_EQ(actual.IsClosed(), expected.IsClosed());
        for (std::size_t i = 0; i < expected.SegmentCount(); ++i)
        {
            const auto actualSegment = actual.SegmentAt(i);
            const auto expectedSegment = expected.SegmentAt(i);
            ASSERT_NE(actualSegment, nullptr);
            ASSERT_NE(expectedSegment, nullptr);
            EXPECT_EQ(actualSegment->Kind(), expectedSegment->Kind());
            for (const double parameter : {0.0, 0.5, 1.0})
            {
                EXPECT_TRUE(actualSegment->PointAt(parameter).AlmostEquals(expectedSegment->PointAt(parameter), 1e-12));
            }
        }
    };
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double maximum = std::numeric_limits<double>::max();
    for (const SCVector2d direction : {SCVector2d{}, SCVector2d{1e-10, 0.0},
             SCVector2d{Geometry::kDefaultEpsilon, 0.0}, SCVector2d{infinity, 0.0},
             SCVector2d{nan, 0.0}, SCVector2d{maximum, maximum}})
    {
        SCOPED_TRACE(direction.DebugString());
        EXPECT_EQ(Mirror(point, origin, direction), point);
        const auto mirroredLine = Mirror(line, origin, direction);
        EXPECT_EQ(mirroredLine.startPoint, line.startPoint);
        EXPECT_EQ(mirroredLine.endPoint, line.endPoint);
        const auto mirroredArc = Mirror(arc, origin, direction);
        EXPECT_EQ(mirroredArc.center, arc.center);
        EXPECT_DOUBLE_EQ(mirroredArc.radius, arc.radius);
        EXPECT_DOUBLE_EQ(mirroredArc.startAngle, arc.startAngle);
        EXPECT_DOUBLE_EQ(mirroredArc.sweepAngle, arc.sweepAngle);
        expectSamePath(Mirror(path, origin, direction), path);
        const auto mirroredPolygon = Mirror(polygon, origin, direction);
        ASSERT_EQ(mirroredPolygon.HoleCount(), polygon.HoleCount());
        expectSamePath(mirroredPolygon.OuterRing(), outer);
        expectSamePath(mirroredPolygon.HoleAt(0), hole);
    }
}

TEST(TransformSamplingTest, UsableMirrorDirectionsReflectArcSamples)
{
    const SCArcSegment2d arc({2.0, 3.0}, 1.5, 0.3, 0.7);
    for (const SCVector2d direction : {SCVector2d{1.0, 0.0}, SCVector2d{-1.0, 0.0},
             SCVector2d{1e-6, 0.0}, SCVector2d{1e6, 0.0}, SCVector2d{0.0, 1.0}})
    {
        SCOPED_TRACE(direction.DebugString());
        const auto mirrored = Mirror(arc, {}, direction);
        ASSERT_TRUE(mirrored.IsValid());
        EXPECT_DOUBLE_EQ(mirrored.radius, arc.radius);
        EXPECT_DOUBLE_EQ(mirrored.sweepAngle, -arc.sweepAngle);
        for (const double parameter : {0.0, 0.5, 1.0})
        {
            const SCPoint2d original = arc.PointAt(parameter);
            const SCPoint2d expected = direction.x == 0.0
                ? SCPoint2d{-original.x, original.y} : SCPoint2d{original.x, -original.y};
            EXPECT_TRUE(mirrored.PointAt(parameter).AlmostEquals(expected, 1e-12));
        }
    }
}

namespace
{
    template <typename Segment>
    class RejectSampleEvaluation : public Segment
    {
    public:
        using Segment::Segment;
        [[nodiscard]] SCPoint2d PointAt(double) const override
        {
            throw std::runtime_error("sampling reached point evaluation");
        }
    };
}

TEST(TransformSamplingTest, SamplingRejectsUnrepresentablePointCountBeforeEvaluation)
{
    const RejectSampleEvaluation<SCLineSegment2d> line(SCPoint2d{}, SCPoint2d{4.0, 0.0});
    const RejectSampleEvaluation<SCArcSegment2d> arc(SCPoint2d{}, 2.0, 0.0, Geometry::kPi * 0.5);
    const Geometry::ISCSegment2d& generic = line;
    for (const std::size_t count : {std::numeric_limits<std::size_t>::max(),
                                   std::vector<SCPoint2d>{}.max_size()})
    {
        SCOPED_TRACE(count);
        EXPECT_THROW(static_cast<void>(Geometry::Sample(line, count)), std::length_error);
        EXPECT_THROW(static_cast<void>(Geometry::Sample(arc, count)), std::length_error);
        EXPECT_THROW(static_cast<void>(Geometry::Sample(generic, count)), std::length_error);
    }
    EXPECT_TRUE(Geometry::Sample(generic, 0).empty());
}

TEST(TransformSamplingTest, MaxAngleSamplingChecksNonFiniteAndOverflowingCounts)
{
    const RejectSampleEvaluation<SCArcSegment2d> arc(SCPoint2d{}, 2.0, 0.0, Geometry::kPi * 0.5);
    for (const double angle : {0.0, -1.0, std::numeric_limits<double>::infinity(),
             -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
    {
        SCOPED_TRACE(angle);
        const auto points = SampleByMaxAngle(arc, angle);
        ASSERT_EQ(points.size(), 2U);
        EXPECT_EQ(points.front(), arc.StartPoint());
        EXPECT_EQ(points.back(), arc.EndPoint());
    }
    for (const double angle : {1e-300, std::numeric_limits<double>::denorm_min()})
    {
        SCOPED_TRACE(angle);
        EXPECT_THROW(static_cast<void>(SampleByMaxAngle(arc, angle)), std::length_error);
    }
}

TEST(TransformSamplingTest, SamplingPreservesRegularCountsAndArcEndpoints)
{
    const SCLineSegment2d line({}, {4.0, 0.0});
    const SCArcSegment2d arc({}, 2.0, 0.0, Geometry::kPi * 0.5);
    const auto linePoints = Geometry::Sample(line, 4);
    const auto arcPoints = SampleByMaxAngle(arc, Geometry::kPi / 8.0);
    ASSERT_EQ(linePoints.size(), 5U);
    ASSERT_EQ(arcPoints.size(), 5U);
    EXPECT_EQ(linePoints.front(), line.startPoint);
    EXPECT_EQ(linePoints[2], SCPoint2d(2.0, 0.0));
    EXPECT_EQ(linePoints.back(), line.endPoint);
    EXPECT_EQ(arcPoints.front(), arc.StartPoint());
    EXPECT_TRUE(arcPoints[2].AlmostEquals({std::sqrt(2.0), std::sqrt(2.0)}, 1e-12));
    EXPECT_EQ(arcPoints.back(), arc.EndPoint());
}
