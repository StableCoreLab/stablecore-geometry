#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <memory>

#include "Geometry.h"

using Geometry::SCArcDirection;
using Geometry::SCArcSegment2d;
using Geometry::SCAxisProjection2d;
using Geometry::SCAxisSample2d;
using Geometry::SCBox2d;
using Geometry::SCClosestPoints2d;
using Geometry::SCIntersectionKind2d;
using Geometry::SCLineSegment2d;
using Geometry::SCPoint2d;
using Geometry::SCPolygon2d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;
using Geometry::SCRingOrientation2d;
using Geometry::ISCSegment2d;
using Geometry::SCSegmentIntersection2d;
using Geometry::SCSegmentKind2;
using Geometry::SCSegmentProjection2d;
using Geometry::SCSegmentSplit2d;
using Geometry::SCSegmentTrim2d;
using Geometry::SCSnapResult2d;
using Geometry::SCVector2d;

TEST(CoreAlgorithmsTest, CoversCurrentCapabilities)
{
    const SCLineSegment2d line(SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0});
    const SCArcSegment2d arc(SCPoint2d{0.0, 0.0}, 2.0, 0.0, Geometry::kPi * 0.5);

    EXPECT_NEAR(Distance(SCPoint2d{1.0, 1.0}, line), 1.0, 1e-12);
    EXPECT_NEAR(Distance(SCPoint2d{1.0, 1.0}, arc), std::abs(std::sqrt(2.0) - 2.0), 1e-12);
    EXPECT_NEAR(line.Length(), 4.0, 1e-12);
    EXPECT_TRUE(arc.Bounds().AlmostEquals(SCBox2d::FromMinMax(SCPoint2d{0.0, 0.0}, SCPoint2d{2.0, 2.0}), 1e-12));

    const SCSegmentProjection2d lineProjection = ProjectPointToLineSegment(SCPoint2d{1.0, 2.0}, line, true);
    EXPECT_TRUE(lineProjection.point.AlmostEquals(SCPoint2d{1.0, 0.0}, 1e-12));
    EXPECT_NEAR(lineProjection.parameter, 0.25, 1e-12);

    const SCSegmentProjection2d arcProjection = ProjectPointToArcSegment(SCPoint2d{1.0, 1.0}, arc, true);
    EXPECT_TRUE(arcProjection.point.AlmostEquals(SCPoint2d{std::sqrt(2.0), std::sqrt(2.0)}, 1e-12));

    EXPECT_TRUE(line.PointAtLength(2.0, true).AlmostEquals(SCPoint2d{2.0, 0.0}, 1e-12));
    EXPECT_TRUE(TangentAt(line, 0.5).AlmostEquals(SCVector2d{1.0, 0.0}, 1e-12));
    EXPECT_TRUE(NormalAt(line, 0.5).AlmostEquals(SCVector2d{0.0, 1.0}, 1e-12));

    const SCLineSegment2d vertical(SCPoint2d{2.0, -1.0}, SCPoint2d{2.0, 1.0});
    const SCSegmentIntersection2d lineLineIntersection = Intersect(line, vertical);
    ASSERT_EQ(lineLineIntersection.kind, SCIntersectionKind2d::Point);
    ASSERT_EQ(lineLineIntersection.pointCount, 1);
    EXPECT_TRUE(lineLineIntersection.points[0].point.AlmostEquals(SCPoint2d{2.0, 0.0}, 1e-12));

    const SCSegmentIntersection2d lineArcIntersection = Intersect(line, arc);
    ASSERT_TRUE(lineArcIntersection.HasIntersection());

    const SCArcSegment2d secondArc(SCPoint2d{2.0, 0.0}, 2.0, Geometry::kPi, -Geometry::kPi * 0.5);
    const SCSegmentIntersection2d arcArcIntersection = Intersect(arc, secondArc);
    ASSERT_TRUE(arcArcIntersection.HasIntersection());

    const SCClosestPoints2d closest = ClosestPoints(line, secondArc);
    ASSERT_TRUE(closest.IsValid());

    const SCPolyline2d ccwRing({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}},
                             SCPolylineClosure::Closed);
    const SCPolyline2d holeRing(
        {SCPoint2d{1.0, 1.0}, SCPoint2d{1.0, 3.0}, SCPoint2d{3.0, 3.0}, SCPoint2d{3.0, 1.0}}, SCPolylineClosure::Closed);
    const SCPolygon2d polygon(ccwRing, {holeRing});
    EXPECT_NEAR(polygon.Area(), 12.0, 1e-12);
    EXPECT_TRUE(Centroid(polygon).AlmostEquals(SCPoint2d{2.0, 2.0}, 1e-12));
    ASSERT_EQ(Orientation(ccwRing), SCRingOrientation2d::CounterClockwise);
    ASSERT_TRUE(IsCounterClockwise(ccwRing));
    ASSERT_TRUE(IsClockwise(holeRing));

    const SCLineSegment2d reversedLine = Reverse(line);
    EXPECT_TRUE(reversedLine.startPoint.AlmostEquals(line.endPoint, 1e-12));
    const SCArcSegment2d reversedArc = Reverse(arc);
    EXPECT_TRUE(reversedArc.StartPoint().AlmostEquals(arc.EndPoint(), 1e-12));
    const SCPolyline2d closedOpen =
        Close(SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{1.0, 0.0}, SCPoint2d{1.0, 1.0}}, SCPolylineClosure::Open));
    ASSERT_TRUE(closedOpen.IsClosed());
    const SCPolyline2d openArcPath(
        {std::make_shared<SCLineSegment2d>(SCPoint2d{0.0, 0.0}, SCPoint2d{1.0, 0.0}),
         std::make_shared<SCArcSegment2d>(
             SCPoint2d{1.0, 1.0}, 1.0, -Geometry::kPi * 0.5, 0.0, SCArcDirection::CounterClockwise)},
        SCPolylineClosure::Open);
    const SCPolyline2d closedArcPath = Close(openArcPath);
    ASSERT_TRUE(closedArcPath.IsClosed());
    ASSERT_EQ(closedArcPath.SegmentAt(0)->Kind(), SCSegmentKind2::Line);
    ASSERT_EQ(closedArcPath.SegmentAt(1)->Kind(), SCSegmentKind2::Arc);
    ASSERT_EQ(closedArcPath.SegmentAt(2)->Kind(), SCSegmentKind2::Line);

    const SCAxisSample2d sample = SampleAxis(line, 0.5);
    EXPECT_TRUE(sample.point.AlmostEquals(SCPoint2d{2.0, 0.0}, 1e-12));
    EXPECT_TRUE(sample.tangent.AlmostEquals(SCVector2d{1.0, 0.0}, 1e-12));
    const SCAxisProjection2d axisProjection = ProjectPointToAxis(SCPoint2d{1.0, 1.0}, line);
    EXPECT_TRUE(axisProjection.projection.point.AlmostEquals(SCPoint2d{1.0, 0.0}, 1e-12));

    const SCSegmentSplit2d split = SplitSegment(line, 0.5);
    ASSERT_TRUE(split.success);
    ASSERT_EQ(split.first->Kind(), SCSegmentKind2::Line);
    ASSERT_EQ(split.second->Kind(), SCSegmentKind2::Line);

    const SCSegmentTrim2d trim = TrimSegment(arc, 0.25, 0.75);
    ASSERT_TRUE(trim.success);
    ASSERT_EQ(trim.segment->Kind(), SCSegmentKind2::Arc);

    const std::unique_ptr<ISCSegment2d> lineClone = line.Clone();
    const std::unique_ptr<ISCSegment2d> arcClone = arc.Clone();
    const std::array<const ISCSegment2d*, 2> snapSegments{lineClone.get(), arcClone.get()};
    const SCSnapResult2d snap = SnapPointToSegments(SCPoint2d{1.0, 1.0}, snapSegments, 2.0);
    ASSERT_TRUE(snap.snapped);
}
