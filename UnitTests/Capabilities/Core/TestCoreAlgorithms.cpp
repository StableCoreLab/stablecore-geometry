#include <gtest/gtest.h>
#include <memory>
#include "Core/ShapeOps.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Geometry2d/SCPolygon2d.h"
#include "Geometry2d/SCPolyline2d.h"
#include "Support/Epsilon.h"

using Geometry::SCArcDirection2d;
using Geometry::SCArcSegment2d;
using Geometry::SCLineSegment2d;
using Geometry::SCPoint2d;
using Geometry::SCPolygon2d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;
using Geometry::SCRingOrientation2d;
using Geometry::SCSegmentKind2d;

TEST(CoreAlgorithmsTest, RingOrientationUsesAreaTolerance)
{
    const SCPolyline2d ring({{0.0, 0.0}, {1e-4, 0.0}, {1e-4, 5e-6}, {0.0, 5e-6}},
                            SCPolylineClosure::Closed);
    ASSERT_TRUE(ring.IsValid());
    EXPECT_EQ(Geometry::Orientation(ring), SCRingOrientation2d::CounterClockwise);
}

TEST(CoreAlgorithmsTest, RingOrientationPreservesValidSmallScaleRings)
{
    for (const double side : {1.0, 1e-4, 1e-6, 1e-8})
    {
        SCOPED_TRACE(side);
        const SCPolyline2d ring({{0, 0}, {side, 0}, {side, side}, {0, side}}, SCPolylineClosure::Closed);
        const SCPolyline2d reversed = Geometry::Reverse(ring);
        ASSERT_TRUE(ring.IsValid());
        ASSERT_TRUE(reversed.IsValid());
        EXPECT_EQ(Geometry::Orientation(ring), SCRingOrientation2d::CounterClockwise);
        EXPECT_TRUE(Geometry::IsCounterClockwise(ring));
        EXPECT_FALSE(Geometry::IsClockwise(ring));
        EXPECT_EQ(Geometry::Orientation(reversed), SCRingOrientation2d::Clockwise);
        EXPECT_TRUE(Geometry::IsClockwise(reversed));
        EXPECT_FALSE(Geometry::IsCounterClockwise(reversed));
    }
    const SCPolyline2d nearCollinear({{0, 0}, {1, 0}, {0.5, 1e-10}}, SCPolylineClosure::Closed);
    ASSERT_TRUE(nearCollinear.IsValid());
    EXPECT_EQ(Geometry::Orientation(nearCollinear), SCRingOrientation2d::Unknown);
    EXPECT_FALSE(Geometry::IsClockwise(nearCollinear));
    EXPECT_FALSE(Geometry::IsCounterClockwise(nearCollinear));
}

TEST(CoreAlgorithmsTest, ComputesPolygonAreaAndRingOrientation)
{
    const SCPolyline2d ccwRing({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}},
                             SCPolylineClosure::Closed);
    const SCPolyline2d holeRing(
        {SCPoint2d{1.0, 1.0}, SCPoint2d{1.0, 3.0}, SCPoint2d{3.0, 3.0}, SCPoint2d{3.0, 1.0}}, SCPolylineClosure::Closed);
    const SCPolygon2d polygon(ccwRing, {holeRing});
    EXPECT_NEAR(polygon.Area(), 12.0, 1e-12);
    EXPECT_TRUE(polygon.Centroid().AlmostEquals(SCPoint2d{2.0, 2.0}, 1e-12));
    ASSERT_EQ(Orientation(ccwRing), SCRingOrientation2d::CounterClockwise);
    ASSERT_TRUE(IsCounterClockwise(ccwRing));
    ASSERT_TRUE(IsClockwise(holeRing));
}

TEST(CoreAlgorithmsTest, ReversesSegmentsAndClosesPaths)
{
    const SCLineSegment2d line(SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0});
    const SCArcSegment2d arc(SCPoint2d{0.0, 0.0}, 2.0, 0.0, Geometry::kPi * 0.5);

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
             SCPoint2d{1.0, 1.0}, 1.0, -Geometry::kPi * 0.5, 0.0, SCArcDirection2d::CounterClockwise)},
        SCPolylineClosure::Open);
    const SCPolyline2d closedArcPath = Close(openArcPath);
    ASSERT_TRUE(closedArcPath.IsClosed());
    ASSERT_EQ(closedArcPath.SegmentAt(0)->Kind(), SCSegmentKind2d::Line);
    ASSERT_EQ(closedArcPath.SegmentAt(1)->Kind(), SCSegmentKind2d::Arc);
    ASSERT_EQ(closedArcPath.SegmentAt(2)->Kind(), SCSegmentKind2d::Line);
}
