#include <gtest/gtest.h>

#include "Geometry.h"

using namespace Geometry;

TEST(GeometryTest, CoversTwoDimensionalPrimitives)
{
    const SCPoint2d a = SCPoint2d::FromXY(0.0, 0.0);
    const SCPoint2d b = SCPoint2d::FromXY(3.0, 4.0);
    const SCVector2d offset = b - a;

    EXPECT_NEAR(Distance(a, b), 5.0, 1e-12);
    ASSERT_EQ(offset, SCVector2d(SCVector2d{3.0, 4.0}));
    EXPECT_NEAR(offset.LengthSquared(), 25.0, 1e-12);
    EXPECT_NEAR(offset.Length(), 5.0, 1e-12);
    ASSERT_EQ(a + offset, b);
    ASSERT_EQ(b - offset, a);
    const SCVector2d zeroVector{0.0, 0.0};
    EXPECT_TRUE(a.AlmostEquals(zeroVector));
    EXPECT_TRUE(zeroVector.AlmostEquals(a));
    const SCPoint3d coordinates{1.0, 2.0, 3.0};
    const SCVector3d components{1.0, 2.0, 3.0};
    EXPECT_TRUE(coordinates.AlmostEquals(components));
    EXPECT_TRUE(components.AlmostEquals(coordinates));

    const SCLineSegment2d line = SCLineSegment2d::FromEndpoints(a, b);
    ASSERT_TRUE(line.IsValid());
    EXPECT_NEAR(line.Length(), 5.0, 1e-12);
    const SCPoint2d lineMidpoint{1.5, 2.0};
    EXPECT_TRUE(line.PointAt(0.5).AlmostEquals(lineMidpoint, 1e-12));
    EXPECT_TRUE(line.Bounds().AlmostEquals(SCBox2d::FromMinMax(SCPoint2d{0.0, 0.0}, SCPoint2d{3.0, 4.0}), 1e-12));
    ASSERT_EQ(line.DebugString().find("SCLineSegment2d{start="), 0);
    ASSERT_EQ(line.Kind(), SCSegmentKind2::Line);

    const SCArcSegment2d arc =
        SCArcSegment2d::FromCenterRadiusStartSweep(SCPoint2d{0.0, 0.0}, 1.0, 0.0, Geometry::kPi * 0.5);
    ASSERT_TRUE(arc.IsValid());
    ASSERT_EQ(arc.Direction(), SCArcDirection::CounterClockwise);
    EXPECT_NEAR(arc.Length(), Geometry::kPi * 0.5, 1e-12);
    const SCPoint2d arcStart{1.0, 0.0};
    const SCPoint2d arcEnd{0.0, 1.0};
    EXPECT_TRUE(arc.StartPoint().AlmostEquals(arcStart, 1e-12));
    EXPECT_TRUE(arc.EndPoint().AlmostEquals(arcEnd, 1e-12));
    EXPECT_TRUE(arc.Bounds().AlmostEquals(SCBox2d::FromMinMax(SCPoint2d{0.0, 0.0}, SCPoint2d{1.0, 1.0}), 1e-12));
    ASSERT_EQ(arc.DebugString().find("SCArcSegment2d{center="), 0);
    ASSERT_EQ(arc.Kind(), SCSegmentKind2::Arc);

    std::unique_ptr<ISCSegment2d> lineSegment = line.Clone();
    std::unique_ptr<ISCSegment2d> arcSegment = arc.Clone();
    ASSERT_EQ(lineSegment->Kind(), SCSegmentKind2::Line);
    ASSERT_EQ(arcSegment->Kind(), SCSegmentKind2::Arc);
    EXPECT_NEAR(lineSegment->Length(), 5.0, 1e-12);
    EXPECT_NEAR(arcSegment->Length(), Geometry::kPi * 0.5, 1e-12);
    EXPECT_TRUE(lineSegment->StartPoint().AlmostEquals(a, 1e-12));
    EXPECT_TRUE(arcSegment->EndPoint().AlmostEquals(arcEnd, 1e-12));

    const auto projection = ProjectPointToSegment(SCPoint2d{3.0, 1.0}, a, b);
    const SCSegmentProjection2d expectedProjection{SCPoint2d{1.56, 2.08}, 0.52, 3.24, true};
    EXPECT_TRUE(projection.AlmostEquals(expectedProjection, 1e-12));

    const auto clampedEndpoint = ProjectPointToSegment(SCPoint2d{5.0, 0.0},
                                                       SCPoint2d{0.0, 0.0}, SCPoint2d{2.0, 0.0}, true);
    EXPECT_TRUE(clampedEndpoint.point.AlmostEquals(SCPoint2d{2.0, 0.0}));
    EXPECT_DOUBLE_EQ(clampedEndpoint.parameter, 1.0);
    EXPECT_TRUE(clampedEndpoint.isOnSegment);

    const auto unboundedEndpoint = ProjectPointToSegment(SCPoint2d{5.0, 0.0},
                                                         SCPoint2d{0.0, 0.0}, SCPoint2d{2.0, 0.0}, false);
    EXPECT_DOUBLE_EQ(unboundedEndpoint.parameter, 2.5);
    EXPECT_FALSE(unboundedEndpoint.isOnSegment);

    const auto shortSegmentProjection = ProjectPointToSegment(
        SCPoint2d{5e-6, 1e-6}, SCPoint2d{0.0, 0.0}, SCPoint2d{1e-5, 0.0});
    EXPECT_NEAR(shortSegmentProjection.parameter, 0.5, 1e-12);
    EXPECT_TRUE(shortSegmentProjection.point.AlmostEquals(SCPoint2d{5e-6, 0.0}, 1e-12));

    const SCBox2d boxA = SCBox2d::FromMinMax(SCPoint2d{0.0, 0.0}, SCPoint2d{2.0, 2.0});
    const SCBox2d boxB = SCBox2d::FromMinMax(SCPoint2d{1.0, 1.0}, SCPoint2d{3.0, 3.0});
    const SCBox2d boxC = SCBox2d::FromMinMax(SCPoint2d{3.1, 3.1}, SCPoint2d{4.0, 4.0});
    const SCBox2d invalidBox = SCBox2d::FromMinMax(SCPoint2d{2.0, 2.0}, SCPoint2d{1.0, 1.0});
    const SCBox2d expectedBox = SCBox2d::FromMinMax(SCPoint2d{0.0, 0.0}, SCPoint2d{2.0, 2.0});
    const SCPoint2d expectedCenter{1.0, 1.0};

    ASSERT_TRUE(boxA.IsValid());
    EXPECT_TRUE(boxA.AlmostEquals(expectedBox, 1e-12));
    ASSERT_FALSE(invalidBox.IsValid());
    EXPECT_NEAR(boxA.Width(), 2.0, 1e-12);
    EXPECT_NEAR(boxA.Height(), 2.0, 1e-12);
    EXPECT_TRUE(boxA.Center().AlmostEquals(expectedCenter, 1e-12));
    ASSERT_TRUE(Contains(boxA, SCPoint2d{1.0, 1.0}));
    ASSERT_TRUE(Intersects(boxA, boxB));
    ASSERT_FALSE(Intersects(boxA, boxC));
    ASSERT_FALSE(Contains(invalidBox, SCPoint2d{1.0, 1.0}));

    const SCPolyline2d openPath({SCPoint2d{0.0, 0.0}, SCPoint2d{3.0, 0.0}, SCPoint2d{3.0, 4.0}}, SCPolylineClosure::Open);
    ASSERT_TRUE(openPath.IsValid());
    ASSERT_FALSE(openPath.IsClosed());
    ASSERT_EQ(openPath.PointCount(), 3);
    ASSERT_EQ(openPath.SegmentCount(), 2);
    const SCPoint2d openMidPoint{3.0, 0.0};
    EXPECT_TRUE(openPath.VertexAt(1).AlmostEquals(openMidPoint, 1e-12));
    EXPECT_TRUE(openPath.Bounds().AlmostEquals(SCBox2d::FromMinMax(SCPoint2d{0.0, 0.0}, SCPoint2d{3.0, 4.0}), 1e-12));
    ASSERT_EQ(openPath.DebugString().find("SCPolyline2d{closure=Open"), 0);

    const SCPolyline2d outerRing({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}},
                               SCPolylineClosure::Closed);
    const SCPolyline2d holeRing({SCPoint2d{1.0, 1.0}, SCPoint2d{1.0, 3.0}, SCPoint2d{3.0, 3.0}, SCPoint2d{3.0, 1.0}},
                              SCPolylineClosure::Closed);
    const SCPolygon2d polygon(outerRing, {holeRing});
    ASSERT_TRUE(polygon.IsValid());
    ASSERT_EQ(polygon.HoleCount(), 1);
    ASSERT_EQ(polygon.PointCount(), 8);
    ASSERT_EQ(polygon.SegmentCount(), 8);
    ASSERT_EQ(polygon.OuterRing().PointCount(), outerRing.PointCount());
    ASSERT_EQ(polygon.HoleAt(0).PointCount(), holeRing.PointCount());
    for (std::size_t i = 0; i < outerRing.PointCount(); ++i)
    {
        EXPECT_TRUE(polygon.OuterRing().VertexAt(i).AlmostEquals(outerRing.VertexAt(i), 1e-12));
        EXPECT_TRUE(polygon.HoleAt(0).VertexAt(i).AlmostEquals(holeRing.VertexAt(i), 1e-12));
    }
    EXPECT_TRUE(polygon.Bounds().AlmostEquals(SCBox2d::FromMinMax(SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 4.0}), 1e-12));
    ASSERT_EQ(polygon.DebugString().find("SCPolygon2d{holeCount=1"), 0);

}
