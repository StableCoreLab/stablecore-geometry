#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <vector>

#include "Core/ShapeOps.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Geometry2d/SCPolyline2d.h"
#include "Support/Epsilon.h"

using Geometry::SCArcDirection;
using Geometry::SCArcSegment2d;
using Geometry::SCBox2d;
using Geometry::SCLineSegment2d;
using Geometry::SCPoint2d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;
using Geometry::Reverse;

TEST(PolylineTest, CoversCurrentCapabilities)
{
    SCPolyline2d emptyPath;
    ASSERT_FALSE(emptyPath.IsValid());
    ASSERT_EQ(emptyPath.SegmentCount(), 0);
    ASSERT_EQ(emptyPath.VertexCount(), 0);

    auto first = std::make_shared<SCLineSegment2d>(SCPoint2d(0.0, 0.0), SCPoint2d(3.0, 0.0));
    auto second = std::make_shared<SCLineSegment2d>(SCPoint2d(3.0, 0.0), SCPoint2d(3.0, 4.0));
    SCPolyline2d openPath({first, second}, SCPolylineClosure::Open);

    ASSERT_TRUE(openPath.IsValid());
    ASSERT_FALSE(openPath.IsClosed());
    ASSERT_EQ(openPath.SegmentCount(), 2);
    ASSERT_EQ(openPath.VertexCount(), 3);
    EXPECT_TRUE(openPath.VertexAt(0).AlmostEquals(SCPoint2d(0.0, 0.0), 1e-12));
    EXPECT_TRUE(openPath.VertexAt(1).AlmostEquals(SCPoint2d(3.0, 0.0), 1e-12));
    EXPECT_TRUE(openPath.VertexAt(2).AlmostEquals(SCPoint2d(3.0, 4.0), 1e-12));
    EXPECT_TRUE(openPath.StartPoint().AlmostEquals(SCPoint2d(0.0, 0.0), 1e-12));
    EXPECT_TRUE(openPath.EndPoint().AlmostEquals(SCPoint2d(3.0, 4.0), 1e-12));
    ASSERT_LT(std::abs(openPath.Length() - 7.0), 1e-12);
    ASSERT_LT(std::abs(openPath.LengthAt(0.5) - 3.5), 1e-12);
    ASSERT_LT(std::abs(openPath.ParameterAtLength(3.5) - 0.5), 1e-12);
    EXPECT_TRUE(openPath.PointAt(0.5).AlmostEquals(SCPoint2d(3.0, 0.5), 1e-12));
    EXPECT_TRUE(openPath.PointAtLength(3.5).AlmostEquals(SCPoint2d(3.0, 0.5), 1e-12));
    EXPECT_TRUE(openPath.PointAtLength(-2.0, true).AlmostEquals(SCPoint2d(0.0, 0.0), 1e-12));
    EXPECT_TRUE(openPath.PointAtLength(9.0, true).AlmostEquals(SCPoint2d(3.0, 4.0), 1e-12));
    EXPECT_TRUE(openPath.PointAtLength(-2.0, false).AlmostEquals(SCPoint2d(-2.0, 0.0), 1e-12));
    EXPECT_TRUE(openPath.PointAtLength(9.0, false).AlmostEquals(SCPoint2d(3.0, 6.0), 1e-12));

    const SCBox2d openBox = openPath.Bounds();
    ASSERT_TRUE(openBox.IsValid());
    EXPECT_TRUE(openBox.MinPoint().AlmostEquals(SCPoint2d(0.0, 0.0), 1e-12));
    EXPECT_TRUE(openBox.MaxPoint().AlmostEquals(SCPoint2d(3.0, 4.0), 1e-12));

    auto broken = std::make_shared<SCLineSegment2d>(SCPoint2d(10.0, 10.0), SCPoint2d(11.0, 10.0));
    SCPolyline2d invalidOpen({first, broken}, SCPolylineClosure::Open);
    ASSERT_FALSE(invalidOpen.IsValid());

    auto closeA = std::make_shared<SCLineSegment2d>(SCPoint2d(0.0, 0.0), SCPoint2d(1.0, 0.0));
    auto closeB = std::make_shared<SCLineSegment2d>(SCPoint2d(1.0, 0.0), SCPoint2d(0.0, 1.0));
    auto closeC = std::make_shared<SCLineSegment2d>(SCPoint2d(0.0, 1.0), SCPoint2d(0.0, 0.0));
    SCPolyline2d closedPath({closeA, closeB, closeC}, SCPolylineClosure::Closed);

    ASSERT_TRUE(closedPath.IsValid());
    ASSERT_TRUE(closedPath.IsClosed());
    ASSERT_EQ(closedPath.VertexCount(), 3);
    EXPECT_TRUE(closedPath.StartPoint().AlmostEquals(SCPoint2d(0.0, 0.0), 1e-12));
    EXPECT_TRUE(closedPath.EndPoint().AlmostEquals(SCPoint2d(0.0, 0.0), 1e-12));

    SCPolyline2d closedByRepeatedPoint({SCPoint2d(0.0, 0.0), SCPoint2d(1.0, 0.0), SCPoint2d(0.0, 1.0), SCPoint2d(0.0, 0.0)},
                                     SCPolylineClosure::Closed);
    ASSERT_TRUE(closedByRepeatedPoint.IsValid());
    ASSERT_EQ(closedByRepeatedPoint.SegmentCount(), 3);
    ASSERT_EQ(closedByRepeatedPoint.VertexCount(), 3);

    auto line = std::make_shared<SCLineSegment2d>(SCPoint2d(0.0, 0.0), SCPoint2d(1.0, 0.0));
    auto arc = std::make_shared<SCArcSegment2d>(SCPoint2d(1.0, 1.0), 1.0, -Geometry::kPi / 2.0, 0.0, SCArcDirection::CounterClockwise);
    SCPolyline2d mixedPath({line, arc}, SCPolylineClosure::Open);

    ASSERT_TRUE(mixedPath.IsValid());
    ASSERT_LT(std::abs(mixedPath.Length() - (1.0 + Geometry::kPi / 2.0)), 1e-12);
    const SCBox2d mixedBox = mixedPath.Bounds();
    ASSERT_TRUE(mixedBox.IsValid());
    EXPECT_TRUE(mixedBox.MinPoint().AlmostEquals(SCPoint2d(0.0, 0.0), 1e-12));
    EXPECT_TRUE(mixedBox.MaxPoint().AlmostEquals(SCPoint2d(2.0, 1.0), 1e-12));

    const SCPolyline2d reversedMixedPath = Reverse(mixedPath);
    ASSERT_TRUE(reversedMixedPath.IsValid());
    ASSERT_EQ(reversedMixedPath.SegmentCount(), 2);
    ASSERT_EQ(reversedMixedPath.SegmentAt(0)->Kind(), Geometry::SCSegmentKind2::Arc);
    ASSERT_EQ(reversedMixedPath.SegmentAt(1)->Kind(), Geometry::SCSegmentKind2::Line);
    EXPECT_TRUE(reversedMixedPath.StartPoint().AlmostEquals(SCPoint2d(2.0, 1.0), 1e-12));
    EXPECT_TRUE(reversedMixedPath.EndPoint().AlmostEquals(SCPoint2d(0.0, 0.0), 1e-12));
}
