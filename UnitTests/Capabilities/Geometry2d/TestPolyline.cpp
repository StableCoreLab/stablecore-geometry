#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <vector>

#include "Core/ShapeOps.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Geometry2d/SCPolyline2d.h"
#include "Support/GeometryTestSupport.h"

using Geometry::SCArcDirection;
using Geometry::SCArcSegment2d;
using Geometry::SCBox2d;
using Geometry::IsEqual;
using Geometry::SCLineSegment2d;
using Geometry::SCPoint2d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;
using Geometry::Reverse;

namespace
{
    constexpr double kPi = 3.141592653589793238462643383279502884;
}

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
    GEOMETRY_TEST_ASSERT_POINT_NEAR(openPath.VertexAt(0), SCPoint2d(0.0, 0.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(openPath.VertexAt(1), SCPoint2d(3.0, 0.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(openPath.VertexAt(2), SCPoint2d(3.0, 4.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(openPath.StartPoint(), SCPoint2d(0.0, 0.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(openPath.EndPoint(), SCPoint2d(3.0, 4.0), 1e-12);
    ASSERT_LT(std::abs(openPath.Length() - 7.0), 1e-12);
    ASSERT_LT(std::abs(openPath.LengthAt(0.5) - 3.5), 1e-12);
    ASSERT_LT(std::abs(openPath.ParameterAtLength(3.5) - 0.5), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(openPath.PointAt(0.5), SCPoint2d(3.0, 0.5), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(openPath.PointAtLength(3.5), SCPoint2d(3.0, 0.5), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(openPath.PointAtLength(-2.0, true), SCPoint2d(0.0, 0.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(openPath.PointAtLength(9.0, true), SCPoint2d(3.0, 4.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(openPath.PointAtLength(-2.0, false), SCPoint2d(-2.0, 0.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(openPath.PointAtLength(9.0, false), SCPoint2d(3.0, 6.0), 1e-12);

    const SCBox2d openBox = openPath.Bounds();
    ASSERT_TRUE(openBox.IsValid());
    GEOMETRY_TEST_ASSERT_POINT_NEAR(openBox.MinPoint(), SCPoint2d(0.0, 0.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(openBox.MaxPoint(), SCPoint2d(3.0, 4.0), 1e-12);

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
    GEOMETRY_TEST_ASSERT_POINT_NEAR(closedPath.StartPoint(), SCPoint2d(0.0, 0.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(closedPath.EndPoint(), SCPoint2d(0.0, 0.0), 1e-12);

    SCPolyline2d closedByRepeatedPoint({SCPoint2d(0.0, 0.0), SCPoint2d(1.0, 0.0), SCPoint2d(0.0, 1.0), SCPoint2d(0.0, 0.0)},
                                     SCPolylineClosure::Closed);
    ASSERT_TRUE(closedByRepeatedPoint.IsValid());
    ASSERT_EQ(closedByRepeatedPoint.SegmentCount(), 3);
    ASSERT_EQ(closedByRepeatedPoint.VertexCount(), 3);

    auto line = std::make_shared<SCLineSegment2d>(SCPoint2d(0.0, 0.0), SCPoint2d(1.0, 0.0));
    auto arc = std::make_shared<SCArcSegment2d>(SCPoint2d(1.0, 1.0), 1.0, -kPi / 2.0, 0.0, SCArcDirection::CounterClockwise);
    SCPolyline2d mixedPath({line, arc}, SCPolylineClosure::Open);

    ASSERT_TRUE(mixedPath.IsValid());
    ASSERT_LT(std::abs(mixedPath.Length() - (1.0 + kPi / 2.0)), 1e-12);
    const SCBox2d mixedBox = mixedPath.Bounds();
    ASSERT_TRUE(mixedBox.IsValid());
    GEOMETRY_TEST_ASSERT_POINT_NEAR(mixedBox.MinPoint(), SCPoint2d(0.0, 0.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(mixedBox.MaxPoint(), SCPoint2d(2.0, 1.0), 1e-12);

    const SCPolyline2d reversedMixedPath = Reverse(mixedPath);
    ASSERT_TRUE(reversedMixedPath.IsValid());
    ASSERT_EQ(reversedMixedPath.SegmentCount(), 2);
    ASSERT_EQ(reversedMixedPath.SegmentAt(0)->Kind(), Geometry::SCSegmentKind2::Arc);
    ASSERT_EQ(reversedMixedPath.SegmentAt(1)->Kind(), Geometry::SCSegmentKind2::Line);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(reversedMixedPath.StartPoint(), SCPoint2d(2.0, 1.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(reversedMixedPath.EndPoint(), SCPoint2d(0.0, 0.0), 1e-12);
}
