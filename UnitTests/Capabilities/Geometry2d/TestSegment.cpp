#include <gtest/gtest.h>
#include <cmath>
#include <concepts>
#include <limits>
#include <type_traits>

#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Support/Epsilon.h"

using Geometry::SCArcDirection;
using Geometry::SCArcSegment2d;
using Geometry::SCBox2d;
using Geometry::SCLineSegment2d;
using Geometry::SCPoint2d;
using Geometry::ISCSegment2d;
using Geometry::SCSegmentKind2;

TEST(SegmentTest, ArcComparisonSeparatesDistanceAndAngleTolerance)
{
    const SCArcSegment2d arc({0.0, 0.0}, 2.0, 0.25, 1.0);
    const SCArcSegment2d rotated({0.0, 0.0}, 2.0, 0.2501, 1.0);
    const SCArcSegment2d moved({2e-6, 0.0}, 2.0, 0.25, 1.0);

    EXPECT_TRUE(arc.AlmostEquals(rotated, 1e-6, 1e-3));
    EXPECT_FALSE(arc.AlmostEquals(rotated, 1e-6, 1e-5));
    EXPECT_FALSE(arc.AlmostEquals(moved, 1e-6, 1e-3));
    EXPECT_FALSE(arc.AlmostEquals(arc, -1.0, 1e-3));
    EXPECT_FALSE(arc.AlmostEquals(arc, 1e-3, std::numeric_limits<double>::infinity()));
    EXPECT_TRUE(arc.AlmostEquals(rotated, 1e-3));
}

TEST(SegmentTest, ArcValidityRejectsOverflowingGeometry)
{
    const SCArcSegment2d endpointOverflow({1e308, 0.0}, 1e308, 0.0,
                                          Geometry::kPi / 2.0);
    const SCArcSegment2d interiorOverflow({1.5e308, 0.0}, 5e307,
                                          -Geometry::kPi / 2.0, Geometry::kPi);
    ASSERT_FALSE(endpointOverflow.StartPoint().IsValid());
    ASSERT_TRUE(interiorOverflow.StartPoint().IsValid());
    ASSERT_TRUE(interiorOverflow.EndPoint().IsValid());
    EXPECT_FALSE(endpointOverflow.IsValid());
    EXPECT_FALSE(interiorOverflow.IsValid());
}

TEST(SegmentTest, CoversCurrentCapabilities)
{
    static_assert(std::is_abstract_v<ISCSegment2d>);
    static_assert(!std::is_abstract_v<SCLineSegment2d>);
    static_assert(!std::is_abstract_v<SCArcSegment2d>);

    const SCPoint2d start(1.0, 2.0);
    const SCPoint2d end(4.0, 6.0);
    const SCLineSegment2d line(start, end);

    ASSERT_EQ(line.Kind(), SCSegmentKind2::Line);
    ASSERT_TRUE(line.IsValid());
    EXPECT_TRUE(line.StartPoint().AlmostEquals(start, 1e-12));
    EXPECT_TRUE(line.EndPoint().AlmostEquals(end, 1e-12));
    ASSERT_LT(std::abs(line.Length() - 5.0), 1e-12);
    EXPECT_TRUE(line.PointAt(0.5).AlmostEquals(SCPoint2d(2.5, 4.0), 1e-12));
    EXPECT_TRUE(line.PointAtLength(2.5).AlmostEquals(SCPoint2d(2.5, 4.0), 1e-12));
    EXPECT_TRUE(line.PointAtLength(-2.5, false).AlmostEquals(SCPoint2d(-0.5, 0.0), 1e-12));
    EXPECT_TRUE(line.PointAtLength(-2.5, true).AlmostEquals(start, 1e-12));

    const ISCSegment2d& lineAsBase = line;
    ASSERT_EQ(lineAsBase.Kind(), SCSegmentKind2::Line);
    ASSERT_LT(std::abs(lineAsBase.Length() - 5.0), 1e-12);
    EXPECT_TRUE(lineAsBase.PointAt(0.5).AlmostEquals(SCPoint2d(2.5, 4.0), 1e-12));

    const SCBox2d lineBox = line.Bounds();
    ASSERT_TRUE(lineBox.IsValid());
    EXPECT_TRUE(lineBox.MinPoint().AlmostEquals(SCPoint2d(1.0, 2.0), 1e-12));
    EXPECT_TRUE(lineBox.MaxPoint().AlmostEquals(SCPoint2d(4.0, 6.0), 1e-12));

    const SCLineSegment2d degenerateLine(start, start);
    ASSERT_FALSE(degenerateLine.IsValid());

    const SCArcSegment2d quarterArc(SCPoint2d(0.0, 0.0), 1.0, 0.0, Geometry::kPi / 2.0, SCArcDirection::CounterClockwise);

    ASSERT_EQ(quarterArc.Kind(), SCSegmentKind2::Arc);
    ASSERT_TRUE(quarterArc.IsValid());
    ASSERT_LT(std::abs(quarterArc.Length() - (Geometry::kPi / 2.0)), 1e-12);
    EXPECT_TRUE(quarterArc.StartPoint().AlmostEquals(SCPoint2d(1.0, 0.0), 1e-12));
    EXPECT_TRUE(quarterArc.EndPoint().AlmostEquals(SCPoint2d(0.0, 1.0), 1e-12));
    EXPECT_TRUE(quarterArc.PointAt(0.5).AlmostEquals(SCPoint2d(std::sqrt(2.0) / 2.0, std::sqrt(2.0) / 2.0), 1e-12));
    EXPECT_TRUE(quarterArc.PointAtLength(quarterArc.Length() * 0.5)
                    .AlmostEquals(SCPoint2d(std::sqrt(2.0) / 2.0, std::sqrt(2.0) / 2.0), 1e-12));

    const SCBox2d arcBox = quarterArc.Bounds();
    ASSERT_TRUE(arcBox.IsValid());
    EXPECT_TRUE(arcBox.MinPoint().AlmostEquals(SCPoint2d(0.0, 0.0), 1e-12));
    EXPECT_TRUE(arcBox.MaxPoint().AlmostEquals(SCPoint2d(1.0, 1.0), 1e-12));

    const ISCSegment2d& arcAsBase = quarterArc;
    ASSERT_EQ(arcAsBase.Kind(), SCSegmentKind2::Arc);
    EXPECT_TRUE(arcAsBase.PointAt(0.0).AlmostEquals(SCPoint2d(1.0, 0.0), 1e-12));
    EXPECT_TRUE(arcAsBase.PointAt(1.0).AlmostEquals(SCPoint2d(0.0, 1.0), 1e-12));

    const SCArcSegment2d clockwiseArc(SCPoint2d(0.0, 0.0), 1.0, Geometry::kPi / 2.0, 0.0, SCArcDirection::Clockwise);

    ASSERT_TRUE(clockwiseArc.IsValid());
    ASSERT_LT(std::abs(clockwiseArc.Length() - (Geometry::kPi / 2.0)), 1e-12);
    EXPECT_TRUE(clockwiseArc.StartPoint().AlmostEquals(SCPoint2d(0.0, 1.0), 1e-12));
    EXPECT_TRUE(clockwiseArc.EndPoint().AlmostEquals(SCPoint2d(1.0, 0.0), 1e-12));
    EXPECT_TRUE(clockwiseArc.PointAt(0.5).AlmostEquals(SCPoint2d(std::sqrt(2.0) / 2.0, std::sqrt(2.0) / 2.0), 1e-12));

    const SCArcSegment2d invalidArc(SCPoint2d(0.0, 0.0), 1.0, 0.0, 0.0, SCArcDirection::CounterClockwise);

    ASSERT_FALSE(invalidArc.IsValid());
}
