#include <gtest/gtest.h>
#include <cmath>
#include <concepts>
#include <limits>
#include <type_traits>

#include "Core/Intersection.h"
#include "Core/Projection.h"
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

TEST(SegmentTest, ExplicitToleranceLineArcIntersectionPreservesSmallSweepParameter)
{
    constexpr double eps = 1e-12;
    const SCArcSegment2d arc({0.0, 0.0}, 1.0, 0.0, 1e-10);
    const SCLineSegment2d line({0.9, 5e-11}, {1.1, 5e-11});
    ASSERT_FALSE(arc.IsValid());
    ASSERT_TRUE(arc.IsValid(eps));
    const auto result = Geometry::Intersect(line, arc, eps);
    ASSERT_EQ(result.pointCount, 1U);
    EXPECT_NEAR(result.points[0].point.y, 5e-11, eps);
    EXPECT_NEAR(result.points[0].parameterOnSecond, 0.5, 1e-5);
}

TEST(SegmentTest, ExplicitToleranceLineArcIntersectionUsesLinearSeparation)
{
    constexpr double eps = 1e-12;
    const SCArcSegment2d arc({0.0, 0.0}, 1e-8, 0.0, Geometry::kPi);
    const SCLineSegment2d crossing({-2e-8, 0.0}, {2e-8, 0.0});
    const auto twoPoints = Geometry::Intersect(crossing, arc, eps);
    ASSERT_EQ(twoPoints.pointCount, 2U);
    EXPECT_EQ(twoPoints.kind, Geometry::SCIntersectionKind2d::Point);

    const SCLineSegment2d separated({-2e-8, 1.1e-8}, {2e-8, 1.1e-8});
    const auto noPoint = Geometry::Intersect(separated, arc, eps);
    EXPECT_EQ(noPoint.pointCount, 0U);
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

    const SCArcSegment2d lengthOverflow({0.0, 0.0}, 1e308, 0.0, Geometry::kPi);
    ASSERT_TRUE(lengthOverflow.StartPoint().IsValid());
    ASSERT_TRUE(lengthOverflow.EndPoint().IsValid());
    EXPECT_FALSE(lengthOverflow.IsValid());
    EXPECT_EQ(lengthOverflow.Length(), 0.0);

    const SCArcSegment2d lengthUnderflow({0.0, 0.0},
                                         std::numeric_limits<double>::denorm_min(), 0.0, 1e-8);
    EXPECT_FALSE(lengthUnderflow.IsValid());
    EXPECT_EQ(lengthUnderflow.Length(), 0.0);
}

TEST(SegmentTest, ExplicitToleranceSamplesShortLineAndArc)
{
    constexpr double length = 1e-10;
    constexpr double eps = 1e-15;
    const SCLineSegment2d line({0.0, 0.0}, {length, 0.0});
    EXPECT_EQ(line.PointAt(0.5), line.startPoint);
    EXPECT_EQ(line.PointAtLength(length * 0.5), line.startPoint);
    EXPECT_TRUE(line.PointAt(0.5, eps).AlmostEquals({length * 0.5, 0.0}, eps));
    EXPECT_TRUE(line.PointAtLength(length * 0.5, false, eps)
                    .AlmostEquals({length * 0.5, 0.0}, eps));
    EXPECT_FALSE(line.Bounds().IsValid());
    EXPECT_TRUE(line.Bounds(eps).AlmostEquals(
        SCBox2d::FromMinMax({0.0, 0.0}, {length, 0.0}), eps));
    EXPECT_EQ(line.PointAtLength(length * 2.0, true, eps), line.endPoint);
    EXPECT_EQ(line.PointAt(0.5, 0.0), line.startPoint);
    EXPECT_EQ(line.PointAtLength(length * 0.5, false,
                                 std::numeric_limits<double>::infinity()), line.startPoint);

    const SCArcSegment2d arc({0.0, 0.0}, length, 0.0, Geometry::kPi * 0.5);
    const SCPoint2d expected{length * std::cos(Geometry::kPi * 0.25),
                             length * std::sin(Geometry::kPi * 0.25)};
    EXPECT_TRUE(arc.PointAt(0.5, eps).AlmostEquals(expected, eps));
    EXPECT_TRUE(arc.PointAtLength(arc.Length() * 0.5, false, eps).AlmostEquals(expected, eps));
    EXPECT_EQ(arc.PointAtLength(arc.Length() * 2.0, true, eps), arc.EndPoint());
    EXPECT_EQ(arc.PointAt(0.5, 0.0), arc.StartPoint());
}

TEST(SegmentTest, ExplicitToleranceAcceptsRepresentableShortSweepArc)
{
    constexpr double epsilon = 1e-9;
    const SCArcSegment2d arc({0.0, 0.0}, 1e6, 0.0, 1e-10);
    EXPECT_FALSE(arc.IsValid());
    ASSERT_TRUE(arc.IsValid(epsilon));
    EXPECT_NEAR(arc.Length(epsilon), 1e-4, 1e-18);
    EXPECT_NEAR(arc.PointAt(0.5, epsilon).y, 5e-5, 1e-12);
    EXPECT_NEAR(arc.PointAtLength(5e-5, false, epsilon).y, 5e-5, 1e-12);
    const SCBox2d box = arc.Bounds(epsilon);
    ASSERT_TRUE(box.IsValid());
    EXPECT_NEAR(box.MaxPoint().y, 1e-4, 1e-12);
    EXPECT_FALSE(arc.IsValid(2e-4));
    EXPECT_FALSE(arc.Bounds(0.0).IsValid());

    const SCArcSegment2d nearCardinal({0.0, 0.0}, 1e6,
                                      Geometry::kPi / 2.0 + 5e-10, 1e-10);
    const SCBox2d narrowBounds = nearCardinal.Bounds(epsilon);
    ASSERT_TRUE(narrowBounds.IsValid());
    EXPECT_LT(narrowBounds.MaxPoint().x, -4e-4);
}

TEST(SegmentTest, BaseSegmentLengthParameterUsesExplicitArcTolerance)
{
    constexpr double epsilon = 1e-9;
    const SCArcSegment2d arc({0.0, 0.0}, 1e6, 0.0, 1e-10);
    const ISCSegment2d& segment = arc;
    ASSERT_FALSE(arc.IsValid());
    ASSERT_TRUE(arc.IsValid(epsilon));
    EXPECT_NEAR(Geometry::ParameterAtLength(segment, arc.Length(epsilon) * 0.5,
                                            false, epsilon), 0.5, 1e-12);
    EXPECT_DOUBLE_EQ(Geometry::ParameterAtLength(segment, arc.Length(epsilon) * 2.0,
                                                true, epsilon), 1.0);
}

TEST(SegmentTest, LineValidityUsesEuclideanLengthAtToleranceBoundary)
{
    constexpr double epsilon = 1e-9;
    const SCLineSegment2d diagonal({0.0, 0.0}, {0.8e-9, 0.8e-9});
    EXPECT_GT(diagonal.Length(), epsilon);
    EXPECT_TRUE(diagonal.IsValid(epsilon));
    EXPECT_TRUE(diagonal.IsValid());
    EXPECT_TRUE(diagonal.PointAt(0.5, epsilon).AlmostEquals({0.4e-9, 0.4e-9}, 1e-15));

    const SCLineSegment2d boundary({0.0, 0.0}, {epsilon, 0.0});
    EXPECT_FALSE(boundary.IsValid(epsilon));
    const SCLineSegment2d overflowing({-1e308, 0.0}, {1e308, 0.0});
    EXPECT_FALSE(overflowing.IsValid(epsilon));
}

TEST(SegmentTest, LineSegmentSupportsSamplingAndBounds)
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
}

TEST(SegmentTest, CounterClockwiseArcSupportsSamplingAndBounds)
{
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
}

TEST(SegmentTest, ClockwiseArcAndZeroSweepValidity)
{
    const SCArcSegment2d clockwiseArc(SCPoint2d(0.0, 0.0), 1.0, Geometry::kPi / 2.0, 0.0, SCArcDirection::Clockwise);

    ASSERT_TRUE(clockwiseArc.IsValid());
    ASSERT_LT(std::abs(clockwiseArc.Length() - (Geometry::kPi / 2.0)), 1e-12);
    EXPECT_TRUE(clockwiseArc.StartPoint().AlmostEquals(SCPoint2d(0.0, 1.0), 1e-12));
    EXPECT_TRUE(clockwiseArc.EndPoint().AlmostEquals(SCPoint2d(1.0, 0.0), 1e-12));
    EXPECT_TRUE(clockwiseArc.PointAt(0.5).AlmostEquals(SCPoint2d(std::sqrt(2.0) / 2.0, std::sqrt(2.0) / 2.0), 1e-12));

    const SCArcSegment2d invalidArc(SCPoint2d(0.0, 0.0), 1.0, 0.0, 0.0, SCArcDirection::CounterClockwise);

    ASSERT_FALSE(invalidArc.IsValid());
}

TEST(SegmentTest, LineSegmentBoundsAndBaseDispatch)
{
    static_assert(std::is_abstract_v<ISCSegment2d>);
    const SCLineSegment2d line(SCPoint2d(1.0, 2.0), SCPoint2d(4.0, 6.0));
    ASSERT_EQ(line.Kind(), SCSegmentKind2::Line);
    ASSERT_TRUE(line.IsValid());
    ASSERT_LT(std::abs(line.Length() - 5.0), 1e-12);
    EXPECT_TRUE(line.PointAt(0.5).AlmostEquals(SCPoint2d(2.5, 4.0), 1e-12));
    EXPECT_TRUE(line.PointAtLength(2.5).AlmostEquals(SCPoint2d(2.5, 4.0), 1e-12));
    EXPECT_TRUE(line.PointAtLength(-2.5, true).AlmostEquals(SCPoint2d(1.0, 2.0), 1e-12));

    const SCBox2d lineBox = line.Bounds();
    ASSERT_TRUE(lineBox.IsValid());
    EXPECT_TRUE(lineBox.MinPoint().AlmostEquals(SCPoint2d(1.0, 2.0), 1e-12));
    EXPECT_TRUE(lineBox.MaxPoint().AlmostEquals(SCPoint2d(4.0, 6.0), 1e-12));

    const ISCSegment2d& lineAsSegment = line;
    ASSERT_EQ(lineAsSegment.Kind(), SCSegmentKind2::Line);
    EXPECT_TRUE(lineAsSegment.PointAt(0.5).AlmostEquals(SCPoint2d(2.5, 4.0), 1e-12));
}

TEST(SegmentTest, ArcSegmentSupportsSamplingAndBounds)
{
    const SCArcSegment2d arc(SCPoint2d(0.0, 0.0), 1.0, 0.0, Geometry::kPi / 2.0, SCArcDirection::CounterClockwise);
    ASSERT_EQ(arc.Kind(), SCSegmentKind2::Arc);
    ASSERT_TRUE(arc.IsValid());
    ASSERT_LT(std::abs(arc.Length() - (Geometry::kPi / 2.0)), 1e-12);
    EXPECT_TRUE(arc.StartPoint().AlmostEquals(SCPoint2d(1.0, 0.0), 1e-12));
    EXPECT_TRUE(arc.EndPoint().AlmostEquals(SCPoint2d(0.0, 1.0), 1e-12));
    EXPECT_TRUE(arc.PointAt(0.5).AlmostEquals(SCPoint2d(std::sqrt(2.0) / 2.0, std::sqrt(2.0) / 2.0), 1e-12));

    const SCBox2d arcBox = arc.Bounds();
    ASSERT_TRUE(arcBox.IsValid());
    EXPECT_TRUE(arcBox.MinPoint().AlmostEquals(SCPoint2d(0.0, 0.0), 1e-12));
    EXPECT_TRUE(arcBox.MaxPoint().AlmostEquals(SCPoint2d(1.0, 1.0), 1e-12));
}
