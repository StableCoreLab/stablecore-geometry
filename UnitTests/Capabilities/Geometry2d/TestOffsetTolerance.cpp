#include <gtest/gtest.h>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "Core/Offset.h"
#include "Core/Relation.h"
#include "Core/ShapeOps.h"
#include "Support/Epsilon.h"

using Geometry::SCArcSegment2d;
using Geometry::SCLineSegment2d;
using Geometry::SCMultiPolygon2d;
using Geometry::SCPoint2d;
using Geometry::SCPolygon2d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;

TEST(OffsetTest, ExplicitTolerancePreservesTinyPolygonOffsets)
{
    constexpr double width = 1e-8;
    constexpr double eps = 1e-15;
    const SCPolygon2d source(SCPolyline2d(
        {{0.0, 0.0}, {width, 0.0}, {width, width}, {0.0, width}}, SCPolylineClosure::Closed));
    ASSERT_TRUE(source.IsValid(eps));

    const Geometry::SCOffsetOptions2d options;
    const SCPolygon2d outward = Offset(source, width / 4.0, options, eps);
    const SCPolygon2d inward = Offset(source, -width / 4.0, options, eps);
    ASSERT_TRUE(outward.IsValid(eps));
    ASSERT_TRUE(inward.IsValid(eps));
    EXPECT_NEAR(outward.Area(eps), 2.25 * width * width, width * width * 1e-5);
    EXPECT_NEAR(inward.Area(eps), 0.25 * width * width, width * width * 1e-5);

    const SCMultiPolygon2d multiOutward = OffsetToMultiPolygon(source, width / 4.0, options, eps);
    ASSERT_EQ(multiOutward.Size(), 1U);
    EXPECT_TRUE(multiOutward.IsValid(eps));
    EXPECT_NEAR(multiOutward[0].Area(eps), outward.Area(eps), width * width * 1e-5);

    const SCMultiPolygon2d collectionOutward = Offset(SCMultiPolygon2d{source}, width / 4.0, options, eps);
    ASSERT_EQ(collectionOutward.Size(), 1U);
    EXPECT_TRUE(collectionOutward.IsValid(eps));
    EXPECT_NEAR(collectionOutward[0].Area(eps), outward.Area(eps), width * width * 1e-5);
}

TEST(OffsetTest, RejectsInvalidExplicitTolerance)
{
    const SCPolygon2d source(SCPolyline2d(
        {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}}, SCPolylineClosure::Closed));
    const SCLineSegment2d line({0.0, 0.0}, {1.0, 0.0});
    const SCArcSegment2d arc({0.0, 0.0}, 1.0, 0.0, Geometry::kPi / 2.0);
    for (double epsilon : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()})
    {
        const Geometry::SCOffsetOptions2d options;
        EXPECT_FALSE(Offset(source, 0.1, options, epsilon).IsValid());
        EXPECT_TRUE(OffsetToMultiPolygon(source, 0.1, options, epsilon).IsEmpty());
        EXPECT_TRUE(Offset(SCMultiPolygon2d{source}, 0.1, options, epsilon).IsEmpty());
        EXPECT_FALSE(Offset(source.OuterRing(), 0.1, options, epsilon).IsValid());
        EXPECT_FALSE(Offset(line, 0.1, epsilon).IsValid());
        EXPECT_FALSE(Offset(arc, 0.1, epsilon).IsValid());
    }
}

TEST(OffsetTest, ExplicitToleranceDoesNotRepairInvalidPolygonWithFixedRetry)
{
    const SCPolygon2d crossed(SCPolyline2d(
        {{0.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}, {2.0, 0.0}}, SCPolylineClosure::Closed));
    ASSERT_FALSE(crossed.IsValid(1e-12));
    const Geometry::SCOffsetOptions2d options;
    EXPECT_FALSE(Offset(crossed, 0.1, options, 1e-12).IsValid(1e-12));
    EXPECT_TRUE(Offset(SCMultiPolygon2d{crossed}, 0.1, options, 1e-12).IsEmpty());
}

TEST(OffsetTest, ExplicitToleranceKeepsSmallArcAfterInwardOffset)
{
    constexpr double radius = 1e-8;
    constexpr double remainingRadius = 5e-10;
    const auto sourceArc = std::make_shared<SCArcSegment2d>(
        SCPoint2d{0.0, 0.0}, radius, 0.0, Geometry::kPi / 2.0);
    const SCPolyline2d source(std::vector<std::shared_ptr<Geometry::ISCSegment2d>>{sourceArc},
                              SCPolylineClosure::Open);
    ASSERT_TRUE(source.IsValid());

    const SCPolyline2d offset = Offset(source, radius - remainingRadius,
                                       Geometry::SCOffsetOptions2d{}, 1e-15);
    ASSERT_TRUE(offset.IsValid());
    ASSERT_EQ(offset.SegmentCount(), 1U);
    const auto segment = offset.SegmentAt(0);
    ASSERT_NE(segment, nullptr);
    ASSERT_EQ(segment->Kind(), Geometry::SCSegmentKind2d::Arc);
    EXPECT_NEAR(static_cast<const SCArcSegment2d&>(*segment).radius, remainingRadius, 1e-20);
}

TEST(OffsetTest, ExplicitToleranceOffsetsSubDefaultLineNextToArc)
{
    constexpr double lineLength = 1e-10;
    constexpr double radius = 1e-8;
    constexpr double distance = 1e-9;
    constexpr double eps = 1e-15;
    const auto line = std::make_shared<SCLineSegment2d>(SCPoint2d{0.0, 0.0},
                                                        SCPoint2d{lineLength, 0.0});
    const auto arc = std::make_shared<SCArcSegment2d>(SCPoint2d{lineLength, radius},
                                                      radius, -Geometry::kPi / 2.0, Geometry::kPi / 2.0);
    const SCPolyline2d source(std::vector<std::shared_ptr<Geometry::ISCSegment2d>>{line, arc},
                              SCPolylineClosure::Open);
    ASSERT_TRUE(source.IsValid(eps));

    const SCPolyline2d shifted = Offset(source, distance, Geometry::SCOffsetOptions2d{}, eps);
    ASSERT_TRUE(shifted.IsValid(eps));
    ASSERT_EQ(shifted.SegmentCount(), 2U);
    EXPECT_NEAR(shifted.StartPoint().y, distance, eps);
    EXPECT_NEAR(shifted.SegmentAt(0)->EndPoint().y, distance, eps);
}

TEST(OffsetTest, ExplicitToleranceOffsetsSubDefaultStandaloneLine)
{
    constexpr double length = 1e-10;
    constexpr double distance = 2e-10;
    constexpr double eps = 1e-15;
    const SCLineSegment2d source({0.0, 0.0}, {length, 0.0});
    ASSERT_FALSE(source.IsValid());
    ASSERT_TRUE(source.IsValid(eps));

    const SCLineSegment2d shifted = Offset(source, distance, eps);
    ASSERT_TRUE(shifted.IsValid(eps));
    EXPECT_NEAR(shifted.startPoint.y, distance, eps);
    EXPECT_NEAR(shifted.endPoint.y, distance, eps);
    EXPECT_NEAR(shifted.Length(), length, eps);
    EXPECT_TRUE(Offset(source, distance).startPoint.AlmostEquals(source.startPoint, eps));
    EXPECT_FALSE(Offset(source, std::numeric_limits<double>::quiet_NaN(), eps).IsValid(eps));
}

TEST(OffsetTest, ExplicitToleranceKeepsSubDefaultStandaloneArcRadius)
{
    constexpr double radius = 1e-8;
    constexpr double remainingRadius = 5e-10;
    constexpr double eps = 1e-15;
    const SCArcSegment2d source({0.0, 0.0}, radius, 0.0, Geometry::kPi / 2.0);
    ASSERT_TRUE(source.IsValid());

    const SCArcSegment2d shifted = Offset(source, radius - remainingRadius, eps);
    ASSERT_TRUE(shifted.IsValid());
    EXPECT_NEAR(shifted.radius, remainingRadius, eps);
    EXPECT_FALSE(Offset(source, radius - remainingRadius).IsValid());
    EXPECT_FALSE(Offset(source, std::numeric_limits<double>::quiet_NaN(), eps).IsValid());
    const SCArcSegment2d overflow({1e308, 0.0}, 1e308, 0.0, Geometry::kPi / 2.0);
    ASSERT_FALSE(overflow.StartPoint().IsValid());
    EXPECT_FALSE(Offset(overflow, 0.0, eps).IsValid());
}

TEST(OffsetTest, ExplicitToleranceOffsetsRepresentableShortSweepArc)
{
    constexpr double epsilon = 1e-9;
    const SCArcSegment2d source({0.0, 0.0}, 1e6, 0.0, 1e-10);
    ASSERT_TRUE(source.IsValid(epsilon));
    const SCArcSegment2d shifted = Offset(source, 100.0, epsilon);
    EXPECT_TRUE(shifted.IsValid(epsilon));
    EXPECT_FALSE(shifted.IsValid());
    EXPECT_NEAR(shifted.radius, 1e6 - 100.0, 1e-9);
}

TEST(OffsetTest, ExplicitToleranceOffsetsShortSweepArcInsidePolyline)
{
    constexpr double epsilon = 1e-9;
    const SCArcSegment2d arc({0.0, 0.0}, 1e6, 0.0, 1e-10);
    const SCPolyline2d source(
        std::vector<std::shared_ptr<Geometry::ISCSegment2d>>{std::make_shared<SCArcSegment2d>(arc)},
        SCPolylineClosure::Open);
    ASSERT_FALSE(source.IsValid());
    ASSERT_TRUE(source.IsValid(epsilon));

    const SCPolyline2d shifted = Offset(source, 100.0, Geometry::SCOffsetOptions2d{}, epsilon);
    ASSERT_TRUE(shifted.IsValid(epsilon));
    ASSERT_EQ(shifted.SegmentCount(), 1U);
    EXPECT_EQ(shifted.SegmentAt(0)->Kind(), Geometry::SCSegmentKind2d::Arc);
    EXPECT_NEAR(shifted.Length(epsilon), (1e6 - 100.0) * 1e-10, 1e-9);
}
