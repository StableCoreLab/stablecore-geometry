#include <cmath>

#include <gtest/gtest.h>

#include "Geometry.h"

using Geometry::ClassifyParallelSegmentProjection;
using Geometry::SCLineSegment2d;
using Geometry::SCParallelSegmentProjectionRelation2d;
using Geometry::SCParallelSegmentProjectionTolerance2d;
using Geometry::SCPoint2d;

namespace
{
    SCParallelSegmentProjectionTolerance2d MakeTolerance(double angular, double projection)
    {
        SCParallelSegmentProjectionTolerance2d tolerance;
        tolerance.angularEpsilon = angular;
        tolerance.projectionEpsilon = projection;
        return tolerance;
    }
}  // namespace

TEST(RelationClassificationTest, TypedToleranceIsValidByDefault)
{
    SCParallelSegmentProjectionTolerance2d tolerance;
    EXPECT_TRUE(tolerance.IsValid());
    EXPECT_GT(tolerance.angularEpsilon, 0.0);
    EXPECT_LT(tolerance.angularEpsilon, 1.0);
    EXPECT_GT(tolerance.projectionEpsilon, 0.0);
}

TEST(RelationClassificationTest, AngularEpsilonAtOneIsInvalidTolerance)
{
    // angularEpsilon == 1 会使任意方向被误判为平行，必须视为无效容差。
    EXPECT_FALSE(MakeTolerance(1.0, 1e-9).IsValid());
    const auto relation = ClassifyParallelSegmentProjection(SCLineSegment2d{SCPoint2d{0.0, 0.0}, SCPoint2d{1.0, 0.0}},
                                                             SCLineSegment2d{SCPoint2d{0.0, 0.0}, SCPoint2d{1.0, 0.0}},
                                                             MakeTolerance(1.0, 1e-9));
    EXPECT_EQ(relation, SCParallelSegmentProjectionRelation2d::InvalidInput);
}

TEST(RelationClassificationTest, AngularEpsilonGreaterThanOneIsInvalidTolerance)
{
    EXPECT_FALSE(MakeTolerance(1.5, 1e-9).IsValid());
    EXPECT_FALSE(MakeTolerance(2.0, 1e-9).IsValid());
    const auto relation = ClassifyParallelSegmentProjection(SCLineSegment2d{SCPoint2d{0.0, 0.0}, SCPoint2d{1.0, 0.0}},
                                                             SCLineSegment2d{SCPoint2d{0.0, 0.0}, SCPoint2d{1.0, 0.0}},
                                                             MakeTolerance(1.0001, 1e-9));
    EXPECT_EQ(relation, SCParallelSegmentProjectionRelation2d::InvalidInput);
}

TEST(RelationClassificationTest, AngularEpsilonZeroOrNegativeIsInvalidTolerance)
{
    EXPECT_FALSE(MakeTolerance(0.0, 1e-9).IsValid());
    EXPECT_FALSE(MakeTolerance(-0.1, 1e-9).IsValid());
}

TEST(RelationClassificationTest, AngularEpsilonJustBelowOneRemainsValid)
{
    // 略小于 1 的合法临界容差仍可分类。
    const SCParallelSegmentProjectionTolerance2d tolerance = MakeTolerance(1.0 - 1e-12, 1e-9);
    ASSERT_TRUE(tolerance.IsValid());
    const auto relation = ClassifyParallelSegmentProjection(SCLineSegment2d{SCPoint2d{0.0, 0.0}, SCPoint2d{10.0, 0.0}},
                                                             SCLineSegment2d{SCPoint2d{2.0, 0.0}, SCPoint2d{8.0, 0.0}},
                                                             tolerance);
    EXPECT_EQ(relation, SCParallelSegmentProjectionRelation2d::PositiveLengthIntersection);
}

TEST(RelationClassificationTest, ProjectionEpsilonMustBeFiniteAndPositive)
{
    EXPECT_FALSE(MakeTolerance(1e-9, 0.0).IsValid());
    EXPECT_FALSE(MakeTolerance(1e-9, -1e-9).IsValid());
    EXPECT_FALSE(MakeTolerance(1e-9, std::numeric_limits<double>::quiet_NaN()).IsValid());
    EXPECT_FALSE(MakeTolerance(1e-9, std::numeric_limits<double>::infinity()).IsValid());
}

TEST(RelationClassificationTest, NonFiniteAngularEpsilonIsInvalidTolerance)
{
    EXPECT_FALSE(MakeTolerance(std::numeric_limits<double>::quiet_NaN(), 1e-9).IsValid());
    EXPECT_FALSE(MakeTolerance(std::numeric_limits<double>::infinity(), 1e-9).IsValid());
}

TEST(RelationClassificationTest, SameDirectionPositiveLengthOverlap)
{
    const auto relation = ClassifyParallelSegmentProjection(SCLineSegment2d{SCPoint2d{0.0, 0.0}, SCPoint2d{10.0, 0.0}},
                                                             SCLineSegment2d{SCPoint2d{2.0, 0.0}, SCPoint2d{8.0, 0.0}},
                                                             MakeTolerance(1e-9, 1e-9));
    EXPECT_EQ(relation, SCParallelSegmentProjectionRelation2d::PositiveLengthIntersection);
}

TEST(RelationClassificationTest, ReversedDirectionPositiveLengthOverlap)
{
    // 反向线段不改变分类。
    const auto relation = ClassifyParallelSegmentProjection(SCLineSegment2d{SCPoint2d{0.0, 0.0}, SCPoint2d{10.0, 0.0}},
                                                             SCLineSegment2d{SCPoint2d{8.0, 0.0}, SCPoint2d{2.0, 0.0}},
                                                             MakeTolerance(1e-9, 1e-9));
    EXPECT_EQ(relation, SCParallelSegmentProjectionRelation2d::PositiveLengthIntersection);
}

TEST(RelationClassificationTest, DisjointSegmentsReturnNoPositiveLengthIntersection)
{
    const auto relation = ClassifyParallelSegmentProjection(SCLineSegment2d{SCPoint2d{0.0, 0.0}, SCPoint2d{5.0, 0.0}},
                                                             SCLineSegment2d{SCPoint2d{10.0, 0.0}, SCPoint2d{15.0, 0.0}},
                                                             MakeTolerance(1e-9, 1e-9));
    EXPECT_EQ(relation, SCParallelSegmentProjectionRelation2d::NoPositiveLengthIntersection);
}

TEST(RelationClassificationTest, EndpointTouchAtSharedEndpoint)
{
    const auto relation = ClassifyParallelSegmentProjection(SCLineSegment2d{SCPoint2d{0.0, 0.0}, SCPoint2d{5.0, 0.0}},
                                                             SCLineSegment2d{SCPoint2d{5.0, 0.0}, SCPoint2d{10.0, 0.0}},
                                                             MakeTolerance(1e-9, 1e-9));
    EXPECT_EQ(relation, SCParallelSegmentProjectionRelation2d::EndpointTouch);
}

TEST(RelationClassificationTest, ProjectionEpsilonSidesClassifyCorrectly)
{
    const double projectionEpsilon = 1.0;
    // overlap == -projectionEpsilon：位于 [-pe, pe] 端点接触。
    auto relation = ClassifyParallelSegmentProjection(
        SCLineSegment2d{SCPoint2d{0.0, 0.0}, SCPoint2d{5.0, 0.0}},
        SCLineSegment2d{SCPoint2d{6.0, 0.0}, SCPoint2d{10.0, 0.0}},
        MakeTolerance(1e-9, projectionEpsilon));
    EXPECT_EQ(relation, SCParallelSegmentProjectionRelation2d::EndpointTouch);
    // overlap == +projectionEpsilon：位于 [-pe, pe] 端点接触。
    relation = ClassifyParallelSegmentProjection(
        SCLineSegment2d{SCPoint2d{0.0, 0.0}, SCPoint2d{5.0, 0.0}},
        SCLineSegment2d{SCPoint2d{4.0, 0.0}, SCPoint2d{10.0, 0.0}},
        MakeTolerance(1e-9, projectionEpsilon));
    EXPECT_EQ(relation, SCParallelSegmentProjectionRelation2d::EndpointTouch);
    // overlap < -projectionEpsilon：分离。
    relation = ClassifyParallelSegmentProjection(
        SCLineSegment2d{SCPoint2d{0.0, 0.0}, SCPoint2d{5.0, 0.0}},
        SCLineSegment2d{SCPoint2d{6.5, 0.0}, SCPoint2d{10.0, 0.0}},
        MakeTolerance(1e-9, projectionEpsilon));
    EXPECT_EQ(relation, SCParallelSegmentProjectionRelation2d::NoPositiveLengthIntersection);
    // overlap > projectionEpsilon：正长度重叠。
    relation = ClassifyParallelSegmentProjection(
        SCLineSegment2d{SCPoint2d{0.0, 0.0}, SCPoint2d{5.0, 0.0}},
        SCLineSegment2d{SCPoint2d{3.0, 0.0}, SCPoint2d{10.0, 0.0}},
        MakeTolerance(1e-9, projectionEpsilon));
    EXPECT_EQ(relation, SCParallelSegmentProjectionRelation2d::PositiveLengthIntersection);
}

TEST(RelationClassificationTest, PerpendicularSegmentsAreNonParallel)
{
    const auto relation = ClassifyParallelSegmentProjection(SCLineSegment2d{SCPoint2d{0.0, 0.0}, SCPoint2d{10.0, 0.0}},
                                                             SCLineSegment2d{SCPoint2d{0.0, 0.0}, SCPoint2d{0.0, 10.0}},
                                                             MakeTolerance(1e-9, 1e-9));
    EXPECT_EQ(relation, SCParallelSegmentProjectionRelation2d::NonParallel);
}

TEST(RelationClassificationTest, NearlyParallelWithinAngularEpsilonIsParallel)
{
    // 方向叉积绝对值约 1e-6，小于 1e-4 角度容差，视为平行；投影区间有正长度重叠。
    const double offset = 1e-6;
    const auto relation = ClassifyParallelSegmentProjection(
        SCLineSegment2d{SCPoint2d{0.0, 0.0}, SCPoint2d{10.0, 0.0}},
        SCLineSegment2d{SCPoint2d{2.0, offset}, SCPoint2d{8.0, offset}},
        MakeTolerance(1e-4, 1e-9));
    EXPECT_EQ(relation, SCParallelSegmentProjectionRelation2d::PositiveLengthIntersection);
}

TEST(RelationClassificationTest, ZeroLengthSegmentIsInvalidInput)
{
    const auto relation = ClassifyParallelSegmentProjection(SCLineSegment2d{SCPoint2d{0.0, 0.0}, SCPoint2d{0.0, 0.0}},
                                                             SCLineSegment2d{SCPoint2d{0.0, 0.0}, SCPoint2d{10.0, 0.0}},
                                                             MakeTolerance(1e-9, 1e-9));
    EXPECT_EQ(relation, SCParallelSegmentProjectionRelation2d::InvalidInput);
}

TEST(RelationClassificationTest, HugeFiniteCoordinatesDoNotOverflow)
{
    // 极大有限坐标：单位向量归一化以最大绝对分量缩放，避免长度平方溢出。
    const double huge = 1e154;
    const auto relation = ClassifyParallelSegmentProjection(
        SCLineSegment2d{SCPoint2d{0.0, 0.0}, SCPoint2d{huge, 0.0}},
        SCLineSegment2d{SCPoint2d{0.0, 0.0}, SCPoint2d{huge, 0.0}},
        MakeTolerance(1e-9, 1e-9));
    EXPECT_EQ(relation, SCParallelSegmentProjectionRelation2d::PositiveLengthIntersection);
}

TEST(RelationClassificationTest, HugeFiniteCoordinatesPerpendicularAreNonParallel)
{
    const double huge = 1e150;
    const auto relation = ClassifyParallelSegmentProjection(
        SCLineSegment2d{SCPoint2d{0.0, 0.0}, SCPoint2d{huge, 0.0}},
        SCLineSegment2d{SCPoint2d{0.0, 0.0}, SCPoint2d{0.0, huge}},
        MakeTolerance(1e-9, 1e-9));
    EXPECT_EQ(relation, SCParallelSegmentProjectionRelation2d::NonParallel);
}

TEST(RelationClassificationTest, VeryShortNonDegenerateSegmentClassifiesNormally)
{
    // 端点不同但长度极短的非退化线段仍按正常关系分类。
    // 注意：SCLineSegment2d::IsValid() 以 kDefaultEpsilon(1e-9) 判定退化，故长度须 > 1e-9。
    const double tiny = 1e-8;
    // projectionEpsilon=1e-7：投影重叠量 1e-8 <= pe，端点接触。
    auto relation = ClassifyParallelSegmentProjection(
        SCLineSegment2d{SCPoint2d{0.0, 0.0}, SCPoint2d{tiny, 0.0}},
        SCLineSegment2d{SCPoint2d{0.0, 0.0}, SCPoint2d{tiny, 0.0}},
        MakeTolerance(1e-9, 1e-7));
    EXPECT_EQ(relation, SCParallelSegmentProjectionRelation2d::EndpointTouch);
    // projectionEpsilon=1e-9：重叠量 1e-8 > pe，正长度重叠。
    relation = ClassifyParallelSegmentProjection(
        SCLineSegment2d{SCPoint2d{0.0, 0.0}, SCPoint2d{tiny, 0.0}},
        SCLineSegment2d{SCPoint2d{0.0, 0.0}, SCPoint2d{tiny, 0.0}},
        MakeTolerance(1e-9, 1e-9));
    EXPECT_EQ(relation, SCParallelSegmentProjectionRelation2d::PositiveLengthIntersection);
}

TEST(RelationClassificationTest, SegmentShorterThanDefaultEpsilonRemainsValidForThisApi)
{
    // SCLineSegment2d 的通用 IsValid 使用默认 epsilon，但本 API 只要求端点不同。
    const double tiny = 1e-10;
    const auto relation = ClassifyParallelSegmentProjection(
        SCLineSegment2d{SCPoint2d{0.0, 0.0}, SCPoint2d{tiny, 0.0}},
        SCLineSegment2d{SCPoint2d{0.0, 0.0}, SCPoint2d{tiny, 0.0}},
        MakeTolerance(1e-9, 1e-9));
    EXPECT_EQ(relation, SCParallelSegmentProjectionRelation2d::EndpointTouch);
}
