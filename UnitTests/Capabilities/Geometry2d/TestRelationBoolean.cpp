#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "Core/Boolean.h"
#include "Core/Relation.h"
#include "Core/ShapeOps.h"
#include "Core/Validation.h"
#include "Geometry2d/SCPathOps.h"
#include "Support/PolygonBooleanFixtures2d.h"

using Geometry::Difference;
using Geometry::Intersect;
using Geometry::SCLineSegment2d;
using Geometry::LocatePoint;
using Geometry::SCMultiPolygon2d;
using Geometry::SCPoint2d;
using Geometry::SCPointContainment2d;
using Geometry::SCPolygon2d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;
using Geometry::Union;
using Geometry::Validate;
using Geometry::Test::TotalArea;

TEST(RelationBooleanTest, HandlesPointContainmentAndBasicBooleanOperations)
{
    const SCLineSegment2d horizontal(SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0});
    ASSERT_EQ(LocatePoint(SCPoint2d{2.0, 0.0}, horizontal), SCPointContainment2d::OnBoundary);
    ASSERT_EQ(LocatePoint(SCPoint2d{2.0, 1.0}, horizontal), SCPointContainment2d::Outside);

    const SCPolyline2d squareRing({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}},
                                SCPolylineClosure::Closed);
    const SCPolygon2d square(squareRing);
    ASSERT_EQ(LocatePoint(SCPoint2d{2.0, 2.0}, square), SCPointContainment2d::Inside);
    ASSERT_EQ(LocatePoint(SCPoint2d{5.0, 2.0}, square), SCPointContainment2d::Outside);
    ASSERT_TRUE(Validate(square).valid);

    const SCPolyline2d clipRing({SCPoint2d{2.0, 0.0}, SCPoint2d{6.0, 0.0}, SCPoint2d{6.0, 4.0}, SCPoint2d{2.0, 4.0}},
                              SCPolylineClosure::Closed);
    const SCPolygon2d clip(clipRing);
    const auto intersection = Intersect(square, clip);
    EXPECT_NEAR(TotalArea(intersection), 8.0, 1e-9);

    const auto united = Union(square, clip);
    EXPECT_NEAR(TotalArea(united), 24.0, 1e-9);

    const SCPolyline2d innerRing({SCPoint2d{1.0, 1.0}, SCPoint2d{3.0, 1.0}, SCPoint2d{3.0, 3.0}, SCPoint2d{1.0, 3.0}},
                               SCPolylineClosure::Closed);
    const SCPolygon2d inner(innerRing);
    const auto difference = Difference(square, inner);
    EXPECT_NEAR(TotalArea(difference), 12.0, 1e-9);

}

TEST(RelationBooleanTest, ShortSegmentDirectionRelationsUseAngularTolerance)
{
    constexpr double scale = 1e-10;
    const SCLineSegment2d horizontal({0.0, 0.0}, {scale, 0.0});
    const SCLineSegment2d tilted({0.0, 0.0}, {scale * std::cos(0.02), scale * std::sin(0.02)});
    const SCLineSegment2d wideAngle({0.0, 0.0}, {scale * std::cos(1.0), scale * std::sin(1.0)});
    const SCLineSegment2d reversed({scale, 0.0}, {0.0, 0.0});

    EXPECT_FALSE(Geometry::IsParallel(horizontal, tilted, 0.01));
    EXPECT_TRUE(Geometry::IsParallel(horizontal, tilted, 0.03));
    EXPECT_TRUE(Geometry::IsSameDirection(horizontal, tilted, 0.03));
    EXPECT_FALSE(Geometry::IsParallel(horizontal, wideAngle, 0.9));
    EXPECT_TRUE(Geometry::IsParallel(horizontal, wideAngle, 1.1));
    EXPECT_TRUE(Geometry::IsAntiParallel(horizontal, reversed, 0.01));
    EXPECT_FALSE(Geometry::IsSameDirection(horizontal, reversed, 0.01));
    EXPECT_FALSE(Geometry::IsParallel(horizontal, tilted, -0.01));
    EXPECT_FALSE(Geometry::IsParallel(horizontal, tilted, std::numeric_limits<double>::infinity()));
}

TEST(RelationBooleanTest, SmallCallerTolerancePreservesBoxIntersectionBoundary)
{
    constexpr double eps = 1e-15;
    const SCPolygon2d subject(SCPolyline2d(
        {{-5e-13, -1.0}, {1.0, -1.0}, {1.0, 1.0}, {-5e-13, 1.0}},
        SCPolylineClosure::Closed));
    const SCPolygon2d clipper(SCPolyline2d(
        {{0.0, -2.0}, {2.0, -2.0}, {2.0, 2.0}, {0.0, 2.0}},
        SCPolylineClosure::Closed));
    ASSERT_TRUE(Validate(subject, eps).valid);
    ASSERT_TRUE(Validate(clipper, eps).valid);

    for (const auto& intersection : {Intersect(subject, clipper, eps), Intersect(clipper, subject, eps)})
    {
        ASSERT_EQ(intersection.Count(), 1U);
        for (std::size_t i = 0; i < intersection[0].OuterRing().PointCount(); ++i)
        {
            EXPECT_GE(intersection[0].OuterRing().VertexAt(i).x, -eps);
        }
        EXPECT_NEAR(TotalArea(intersection), 2.0, 1e-12);
    }
}

TEST(RelationBooleanTest, SmallCallerTolerancePreservesNonAxisAlignedTriangleIntersection)
{
    constexpr double scale = 1e-10;
    constexpr double eps = 1e-15;
    const SCPolygon2d first(SCPolyline2d(
        {{0.0, 0.0}, {4.0 * scale, 0.0}, {0.0, 4.0 * scale}}, SCPolylineClosure::Closed));
    const SCPolygon2d second(SCPolyline2d(
        {{scale, 0.0}, {5.0 * scale, 0.0}, {scale, 4.0 * scale}}, SCPolylineClosure::Closed));
    ASSERT_TRUE(first.IsValid(eps));
    ASSERT_TRUE(second.IsValid(eps));
    EXPECT_TRUE(Geometry::NormalizePolygonByLines(first, eps).IsValid(eps));
    EXPECT_TRUE(Geometry::NormalizePolygonByLines(second, eps).IsValid(eps));
    EXPECT_EQ(LocatePoint({2.0 * scale, scale}, first, eps), SCPointContainment2d::Inside);
    EXPECT_EQ(LocatePoint({scale, 0.0}, first, eps), SCPointContainment2d::OnBoundary);
    EXPECT_EQ(LocatePoint({scale, 0.0}, SCLineSegment2d({0.0, 0.0}, {4.0 * scale, 0.0}), eps),
              SCPointContainment2d::OnBoundary);

    const auto intersection = Intersect(first, second, eps);
    ASSERT_EQ(intersection.Count(), 1U);
    EXPECT_TRUE(intersection[0].IsValid(eps));
    EXPECT_NEAR(intersection[0].Area(eps), 4.5 * scale * scale, 1e-23);
}

TEST(RelationBooleanTest, SmallCallerTolerancePreservesTinyBoxIntersection)
{
    constexpr double eps = 1e-15;
    const SCPolygon2d first(SCPolyline2d(
        {{0.0, 0.0}, {1e-7, 0.0}, {1e-7, 1e-7}, {0.0, 1e-7}}, SCPolylineClosure::Closed));
    const SCPolygon2d second(SCPolyline2d(
        {{5e-8, 0.0}, {1.5e-7, 0.0}, {1.5e-7, 1e-7}, {5e-8, 1e-7}},
        SCPolylineClosure::Closed));
    ASSERT_TRUE(first.IsValid());
    ASSERT_TRUE(second.IsValid());
    ASSERT_TRUE(Validate(first, eps).valid);
    ASSERT_TRUE(Validate(second, eps).valid);
    EXPECT_NEAR(first.Centroid(eps).x, 5e-8, 1e-12);
    EXPECT_NEAR(first.Centroid(eps).y, 5e-8, 1e-12);
    EXPECT_TRUE(Geometry::NormalizePolygonByLines(first, eps).IsValid());
    EXPECT_TRUE(Geometry::NormalizePolygonByLines(second, eps).IsValid());

    for (const auto& intersection : {Intersect(first, second, eps), Intersect(second, first, eps)})
    {
        ASSERT_EQ(intersection.Count(), 1U);
        EXPECT_NEAR(TotalArea(intersection), 5e-15, 1e-17);
        for (std::size_t i = 0; i < intersection[0].OuterRing().PointCount(); ++i)
        {
            const SCPoint2d point = intersection[0].OuterRing().VertexAt(i);
            EXPECT_GE(point.x, 5e-8 - eps);
            EXPECT_LE(point.x, 1e-7 + eps);
            EXPECT_GE(point.y, -eps);
            EXPECT_LE(point.y, 1e-7 + eps);
        }
    }
}

TEST(RelationBooleanTest, SmallCallerTolerancePreservesTinyBoxDifference)
{
    constexpr double eps = 1e-15;
    constexpr double width = 1e-6;
    const SCPolygon2d first(SCPolyline2d(
        {{0.0, 0.0}, {width, 0.0}, {width, width}, {0.0, width}},
        SCPolylineClosure::Closed));
    const SCPolygon2d second(SCPolyline2d(
        {{0.0, 0.0}, {width / 2.0, 0.0}, {width / 2.0, width}, {0.0, width}},
        SCPolylineClosure::Closed));
    ASSERT_TRUE(first.IsValid(eps));
    ASSERT_TRUE(second.IsValid(eps));

    const auto difference = Difference(first, second, eps);
    ASSERT_EQ(difference.Count(), 1U);
    EXPECT_TRUE(difference.IsValid(eps));
    EXPECT_NEAR(difference[0].Area(eps), width * width / 2.0, 1e-17);
    EXPECT_NEAR(difference[0].Bounds(eps).MinPoint().x, width / 2.0, eps);
}

TEST(RelationBooleanTest, SmallCallerTolerancePreservesRepairedTinyBoxBoolean)
{
    constexpr double eps = 1e-15;
    for (double width : {1e-6, 1e-7, 1e-8})
    {
        SCOPED_TRACE(width);
        const SCPolygon2d first(SCPolyline2d(
            {{0.0, 0.0}, {width, 0.0}, {width, width}, {0.0, width}},
            SCPolylineClosure::Closed));
        const SCPolygon2d second(SCPolyline2d(
            {{0.0, 0.0}, {width / 2.0, 0.0}, {width / 2.0 - width / 1000.0, 0.0},
             {width / 2.0, width}, {0.0, width}},
            SCPolylineClosure::Closed));
        ASSERT_TRUE(first.IsValid(eps));
        ASSERT_FALSE(second.IsValid(eps));

        const auto difference = Difference(first, second, eps);
        ASSERT_EQ(difference.Count(), 1U);
        EXPECT_TRUE(difference.IsValid(eps));
        EXPECT_NEAR(difference[0].Area(eps), width * width / 2.0, width * width * 1e-5);

        const auto intersection = Intersect(first, second, eps);
        ASSERT_EQ(intersection.Count(), 1U);
        EXPECT_TRUE(intersection.IsValid(eps));
        EXPECT_NEAR(intersection[0].Area(eps), width * width / 2.0, width * width * 1e-5);

        const auto united = Union(first, second, eps);
        ASSERT_FALSE(united.IsEmpty());
        EXPECT_TRUE(united.IsValid(eps));
        EXPECT_EQ(united.Count(), 1U);
        EXPECT_NEAR(TotalArea(united), width * width, width * width * 1e-5);
    }
}

TEST(RelationBooleanTest, RepairedSubDefaultScaleBoxRespectsCallerTolerance)
{
    constexpr double eps = 1e-15;
    constexpr double width = 1e-10;
    const SCPolygon2d first(SCPolyline2d(
        {{0.0, 0.0}, {width, 0.0}, {width, width}, {0.0, width}},
        SCPolylineClosure::Closed));
    const SCPolygon2d second(SCPolyline2d(
        {{0.0, 0.0}, {width / 2.0, 0.0}, {width / 2.0 - width / 1000.0, 0.0},
         {width / 2.0, width}, {0.0, width}},
        SCPolylineClosure::Closed));
    ASSERT_TRUE(first.IsValid(eps));
    ASSERT_FALSE(second.IsValid(eps));

    for (const SCPolygon2d& damaged : {second, SCPolygon2d(Reverse(second.OuterRing()))})
    {
        const auto intersection = Intersect(first, damaged, eps);
        ASSERT_EQ(intersection.Count(), 1U);
        ASSERT_TRUE(intersection.IsValid(eps));
        EXPECT_NEAR(intersection[0].Area(eps), width * width / 2.0, width * width * 1e-5);

        const auto difference = Difference(first, damaged, eps);
        ASSERT_EQ(difference.Count(), 1U);
        ASSERT_TRUE(difference.IsValid(eps));
        EXPECT_NEAR(difference[0].Area(eps), width * width / 2.0, width * width * 1e-5);

        const auto united = Union(first, damaged, eps);
        ASSERT_EQ(united.Count(), 1U);
        ASSERT_TRUE(united.IsValid(eps));
        EXPECT_NEAR(united[0].Area(eps), width * width, width * width * 1e-5);
    }
}

TEST(RelationBooleanTest, SmallCallerToleranceMergesVerticallyRepairedBoxUnion)
{
    constexpr double eps = 1e-15;
    constexpr double width = 1e-6;
    const SCPolygon2d first(SCPolyline2d(
        {{0.0, 0.0}, {width, 0.0}, {width, width}, {0.0, width}},
        SCPolylineClosure::Closed));
    const SCPolygon2d second(SCPolyline2d(
        {{0.0, 0.0}, {width, 0.0}, {width, width / 2.0},
         {width, width / 2.0 - width / 1000.0}, {0.0, width / 2.0}},
        SCPolylineClosure::Closed));
    ASSERT_FALSE(second.IsValid(eps));

    const auto united = Union(first, second, eps);
    ASSERT_EQ(united.Count(), 1U);
    EXPECT_TRUE(united.IsValid(eps));
    EXPECT_NEAR(united[0].Area(eps), width * width, width * width * 1e-5);
}
