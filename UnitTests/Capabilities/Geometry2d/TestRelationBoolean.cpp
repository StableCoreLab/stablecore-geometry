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

TEST(RelationBooleanTest, OffsetBoxUnionDoesNotFillMissingCorners)
{
    constexpr double shift = 0.8284271247461901;
    const SCPolygon2d first(SCPolyline2d(
        {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}}, SCPolylineClosure::Closed));
    const SCPolygon2d second(SCPolyline2d(
        {{shift, shift}, {2.0 + shift, shift}, {2.0 + shift, 2.0 + shift}, {shift, 2.0 + shift}},
        SCPolylineClosure::Closed));

    const auto united = Union(first, second);
    ASSERT_FALSE(united.IsEmpty());
    EXPECT_NEAR(TotalArea(united), 8.0 - (2.0 - shift) * (2.0 - shift), 1e-8);
}

TEST(RelationBooleanTest, TinyOffsetBoxUnionPreservesConnectedMissingCorners)
{
    constexpr double eps = 1e-15;
    constexpr double width = 1e-8;
    const SCPolygon2d first(SCPolyline2d(
        {{0.0, 0.0}, {width, 0.0}, {width, width}, {0.0, width}}, SCPolylineClosure::Closed));
    const SCPolygon2d second(SCPolyline2d(
        {{width / 2.0, width / 2.0}, {1.5 * width, width / 2.0},
         {1.5 * width - width / 1000.0, width / 2.0},
         {1.5 * width, 1.5 * width}, {width / 2.0, 1.5 * width}},
        SCPolylineClosure::Closed));
    ASSERT_TRUE(first.IsValid(eps));
    ASSERT_FALSE(second.IsValid(eps));

    const auto overlap = Intersect(first, second, eps);
    const auto firstOnly = Difference(first, second, eps);
    const auto secondOnly = Difference(second, first, eps);
    EXPECT_NEAR(TotalArea(overlap), 0.25 * width * width, width * width * 1e-5);
    EXPECT_NEAR(TotalArea(firstOnly), 0.75 * width * width, width * width * 1e-5);
    EXPECT_NEAR(TotalArea(secondOnly), 0.75 * width * width, width * width * 1e-5);

    const SCPolygon2d repairedSecond(SCPolyline2d(
        {{width / 2.0, width / 2.0}, {1.5 * width, width / 2.0},
         {1.5 * width, 1.5 * width}, {width / 2.0, 1.5 * width}}, SCPolylineClosure::Closed));
    const auto validUnion = Union(first, repairedSecond, eps);
    ASSERT_EQ(validUnion.Count(), 1U);
    EXPECT_NEAR(TotalArea(validUnion), 1.75 * width * width, width * width * 1e-5);

    const auto united = Union(first, second, eps);
    ASSERT_EQ(united.Count(), 1U);
    EXPECT_TRUE(united.IsValid(eps));
    EXPECT_NEAR(TotalArea(united), 1.75 * width * width, width * width * 1e-5);
    EXPECT_EQ(LocatePoint({width / 4.0, 1.25 * width}, united[0], eps), SCPointContainment2d::Outside);
    EXPECT_EQ(LocatePoint({1.25 * width, width / 4.0}, united[0], eps), SCPointContainment2d::Outside);
    EXPECT_EQ(LocatePoint({width / 4.0, width / 4.0}, united[0], eps), SCPointContainment2d::Inside);
    EXPECT_EQ(LocatePoint({1.25 * width, 1.25 * width}, united[0], eps), SCPointContainment2d::Inside);
}

TEST(RelationBooleanTest, TinyOverlappingBoxesFormSinglePolygonAcrossAxisLayouts)
{
    constexpr double eps = 1e-15;
    constexpr double width = 1e-8;
    struct BoxCase
    {
        const char* name;
        double minX;
        double minY;
        double maxX;
        double maxY;
    };
    const std::vector<BoxCase> cases{
        {"horizontal", 0.5, 0.0, 1.5, 1.0},
        {"vertical", 0.0, 0.5, 1.0, 1.5},
        {"aligned_bottom", 0.5, 0.0, 1.5, 0.75},
        {"contained", 0.25, 0.25, 0.75, 0.75},
        {"identical", 0.0, 0.0, 1.0, 1.0},
        {"horizontal_touch", 1.0, 0.0, 2.0, 1.0},
        {"vertical_touch", 0.0, 1.0, 1.0, 2.0},
        {"horizontal_partial_touch", 1.0, 0.25, 2.0, 0.75},
        {"vertical_partial_touch", 0.25, 1.0, 0.75, 2.0}};
    const SCPolygon2d first(SCPolyline2d(
        {{0.0, 0.0}, {width, 0.0}, {width, width}, {0.0, width}}, SCPolylineClosure::Closed));
    ASSERT_TRUE(first.IsValid(eps));
    for (const BoxCase& box : cases)
    {
        SCOPED_TRACE(box.name);
        const double minX = box.minX * width;
        const double minY = box.minY * width;
        const double maxX = box.maxX * width;
        const double maxY = box.maxY * width;
        const SCPolygon2d second(SCPolyline2d(
            {{minX, minY}, {maxX, minY}, {maxX, maxY}, {minX, maxY}}, SCPolylineClosure::Closed));
        ASSERT_TRUE(second.IsValid(eps));
        const double overlapWidth = std::max(0.0, std::min(width, maxX) - std::max(0.0, minX));
        const double overlapHeight = std::max(0.0, std::min(width, maxY) - std::max(0.0, minY));
        const double expectedArea = width * width + (maxX - minX) * (maxY - minY) -
                                    overlapWidth * overlapHeight;
        for (const auto& operands : {std::pair<const SCPolygon2d*, const SCPolygon2d*>{&first, &second},
                                     {&second, &first}})
        {
            const auto united = Union(*operands.first, *operands.second, eps);
            ASSERT_EQ(united.Count(), 1U);
            EXPECT_TRUE(united.IsValid(eps));
            EXPECT_NEAR(TotalArea(united), expectedArea, width * width * 1e-5);
        }
    }
}

TEST(RelationBooleanTest, TinyBoxesMeetingAtOnlyOnePointRemainSeparate)
{
    constexpr double eps = 1e-15;
    constexpr double width = 1e-8;
    const SCPolygon2d first(SCPolyline2d(
        {{0.0, 0.0}, {width, 0.0}, {width, width}, {0.0, width}}, SCPolylineClosure::Closed));
    const SCPolygon2d cornerTouch(SCPolyline2d(
        {{width, width}, {2.0 * width, width}, {2.0 * width, 2.0 * width}, {width, 2.0 * width}},
        SCPolylineClosure::Closed));
    const SCPolygon2d disjoint(SCPolyline2d(
        {{2.0 * width, 0.0}, {3.0 * width, 0.0}, {3.0 * width, width}, {2.0 * width, width}},
        SCPolylineClosure::Closed));

    for (const SCPolygon2d* second : {&cornerTouch, &disjoint})
    {
        const auto united = Union(first, *second, eps);
        EXPECT_EQ(united.Count(), 2U);
        EXPECT_NEAR(TotalArea(united), 2.0 * width * width, width * width * 1e-5);
    }
}

TEST(RelationBooleanTest, SmallCallerTolerancePreservesNarrowInset)
{
    constexpr double eps = 1e-15;
    constexpr double gap = 1e-9;
    const SCPolygon2d inset(SCPolyline2d(
        {{0.0, 0.0}, {6.0, 0.0}, {6.0, 1.0}, {4.0, 1.0},
         {4.0, 1.0 + gap}, {6.0, 1.0 + gap}, {6.0, 4.0}, {0.0, 4.0}},
        SCPolylineClosure::Closed));
    const SCPolygon2d clip(SCPolyline2d(
        {{2.0, -1.0}, {8.0, -1.0}, {8.0, 3.0}, {2.0, 3.0}},
        SCPolylineClosure::Closed));
    ASSERT_TRUE(clip.IsValid());
    EXPECT_FALSE(inset.IsValid());
    EXPECT_TRUE(inset.IsValid(eps));
    EXPECT_TRUE(Validate(inset, eps).valid);
    EXPECT_NEAR(inset.Perimeter(eps), 24.0, 1e-12);
    EXPECT_EQ(LocatePoint(inset.Centroid(eps), inset, eps), SCPointContainment2d::Inside);
    ASSERT_TRUE(inset.Bounds(eps).IsValid());
    EXPECT_NEAR(inset.Bounds(eps).MaxPoint().x, 6.0, eps);
    Geometry::SCMultiPolyline2d boundaries;
    boundaries.Add(inset.OuterRing());
    const auto direct = Geometry::BuildMultiPolygonByLines(boundaries, eps);
    ASSERT_EQ(direct.Count(), 1U);
    EXPECT_NEAR(direct[0].Area(eps), 24.0 - 2.0 * gap, 1e-11);

    const SCPolygon2d normalized = Geometry::NormalizePolygonByLines(inset, eps);
    ASSERT_TRUE(normalized.IsValid(eps));
    EXPECT_NEAR(normalized.Area(eps), 24.0 - 2.0 * gap, 1e-11);

    const auto intersection = Intersect(inset, clip, eps);
    ASSERT_EQ(intersection.Count(), 1U);
    EXPECT_FALSE(intersection.IsValid());
    EXPECT_TRUE(intersection.IsValid(eps));
    EXPECT_NEAR(intersection[0].Area(eps), 12.0 - 2.0 * gap, 1e-11);
    ASSERT_TRUE(intersection.Bounds(eps).IsValid());
    EXPECT_NEAR(intersection.Bounds(eps).MinPoint().x, 2.0, eps);

    const auto areaWithTolerance = [eps](const SCMultiPolygon2d& polygons) {
        double area = 0.0;
        for (const SCPolygon2d& polygon : polygons.Polygons())
        {
            area += polygon.Area(eps);
        }
        return area;
    };
    const auto reverseDifference = Difference(clip, inset, eps);
    ASSERT_FALSE(reverseDifference.IsEmpty());
    EXPECT_TRUE(reverseDifference.IsValid(eps));
    EXPECT_NEAR(areaWithTolerance(reverseDifference), 12.0 + 2.0 * gap, 1e-11);

    const auto united = Union(inset, clip, eps);
    ASSERT_FALSE(united.IsEmpty());
    EXPECT_TRUE(united.IsValid(eps));
    EXPECT_NEAR(areaWithTolerance(united), 36.0, 1e-11);
}

TEST(RelationBooleanTest, RejectsInvalidBooleanTolerance)
{
    const SCPolygon2d first(SCPolyline2d(
        {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}}, SCPolylineClosure::Closed));
    const SCPolygon2d second(SCPolyline2d(
        {{1.0, 0.0}, {3.0, 0.0}, {3.0, 2.0}, {1.0, 2.0}}, SCPolylineClosure::Closed));
    ASSERT_TRUE(first.IsValid());
    ASSERT_TRUE(second.IsValid());

    for (double eps : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN()})
    {
        EXPECT_TRUE(Intersect(first, second, eps).IsEmpty());
        EXPECT_TRUE(Union(first, second, eps).IsEmpty());
        EXPECT_TRUE(Difference(first, second, eps).IsEmpty());
    }
}

TEST(RelationBooleanTest, HandlesCrossAndNestedBooleanOperations)
{
    const SCPolygon2d square(SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}},
                                          SCPolylineClosure::Closed));
    const SCPolygon2d horizontalBar(SCPolyline2d(
        {SCPoint2d{0.0, 1.0}, SCPoint2d{4.0, 1.0}, SCPoint2d{4.0, 3.0}, SCPoint2d{0.0, 3.0}}, SCPolylineClosure::Closed));
    const SCPolygon2d verticalBar(SCPolyline2d({SCPoint2d{1.0, 0.0}, SCPoint2d{3.0, 0.0}, SCPoint2d{3.0, 4.0}, SCPoint2d{1.0, 4.0}},
                                           SCPolylineClosure::Closed));

    const auto crossUnion = Union(horizontalBar, verticalBar);
    ASSERT_GE(crossUnion.Count(), 1);
    EXPECT_NEAR(TotalArea(crossUnion), 12.0, 1e-9);

    const auto crossDifference = Difference(horizontalBar, verticalBar);
    ASSERT_GE(crossDifference.Count(), 1);
    EXPECT_NEAR(TotalArea(crossDifference), 4.0, 1e-9);

    const SCPolygon2d outer(SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{6.0, 0.0}, SCPoint2d{6.0, 6.0}, SCPoint2d{0.0, 6.0}},
                                     SCPolylineClosure::Closed));
    const SCPolygon2d nested(SCPolyline2d({SCPoint2d{2.0, 2.0}, SCPoint2d{4.0, 2.0}, SCPoint2d{4.0, 4.0}, SCPoint2d{2.0, 4.0}},
                                      SCPolylineClosure::Closed));

    const auto containedUnion = Union(outer, nested);
    ASSERT_GE(containedUnion.Count(), 1);
    EXPECT_NEAR(TotalArea(containedUnion), 36.0, 1e-9);

    const auto containedIntersection = Intersect(outer, nested);
    ASSERT_GE(containedIntersection.Count(), 1);
    EXPECT_NEAR(TotalArea(containedIntersection), 4.0, 1e-9);

    const auto containedDifference = Difference(outer, nested);
    ASSERT_GE(containedDifference.Count(), 1);
    EXPECT_NEAR(TotalArea(containedDifference), 32.0, 1e-9);

    const SCPolygon2d overlapStripA(SCPolyline2d(
        {SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 2.0}, SCPoint2d{0.0, 2.0}}, SCPolylineClosure::Closed));
    const SCPolygon2d overlapStripB(SCPolyline2d(
        {SCPoint2d{2.0, 0.0}, SCPoint2d{6.0, 0.0}, SCPoint2d{6.0, 2.0}, SCPoint2d{2.0, 2.0}}, SCPolylineClosure::Closed));
    EXPECT_NEAR(TotalArea(Intersect(overlapStripA, overlapStripB)), 4.0, 1e-9);
    EXPECT_NEAR(TotalArea(Union(overlapStripA, overlapStripB)), 12.0, 1e-9);
    EXPECT_NEAR(TotalArea(Difference(overlapStripA, overlapStripB)), 4.0, 1e-9);

}
