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
    ASSERT_EQ(validUnion.Size(), 1U);
    EXPECT_NEAR(TotalArea(validUnion), 1.75 * width * width, width * width * 1e-5);

    const auto united = Union(first, second, eps);
    ASSERT_EQ(united.Size(), 1U);
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
            ASSERT_EQ(united.Size(), 1U);
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
        EXPECT_EQ(united.Size(), 2U);
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
    ASSERT_EQ(direct.Size(), 1U);
    EXPECT_NEAR(direct[0].Area(eps), 24.0 - 2.0 * gap, 1e-11);

    const SCPolygon2d normalized = Geometry::NormalizePolygonByLines(inset, eps);
    ASSERT_TRUE(normalized.IsValid(eps));
    EXPECT_NEAR(normalized.Area(eps), 24.0 - 2.0 * gap, 1e-11);

    const auto intersection = Intersect(inset, clip, eps);
    ASSERT_EQ(intersection.Size(), 1U);
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
    ASSERT_GE(crossUnion.Size(), 1);
    EXPECT_NEAR(TotalArea(crossUnion), 12.0, 1e-9);

    const auto crossDifference = Difference(horizontalBar, verticalBar);
    ASSERT_GE(crossDifference.Size(), 1);
    EXPECT_NEAR(TotalArea(crossDifference), 4.0, 1e-9);

    const SCPolygon2d outer(SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{6.0, 0.0}, SCPoint2d{6.0, 6.0}, SCPoint2d{0.0, 6.0}},
                                     SCPolylineClosure::Closed));
    const SCPolygon2d nested(SCPolyline2d({SCPoint2d{2.0, 2.0}, SCPoint2d{4.0, 2.0}, SCPoint2d{4.0, 4.0}, SCPoint2d{2.0, 4.0}},
                                      SCPolylineClosure::Closed));

    const auto containedUnion = Union(outer, nested);
    ASSERT_GE(containedUnion.Size(), 1);
    EXPECT_NEAR(TotalArea(containedUnion), 36.0, 1e-9);

    const auto containedIntersection = Intersect(outer, nested);
    ASSERT_GE(containedIntersection.Size(), 1);
    EXPECT_NEAR(TotalArea(containedIntersection), 4.0, 1e-9);

    const auto containedDifference = Difference(outer, nested);
    ASSERT_GE(containedDifference.Size(), 1);
    EXPECT_NEAR(TotalArea(containedDifference), 32.0, 1e-9);

    const SCPolygon2d overlapStripA(SCPolyline2d(
        {SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 2.0}, SCPoint2d{0.0, 2.0}}, SCPolylineClosure::Closed));
    const SCPolygon2d overlapStripB(SCPolyline2d(
        {SCPoint2d{2.0, 0.0}, SCPoint2d{6.0, 0.0}, SCPoint2d{6.0, 2.0}, SCPoint2d{2.0, 2.0}}, SCPolylineClosure::Closed));
    EXPECT_NEAR(TotalArea(Intersect(overlapStripA, overlapStripB)), 4.0, 1e-9);
    EXPECT_NEAR(TotalArea(Union(overlapStripA, overlapStripB)), 12.0, 1e-9);
    EXPECT_NEAR(TotalArea(Difference(overlapStripA, overlapStripB)), 4.0, 1e-9);

}

TEST(RelationBooleanTest, RepairedBoxBooleanPreservesUniformScale)
{
    for (const double width : {1.0, 1e-4, 1e-8})
    {
        SCOPED_TRACE(width);
        const double eps = 1e-7 * width;
        const SCPolygon2d first(SCPolyline2d({{0, 0}, {width, 0}, {width, width}, {0, width}},
            SCPolylineClosure::Closed));
        const SCPolygon2d second(SCPolyline2d({{0.5 * width, 0.5 * width}, {1.5 * width, 0.5 * width},
            {1.499 * width, 0.5 * width}, {1.5 * width, 1.5 * width}, {0.5 * width, 1.5 * width}},
            SCPolylineClosure::Closed));
        ASSERT_TRUE(first.IsValid(eps));
        ASSERT_FALSE(second.IsValid(eps));
        const auto area = [&](const SCMultiPolygon2d& polygons) {
            double result = 0.0;
            for (std::size_t i = 0; i < polygons.Size(); ++i) result += polygons[i].Area(eps);
            return result / (width * width);
        };
        for (const bool reversed : {false, true})
        {
            const auto operand = reversed ? SCPolygon2d(Geometry::Reverse(second.OuterRing())) : second;
            const auto united = Union(first, operand, eps);
            ASSERT_EQ(united.Size(), 1U);
            EXPECT_NEAR(area(united), 1.75, 1e-5);
            const auto reverseUnion = Union(operand, first, eps);
            ASSERT_EQ(reverseUnion.Size(), 1U);
            EXPECT_NEAR(area(reverseUnion), 1.75, 1e-5);
            EXPECT_NEAR(area(Intersect(first, operand, eps)), 0.25, 1e-5);
            EXPECT_NEAR(area(Difference(first, operand, eps)), 0.75, 1e-5);
            EXPECT_NEAR(area(Difference(operand, first, eps)), 0.75, 1e-5);
        }
    }
}
