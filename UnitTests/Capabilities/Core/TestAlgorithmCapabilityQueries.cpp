#include <gtest/gtest.h>

#include <array>
#include <memory>
#include <vector>

#include "Geometry.h"

using Geometry::SCBox3d;
using Geometry::SCGeometryTolerance3d;
using Geometry::SCIntersectionKind2d;
using Geometry::SCIntervald;
using Geometry::SCLine3d;
using Geometry::SCLineCurve3d;
using Geometry::SCLineSegment3d;
using Geometry::SCPoint2d;
using Geometry::SCPoint3d;
using Geometry::SCPointContainment3d;
using Geometry::SCPolyline2d;
using Geometry::SCVector3d;
using Geometry::SCSegmentSearch3d;

TEST(AlgorithmCapabilitiesTest, SegmentSearch3dNearestUsesProvidedTolerance)
{
    SCSegmentSearch3d search;
    const SCLine3d line = SCLine3d::FromOriginAndDirection(SCPoint3d{0.0, 0.0, 0.0}, Geometry::SCVector3d{1e-6, 0.0, 0.0});
    search.Add(SCLineCurve3d::FromLine(line, Geometry::SCIntervald{0.0, 1.0}));

    const auto strictHit = search.Nearest(SCPoint3d{0.0, 1.0, 0.0}, SCGeometryTolerance3d{1e-9, 1e-9, 1e-9});
    ASSERT_TRUE(strictHit.has_value());
    EXPECT_TRUE(strictHit->point.AlmostEquals(SCPoint3d{0.0, 0.0, 0.0}, 1e-12));

    const auto looseHit = search.Nearest(SCPoint3d{0.0, 1.0, 0.0}, SCGeometryTolerance3d{1e-3, 1e-9, 1e-9});
    EXPECT_FALSE(looseHit.has_value());

    SCSegmentSearch3d tinySearch;
    const SCLine3d tinyLine = SCLine3d::FromOriginAndDirection(
        {0.0, 0.0, 0.0}, {1e-10, 0.0, 0.0});
    tinySearch.Add(SCLineCurve3d::FromLine(tinyLine, SCIntervald{0.0, 1.0}));
    EXPECT_FALSE(tinySearch.IsValid());
    const auto tinyHit = tinySearch.Nearest(
        {0.0, 1.0, 0.0}, SCGeometryTolerance3d{1e-12, 1e-12, 1e-12});
    ASSERT_TRUE(tinyHit.has_value());
    EXPECT_EQ(tinyHit->id, 0U);
}

TEST(AlgorithmCapabilitiesTest, SegmentSearch3dUsesBoxIndexForExactQueries)
{
    SCSegmentSearch3d search;
    for (std::size_t id = 0; id < 32; ++id)
    {
        const double x = static_cast<double>(id);
        search.Add(SCLineSegment3d{SCPoint3d{x, 0.0, 0.0}, SCPoint3d{x + 0.4, 0.0, 0.0}});
    }

    const SCPoint3d queryPoint{20.2, 0.3, 0.0};
    const auto nearest = search.Nearest(queryPoint);
    ASSERT_TRUE(nearest.has_value());
    EXPECT_EQ(nearest->id, 20U);
    EXPECT_NEAR(nearest->distanceSquared, 0.09, 1e-12);

    const auto nearestThree = search.QueryKNearest(queryPoint, 3);
    ASSERT_EQ(nearestThree.size(), 3U);
    EXPECT_EQ(nearestThree[0].id, 20U);
    EXPECT_EQ(nearestThree[1].id, 19U);
    EXPECT_EQ(nearestThree[2].id, 21U);

    const auto within = search.QueryWithinDistance(queryPoint, 0.31);
    ASSERT_EQ(within.size(), 1U);
    EXPECT_EQ(within.front().id, 20U);

    const auto intersecting = search.QueryIntersecting(
        SCBox3d::FromMinMax(SCPoint3d{19.9, -0.1, -0.1}, SCPoint3d{20.1, 0.1, 0.1}));
    ASSERT_EQ(intersecting.size(), 1U);
    EXPECT_EQ(intersecting.front(), 20U);

    EXPECT_TRUE(search.Remove(20U));
    const auto nearestAfterRemove = search.Nearest(queryPoint);
    ASSERT_TRUE(nearestAfterRemove.has_value());
    EXPECT_EQ(nearestAfterRemove->id, 19U);
}

TEST(AlgorithmCapabilitiesTest, SegmentSearch3dBoxQueryChecksCurrentCurveBounds)
{
    const SCBox3d queryBox = SCBox3d::FromMinMax({0.5, -0.5, -0.5}, {1.5, 0.5, 0.5});
    const SCBox3d distantBox = SCBox3d::FromMinMax({100.0, 100.0, 100.0},
                                                   {101.0, 101.0, 101.0});
    const SCLineCurve3d curve = SCLineCurve3d::FromLine(
        SCLine3d::FromOriginAndDirection({0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}),
        SCIntervald{0.0, 1.0});
    ASSERT_TRUE(curve.IsValid());

    SCSegmentSearch3d staleBounds({Geometry::SCSegmentSearchEntry3d{
        7, std::make_shared<SCLineCurve3d>(curve), distantBox}});
    EXPECT_EQ(staleBounds.QueryIntersecting(queryBox), std::vector<std::size_t>{7});
    SCSegmentSearch3d missingBounds({Geometry::SCSegmentSearchEntry3d{
        10, std::make_shared<SCLineCurve3d>(curve), SCBox3d{}}});
    EXPECT_FALSE(missingBounds.IsValid());
    EXPECT_EQ(missingBounds.QueryIntersecting(queryBox), std::vector<std::size_t>{10});
    ASSERT_EQ(missingBounds.QueryWithinDistance({1.0, 0.5, 0.0}, 1.0).size(), 1U);

    SCSegmentSearch3d emptyCurve({Geometry::SCSegmentSearchEntry3d{8, {}, queryBox}});
    EXPECT_TRUE(emptyCurve.QueryIntersecting(queryBox).empty());

    const SCLineCurve3d invalidCurve = SCLineCurve3d::FromLine(
        SCLine3d::FromOriginAndDirection({0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}),
        SCIntervald{1.0, 0.0});
    ASSERT_FALSE(invalidCurve.IsValid());
    SCSegmentSearch3d invalidGeometry({Geometry::SCSegmentSearchEntry3d{
        9, std::make_shared<SCLineCurve3d>(invalidCurve), queryBox}});
    EXPECT_FALSE(invalidGeometry.IsValid());
    EXPECT_TRUE(invalidGeometry.QueryIntersecting(queryBox).empty());
}

TEST(AlgorithmCapabilitiesTest, SegmentSearch3dBoxQueryUsesExplicitToleranceForShortCurve)
{
    const SCLineCurve3d shortCurve = SCLineCurve3d::FromLine(
        SCLine3d::FromOriginAndDirection({0.0, 0.0, 0.0}, {1e-10, 0.0, 0.0}),
        SCIntervald{0.0, 1.0});
    ASSERT_FALSE(shortCurve.IsValid());
    SCGeometryTolerance3d tolerance{};
    tolerance.distanceEpsilon = 1e-12;
    ASSERT_TRUE(shortCurve.IsValid(tolerance));
    SCSegmentSearch3d search;
    const std::size_t id = search.Add(shortCurve);
    const SCBox3d box = SCBox3d::FromMinMax({-1e-11, -1e-11, -1e-11},
                                            {1.1e-10, 1e-11, 1e-11});
    EXPECT_EQ(search.QueryIntersecting(box, 1e-12), std::vector<std::size_t>{id});
}

TEST(AlgorithmCapabilitiesTest, ThreeDimensionalPointContainmentHasItsOwnEnum)
{
    const SCLine3d line = SCLine3d::FromOriginAndDirection(SCPoint3d{}, SCVector3d{1.0, 0.0, 0.0});
    const SCLineCurve3d curve = SCLineCurve3d::FromLine(line, SCIntervald{0.0, 1.0});
    const SCPointContainment3d containment = LocatePoint(SCPoint3d{0.5, 0.0, 0.0}, curve);
    EXPECT_EQ(containment, SCPointContainment3d::OnBoundary);
}

TEST(AlgorithmCapabilitiesTest, PolylineIntersectionReturnsSharedCrossing)
{
    const SCPolyline2d first(std::vector<SCPoint2d>{SCPoint2d{0.0, 0.0}, SCPoint2d{2.0, 0.0}, SCPoint2d{4.0, 0.0}});
    const SCPolyline2d second(std::vector<SCPoint2d>{SCPoint2d{2.0, -1.0}, SCPoint2d{2.0, 1.0}});

    const auto intersections = Geometry::Intersect(first, second);
    ASSERT_EQ(intersections.size(), 1U);
    EXPECT_EQ(intersections[0].kind, SCIntersectionKind2d::Point);
    EXPECT_TRUE(intersections[0].point.AlmostEquals(SCPoint2d{2.0, 0.0}, 1e-12));
}

TEST(AlgorithmCapabilitiesTest, SnapPointToSegments3dFindsNearestSegment)
{
    const SCLineSegment3d first{SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{2.0, 0.0, 0.0}};
    const SCLineSegment3d second{SCPoint3d{0.0, 3.0, 0.0}, SCPoint3d{2.0, 3.0, 0.0}};
    const std::array<const SCLineSegment3d*, 2> segments{&first, &second};
    const auto snapped = Geometry::SnapPointToSegments3d(SCPoint3d{1.0, 0.2, 0.0}, segments, 10.0);

    ASSERT_TRUE(snapped.snapped);
    EXPECT_EQ(snapped.segmentIndex, 0U);
    EXPECT_TRUE(snapped.point.AlmostEquals(SCPoint3d{1.0, 0.0, 0.0}, 1e-12));
}

TEST(AlgorithmCapabilitiesTest, SnapPointToCurves3dFindsNearestCurve)
{
    const SCLine3d firstLine = SCLine3d::FromOriginAndDirection({0.0, 0.0, 0.0}, {2.0, 0.0, 0.0});
    const SCLine3d secondLine = SCLine3d::FromOriginAndDirection({0.0, 3.0, 0.0}, {2.0, 0.0, 0.0});
    const SCLineCurve3d first = SCLineCurve3d::FromLine(firstLine, SCIntervald{0.0, 1.0});
    const SCLineCurve3d second = SCLineCurve3d::FromLine(secondLine, SCIntervald{0.0, 1.0});
    const std::array<const Geometry::ISCCurve3d*, 2> curves{&first, &second};
    const auto snapped = Geometry::SnapPointToCurves3d(SCPoint3d{1.0, 0.2, 0.0}, curves, 10.0);

    ASSERT_TRUE(snapped.snapped);
    EXPECT_EQ(snapped.segmentIndex, 0U);
    EXPECT_TRUE(snapped.point.AlmostEquals(SCPoint3d{1.0, 0.0, 0.0}, 1e-12));
}
