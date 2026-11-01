#include <gtest/gtest.h>
#include <limits>
#include <memory>

#include "Core/PolygonRelation2d.h"
#include "Geometry2d/SCBoxTree2d.h"
#include "Geometry2d/SCKDTree2d.h"
#include "Geometry2d/SCSegmentSearch2d.h"

using Geometry::SCArcSegment2d;
using Geometry::SCBox2d;
using Geometry::SCBoxTree2d;
using Geometry::SCKDTree2d;
using Geometry::SCLineSegment2d;
using Geometry::SCMultiPolygon2d;
using Geometry::SCPoint2d;
using Geometry::SCPolygon2d;
using Geometry::PolygonContainment2d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;
using Geometry::SCSegmentSearch2d;

TEST(TopologyIndexingTest, BoxTreeQueriesOverlappingBoxes)
{
    SCBoxTree2d boxTree;
    boxTree.Add(1, SCBox2d::FromMinMax(SCPoint2d{0.0, 0.0}, SCPoint2d{2.0, 2.0}));
    boxTree.Add(2, SCBox2d::FromMinMax(SCPoint2d{3.0, 3.0}, SCPoint2d{4.0, 4.0}));
    ASSERT_EQ(boxTree.Query(SCBox2d::FromMinMax(SCPoint2d{1.0, 1.0}, SCPoint2d{3.1, 3.1})).size(), 2);
}

TEST(TopologyIndexingTest, KdTreeQueriesAndFindsNearestPoint)
{
    SCKDTree2d kdTree;
    kdTree.Add(7, SCPoint2d{1.0, 1.0});
    kdTree.Add(8, SCPoint2d{3.0, 3.0});
    ASSERT_EQ(kdTree.Query(SCPoint2d{1.0, 1.0}).size(), 1);
    ASSERT_EQ(kdTree.Nearest(SCPoint2d{2.9, 3.1})->id, 8);
}

TEST(TopologyIndexingTest, SegmentSearchQueriesAndFindsNearestSegment)
{
    SCSegmentSearch2d search;
    const std::size_t lineId = search.Add(SCLineSegment2d(SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}));
    const std::size_t arcId = search.Add(SCArcSegment2d(SCPoint2d{0.0, 0.0}, 2.0, 0.0, 1.0));
    (void)arcId;
    ASSERT_GE(search.QueryWithinDistance(SCPoint2d{2.0, 0.5}, 1.0).size(), 1);
    ASSERT_EQ(search.Nearest(SCPoint2d{2.0, -0.2})->id, lineId);
}

TEST(TopologyIndexingTest, SegmentSearchKeepsInvalidEntriesOutOfQueries)
{
    SCSegmentSearch2d search;
    const std::size_t invalidId = search.Add(std::shared_ptr<const Geometry::ISCSegment2d>{});
    const SCLineSegment2d line({0.0, 0.0}, {2.0, 0.0});
    const std::size_t lineId = search.Add(line);
    ASSERT_TRUE(search.Contains(invalidId));
    ASSERT_NE(search.Find(invalidId), nullptr);
    EXPECT_FALSE(search.IsValid());

    const SCBox2d box = SCBox2d::FromMinMax({0.5, -0.5}, {1.5, 0.5});
    EXPECT_EQ(search.QueryIntersecting(box), std::vector<std::size_t>{lineId});
    EXPECT_EQ(search.QueryIntersecting(line), std::vector<std::size_t>{lineId});
    ASSERT_EQ(search.QueryWithinDistance({1.0, 0.5}, 1.0).size(), 1U);
    EXPECT_TRUE(search.QueryWithinDistance({1.0, 0.5}, -1.0).empty());
    ASSERT_TRUE(search.Nearest({1.0, 0.5}).has_value());
    EXPECT_EQ(search.Nearest({1.0, 0.5})->id, lineId);
    ASSERT_EQ(search.QueryKNearest({1.0, 0.5}, 2).size(), 1U);

    SCSegmentSearch2d stale({Geometry::SCSegmentSearchEntry2d{42, {}, box}});
    EXPECT_TRUE(stale.QueryIntersecting(box).empty());
    EXPECT_TRUE(stale.QueryIntersecting(line).empty());
    SCSegmentSearch2d staleBounds({Geometry::SCSegmentSearchEntry2d{
        43, std::make_shared<SCLineSegment2d>(line),
        SCBox2d::FromMinMax({100.0, 100.0}, {101.0, 101.0})}});
    EXPECT_EQ(staleBounds.QueryIntersecting(box), std::vector<std::size_t>{43});

    EXPECT_TRUE(search.Remove(invalidId));
    EXPECT_TRUE(search.IsValid());
}

TEST(TopologyIndexingTest, SegmentSearchBoxQueryUsesExplicitToleranceForShortSegment)
{
    const SCLineSegment2d shortLine({0.0, 0.0}, {1e-10, 0.0});
    ASSERT_FALSE(shortLine.IsValid());
    ASSERT_TRUE(shortLine.IsValid(1e-12));
    SCSegmentSearch2d search;
    const std::size_t id = search.Add(shortLine);
    const SCBox2d box = SCBox2d::FromMinMax({-1e-11, -1e-11}, {1.1e-10, 1e-11});
    EXPECT_EQ(search.QueryIntersecting(box, 1e-12), std::vector<std::size_t>{id});
    const SCLineSegment2d crossing({5e-11, -5e-11}, {5e-11, 5e-11});
    EXPECT_EQ(search.QueryIntersecting(crossing, 1e-12), std::vector<std::size_t>{id});
}

TEST(TopologyIndexingTest, PolygonRelationsAndTopologyAreConsistent)
{
    const SCPolygon2d outer(SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{10.0, 0.0}, SCPoint2d{10.0, 10.0}, SCPoint2d{0.0, 10.0}},
                                     SCPolylineClosure::Closed));
    const SCPolygon2d inner(SCPolyline2d({SCPoint2d{2.0, 2.0}, SCPoint2d{4.0, 2.0}, SCPoint2d{4.0, 4.0}, SCPoint2d{2.0, 4.0}},
                                     SCPolylineClosure::Closed));
    ASSERT_EQ(Relate(outer, inner), PolygonContainment2d::FirstContainsSecond);
    ASSERT_EQ(Relate(inner, outer), PolygonContainment2d::SecondContainsFirst);
    const auto topology = BuildPolygonTopology(SCMultiPolygon2d{{outer, inner}});
    ASSERT_EQ(topology.Roots().size(), 1);
    const auto duplicateTopology = BuildPolygonTopology(SCMultiPolygon2d{outer, outer});
    ASSERT_EQ(duplicateTopology.Roots().size(), 1);
    ASSERT_EQ(duplicateTopology.ParentOf(1), 0);

    const SCPolygon2d touching(SCPolyline2d(
        {SCPoint2d{10.0, 2.0}, SCPoint2d{12.0, 2.0}, SCPoint2d{12.0, 4.0}, SCPoint2d{10.0, 4.0}}, SCPolylineClosure::Closed));
    const PolygonContainment2d touchingRelation = Relate(outer, touching);
    ASSERT_TRUE(touchingRelation == PolygonContainment2d::Touching ||
                touchingRelation == PolygonContainment2d::Intersecting);
    ASSERT_EQ(Relate(outer, outer), PolygonContainment2d::Equal);
    ASSERT_TRUE(Contains(outer, outer));

    const SCPolygon2d intersecting(SCPolyline2d(
        {SCPoint2d{8.0, 8.0}, SCPoint2d{12.0, 8.0}, SCPoint2d{12.0, 12.0}, SCPoint2d{8.0, 12.0}}, SCPolylineClosure::Closed));
    ASSERT_EQ(Relate(outer, intersecting), PolygonContainment2d::Intersecting);

    const SCPolygon2d sharedEdgeTouch(SCPolyline2d(
        {SCPoint2d{10.0, 0.0}, SCPoint2d{12.0, 0.0}, SCPoint2d{12.0, 10.0}, SCPoint2d{10.0, 10.0}}, SCPolylineClosure::Closed));
    ASSERT_EQ(Relate(outer, sharedEdgeTouch), PolygonContainment2d::Touching);

    const SCPolygon2d overlapWithInterior(SCPolyline2d(
        {SCPoint2d{8.0, 0.0}, SCPoint2d{12.0, 0.0}, SCPoint2d{12.0, 6.0}, SCPoint2d{8.0, 6.0}}, SCPolylineClosure::Closed));
    ASSERT_EQ(Relate(outer, overlapWithInterior), PolygonContainment2d::Intersecting);

    const SCPolygon2d sameOuterWithHole(
        SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{10.0, 0.0}, SCPoint2d{10.0, 10.0}, SCPoint2d{0.0, 10.0}},
                   SCPolylineClosure::Closed),
        {SCPolyline2d({SCPoint2d{3.0, 3.0}, SCPoint2d{3.0, 7.0}, SCPoint2d{7.0, 7.0}, SCPoint2d{7.0, 3.0}},
                    SCPolylineClosure::Closed)});
    ASSERT_EQ(Relate(outer, sameOuterWithHole), PolygonContainment2d::FirstContainsSecond);

    const SCPolygon2d sameOuterWithDifferentHole(
        SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{10.0, 0.0}, SCPoint2d{10.0, 10.0}, SCPoint2d{0.0, 10.0}},
                   SCPolylineClosure::Closed),
        {SCPolyline2d({SCPoint2d{1.0, 1.0}, SCPoint2d{1.0, 2.0}, SCPoint2d{2.0, 2.0}, SCPoint2d{2.0, 1.0}},
                    SCPolylineClosure::Closed)});
    ASSERT_EQ(Relate(sameOuterWithHole, sameOuterWithDifferentHole), PolygonContainment2d::Intersecting);

    const SCPolygon2d boundaryOverlappingContained(SCPolyline2d(
        {SCPoint2d{2.0, 0.0}, SCPoint2d{6.0, 0.0}, SCPoint2d{6.0, 10.0}, SCPoint2d{2.0, 10.0}}, SCPolylineClosure::Closed));
    ASSERT_EQ(Relate(outer, boundaryOverlappingContained), PolygonContainment2d::FirstContainsSecond);

    const SCPolygon2d noisyOuter(SCPolyline2d({SCPoint2d{0.0, 0.0},
                                           SCPoint2d{10.0, 0.0},
                                           SCPoint2d{10.0, 5.0},
                                           SCPoint2d{10.0, 5.000000001},
                                           SCPoint2d{10.0, 10.0},
                                           SCPoint2d{0.0, 10.0},
                                           SCPoint2d{0.0, 5.000000001},
                                           SCPoint2d{0.0, 5.0}},
                                          SCPolylineClosure::Closed));
    const SCPolygon2d smallInner(SCPolyline2d({SCPoint2d{2.0, 2.0}, SCPoint2d{3.0, 2.0}, SCPoint2d{3.0, 3.0}, SCPoint2d{2.0, 3.0}},
                                          SCPolylineClosure::Closed));
    const auto normalizedTopology = BuildPolygonTopology(SCMultiPolygon2d{noisyOuter, smallInner});
    ASSERT_EQ(normalizedTopology.Roots().size(), 1);
    ASSERT_EQ(normalizedTopology.ParentOf(1), 0);
    ASSERT_EQ(Relate(noisyOuter, smallInner), PolygonContainment2d::FirstContainsSecond);
}

TEST(TopologyIndexingTest, SmallConcaveHolePreventsFalseContainment)
{
    constexpr double eps = 1e-15;
    constexpr double width = 1e-10;
    constexpr double origin = 2e-8;
    const SCPolyline2d outerRing(
        {{0.0, 0.0}, {1e-7, 0.0}, {1e-7, 1e-7}, {0.0, 1e-7}},
        SCPolylineClosure::Closed);
    const SCPolyline2d holeRing(
        {{origin, origin}, {origin + 4 * width, origin},
         {origin + 4 * width, origin + 4 * width},
         {origin + 3 * width, origin + 4 * width},
         {origin + 3 * width, origin + width},
         {origin + width, origin + width},
         {origin + width, origin + 4 * width},
         {origin, origin + 4 * width}},
        SCPolylineClosure::Closed);
    ASSERT_FALSE(holeRing.IsValid());
    ASSERT_TRUE(holeRing.IsValid(eps));

    const SCPolygon2d solid(outerRing);
    const SCPolygon2d withHole(outerRing, {holeRing});
    ASSERT_TRUE(solid.IsValid(eps));
    ASSERT_TRUE(withHole.IsValid(eps));
    EXPECT_FALSE(Geometry::Contains(withHole, solid, eps));
    EXPECT_EQ(Geometry::Relate(withHole, solid, eps), PolygonContainment2d::SecondContainsFirst);
}

TEST(TopologyIndexingTest, TopologyBuildRejectsInvalidToleranceAndClearsPreviousResult)
{
    const SCPolygon2d square(SCPolyline2d(
        {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}},
        SCPolylineClosure::Closed));
    ASSERT_TRUE(square.IsValid());
    Geometry::PolygonTopology2d topology;
    for (double eps : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN()})
    {
        ASSERT_TRUE(topology.Build(SCMultiPolygon2d{square}));
        ASSERT_EQ(topology.Count(), 1U);
        EXPECT_FALSE(topology.Build(SCMultiPolygon2d{square}, eps));
        EXPECT_TRUE(topology.IsEmpty());
        EXPECT_TRUE(topology.Roots().empty());
    }
}
