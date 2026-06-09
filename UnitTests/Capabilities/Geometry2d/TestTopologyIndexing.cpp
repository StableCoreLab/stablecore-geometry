#include <gtest/gtest.h>
#include <memory>

#include "Brep/Topology.h"
#include "Geometry2d/SCBoxTree2d.h"
#include "Geometry2d/SCKDTree2d.h"
#include "Geometry2d/SCSegmentSearch2d.h"
#include "Support/GeometryTestSupport.h"

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

TEST(TopologyIndexingTest, CoversCurrentCapabilities)
{
    SCBoxTree2d boxTree;
    boxTree.Add(1, SCBox2d::FromMinMax(SCPoint2d{0.0, 0.0}, SCPoint2d{2.0, 2.0}));
    boxTree.Add(2, SCBox2d::FromMinMax(SCPoint2d{3.0, 3.0}, SCPoint2d{4.0, 4.0}));
    ASSERT_EQ(boxTree.Query(SCBox2d::FromMinMax(SCPoint2d{1.0, 1.0}, SCPoint2d{3.1, 3.1})).size(), 2);

    SCKDTree2d kdTree;
    kdTree.Add(7, SCPoint2d{1.0, 1.0});
    kdTree.Add(8, SCPoint2d{3.0, 3.0});
    ASSERT_EQ(kdTree.Query(SCPoint2d{1.0, 1.0}).size(), 1);
    ASSERT_EQ(kdTree.Nearest(SCPoint2d{2.9, 3.1})->id, 8);

    SCSegmentSearch2d search;
    const std::size_t lineId = search.Add(SCLineSegment2d(SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}));
    const std::size_t arcId = search.Add(SCArcSegment2d(SCPoint2d{0.0, 0.0}, 2.0, 0.0, 1.0));
    (void)arcId;
    ASSERT_GE(search.QueryWithinDistance(SCPoint2d{2.0, 0.5}, 1.0).size(), 1);
    ASSERT_EQ(search.Nearest(SCPoint2d{2.0, -0.2})->id, lineId);

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
