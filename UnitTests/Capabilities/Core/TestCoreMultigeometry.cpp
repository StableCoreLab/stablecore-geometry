#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <vector>

#include "Geometry.h"

using Geometry::SCArcSegment2d;
using Geometry::SCBox2d;
using Geometry::SCBoxTree2d;
using Geometry::SCKDTree2d;
using Geometry::SCLineSegment2d;
using Geometry::SCMultiPolygon2d;
using Geometry::SCMultiPolyline2d;
using Geometry::SCPoint2d;
using Geometry::SCPolygon2d;
using Geometry::PolygonContainment2d;
using Geometry::PolygonTopology2d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;
using Geometry::SCSegmentSearch2d;

namespace
{
    constexpr double kPi = 3.141592653589793238462643383279502884;
}

TEST(CoreMultiGeometryTest, CoversCurrentCapabilities)
{
    const SCPolyline2d outerRing({SCPoint2d{0.0, 0.0}, SCPoint2d{6.0, 0.0}, SCPoint2d{6.0, 6.0}, SCPoint2d{0.0, 6.0}},
                               SCPolylineClosure::Closed);
    const SCPolyline2d innerRing({SCPoint2d{1.0, 1.0}, SCPoint2d{2.0, 1.0}, SCPoint2d{2.0, 2.0}, SCPoint2d{1.0, 2.0}},
                               SCPolylineClosure::Closed);
    const SCPolyline2d siblingRing({SCPoint2d{8.0, 8.0}, SCPoint2d{10.0, 8.0}, SCPoint2d{10.0, 10.0}, SCPoint2d{8.0, 10.0}},
                                 SCPolylineClosure::Closed);

    SCMultiPolyline2d multiPolyline({outerRing, innerRing});
    ASSERT_EQ(multiPolyline.Count(), 2);
    ASSERT_TRUE(multiPolyline.IsValid());
    ASSERT_EQ(multiPolyline.PointCount(), outerRing.PointCount() + innerRing.PointCount());
    ASSERT_TRUE(multiPolyline.Bounds().IsValid());

    const SCPolygon2d outerPolygon(outerRing);
    const SCPolygon2d innerPolygon(innerRing);
    const SCPolygon2d siblingPolygon(siblingRing);
    SCMultiPolygon2d multiPolygon({outerPolygon, innerPolygon, siblingPolygon});

    ASSERT_EQ(multiPolygon.Count(), 3);
    ASSERT_TRUE(multiPolygon.IsValid());
    ASSERT_EQ(multiPolygon.HoleCount(), 0);

    ASSERT_TRUE(ContainsPoint(outerPolygon, SCPoint2d{0.5, 0.5}));
    ASSERT_FALSE(ContainsPoint(outerPolygon, SCPoint2d{8.5, 8.5}));
    ASSERT_TRUE(Contains(outerPolygon, innerPolygon));
    ASSERT_EQ(Relate(outerPolygon, innerPolygon), PolygonContainment2d::FirstContainsSecond);

    PolygonTopology2d topology = BuildPolygonTopology(multiPolygon);
    ASSERT_EQ(topology.Count(), 3);
    ASSERT_EQ(topology.Roots().size(), 2);
    ASSERT_EQ(topology.ParentOf(1), 0);
    ASSERT_EQ(topology.ChildrenOf(0).size(), 1);
    ASSERT_TRUE(topology.IsValid());

    SCBoxTree2d boxTree;
    boxTree.Add(1, SCBox2d::FromMinMax(SCPoint2d{0.0, 0.0}, SCPoint2d{2.0, 2.0}));
    boxTree.Add(2, SCBox2d::FromMinMax(SCPoint2d{1.0, 1.0}, SCPoint2d{3.0, 3.0}));
    boxTree.Add(3, SCBox2d::FromMinMax(SCPoint2d{5.0, 5.0}, SCPoint2d{6.0, 6.0}));
    const auto overlaps = boxTree.Query(SCBox2d::FromMinMax(SCPoint2d{1.5, 1.5}, SCPoint2d{2.5, 2.5}));
    ASSERT_EQ(overlaps.size(), 2);
    ASSERT_EQ(boxTree.QueryContaining(SCPoint2d{1.0, 1.0}).size(), 2);
    ASSERT_TRUE(boxTree.Contains(3));

    SCKDTree2d kdTree;
    kdTree.Add(1, SCPoint2d{0.0, 0.0});
    kdTree.Add(2, SCPoint2d{1.0, 1.0});
    kdTree.Add(3, SCPoint2d{5.0, 5.0});
    const auto exact = kdTree.Query(SCPoint2d{1.0, 1.0});
    ASSERT_EQ(exact.size(), 1);
    ASSERT_EQ(exact[0], 2);
    const auto nearest = kdTree.Nearest(SCPoint2d{1.2, 1.2});
    ASSERT_TRUE(nearest.has_value());
    ASSERT_EQ(nearest->id, 2);

    SCSegmentSearch2d segmentSearch;
    const std::size_t lineId = segmentSearch.Add(SCLineSegment2d(SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}));
    const std::size_t arcId = segmentSearch.Add(SCArcSegment2d(SCPoint2d{4.0, 0.0}, 1.0, kPi / 2.0, kPi / 2.0));
    ASSERT_TRUE(segmentSearch.Contains(lineId));
    ASSERT_TRUE(segmentSearch.Contains(arcId));
    ASSERT_TRUE(segmentSearch.QueryIntersecting(SCBox2d::FromMinMax(SCPoint2d{10.0, 10.0}, SCPoint2d{11.0, 11.0})).empty());
    const auto nearestSegment = segmentSearch.Nearest(SCPoint2d{2.0, 1.0});
    ASSERT_TRUE(nearestSegment.has_value());
    ASSERT_EQ(nearestSegment->id, lineId);
}
