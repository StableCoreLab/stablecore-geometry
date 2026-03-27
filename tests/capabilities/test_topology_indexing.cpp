#include <gtest/gtest.h>
#include <cassert>
#include <memory>

#include "sdk/GeometryBoxTree.h"
#include "sdk/GeometryKDTree.h"
#include "sdk/GeometrySegmentSearch.h"
#include "sdk/GeometryTopology.h"
#include "support/GTestCompat.h"
#include "support/GeometryTestSupport.h"

using geometry::sdk::ArcSegment2d;
using geometry::sdk::Box2d;
using geometry::sdk::GeometryBoxTree2d;
using geometry::sdk::GeometryKDTree2d;
using geometry::sdk::GeometrySegmentSearch2d;
using geometry::sdk::LineSegment2d;
using geometry::sdk::MultiPolygon2d;
using geometry::sdk::Point2d;
using geometry::sdk::Polygon2d;
using geometry::sdk::PolygonContainment2d;
using geometry::sdk::Polyline2d;
using geometry::sdk::PolylineClosure;

TEST(TopologyIndexingTest, CoversCurrentCapabilities)
{
    GeometryBoxTree2d boxTree;
    boxTree.Add(1, Box2d::FromMinMax(Point2d{0.0, 0.0}, Point2d{2.0, 2.0}));
    boxTree.Add(2, Box2d::FromMinMax(Point2d{3.0, 3.0}, Point2d{4.0, 4.0}));
    assert(boxTree.Query(Box2d::FromMinMax(Point2d{1.0, 1.0}, Point2d{3.1, 3.1})).size() == 2);

    GeometryKDTree2d kdTree;
    kdTree.Add(7, Point2d{1.0, 1.0});
    kdTree.Add(8, Point2d{3.0, 3.0});
    assert(kdTree.Query(Point2d{1.0, 1.0}).size() == 1);
    assert(kdTree.Nearest(Point2d{2.9, 3.1})->id == 8);

    GeometrySegmentSearch2d search;
    const std::size_t lineId = search.Add(LineSegment2d(Point2d{0.0, 0.0}, Point2d{4.0, 0.0}));
    const std::size_t arcId = search.Add(ArcSegment2d(Point2d{0.0, 0.0}, 2.0, 0.0, 1.0));
    (void)arcId;
    assert(search.QueryWithinDistance(Point2d{2.0, 0.5}, 1.0).size() >= 1);
    assert(search.Nearest(Point2d{2.0, -0.2})->id == lineId);

    const Polygon2d outer(
        Polyline2d(
            {Point2d{0.0, 0.0}, Point2d{10.0, 0.0}, Point2d{10.0, 10.0}, Point2d{0.0, 10.0}},
            PolylineClosure::Closed));
    const Polygon2d inner(
        Polyline2d(
            {Point2d{2.0, 2.0}, Point2d{4.0, 2.0}, Point2d{4.0, 4.0}, Point2d{2.0, 4.0}},
            PolylineClosure::Closed));
    assert(geometry::sdk::Relate(outer, inner) == PolygonContainment2d::FirstContainsSecond);
    const auto topology = geometry::sdk::BuildPolygonTopology(MultiPolygon2d{{outer, inner}});
    assert(topology.Roots().size() == 1); const auto duplicateTopology = geometry::sdk::BuildPolygonTopology(MultiPolygon2d{outer, outer}); assert(duplicateTopology.Roots().size() == 1); assert(duplicateTopology.ParentOf(1) == 0);

    const Polygon2d touching(
        Polyline2d(
            {Point2d{10.0, 2.0}, Point2d{12.0, 2.0}, Point2d{12.0, 4.0}, Point2d{10.0, 4.0}},
            PolylineClosure::Closed));
    const PolygonContainment2d touchingRelation = geometry::sdk::Relate(outer, touching);
    assert(
        touchingRelation == PolygonContainment2d::Touching ||
        touchingRelation == PolygonContainment2d::Intersecting);
    assert(geometry::sdk::Relate(outer, outer) == PolygonContainment2d::Equal);
    assert(geometry::sdk::Contains(outer, outer));

    const Polygon2d intersecting(
        Polyline2d(
            {Point2d{8.0, 8.0}, Point2d{12.0, 8.0}, Point2d{12.0, 12.0}, Point2d{8.0, 12.0}},
            PolylineClosure::Closed));
    assert(geometry::sdk::Relate(outer, intersecting) == PolygonContainment2d::Intersecting);
}


