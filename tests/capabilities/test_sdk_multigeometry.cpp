#include <gtest/gtest.h>
#include <cassert>
#include <cmath>
#include <memory>
#include <vector>

#include "sdk/Geometry.h"

using geometry::Point2d;
using geometry::sdk::ArcSegment2d;
using geometry::sdk::Box2d;
using geometry::sdk::GeometryBoxTree2d;
using geometry::sdk::GeometryKDTree2d;
using geometry::sdk::GeometrySegmentSearch2d;
using geometry::sdk::LineSegment2d;
using geometry::sdk::MultiPolygon2d;
using geometry::sdk::MultiPolyline2d;
using geometry::sdk::Polygon2d;
using geometry::sdk::PolygonContainment2d;
using geometry::sdk::PolygonTopology2d;
using geometry::sdk::Polyline2d;
using geometry::sdk::PolylineClosure;

namespace
{
constexpr double kPi = 3.141592653589793238462643383279502884;
}

TEST(SdkMultigeometryTest, CoversCurrentCapabilities)
{
    const Polyline2d outerRing(
        {Point2d{0.0, 0.0}, Point2d{6.0, 0.0}, Point2d{6.0, 6.0}, Point2d{0.0, 6.0}},
        PolylineClosure::Closed);
    const Polyline2d innerRing(
        {Point2d{1.0, 1.0}, Point2d{2.0, 1.0}, Point2d{2.0, 2.0}, Point2d{1.0, 2.0}},
        PolylineClosure::Closed);
    const Polyline2d siblingRing(
        {Point2d{8.0, 8.0}, Point2d{10.0, 8.0}, Point2d{10.0, 10.0}, Point2d{8.0, 10.0}},
        PolylineClosure::Closed);

    MultiPolyline2d multiPolyline({outerRing, innerRing});
    assert(multiPolyline.Count() == 2);
    assert(multiPolyline.IsValid());
    assert(multiPolyline.PointCount() == outerRing.PointCount() + innerRing.PointCount());
    assert(multiPolyline.Bounds().IsValid());

    const Polygon2d outerPolygon(outerRing);
    const Polygon2d innerPolygon(innerRing);
    const Polygon2d siblingPolygon(siblingRing);
    MultiPolygon2d multiPolygon({outerPolygon, innerPolygon, siblingPolygon});

    assert(multiPolygon.Count() == 3);
    assert(multiPolygon.IsValid());
    assert(multiPolygon.HoleCount() == 0);

    assert(geometry::sdk::ContainsPoint(outerPolygon, Point2d{0.5, 0.5}));
    assert(!geometry::sdk::ContainsPoint(outerPolygon, Point2d{8.5, 8.5}));
    assert(geometry::sdk::Contains(outerPolygon, innerPolygon));
    assert(geometry::sdk::Relate(outerPolygon, innerPolygon) == PolygonContainment2d::FirstContainsSecond);

    PolygonTopology2d topology = geometry::sdk::BuildPolygonTopology(multiPolygon);
    assert(topology.Count() == 3);
    assert(topology.Roots().size() == 2);
    assert(topology.ParentOf(1) == 0);
    assert(topology.ChildrenOf(0).size() == 1);
    assert(topology.IsValid());

    GeometryBoxTree2d boxTree;
    boxTree.Add(1, Box2d::FromMinMax(Point2d{0.0, 0.0}, Point2d{2.0, 2.0}));
    boxTree.Add(2, Box2d::FromMinMax(Point2d{1.0, 1.0}, Point2d{3.0, 3.0}));
    boxTree.Add(3, Box2d::FromMinMax(Point2d{5.0, 5.0}, Point2d{6.0, 6.0}));
    const auto overlaps = boxTree.Query(Box2d::FromMinMax(Point2d{1.5, 1.5}, Point2d{2.5, 2.5}));
    assert(overlaps.size() == 2);
    assert(boxTree.QueryContaining(Point2d{1.0, 1.0}).size() == 2);
    assert(boxTree.Contains(3));

    GeometryKDTree2d kdTree;
    kdTree.Add(1, Point2d{0.0, 0.0});
    kdTree.Add(2, Point2d{1.0, 1.0});
    kdTree.Add(3, Point2d{5.0, 5.0});
    const auto exact = kdTree.Query(Point2d{1.0, 1.0});
    assert(exact.size() == 1 && exact[0] == 2);
    const auto nearest = kdTree.Nearest(Point2d{1.2, 1.2});
    assert(nearest.has_value());
    assert(nearest->id == 2);

    GeometrySegmentSearch2d segmentSearch;
    const std::size_t lineId = segmentSearch.Add(LineSegment2d(Point2d{0.0, 0.0}, Point2d{4.0, 0.0}));
    const std::size_t arcId = segmentSearch.Add(ArcSegment2d(
        Point2d{4.0, 0.0},
        1.0,
        kPi / 2.0,
        kPi / 2.0));
    assert(segmentSearch.Contains(lineId));
    assert(segmentSearch.Contains(arcId));
    assert(segmentSearch.QueryIntersecting(Box2d::FromMinMax(Point2d{10.0, 10.0}, Point2d{11.0, 11.0})).empty());
    const auto nearestSegment = segmentSearch.Nearest(Point2d{2.0, 1.0});
    assert(nearestSegment.has_value());
    assert(nearestSegment->id == lineId);
}



