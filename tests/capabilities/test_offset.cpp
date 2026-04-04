#include <gtest/gtest.h>
#include <cassert>
#include <cmath>
#include <memory>
#include <vector>

#include "sdk/GeometryOffset.h"
#include "sdk/GeometryRelation.h"
#include "sdk/GeometryShapeOps.h"
#include "support/GTestCompat.h"
#include "support/GeometryTestSupport.h"

using geometry::sdk::ArcSegment2d;
using geometry::sdk::LineSegment2d;
using geometry::sdk::MultiPolygon2d;
using geometry::sdk::Point2d;
using geometry::sdk::PointContainment2d;
using geometry::sdk::Polygon2d;
using geometry::sdk::Polyline2d;
using geometry::sdk::PolylineClosure;

TEST(OffsetTest, CoversCurrentCapabilities)
{
    const LineSegment2d line(Point2d{0.0, 0.0}, Point2d{4.0, 0.0});
    const LineSegment2d shiftedLine = geometry::sdk::Offset(line, 2.0);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(shiftedLine.startPoint, (Point2d{0.0, 2.0}), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(shiftedLine.endPoint, (Point2d{4.0, 2.0}), 1e-12);

    const ArcSegment2d ccwArc(
        Point2d{0.0, 0.0},
        5.0,
        0.0,
        std::acos(-1.0) * 0.5);
    const ArcSegment2d insetArc = geometry::sdk::Offset(ccwArc, 1.0);
    GEOMETRY_TEST_ASSERT_NEAR(insetArc.radius, 4.0, 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(insetArc.StartPoint(), (Point2d{4.0, 0.0}), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(insetArc.EndPoint(), (Point2d{0.0, 4.0}), 1e-12);

    const Polyline2d openPath(
        {Point2d{0.0, 0.0}, Point2d{4.0, 0.0}, Point2d{4.0, 3.0}},
        PolylineClosure::Open);
    const Polyline2d openOffset = geometry::sdk::Offset(openPath, 1.0);
    assert(openOffset.IsValid());
    assert(openOffset.PointCount() == 3);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(openOffset.PointAt(0), (Point2d{0.0, 1.0}), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(openOffset.PointAt(1), (Point2d{3.0, 1.0}), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(openOffset.PointAt(2), (Point2d{3.0, 3.0}), 1e-12);

    const Polyline2d ccwRing(
        {Point2d{0.0, 0.0}, Point2d{4.0, 0.0}, Point2d{4.0, 4.0}, Point2d{0.0, 4.0}},
        PolylineClosure::Closed);
    const Polygon2d polygon(ccwRing);
    const Polygon2d grownPolygon = geometry::sdk::Offset(polygon, 1.0);
    assert(grownPolygon.IsValid());
    GEOMETRY_TEST_ASSERT_POINT_NEAR(grownPolygon.OuterRing().PointAt(0), (Point2d{-1.0, -1.0}), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(grownPolygon.OuterRing().PointAt(2), (Point2d{5.0, 5.0}), 1e-12);

    const Polyline2d holeRing(
        {Point2d{1.0, 1.0}, Point2d{1.0, 3.0}, Point2d{3.0, 3.0}, Point2d{3.0, 1.0}},
        PolylineClosure::Closed);
    const Polygon2d polygonWithHole(ccwRing, {holeRing});
    const Polygon2d offsetWithHole = geometry::sdk::Offset(polygonWithHole, 0.25);
    assert(offsetWithHole.IsValid());
    assert(offsetWithHole.Area() > polygonWithHole.Area());
    GEOMETRY_TEST_ASSERT_POINT_NEAR(offsetWithHole.OuterRing().PointAt(0), (Point2d{-0.25, -0.25}), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(offsetWithHole.OuterRing().PointAt(2), (Point2d{4.25, 4.25}), 1e-12);

    const Polygon2d concave(
        Polyline2d(
            {Point2d{0.0, 0.0}, Point2d{5.0, 0.0}, Point2d{5.0, 1.0}, Point2d{2.0, 1.0}, Point2d{2.0, 4.0}, Point2d{0.0, 4.0}},
            PolylineClosure::Closed));
    const Polygon2d concaveOffset = geometry::sdk::Offset(concave, 0.5);
    assert(concaveOffset.IsValid());
    assert(concaveOffset.Area() > concave.Area());

    const MultiPolygon2d disjoint{
        Polygon2d(Polyline2d(
            {Point2d{0.0, 0.0}, Point2d{2.0, 0.0}, Point2d{2.0, 2.0}, Point2d{0.0, 2.0}},
            PolylineClosure::Closed)),
        Polygon2d(Polyline2d(
            {Point2d{4.0, 0.0}, Point2d{6.0, 0.0}, Point2d{6.0, 2.0}, Point2d{4.0, 2.0}},
            PolylineClosure::Closed))};
    const MultiPolygon2d expandedDisjoint = geometry::sdk::Offset(disjoint, 0.5);
    assert(expandedDisjoint.Count() == 2);

    const Polygon2d clockwiseOuter(
        Polyline2d(
            {Point2d{0.0, 0.0}, Point2d{0.0, 4.0}, Point2d{4.0, 4.0}, Point2d{4.0, 0.0}},
            PolylineClosure::Closed));
    const Polygon2d clockwiseExpanded = geometry::sdk::Offset(clockwiseOuter, 0.5);
    assert(clockwiseExpanded.IsValid());
    assert(clockwiseExpanded.Area() > clockwiseOuter.Area());

    const Polygon2d narrowDonut(
        Polyline2d(
            {Point2d{0.0, 0.0}, Point2d{10.0, 0.0}, Point2d{10.0, 6.0}, Point2d{0.0, 6.0}},
            PolylineClosure::Closed),
        {Polyline2d(
            {Point2d{3.5, 2.0}, Point2d{3.5, 4.0}, Point2d{6.5, 4.0}, Point2d{6.5, 2.0}},
            PolylineClosure::Closed)});
    const Polygon2d inwardRecovered = geometry::sdk::Offset(narrowDonut, -0.75);
    assert(inwardRecovered.IsValid());
    assert(inwardRecovered.Area() < narrowDonut.Area());
    for (std::size_t i = 0; i < inwardRecovered.OuterRing().PointCount(); ++i)
    {
        const PointContainment2d containment =
            geometry::sdk::LocatePoint(inwardRecovered.OuterRing().PointAt(i), narrowDonut);
        assert(containment != PointContainment2d::Outside);
    }

    const MultiPolygon2d multiWithHole{
        narrowDonut,
        Polygon2d(
            Polyline2d(
                {Point2d{14.0, 0.0}, Point2d{20.0, 0.0}, Point2d{20.0, 5.0}, Point2d{14.0, 5.0}},
                PolylineClosure::Closed))};
    const MultiPolygon2d inwardMultiRecovered = geometry::sdk::Offset(multiWithHole, -0.8);
    assert(!inwardMultiRecovered.IsEmpty());
    for (std::size_t polygonIndex = 0; polygonIndex < inwardMultiRecovered.Count(); ++polygonIndex)
    {
        const Polygon2d& recovered = inwardMultiRecovered[polygonIndex];
        for (std::size_t pointIndex = 0; pointIndex < recovered.OuterRing().PointCount(); ++pointIndex)
        {
            const Point2d point = recovered.OuterRing().PointAt(pointIndex);
            const bool inFirst = geometry::sdk::LocatePoint(point, narrowDonut) != PointContainment2d::Outside;
            const bool inSecond =
                geometry::sdk::LocatePoint(point, multiWithHole[1]) != PointContainment2d::Outside;
            assert(inFirst || inSecond);
        }
    }
}

TEST(OffsetTest, PreservesSinglePolygonHoleSemanticsAfterRebuild)
{
    const Polygon2d source(
        Polyline2d(
            {Point2d{0.0, 0.0}, Point2d{4.0, 0.0}, Point2d{4.0, 4.0}, Point2d{0.0, 4.0}},
            PolylineClosure::Closed),
        {Polyline2d(
            {Point2d{1.0, 1.0}, Point2d{1.0, 3.0}, Point2d{3.0, 3.0}, Point2d{3.0, 1.0}},
            PolylineClosure::Closed)});

    const Polygon2d outward = geometry::sdk::Offset(source, 0.5);
    assert(outward.IsValid());
    assert(outward.HoleCount() == 1);
    assert(outward.Area() > source.Area());
    GEOMETRY_TEST_ASSERT_POINT_NEAR(outward.OuterRing().PointAt(0), (Point2d{-0.5, -0.5}), 1e-9);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(outward.OuterRing().PointAt(2), (Point2d{4.5, 4.5}), 1e-9);

    const Polyline2d recoveredHole = outward.HoleAt(0);
    GEOMETRY_TEST_ASSERT_NEAR(recoveredHole.Bounds().MinPoint().x, 1.5, 1e-9);
    GEOMETRY_TEST_ASSERT_NEAR(recoveredHole.Bounds().MinPoint().y, 1.5, 1e-9);
    GEOMETRY_TEST_ASSERT_NEAR(recoveredHole.Bounds().MaxPoint().x, 2.5, 1e-9);
    GEOMETRY_TEST_ASSERT_NEAR(recoveredHole.Bounds().MaxPoint().y, 2.5, 1e-9);

    const Polygon2d inward = geometry::sdk::Offset(source, -0.4);
    assert(inward.IsValid());
    assert(inward.HoleCount() == 1);
    assert(inward.Area() < source.Area());
    for (std::size_t i = 0; i < inward.OuterRing().PointCount(); ++i)
    {
        assert(geometry::sdk::LocatePoint(inward.OuterRing().PointAt(i), source) != PointContainment2d::Outside);
    }
}

TEST(OffsetTest, RecoversRepresentativeReverseEdgeSelfIntersectionCase)
{
    const Polygon2d source(
        Polyline2d(
            {Point2d{0.0, 0.0}, Point2d{6.0, 0.0}, Point2d{6.0, 1.0}, Point2d{2.5, 1.0}, Point2d{2.5, 4.0}, Point2d{6.0, 4.0},
             Point2d{6.0, 5.0}, Point2d{0.0, 5.0}},
            PolylineClosure::Closed));

    const Polygon2d outward = geometry::sdk::Offset(source, 0.5);
    assert(outward.IsValid());
    assert(outward.Area() > source.Area());

    const Polygon2d inward = geometry::sdk::Offset(source, -0.4);
    assert(inward.IsValid());
    assert(inward.Area() < source.Area());
    for (std::size_t i = 0; i < inward.OuterRing().PointCount(); ++i)
    {
        assert(geometry::sdk::LocatePoint(inward.OuterRing().PointAt(i), source) != PointContainment2d::Outside);
    }
}

TEST(OffsetTest, SupportsNarrowBridgeSplitViaMultiPolygonOffsetApi)
{
    const Polygon2d source(
        Polyline2d(
            {Point2d{0.0, 0.0}, Point2d{2.0, 0.0}, Point2d{2.0, 0.8}, Point2d{4.0, 0.8}, Point2d{4.0, 0.0}, Point2d{6.0, 0.0},
             Point2d{6.0, 2.0}, Point2d{4.0, 2.0}, Point2d{4.0, 1.2}, Point2d{2.0, 1.2}, Point2d{2.0, 2.0}, Point2d{0.0, 2.0}},
            PolylineClosure::Closed));

    const MultiPolygon2d split = geometry::sdk::OffsetToMultiPolygon(source, -0.35);
    assert(!split.IsEmpty());
    assert(split.Count() >= 2);
    for (std::size_t polygonIndex = 0; polygonIndex < split.Count(); ++polygonIndex)
    {
        const Polygon2d& piece = split[polygonIndex];
        assert(piece.IsValid());
        for (std::size_t pointIndex = 0; pointIndex < piece.OuterRing().PointCount(); ++pointIndex)
        {
            assert(geometry::sdk::LocatePoint(piece.OuterRing().PointAt(pointIndex), source) != PointContainment2d::Outside);
        }
    }
}


