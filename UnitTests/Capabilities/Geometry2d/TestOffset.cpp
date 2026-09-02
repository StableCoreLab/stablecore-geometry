#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include "Core/Offset.h"
#include "Core/Relation.h"
#include "Core/ShapeOps.h"
#include "Support/Epsilon.h"

using Geometry::SCArcSegment2d;
using Geometry::SCLineSegment2d;
using Geometry::SCMultiPolygon2d;
using Geometry::SCPoint2d;
using Geometry::SCPointContainment2d;
using Geometry::SCPolygon2d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;

TEST(OffsetTest, OffsetsLineSegment)
{
    const SCLineSegment2d line(SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0});
    const SCLineSegment2d shiftedLine = Offset(line, 2.0);
    EXPECT_NEAR(shiftedLine.startPoint.x, 0.0, 1e-12);
    EXPECT_NEAR(shiftedLine.startPoint.y, 2.0, 1e-12);
    EXPECT_NEAR(shiftedLine.endPoint.x, 4.0, 1e-12);
    EXPECT_NEAR(shiftedLine.endPoint.y, 2.0, 1e-12);

}

TEST(OffsetTest, OffsetsLineAndArcSegments)
{
    const SCArcSegment2d ccwArc(SCPoint2d{0.0, 0.0}, 5.0, 0.0, Geometry::kPi * 0.5);
    const SCArcSegment2d insetArc = Offset(ccwArc, 1.0);
    EXPECT_NEAR(insetArc.radius, 4.0, 1e-12);
    const SCPoint2d insetStart = insetArc.StartPoint();
    EXPECT_NEAR(insetStart.x, 4.0, 1e-12);
    EXPECT_NEAR(insetStart.y, 0.0, 1e-12);
    const SCPoint2d insetEnd = insetArc.EndPoint();
    EXPECT_NEAR(insetEnd.x, 0.0, 1e-12);
    EXPECT_NEAR(insetEnd.y, 4.0, 1e-12);

    const SCPolyline2d openPath({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 3.0}}, SCPolylineClosure::Open);
    const SCPolyline2d openOffset = Offset(openPath, 1.0);
    ASSERT_TRUE(openOffset.IsValid());
    ASSERT_EQ(openOffset.PointCount(), 3);
    const SCPoint2d firstOpenOffsetPoint = openOffset.PointAt(0);
    EXPECT_NEAR(firstOpenOffsetPoint.x, 0.0, 1e-12);
    EXPECT_NEAR(firstOpenOffsetPoint.y, 1.0, 1e-12);
    const SCPoint2d secondOpenOffsetPoint = openOffset.PointAt(1);
    EXPECT_NEAR(secondOpenOffsetPoint.x, 3.0, 1e-12);
    EXPECT_NEAR(secondOpenOffsetPoint.y, 1.0, 1e-12);
    const SCPoint2d thirdOpenOffsetPoint = openOffset.PointAt(2);
    EXPECT_NEAR(thirdOpenOffsetPoint.x, 3.0, 1e-12);
    EXPECT_NEAR(thirdOpenOffsetPoint.y, 3.0, 1e-12);

}

TEST(OffsetTest, OffsetsPolygonAndMultiPolygonShapes)
{
    const SCPolyline2d ccwRing({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}},
                             SCPolylineClosure::Closed);
    const SCPolygon2d polygon(ccwRing);
    const SCPolygon2d grownPolygon = Offset(polygon, 1.0);
    ASSERT_TRUE(grownPolygon.IsValid());
    const SCPoint2d firstGrownPoint = grownPolygon.OuterRing().PointAt(0);
    EXPECT_NEAR(firstGrownPoint.x, -1.0, 1e-12);
    EXPECT_NEAR(firstGrownPoint.y, -1.0, 1e-12);
    const SCPoint2d thirdGrownPoint = grownPolygon.OuterRing().PointAt(2);
    EXPECT_NEAR(thirdGrownPoint.x, 5.0, 1e-12);
    EXPECT_NEAR(thirdGrownPoint.y, 5.0, 1e-12);

    const SCPolyline2d holeRing({SCPoint2d{1.0, 1.0}, SCPoint2d{1.0, 3.0}, SCPoint2d{3.0, 3.0}, SCPoint2d{3.0, 1.0}},
                              SCPolylineClosure::Closed);
    const SCPolygon2d polygonWithHole(ccwRing, {holeRing});
    const SCPolygon2d offsetWithHole = Offset(polygonWithHole, 0.25);
    ASSERT_TRUE(offsetWithHole.IsValid());
    ASSERT_GT(offsetWithHole.Area(), polygonWithHole.Area());
    const SCPoint2d firstOffsetPoint = offsetWithHole.OuterRing().PointAt(0);
    EXPECT_NEAR(firstOffsetPoint.x, -0.25, 1e-12);
    EXPECT_NEAR(firstOffsetPoint.y, -0.25, 1e-12);
    const SCPoint2d thirdOffsetPoint = offsetWithHole.OuterRing().PointAt(2);
    EXPECT_NEAR(thirdOffsetPoint.x, 4.25, 1e-12);
    EXPECT_NEAR(thirdOffsetPoint.y, 4.25, 1e-12);

    const SCPolygon2d concave(SCPolyline2d({SCPoint2d{0.0, 0.0},
                                        SCPoint2d{5.0, 0.0},
                                        SCPoint2d{5.0, 1.0},
                                        SCPoint2d{2.0, 1.0},
                                        SCPoint2d{2.0, 4.0},
                                        SCPoint2d{0.0, 4.0}},
                                       SCPolylineClosure::Closed));
    const SCPolygon2d concaveOffset = Offset(concave, 0.5);
    ASSERT_TRUE(concaveOffset.IsValid());
    ASSERT_GT(concaveOffset.Area(), concave.Area());

    const SCMultiPolygon2d disjoint{
        SCPolygon2d(SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{2.0, 0.0}, SCPoint2d{2.0, 2.0}, SCPoint2d{0.0, 2.0}},
                             SCPolylineClosure::Closed)),
        SCPolygon2d(SCPolyline2d({SCPoint2d{4.0, 0.0}, SCPoint2d{6.0, 0.0}, SCPoint2d{6.0, 2.0}, SCPoint2d{4.0, 2.0}},
                             SCPolylineClosure::Closed))};
    const SCMultiPolygon2d expandedDisjoint = Offset(disjoint, 0.5);
    ASSERT_EQ(expandedDisjoint.Count(), 2);

    const SCPolygon2d clockwiseOuter(SCPolyline2d(
        {SCPoint2d{0.0, 0.0}, SCPoint2d{0.0, 4.0}, SCPoint2d{4.0, 4.0}, SCPoint2d{4.0, 0.0}}, SCPolylineClosure::Closed));
    const SCPolygon2d clockwiseExpanded = Offset(clockwiseOuter, 0.5);
    ASSERT_TRUE(clockwiseExpanded.IsValid());
    ASSERT_GT(clockwiseExpanded.Area(), clockwiseOuter.Area());

    const SCPolygon2d narrowDonut(
        SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{10.0, 0.0}, SCPoint2d{10.0, 6.0}, SCPoint2d{0.0, 6.0}},
                   SCPolylineClosure::Closed),
        {SCPolyline2d({SCPoint2d{3.5, 2.0}, SCPoint2d{3.5, 4.0}, SCPoint2d{6.5, 4.0}, SCPoint2d{6.5, 2.0}},
                    SCPolylineClosure::Closed)});
    const SCPolygon2d inwardRecovered = Offset(narrowDonut, -0.75);
    ASSERT_TRUE(inwardRecovered.IsValid());
    ASSERT_LT(inwardRecovered.Area(), narrowDonut.Area());
    for (std::size_t i = 0; i < inwardRecovered.OuterRing().PointCount(); ++i)
    {
        const SCPointContainment2d containment = LocatePoint(inwardRecovered.OuterRing().PointAt(i), narrowDonut);
        ASSERT_NE(containment, SCPointContainment2d::Outside);
    }

    const SCMultiPolygon2d multiWithHole{
        narrowDonut,
        SCPolygon2d(SCPolyline2d({SCPoint2d{14.0, 0.0}, SCPoint2d{20.0, 0.0}, SCPoint2d{20.0, 5.0}, SCPoint2d{14.0, 5.0}},
                             SCPolylineClosure::Closed))};
    const SCMultiPolygon2d inwardMultiRecovered = Offset(multiWithHole, -0.8);
    ASSERT_FALSE(inwardMultiRecovered.IsEmpty());
    for (std::size_t polygonIndex = 0; polygonIndex < inwardMultiRecovered.Count(); ++polygonIndex)
    {
        const SCPolygon2d& recovered = inwardMultiRecovered[polygonIndex];
        for (std::size_t pointIndex = 0; pointIndex < recovered.OuterRing().PointCount(); ++pointIndex)
        {
            const SCPoint2d point = recovered.OuterRing().PointAt(pointIndex);
            const bool inFirst = LocatePoint(point, narrowDonut) != SCPointContainment2d::Outside;
            const bool inSecond = LocatePoint(point, multiWithHole[1]) != SCPointContainment2d::Outside;
            ASSERT_TRUE(inFirst || inSecond);
        }
    }
}

TEST(OffsetTest, PreservesSinglePolygonHoleSemanticsAfterRebuild)
{
    const SCPolygon2d source(SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}},
                                      SCPolylineClosure::Closed),
                           {SCPolyline2d({SCPoint2d{1.0, 1.0}, SCPoint2d{1.0, 3.0}, SCPoint2d{3.0, 3.0}, SCPoint2d{3.0, 1.0}},
                                       SCPolylineClosure::Closed)});

    const SCPolygon2d outward = Offset(source, 0.5);
    ASSERT_TRUE(outward.IsValid());
    ASSERT_EQ(outward.HoleCount(), 1);
    ASSERT_GT(outward.Area(), source.Area());
    const SCPoint2d firstOutwardPoint = outward.OuterRing().PointAt(0);
    EXPECT_NEAR(firstOutwardPoint.x, -0.5, 1e-9);
    EXPECT_NEAR(firstOutwardPoint.y, -0.5, 1e-9);
    const SCPoint2d thirdOutwardPoint = outward.OuterRing().PointAt(2);
    EXPECT_NEAR(thirdOutwardPoint.x, 4.5, 1e-9);
    EXPECT_NEAR(thirdOutwardPoint.y, 4.5, 1e-9);

    const SCPolyline2d recoveredHole = outward.HoleAt(0);
    const Geometry::SCBox2d recoveredHoleBounds = recoveredHole.Bounds();
    EXPECT_NEAR(recoveredHoleBounds.MinPoint().x, 1.5, 1e-9);
    EXPECT_NEAR(recoveredHoleBounds.MinPoint().y, 1.5, 1e-9);
    EXPECT_NEAR(recoveredHoleBounds.MaxPoint().x, 2.5, 1e-9);
    EXPECT_NEAR(recoveredHoleBounds.MaxPoint().y, 2.5, 1e-9);

    const SCPolygon2d inward = Offset(source, -0.4);
    ASSERT_TRUE(inward.IsValid());
    ASSERT_EQ(inward.HoleCount(), 1);
    ASSERT_LT(inward.Area(), source.Area());
    for (std::size_t i = 0; i < inward.OuterRing().PointCount(); ++i)
    {
        ASSERT_NE(LocatePoint(inward.OuterRing().PointAt(i), source), SCPointContainment2d::Outside);
    }
}

TEST(OffsetTest, RecoversRepresentativeReverseEdgeSelfIntersectionCase)
{
    const SCPolygon2d source(SCPolyline2d({SCPoint2d{0.0, 0.0},
                                       SCPoint2d{6.0, 0.0},
                                       SCPoint2d{6.0, 1.0},
                                       SCPoint2d{2.5, 1.0},
                                       SCPoint2d{2.5, 4.0},
                                       SCPoint2d{6.0, 4.0},
                                       SCPoint2d{6.0, 5.0},
                                       SCPoint2d{0.0, 5.0}},
                                      SCPolylineClosure::Closed));

    const SCPolygon2d outward = Offset(source, 0.5);
    ASSERT_TRUE(outward.IsValid());
    ASSERT_GT(outward.Area(), source.Area());

    const SCPolygon2d inward = Offset(source, -0.4);
    ASSERT_TRUE(inward.IsValid());
    ASSERT_LT(inward.Area(), source.Area());
    for (std::size_t i = 0; i < inward.OuterRing().PointCount(); ++i)
    {
        ASSERT_NE(LocatePoint(inward.OuterRing().PointAt(i), source), SCPointContainment2d::Outside);
    }
}

TEST(OffsetTest, SupportsNarrowBridgeSplitViaMultiPolygonOffsetApi)
{
    const SCPolygon2d source(SCPolyline2d({SCPoint2d{0.0, 0.0},
                                       SCPoint2d{2.0, 0.0},
                                       SCPoint2d{2.0, 0.8},
                                       SCPoint2d{4.0, 0.8},
                                       SCPoint2d{4.0, 0.0},
                                       SCPoint2d{6.0, 0.0},
                                       SCPoint2d{6.0, 2.0},
                                       SCPoint2d{4.0, 2.0},
                                       SCPoint2d{4.0, 1.2},
                                       SCPoint2d{2.0, 1.2},
                                       SCPoint2d{2.0, 2.0},
                                       SCPoint2d{0.0, 2.0}},
                                      SCPolylineClosure::Closed));

    const SCMultiPolygon2d split = OffsetToMultiPolygon(source, -0.35);
    ASSERT_FALSE(split.IsEmpty());
    ASSERT_GE(split.Count(), 2);
    for (std::size_t polygonIndex = 0; polygonIndex < split.Count(); ++polygonIndex)
    {
        const SCPolygon2d& piece = split[polygonIndex];
        ASSERT_TRUE(piece.IsValid());
        for (std::size_t pointIndex = 0; pointIndex < piece.OuterRing().PointCount(); ++pointIndex)
        {
            ASSERT_NE(LocatePoint(piece.OuterRing().PointAt(pointIndex), source), SCPointContainment2d::Outside);
        }
    }
}
