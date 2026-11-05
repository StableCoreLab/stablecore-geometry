#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "Core/Relation.h"
#include "Core/ShapeOps.h"
#include "Core/Validation.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Geometry2d/SCPolygon2d.h"

using Geometry::SCArcDirection2d;
using Geometry::SCArcSegment2d;
using Geometry::SCBox2d;
using Geometry::IsEqual;
using Geometry::SCLineSegment2d;
using Geometry::LocatePoint;
using Geometry::Orientation;
using Geometry::SCPoint2d;
using Geometry::SCPointContainment2d;
using Geometry::SCPolygon2d;
using Geometry::SCPolygonValidation2d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;
using Geometry::SCRingOrientation2d;
using Geometry::Validate;

namespace
{
    SCPolyline2d MakeClosedRing(const std::vector<std::pair<SCPoint2d, SCPoint2d>>& edges)
    {
        std::vector<std::shared_ptr<Geometry::ISCSegment2d>> segments;
        segments.reserve(edges.size());
        for (const auto& edge : edges)
        {
            segments.push_back(std::make_shared<SCLineSegment2d>(edge.first, edge.second));
        }
        return SCPolyline2d(std::move(segments), SCPolylineClosure::Closed);
    }
}  // namespace

TEST(PolygonTest, CoversPolygonMetricsAndBounds)
{
    SCPolyline2d outerRing = MakeClosedRing({
        {SCPoint2d(0.0, 0.0), SCPoint2d(4.0, 0.0)},
        {SCPoint2d(4.0, 0.0), SCPoint2d(4.0, 4.0)},
        {SCPoint2d(4.0, 4.0), SCPoint2d(0.0, 4.0)},
        {SCPoint2d(0.0, 4.0), SCPoint2d(0.0, 0.0)},
    });

    SCPolyline2d holeRing = MakeClosedRing({
        {SCPoint2d(1.0, 1.0), SCPoint2d(1.0, 3.0)},
        {SCPoint2d(1.0, 3.0), SCPoint2d(3.0, 3.0)},
        {SCPoint2d(3.0, 3.0), SCPoint2d(3.0, 1.0)},
        {SCPoint2d(3.0, 1.0), SCPoint2d(1.0, 1.0)},
    });

    SCPolygon2d polygon(outerRing, {holeRing});
    ASSERT_TRUE(polygon.IsValid());
    ASSERT_TRUE(polygon.OuterRing().IsClosed());
    ASSERT_EQ(polygon.HoleCount(), 1);
    ASSERT_TRUE(polygon.HoleAt(0).IsClosed());

    ASSERT_LT(std::abs(polygon.Area() - 12.0), 1e-12);
    ASSERT_LT(std::abs(polygon.Perimeter() - 24.0), 1e-12);
    EXPECT_TRUE(polygon.Centroid().AlmostEquals(SCPoint2d(2.0, 2.0), 1e-12));

    const SCBox2d box = polygon.Bounds();
    ASSERT_TRUE(box.IsValid());
    EXPECT_TRUE(box.MinPoint().AlmostEquals(SCPoint2d(0.0, 0.0), 1e-12));
    EXPECT_TRUE(box.MaxPoint().AlmostEquals(SCPoint2d(4.0, 4.0), 1e-12));

}

TEST(PolygonTest, SmallCallerToleranceRetainsSubDefaultRingEdges)
{
    constexpr double width = 1e-10;
    constexpr double eps = 1e-15;
    const SCPolyline2d ring(
        {{0.0, 0.0}, {width, 0.0}, {width, width}, {0.0, width}}, SCPolylineClosure::Closed);
    ASSERT_EQ(ring.PointCount(), 4U);
    EXPECT_FALSE(ring.IsValid());
    EXPECT_TRUE(ring.IsValid(eps));
    const SCPolygon2d polygon(ring);
    EXPECT_TRUE(polygon.IsValid(eps));
    EXPECT_NEAR(polygon.Area(eps), width * width, width * width * 1e-5);
    ASSERT_TRUE(polygon.Bounds(eps).IsValid());
    EXPECT_TRUE(polygon.Bounds(eps).MaxPoint().AlmostEquals({width, width}, eps));
}

TEST(PolygonTest, RejectsOverflowingCombinedPerimeter)
{
    const SCPolyline2d outer(
        {{0.0, 0.0}, {6e307, 0.0}, {6e307, 1.0}, {0.0, 1.0}},
        SCPolylineClosure::Closed);
    const SCPolyline2d hole(
        {{1e307, 0.25}, {5e307, 0.25}, {5e307, 0.75}, {1e307, 0.75}},
        SCPolylineClosure::Closed);
    ASSERT_TRUE(outer.IsValid());
    ASSERT_TRUE(hole.IsValid());
    const SCPolygon2d polygon(outer, {hole});
    EXPECT_FALSE(polygon.IsValid());
}

TEST(PolygonTest, RejectsUnrepresentableArea)
{
    const SCPolyline2d outer(
        {{0.0, 0.0}, {1e155, 0.0}, {1e155, 1e155}, {0.0, 1e155}},
        SCPolylineClosure::Closed);
    ASSERT_TRUE(outer.IsValid());
    const SCPolygon2d polygon(outer);
    EXPECT_FALSE(polygon.IsValid());
}

TEST(PolygonTest, RejectsInvalidHoleConfigurations)
{
    SCPolyline2d outerRing = MakeClosedRing({
        {SCPoint2d(0.0, 0.0), SCPoint2d(4.0, 0.0)},
        {SCPoint2d(4.0, 0.0), SCPoint2d(4.0, 4.0)},
        {SCPoint2d(4.0, 4.0), SCPoint2d(0.0, 4.0)},
        {SCPoint2d(0.0, 4.0), SCPoint2d(0.0, 0.0)},
    });
    SCPolyline2d ccwHole = MakeClosedRing({
        {SCPoint2d(1.0, 1.0), SCPoint2d(3.0, 1.0)},
        {SCPoint2d(3.0, 1.0), SCPoint2d(3.0, 3.0)},
        {SCPoint2d(3.0, 3.0), SCPoint2d(1.0, 3.0)},
        {SCPoint2d(1.0, 3.0), SCPoint2d(1.0, 1.0)},
    });
    const SCPolygon2d polygonWithCcwHole(outerRing, {ccwHole});
    ASSERT_TRUE(polygonWithCcwHole.IsValid());
    EXPECT_DOUBLE_EQ(polygonWithCcwHole.Area(), 12.0);

    SCPolyline2d outsideHole = MakeClosedRing({
        {SCPoint2d(5.0, 5.0), SCPoint2d(6.0, 5.0)},
        {SCPoint2d(6.0, 5.0), SCPoint2d(6.0, 6.0)},
        {SCPoint2d(6.0, 6.0), SCPoint2d(5.0, 6.0)},
        {SCPoint2d(5.0, 6.0), SCPoint2d(5.0, 5.0)},
    });
    ASSERT_FALSE(SCPolygon2d(outerRing, {outsideHole}).IsValid());

    SCPolyline2d crossingHole = MakeClosedRing({
        {SCPoint2d(3.0, 3.0), SCPoint2d(5.0, 3.0)},
        {SCPoint2d(5.0, 3.0), SCPoint2d(5.0, 5.0)},
        {SCPoint2d(5.0, 5.0), SCPoint2d(3.0, 5.0)},
        {SCPoint2d(3.0, 5.0), SCPoint2d(3.0, 3.0)},
    });
    ASSERT_FALSE(SCPolygon2d(outerRing, {crossingHole}).IsValid());

    SCPolyline2d nestedHoleA = MakeClosedRing({
        {SCPoint2d(0.5, 0.5), SCPoint2d(0.5, 3.5)},
        {SCPoint2d(0.5, 3.5), SCPoint2d(3.5, 3.5)},
        {SCPoint2d(3.5, 3.5), SCPoint2d(3.5, 0.5)},
        {SCPoint2d(3.5, 0.5), SCPoint2d(0.5, 0.5)},
    });
    SCPolyline2d nestedHoleB = MakeClosedRing({
        {SCPoint2d(1.5, 1.5), SCPoint2d(1.5, 2.5)},
        {SCPoint2d(1.5, 2.5), SCPoint2d(2.5, 2.5)},
        {SCPoint2d(2.5, 2.5), SCPoint2d(2.5, 1.5)},
        {SCPoint2d(2.5, 1.5), SCPoint2d(1.5, 1.5)},
    });
    ASSERT_FALSE(SCPolygon2d(outerRing, {nestedHoleA, nestedHoleB}).IsValid());

}

TEST(PolygonTest, SupportsCurvedBoundaryPolygons)
{
    std::vector<std::shared_ptr<Geometry::ISCSegment2d>> arcSegments;
    arcSegments.push_back(std::make_shared<SCLineSegment2d>(SCPoint2d(0.0, 0.0), SCPoint2d(1.0, 0.0)));
    arcSegments.push_back(std::make_shared<SCArcSegment2d>(
        SCPoint2d(0.0, 0.0), 1.0, 0.0, Geometry::kPi * 0.5, SCArcDirection2d::CounterClockwise));
    arcSegments.push_back(std::make_shared<SCLineSegment2d>(SCPoint2d(0.0, 1.0), SCPoint2d(0.0, 0.0)));

    SCPolygon2d quarterDisk(SCPolyline2d(std::move(arcSegments), SCPolylineClosure::Closed));
    ASSERT_TRUE(quarterDisk.IsValid());
    ASSERT_LT(std::abs(quarterDisk.Area() - (Geometry::kPi / 4.0)), 1e-12);
    ASSERT_LT(std::abs(quarterDisk.Perimeter() - (2.0 + Geometry::kPi * 0.5)), 1e-12);
    EXPECT_TRUE(quarterDisk.Centroid().AlmostEquals(
        SCPoint2d(4.0 / (3.0 * Geometry::kPi), 4.0 / (3.0 * Geometry::kPi)), 1e-12));
    ASSERT_EQ(LocatePoint(SCPoint2d(0.2, 0.2), quarterDisk), SCPointContainment2d::Inside);
    ASSERT_EQ(LocatePoint(SCPoint2d(0.9, 0.9), quarterDisk), SCPointContainment2d::Outside);
    ASSERT_EQ(LocatePoint(SCPoint2d(std::sqrt(0.5), std::sqrt(0.5)), quarterDisk), SCPointContainment2d::OnBoundary);

    const double pi = Geometry::kPi;
    std::vector<std::shared_ptr<Geometry::ISCSegment2d>> circleSegments;
    circleSegments.push_back(std::make_shared<SCArcSegment2d>(SCPoint2d(0.0, 0.0), 1.0, 0.0, pi));
    circleSegments.push_back(std::make_shared<SCArcSegment2d>(SCPoint2d(0.0, 0.0), 1.0, pi, pi));
    SCPolyline2d fullCircleRing(std::move(circleSegments), SCPolylineClosure::Closed);
    SCPolygon2d fullCircle(fullCircleRing);
    const SCPolygonValidation2d fullCircleValidation = Validate(fullCircleRing);
    ASSERT_TRUE(fullCircleValidation.valid);
    ASSERT_TRUE(fullCircle.IsValid());
    ASSERT_EQ(Orientation(fullCircleRing), SCRingOrientation2d::CounterClockwise);
    ASSERT_EQ(LocatePoint(SCPoint2d(0.0, 0.0), fullCircle), SCPointContainment2d::Inside);
    EXPECT_TRUE(fullCircle.Centroid().AlmostEquals(SCPoint2d(0.0, 0.0), 1e-12));
}

TEST(PolygonTest, TranslationPreservesRectangleMeasurementsAndOrientation)
{
    for (const double origin : {0.0, 1e9, -1e9})
    {
        SCOPED_TRACE(origin);
        const SCPolyline2d ring({{origin, -origin}, {origin + 3, -origin},
            {origin + 3, -origin + 2}, {origin, -origin + 2}}, SCPolylineClosure::Closed);
        ASSERT_TRUE(ring.IsValid());
        for (const bool reverse : {false, true})
        {
            SCOPED_TRACE(reverse);
            const SCPolyline2d boundary = reverse ? Geometry::Reverse(ring) : ring;
            const SCPolygon2d polygon(boundary);
            EXPECT_EQ(Orientation(boundary), reverse ? SCRingOrientation2d::Clockwise : SCRingOrientation2d::CounterClockwise);
            EXPECT_TRUE(polygon.IsValid());
            EXPECT_NEAR(polygon.Area(), 6.0, 1e-12);
            EXPECT_NEAR(polygon.Centroid().x, origin + 1.5, 2e-7);
            EXPECT_NEAR(polygon.Centroid().y, -origin + 1.0, 2e-7);
        }
    }
}

TEST(PolygonTest, TranslationPreservesMixedArcMeasurements)
{
    for (const double origin : {0.0, 1e9, -1e9})
    {
        SCOPED_TRACE(origin);
        const SCPoint2d center{origin, -origin};
        std::vector<std::shared_ptr<Geometry::ISCSegment2d>> segments;
        segments.push_back(std::make_shared<SCLineSegment2d>(center, SCPoint2d{origin + 1, -origin}));
        segments.push_back(std::make_shared<SCArcSegment2d>(center, 1.0, 0.0, Geometry::kPi * 0.5));
        segments.push_back(std::make_shared<SCLineSegment2d>(SCPoint2d{origin, -origin + 1}, center));
        const SCPolyline2d boundary(std::move(segments), SCPolylineClosure::Closed);
        ASSERT_TRUE(boundary.IsValid());
        for (const bool reverse : {false, true})
        {
            SCOPED_TRACE(reverse);
            const SCPolygon2d polygon(reverse ? Geometry::Reverse(boundary) : boundary);
            EXPECT_TRUE(polygon.IsValid());
            EXPECT_NEAR(polygon.Area(), Geometry::kPi / 4.0, 1e-12);
            EXPECT_NEAR(polygon.Centroid().x, origin + 4.0 / (3.0 * Geometry::kPi), 3e-7);
            EXPECT_NEAR(polygon.Centroid().y, -origin + 4.0 / (3.0 * Geometry::kPi), 3e-7);
        }
    }
}

TEST(PolygonTest, RingDirectionsDoNotChangeTranslatedHoleCentroid)
{
    for (const double origin : {0.0, 1e9, -1e9})
    {
        SCOPED_TRACE(origin);
        const SCPolyline2d outer({{origin, -origin}, {origin + 4, -origin},
            {origin + 4, -origin + 4}, {origin, -origin + 4}}, SCPolylineClosure::Closed);
        const SCPolyline2d hole({{origin + 1, -origin + 1}, {origin + 1, -origin + 2},
            {origin + 2, -origin + 2}, {origin + 2, -origin + 1}}, SCPolylineClosure::Closed);
        for (const bool reverseOuter : {false, true})
            for (const bool reverseHole : {false, true})
            {
                SCOPED_TRACE(reverseOuter);
                SCOPED_TRACE(reverseHole);
                const SCPolygon2d polygon(reverseOuter ? Geometry::Reverse(outer) : outer,
                    {reverseHole ? Geometry::Reverse(hole) : hole});
                EXPECT_TRUE(polygon.IsValid());
                EXPECT_NEAR(polygon.Area(), 15.0, 1e-12);
                EXPECT_NEAR(polygon.Centroid().x, origin + 30.5 / 15.0, 3e-7);
                EXPECT_NEAR(polygon.Centroid().y, -origin + 30.5 / 15.0, 3e-7);
            }
    }
}

TEST(PolygonTest, ExplicitPerimeterToleranceRetainsSmallSweepOuterAndHoleArcs)
{
    for (const double radius : {1.0, 1e10})
    {
        SCOPED_TRACE(radius);
        const double eps = radius * 1e-15;
        const SCPolyline2d square({{-2 * radius, -2 * radius}, {2 * radius, -2 * radius},
            {2 * radius, 2 * radius}, {-2 * radius, 2 * radius}}, SCPolylineClosure::Closed);
        for (const double sweep : {0.5, 1e-10})
        {
            SCOPED_TRACE(sweep);
            const SCArcSegment2d arc(SCPoint2d{}, radius, 0.0, sweep);
            ASSERT_TRUE(arc.IsValid(eps));
            EXPECT_EQ(arc.IsValid(), sweep == 0.5);
            std::vector<std::shared_ptr<Geometry::ISCSegment2d>> segments;
            segments.push_back(std::make_shared<SCLineSegment2d>(SCPoint2d{}, arc.StartPoint()));
            segments.push_back(std::make_shared<SCArcSegment2d>(arc));
            segments.push_back(std::make_shared<SCLineSegment2d>(arc.EndPoint(), SCPoint2d{}));
            const SCPolyline2d sector(std::move(segments), SCPolylineClosure::Closed);
            ASSERT_TRUE(sector.IsValid(eps));
            const SCPolygon2d diskSector(sector);
            const SCPolygon2d squareWithHole(square, {sector});
            ASSERT_TRUE(diskSector.IsValid(eps));
            ASSERT_TRUE(squareWithHole.IsValid(eps));
            EXPECT_NEAR(diskSector.Perimeter(eps), radius * (2.0 + sweep), radius * 1e-13);
            EXPECT_NEAR(squareWithHole.Perimeter(eps), radius * (18.0 + sweep), radius * 1e-13);
            if (sweep == 0.5 || radius == 1e10)
            {
                ASSERT_TRUE(diskSector.IsValid());
                ASSERT_TRUE(squareWithHole.IsValid());
                EXPECT_NEAR(diskSector.Perimeter(), diskSector.Perimeter(eps), radius * 1e-13);
                EXPECT_NEAR(squareWithHole.Perimeter(), squareWithHole.Perimeter(eps), radius * 1e-13);
            }
        }
    }
}
