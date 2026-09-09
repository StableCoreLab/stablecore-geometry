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

using Geometry::SCArcDirection;
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
        SCPoint2d(0.0, 0.0), 1.0, 0.0, Geometry::kPi * 0.5, SCArcDirection::CounterClockwise));
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
    EXPECT_TRUE(Geometry::Centroid(fullCircle).AlmostEquals(SCPoint2d(0.0, 0.0), 1e-12));
}
