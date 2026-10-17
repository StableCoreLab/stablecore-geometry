#include <gtest/gtest.h>

#include "Core/Metrics.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Geometry2d/SCPolyline2d.h"

TEST(Measure2dTest, PointSegmentDistance)
{
    const Geometry::SCLineSegment2d segment({0.0, 0.0}, {2.0, 0.0});
    EXPECT_NEAR(Geometry::DistanceSquared(Geometry::SCPoint2d{1.0, 2.0}, segment), 4.0, 1e-12);
    EXPECT_NEAR(Geometry::Distance(Geometry::SCPoint2d{1.0, 2.0}, segment), 2.0, 1e-12);
}

TEST(Measure2dTest, PolylineBounds)
{
    const Geometry::SCPolyline2d polyline(
        {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}}, Geometry::SCPolylineClosure::Closed);
    const Geometry::SCBox2d bounds = polyline.Bounds();
    EXPECT_NEAR(bounds.MinPoint().x, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().y, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().x, 2.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().y, 2.0, 1e-12);
}
