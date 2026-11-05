#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCPolylineCurve2d.h"
#include "Support/Epsilon.h"

TEST(PolylineCurve2dTest, PolylineUvAdapterPreservesParametersAndClone)
{
    Geometry::SCPolylineCurve2d adapter(Geometry::SCPolyline2d({{0,0},{2,0},{2,3}}));
    const Geometry::ISCCurve2d& curve = adapter;
    ASSERT_TRUE(curve.IsValid());
    EXPECT_DOUBLE_EQ(curve.ParameterRange().max, 2.0);
    const auto vertex = curve.Evaluate(1.0);
    EXPECT_EQ(vertex.point, Geometry::SCPoint2d(2,0));
    EXPECT_EQ(vertex.firstDerivative, Geometry::SCVector2d(0,3));
    EXPECT_EQ(curve.PointAt(2.0), Geometry::SCPoint2d(2,3));
    EXPECT_FALSE(curve.PointAt(-1.0).IsValid());
    const auto copy = curve.Clone();
    adapter = Geometry::SCPolylineCurve2d(Geometry::SCPolyline2d({{0,0},{9,0}}));
    EXPECT_EQ(copy->PointAt(2.0), Geometry::SCPoint2d(2,3));
    EXPECT_DOUBLE_EQ(copy->ParameterRange().max, 2.0);
    const Geometry::SCPolylineCurve2d closed(Geometry::SCPolyline2d(
        {{0,0},{2,0},{0,2}}, Geometry::SCPolylineClosure::Closed));
    EXPECT_EQ(closed.PointAt(3.0), closed.PointAt(0.0));
}

TEST(PolylineCurve2dTest, PolylineUvAdapterEvaluatesArcAndBounds)
{
    const auto arc = std::make_shared<Geometry::SCArcSegment2d>(Geometry::SCPoint2d{}, 2.0, 0.0, Geometry::kPi);
    const Geometry::SCPolylineCurve2d adapter(Geometry::SCPolyline2d(
        std::vector<std::shared_ptr<Geometry::ISCSegment2d>>{arc}, Geometry::SCPolylineClosure::Open));
    ASSERT_TRUE(adapter.IsValid());
    const auto middle = adapter.Evaluate(0.5);
    EXPECT_TRUE(middle.point.AlmostEquals({0,2}, 1e-12));
    EXPECT_TRUE(middle.firstDerivative.AlmostEquals({-2 * Geometry::kPi,0}, 1e-12));
    EXPECT_EQ(middle.derivativeOrder, 1);
    EXPECT_NEAR(adapter.Bounds().MinPoint().x, -2.0, 1e-12);
    EXPECT_NEAR(adapter.Bounds().MaxPoint().y, 2.0, 1e-12);
}
