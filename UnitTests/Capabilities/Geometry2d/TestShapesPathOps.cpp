#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <vector>

#include "Geometry.h"

using Geometry::SCCircle2d;
using Geometry::SCEllipse2d;
using Geometry::SCLineSegment2d;
using Geometry::SCPoint2d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;
using Geometry::SCRectangle2d;
using Geometry::SubPolyline;

TEST(ShapesPathopsTest, BuildsPrimitiveShapeContracts)
{
    const SCCircle2d circle(SCPoint2d{1.0, 2.0}, 3.0);
    ASSERT_TRUE(circle.IsValid());
    EXPECT_NEAR(circle.Area(), 28.274333882308138, 1e-9);
    EXPECT_TRUE(circle.Bounds().MinPoint().AlmostEquals(SCPoint2d{-2.0, -1.0}, 1e-12));
    ASSERT_TRUE(circle.ToPolygon(16).IsValid());

    const SCRectangle2d rectangle(SCPoint2d{0.0, 0.0}, 4.0, 2.0, 0.0);
    ASSERT_TRUE(rectangle.IsValid());
    EXPECT_NEAR(rectangle.Area(), 8.0, 1e-12);
    EXPECT_TRUE(rectangle.ToPolygon().OuterRing().VertexAt(0).AlmostEquals(SCPoint2d{-2.0, -1.0}, 1e-12));

    const SCEllipse2d ellipse(SCPoint2d{0.0, 0.0}, 3.0, 2.0, 0.0);
    ASSERT_TRUE(ellipse.IsValid());
    EXPECT_NEAR(ellipse.Area(), 18.84955592153876, 1e-9);
    ASSERT_TRUE(ellipse.ToPolygon(32).IsValid());

}

TEST(ShapesPathopsTest, RotatedEllipseBoundsContainAnalyticExtrema)
{
    const SCEllipse2d ellipse(SCPoint2d{4.0, -3.0}, 3.0, 2.0, std::numbers::pi_v<double> / 4.0);
    ASSERT_TRUE(ellipse.IsValid());

    const double extent = std::sqrt(6.5);
    const auto expected = Geometry::SCBox2d::FromMinMax(
        SCPoint2d{4.0 - extent, -3.0 - extent}, SCPoint2d{4.0 + extent, -3.0 + extent});
    EXPECT_TRUE(ellipse.Bounds(8).AlmostEquals(expected, 1e-12));
    EXPECT_TRUE(ellipse.Bounds(128).AlmostEquals(expected, 1e-12));
    EXPECT_FALSE(ellipse.Bounds(7).IsValid());
}

TEST(ShapesPathopsTest, PrimitiveShapeComparisonUsesStoredParameters)
{
    const SCCircle2d circle({1.0, 2.0}, 3.0);
    EXPECT_TRUE(circle == SCCircle2d({1.0, 2.0}, 3.0));
    EXPECT_FALSE(circle == SCCircle2d({1.0, 2.0}, 3.1));
    EXPECT_TRUE(circle.AlmostEquals(SCCircle2d({1.0 + 1e-8, 2.0}, 3.0 + 1e-8), 2e-8));
    EXPECT_FALSE(circle.AlmostEquals(SCCircle2d({1.0, 2.0}, 3.0), -1.0));
    EXPECT_FALSE(circle.AlmostEquals(SCCircle2d({1.0, 2.0}, 3.0),
                                     std::numeric_limits<double>::infinity()));

    const SCEllipse2d ellipse({0.0, 0.0}, 3.0, 2.0, 0.25);
    EXPECT_TRUE(ellipse == SCEllipse2d({0.0, 0.0}, 3.0, 2.0, 0.25));
    EXPECT_TRUE(ellipse.AlmostEquals(SCEllipse2d({1e-8, 0.0}, 3.0 + 1e-8, 2.0, 0.25 + 1e-8),
                                     2e-8, 2e-8));
    EXPECT_FALSE(ellipse.AlmostEquals(SCEllipse2d({1e-8, 0.0}, 3.0 + 1e-8, 2.0, 0.25 + 1e-8),
                                      2e-8, 1e-10));
    EXPECT_FALSE(ellipse.AlmostEquals(SCEllipse2d({0.0, 0.0}, 3.0, 2.0, 0.25 + 2.0 * std::numbers::pi),
                                      1e-8));
    EXPECT_FALSE(ellipse == SCEllipse2d({0.0, 0.0}, 2.0, 3.0, 0.25));

    const SCRectangle2d rectangle({2.0, -1.0}, 4.0, 2.0, 0.5);
    EXPECT_TRUE(rectangle == SCRectangle2d({2.0, -1.0}, 4.0, 2.0, 0.5));
    EXPECT_TRUE(rectangle.AlmostEquals(SCRectangle2d({2.0, -1.0 + 1e-8}, 4.0, 2.0 + 1e-8, 0.5),
                                       2e-8));
    EXPECT_TRUE(rectangle.AlmostEquals(SCRectangle2d({2.0, -1.0}, 4.0, 2.0, 0.5 + 1e-8),
                                       1e-10, 2e-8));
    EXPECT_FALSE(rectangle.AlmostEquals(SCRectangle2d({2.0, -1.0}, 4.0, 2.0, 0.5 + 1e-8),
                                        2e-8, 1e-10));
    EXPECT_FALSE(rectangle.AlmostEquals(SCRectangle2d({2.0, -1.0}, 4.0, 2.0, 0.6), 1e-8));
    EXPECT_FALSE(rectangle == SCRectangle2d({2.0, -1.0}, 2.0, 4.0, 0.5));
}

TEST(ShapesPathopsTest, ExtractsPolylineSubpaths)
{
    const SCPolyline2d path({SCPoint2d{0.0, 0.0}, SCPoint2d{3.0, 0.0}, SCPoint2d{3.0, 4.0}}, SCPolylineClosure::Open);
    const SCPolyline2d sub = SubPolyline(path, 2.0, 5.0);
    ASSERT_TRUE(sub.IsValid());
    EXPECT_TRUE(sub.VertexAt(0).AlmostEquals(SCPoint2d{2.0, 0.0}, 1e-12));
    EXPECT_TRUE(sub.VertexAt(sub.PointCount() - 1).AlmostEquals(SCPoint2d{3.0, 2.0}, 1e-12));

}

TEST(ShapesPathopsTest, ExtractsSubDefaultPolylineWithExplicitTolerance)
{
    constexpr double length = 1e-10;
    constexpr double eps = 1e-15;
    const SCPolyline2d path({{0.0, 0.0}, {length, 0.0}, {length, length}}, SCPolylineClosure::Open);
    ASSERT_FALSE(path.IsValid());
    ASSERT_TRUE(path.IsValid(eps));

    const SCPolyline2d sub = SubPolyline(path, 0.5 * length, 1.5 * length, eps);
    ASSERT_TRUE(sub.IsValid(eps));
    ASSERT_EQ(sub.PointCount(), 3U);
    EXPECT_TRUE(sub.VertexAt(0).AlmostEquals({0.5 * length, 0.0}, eps));
    EXPECT_TRUE(sub.VertexAt(1).AlmostEquals({length, 0.0}, eps));
    EXPECT_TRUE(sub.VertexAt(2).AlmostEquals({length, 0.5 * length}, eps));
    EXPECT_FALSE(SubPolyline(path, 0.5 * length, 1.5 * length).IsValid(eps));
    EXPECT_FALSE(SubPolyline(path, 0.5 * length, 1.5 * length, 0.0).IsValid(eps));
    EXPECT_FALSE(SubPolyline(path, 0.5 * length, 1.5 * length,
                             std::numeric_limits<double>::infinity()).IsValid(eps));
}

TEST(ShapesPathopsTest, SubPolylineRetainsArcLengthAndGeometry)
{
    const auto arc = std::make_shared<Geometry::SCArcSegment2d>(
        SCPoint2d{0.0, 0.0}, 2.0, 0.0, std::numbers::pi_v<double>);
    const SCPolyline2d path(std::vector<std::shared_ptr<Geometry::ISCSegment2d>>{arc},
                            SCPolylineClosure::Open);
    ASSERT_TRUE(path.IsValid());

    const SCPolyline2d sub = SubPolyline(path, 0.25 * path.Length(), 0.75 * path.Length(), 1e-12);
    ASSERT_TRUE(sub.IsValid(1e-12));
    ASSERT_EQ(sub.SegmentCount(), 1U);
    EXPECT_EQ(sub.SegmentAt(0)->Kind(), Geometry::SCSegmentKind2d::Arc);
    EXPECT_NEAR(sub.Length(), std::numbers::pi_v<double>, 1e-12);
    EXPECT_TRUE(sub.StartPoint().AlmostEquals({std::sqrt(2.0), std::sqrt(2.0)}, 1e-12));
    EXPECT_TRUE(sub.EndPoint().AlmostEquals({-std::sqrt(2.0), std::sqrt(2.0)}, 1e-12));
}

TEST(ShapesPathopsTest, SubPolylineIncludesClosingSegment)
{
    const SCPolyline2d path({{0.0, 0.0}, {2.0, 0.0}, {0.0, 2.0}}, SCPolylineClosure::Closed);
    ASSERT_TRUE(path.IsValid());

    const SCPolyline2d sub = SubPolyline(path, path.Length() - 1.5, path.Length() - 0.5, 1e-12);
    ASSERT_TRUE(sub.IsValid());
    ASSERT_EQ(sub.SegmentCount(), 1U);
    EXPECT_TRUE(sub.StartPoint().AlmostEquals({0.0, 1.5}, 1e-12));
    EXPECT_TRUE(sub.EndPoint().AlmostEquals({0.0, 0.5}, 1e-12));
}

TEST(ShapesPathopsTest, SubPolylineSkipsSubToleranceArcBeforeSelectedLine)
{
    constexpr double eps = 1e-12;
    const auto arc = std::make_shared<Geometry::SCArcSegment2d>(
        SCPoint2d{0.0, 0.0}, 1e-14, 0.0, std::numbers::pi_v<double>);
    const auto line = std::make_shared<SCLineSegment2d>(arc->EndPoint(), SCPoint2d{2.0, 0.0});
    const SCPolyline2d path(std::vector<std::shared_ptr<Geometry::ISCSegment2d>>{arc, line},
                            SCPolylineClosure::Open);
    ASSERT_TRUE(path.IsValid(eps));
    EXPECT_NEAR(path.Length(eps), path.Length(), eps);
    EXPECT_TRUE(path.PointAtLength(arc->Length() * 0.5, false, eps)
                    .AlmostEquals(arc->PointAt(0.5), 1e-16));
    EXPECT_GT(path.Bounds(eps).MaxPoint().y, 0.0);

    const SCPolyline2d sub = SubPolyline(path, arc->Length() + 0.5, arc->Length() + 1.5, eps);
    ASSERT_TRUE(sub.IsValid(eps));
    ASSERT_EQ(sub.SegmentCount(), 1U);
    EXPECT_EQ(sub.SegmentAt(0)->Kind(), Geometry::SCSegmentKind2d::Line);
    EXPECT_NEAR(sub.StartPoint().x, 0.5, eps);
    EXPECT_NEAR(sub.EndPoint().x, 1.5, eps);
}

TEST(ShapesPathopsTest, SubPolylineClipsShortSweepArcWithExplicitTolerance)
{
    constexpr double eps = 1e-9;
    const auto line = std::make_shared<SCLineSegment2d>(
        SCPoint2d{1e6 - 1e-4, 0.0}, SCPoint2d{1e6, 0.0});
    const auto arc = std::make_shared<Geometry::SCArcSegment2d>(
        SCPoint2d{0.0, 0.0}, 1e6, 0.0, 1e-10);
    const SCPolyline2d path(std::vector<std::shared_ptr<Geometry::ISCSegment2d>>{line, arc},
                            SCPolylineClosure::Open);
    ASSERT_TRUE(path.IsValid(eps));

    const double start = line->Length() + 2e-5;
    const double end = line->Length() + 8e-5;
    EXPECT_FALSE(SubPolyline(path, start, end).IsValid());
    const SCPolyline2d clipped = SubPolyline(path, start, end, eps);
    ASSERT_TRUE(clipped.IsValid(eps));
    ASSERT_EQ(clipped.SegmentCount(), 1U);
    EXPECT_EQ(clipped.SegmentAt(0)->Kind(), Geometry::SCSegmentKind2d::Arc);
    EXPECT_NEAR(clipped.StartPoint().y, 2e-5, 1e-9);
    EXPECT_NEAR(clipped.EndPoint().y, 8e-5, 1e-9);
}

TEST(ShapesPathopsTest, SubPolylineRejectsArcWithOverflowingEndpoint)
{
    const auto arc = std::make_shared<Geometry::SCArcSegment2d>(
        SCPoint2d{1e308, 0.0}, 1e308, 0.0, std::numbers::pi_v<double> / 2.0);
    const SCPolyline2d path(std::vector<std::shared_ptr<Geometry::ISCSegment2d>>{arc},
                            SCPolylineClosure::Open);
    ASSERT_FALSE(arc->StartPoint().IsValid());

    const SCPolyline2d sub = SubPolyline(path, 0.0, 0.5 * path.Length(), 1e-12);
    EXPECT_FALSE(sub.IsValid(1e-12));
}

TEST(ShapesPathopsTest, SubPolylineRejectsArcWithOverflowingInterior)
{
    const auto arc = std::make_shared<Geometry::SCArcSegment2d>(
        SCPoint2d{1.5e308, 0.0}, 5e307, -std::numbers::pi_v<double> / 2.0,
        std::numbers::pi_v<double>);
    const SCPolyline2d path(std::vector<std::shared_ptr<Geometry::ISCSegment2d>>{arc},
                            SCPolylineClosure::Open);
    ASSERT_TRUE(arc->StartPoint().IsValid());
    ASSERT_TRUE(arc->EndPoint().IsValid());
    ASSERT_FALSE((SCPoint2d{arc->center.x + arc->radius, arc->center.y}).IsValid());

    const SCPolyline2d sub = SubPolyline(path, 0.0, path.Length(), 1e-12);
    EXPECT_FALSE(sub.IsValid(1e-12));
}
