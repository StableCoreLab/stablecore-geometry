#include <gtest/gtest.h>

#include "Geometry.h"
#include "Support/GeometryTestSupport.h"

using Geometry::BuildMultiPolygonByLines;
using Geometry::SCCircle2d;
using Geometry::CutPolygon;
using Geometry::SCEllipse2d;
using Geometry::SCLineSegment2d;
using Geometry::SCMultiPolyline2d;
using Geometry::NormalizePolygonByLines;
using Geometry::SCPoint2d;
using Geometry::SCPolygon2d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;
using Geometry::SCRectangle2d;
using Geometry::SubPolyline;

TEST(ShapesPathopsTest, CoversCurrentCapabilities)
{
    const SCCircle2d circle(SCPoint2d{1.0, 2.0}, 3.0);
    ASSERT_TRUE(circle.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(circle.Area(), 28.274333882308138, 1e-9);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(circle.Bounds().MinPoint(), (SCPoint2d{-2.0, -1.0}), 1e-12);
    ASSERT_TRUE(circle.ToPolygon(16).IsValid());

    const SCRectangle2d rectangle(SCPoint2d{0.0, 0.0}, 4.0, 2.0, 0.0);
    ASSERT_TRUE(rectangle.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(rectangle.Area(), 8.0, 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(rectangle.ToPolygon().OuterRing().PointAt(0), (SCPoint2d{-2.0, -1.0}), 1e-12);

    const SCEllipse2d ellipse(SCPoint2d{0.0, 0.0}, 3.0, 2.0, 0.0);
    ASSERT_TRUE(ellipse.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(ellipse.Area(), 18.84955592153876, 1e-9);
    ASSERT_TRUE(ellipse.ToPolygon(32).IsValid());

    const SCPolyline2d path({SCPoint2d{0.0, 0.0}, SCPoint2d{3.0, 0.0}, SCPoint2d{3.0, 4.0}}, SCPolylineClosure::Open);
    const SCPolyline2d sub = SubPolyline(path, 2.0, 5.0);
    ASSERT_TRUE(sub.IsValid());
    GEOMETRY_TEST_ASSERT_POINT_NEAR(sub.PointAt(0), (SCPoint2d{2.0, 0.0}), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(sub.PointAt(sub.PointCount() - 1), (SCPoint2d{3.0, 2.0}), 1e-12);

    const SCPolygon2d square(SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}},
                                      SCPolylineClosure::Closed));
    const auto cut = CutPolygon(square, SCLineSegment2d(SCPoint2d{2.0, -1.0}, SCPoint2d{2.0, 5.0}));
    ASSERT_TRUE(cut.success);
    ASSERT_EQ(cut.left.Count(), 1);
    ASSERT_EQ(cut.right.Count(), 1);
    GEOMETRY_TEST_ASSERT_NEAR(cut.left[0].Area(), 8.0, 1e-9);
    GEOMETRY_TEST_ASSERT_NEAR(cut.right[0].Area(), 8.0, 1e-9);

    const SCPolygon2d donut(SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{8.0, 0.0}, SCPoint2d{8.0, 8.0}, SCPoint2d{0.0, 8.0}},
                                     SCPolylineClosure::Closed),
                          {SCPolyline2d({SCPoint2d{3.0, 3.0}, SCPoint2d{3.0, 5.0}, SCPoint2d{5.0, 5.0}, SCPoint2d{5.0, 3.0}},
                                      SCPolylineClosure::Closed)});
    const auto donutCut = CutPolygon(donut, SCLineSegment2d(SCPoint2d{4.0, -2.0}, SCPoint2d{4.0, 10.0}));
    ASSERT_TRUE(donutCut.success);
    double leftArea = 0.0;
    double rightArea = 0.0;
    for (std::size_t i = 0; i < donutCut.left.Count(); ++i)
    {
        leftArea += donutCut.left[i].Area();
    }
    for (std::size_t i = 0; i < donutCut.right.Count(); ++i)
    {
        rightArea += donutCut.right[i].Area();
    }
    GEOMETRY_TEST_ASSERT_NEAR(leftArea, 30.0, 1e-6);
    GEOMETRY_TEST_ASSERT_NEAR(rightArea, 30.0, 1e-6);

    const SCMultiPolyline2d closedLines{
        SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{2.0, 0.0}, SCPoint2d{2.0, 2.0}, SCPoint2d{0.0, 2.0}},
                   SCPolylineClosure::Closed),
        SCPolyline2d({SCPoint2d{5.0, 5.0}, SCPoint2d{6.0, 5.0}, SCPoint2d{6.0, 6.0}, SCPoint2d{5.0, 6.0}},
                   SCPolylineClosure::Closed)};
    const auto closedMulti = BuildMultiPolygonByLines(closedLines);
    ASSERT_EQ(closedMulti.Count(), 2);

    const SCMultiPolyline2d openSquareLines{SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}}, SCPolylineClosure::Open),
                                          SCPolyline2d({SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}}, SCPolylineClosure::Open),
                                          SCPolyline2d({SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}}, SCPolylineClosure::Open),
                                          SCPolyline2d({SCPoint2d{0.0, 4.0}, SCPoint2d{0.0, 0.0}}, SCPolylineClosure::Open)};
    const auto openSquare = BuildMultiPolygonByLines(openSquareLines);
    ASSERT_EQ(openSquare.Count(), 1);
    ASSERT_EQ(openSquare[0].HoleCount(), 0);
    GEOMETRY_TEST_ASSERT_NEAR(openSquare[0].Area(), 16.0, 1e-9);

    const SCMultiPolyline2d nestedOpenLines{SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{6.0, 0.0}}, SCPolylineClosure::Open),
                                          SCPolyline2d({SCPoint2d{6.0, 0.0}, SCPoint2d{6.0, 6.0}}, SCPolylineClosure::Open),
                                          SCPolyline2d({SCPoint2d{6.0, 6.0}, SCPoint2d{0.0, 6.0}}, SCPolylineClosure::Open),
                                          SCPolyline2d({SCPoint2d{0.0, 6.0}, SCPoint2d{0.0, 0.0}}, SCPolylineClosure::Open),
                                          SCPolyline2d({SCPoint2d{2.0, 2.0}, SCPoint2d{4.0, 2.0}}, SCPolylineClosure::Open),
                                          SCPolyline2d({SCPoint2d{4.0, 2.0}, SCPoint2d{4.0, 4.0}}, SCPolylineClosure::Open),
                                          SCPolyline2d({SCPoint2d{4.0, 4.0}, SCPoint2d{2.0, 4.0}}, SCPolylineClosure::Open),
                                          SCPolyline2d({SCPoint2d{2.0, 4.0}, SCPoint2d{2.0, 2.0}}, SCPolylineClosure::Open)};
    const auto nested = BuildMultiPolygonByLines(nestedOpenLines);
    ASSERT_EQ(nested.Count(), 1);
    ASSERT_EQ(nested[0].HoleCount(), 1);
    GEOMETRY_TEST_ASSERT_NEAR(nested[0].Area(), 32.0, 1e-9);

    const SCMultiPolyline2d branchedLines{SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}}, SCPolylineClosure::Open),
                                        SCPolyline2d({SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}}, SCPolylineClosure::Open),
                                        SCPolyline2d({SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}}, SCPolylineClosure::Open),
                                        SCPolyline2d({SCPoint2d{0.0, 4.0}, SCPoint2d{0.0, 0.0}}, SCPolylineClosure::Open),
                                        SCPolyline2d({SCPoint2d{2.0, -1.0}, SCPoint2d{2.0, 2.0}}, SCPolylineClosure::Open)};
    const auto branched = BuildMultiPolygonByLines(branchedLines);
    ASSERT_EQ(branched.Count(), 1);
    GEOMETRY_TEST_ASSERT_NEAR(branched[0].Area(), 16.0, 1e-9);

    const SCMultiPolyline2d dirtyNearClosedLines{
        SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{0.0, 4.0}, SCPoint2d{0.0, 0.12}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{2.0, 4.0}, SCPoint2d{2.0, 5.0}}, SCPolylineClosure::Open)};
    const auto dirtyNearClosed = BuildMultiPolygonByLines(dirtyNearClosedLines);
    ASSERT_EQ(dirtyNearClosed.Count(), 1);
    GEOMETRY_TEST_ASSERT_NEAR(dirtyNearClosed[0].Area(), 16.0, 1e-6);

    const SCMultiPolyline2d autoExtendLines{
        SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{0.0, 4.0}, SCPoint2d{0.0, 1.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{-0.15, 1.0}, SCPoint2d{-0.15, 0.25}}, SCPolylineClosure::Open)};
    const auto autoExtended = BuildMultiPolygonByLines(autoExtendLines);
    ASSERT_EQ(autoExtended.Count(), 1);
    ASSERT_GE(autoExtended[0].Area(), 15.5);
    ASSERT_LE(autoExtended[0].Area(), 16.5);

    const SCMultiPolyline2d ambiguousFakeLines{
        SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{0.0, 4.0}, SCPoint2d{0.0, 0.25}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{-0.15, 0.25}, SCPoint2d{-0.15, 0.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{-0.15, 0.0}, SCPoint2d{0.15, 0.0}}, SCPolylineClosure::Open)};
    const auto ambiguousFake = BuildMultiPolygonByLines(ambiguousFakeLines);
    ASSERT_EQ(ambiguousFake.Count(), 1);
    ASSERT_EQ(ambiguousFake[0].HoleCount(), 0);
    ASSERT_GE(ambiguousFake[0].Area(), 15.5);
    ASSERT_LE(ambiguousFake[0].Area(), 16.5);

    const SCMultiPolyline2d branchHeavyAmbiguousLines{
        SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{6.0, 0.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{6.0, 0.0}, SCPoint2d{6.0, 6.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{6.0, 6.0}, SCPoint2d{0.0, 6.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{0.0, 6.0}, SCPoint2d{0.0, 0.4}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{-0.2, 0.4}, SCPoint2d{-0.2, 0.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{-0.2, 0.0}, SCPoint2d{0.3, 0.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{3.0, 6.0}, SCPoint2d{3.0, 7.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{0.0, 3.0}, SCPoint2d{-1.0, 3.0}}, SCPolylineClosure::Open)};
    const auto branchHeavyAmbiguous = BuildMultiPolygonByLines(branchHeavyAmbiguousLines);
    ASSERT_EQ(branchHeavyAmbiguous.Count(), 1);
    ASSERT_EQ(branchHeavyAmbiguous[0].HoleCount(), 0);
    ASSERT_GE(branchHeavyAmbiguous[0].Area(), 35.0);
    ASSERT_LE(branchHeavyAmbiguous[0].Area(), 37.0);

    const SCMultiPolyline2d branchScoredAmbiguousLines{
        SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{0.0, 4.0}, SCPoint2d{0.0, 0.4}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{-0.2, 0.4}, SCPoint2d{-0.2, -0.2}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{-0.2, -0.2}, SCPoint2d{1.0, -0.2}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{2.0, -1.0}, SCPoint2d{2.0, 2.0}}, SCPolylineClosure::Open)};
    const auto branchScoredAmbiguous = BuildMultiPolygonByLines(branchScoredAmbiguousLines);
    ASSERT_EQ(branchScoredAmbiguous.Count(), 1);
    ASSERT_EQ(branchScoredAmbiguous[0].HoleCount(), 0);
    GEOMETRY_TEST_ASSERT_NEAR(branchScoredAmbiguous[0].Area(), 16.0, 1e-6);

    const SCPolygon2d noisyBoundary(SCPolyline2d({SCPoint2d{0.0, 0.0},
                                              SCPoint2d{4.0, 0.0},
                                              SCPoint2d{4.0, 2.0},
                                              SCPoint2d{4.0, 2.000000001},
                                              SCPoint2d{4.0, 4.0},
                                              SCPoint2d{0.0, 4.0},
                                              SCPoint2d{0.0, 2.000000001},
                                              SCPoint2d{0.0, 2.0}},
                                             SCPolylineClosure::Closed));
    const SCPolygon2d normalizedByLines = NormalizePolygonByLines(noisyBoundary);
    ASSERT_TRUE(normalizedByLines.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(normalizedByLines.Area(), 16.0, 1e-6);
}
