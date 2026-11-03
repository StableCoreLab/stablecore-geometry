#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <vector>

#include "Geometry.h"

using Geometry::BuildMultiPolygonByLines;
using Geometry::CutPolygon;
using Geometry::SCLineSegment2d;
using Geometry::SCMultiPolyline2d;
using Geometry::NormalizePolygonByLines;
using Geometry::SCPoint2d;
using Geometry::SCPolygon2d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;

TEST(ShapesPathopsTest, CutsSquareIntoEqualParts)
{
    const SCPolygon2d square(SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}},
                                      SCPolylineClosure::Closed));
    const auto cut = CutPolygonParts(square, SCLineSegment2d(SCPoint2d{2.0, -1.0}, SCPoint2d{2.0, 5.0}));
    ASSERT_TRUE(cut.has_value());
    ASSERT_EQ(cut->left.Size(), 1);
    ASSERT_EQ(cut->right.Size(), 1);
    EXPECT_NEAR(cut->left[0].Area(), 8.0, 1e-9);
    EXPECT_NEAR(cut->right[0].Area(), 8.0, 1e-9);
}

TEST(ShapesPathopsTest, CutsPolygonWithHole)
{
    const SCPolygon2d donut(SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{8.0, 0.0}, SCPoint2d{8.0, 8.0}, SCPoint2d{0.0, 8.0}},
                                     SCPolylineClosure::Closed),
                          {SCPolyline2d({SCPoint2d{3.0, 3.0}, SCPoint2d{3.0, 5.0}, SCPoint2d{5.0, 5.0}, SCPoint2d{5.0, 3.0}},
                                      SCPolylineClosure::Closed)});
    const auto donutCut = CutPolygonParts(donut, SCLineSegment2d(SCPoint2d{4.0, -2.0}, SCPoint2d{4.0, 10.0}));
    ASSERT_TRUE(donutCut.has_value());
    double leftArea = 0.0;
    double rightArea = 0.0;
    for (std::size_t i = 0; i < donutCut->left.Size(); ++i)
    {
        leftArea += donutCut->left[i].Area();
    }
    for (std::size_t i = 0; i < donutCut->right.Size(); ++i)
    {
        rightArea += donutCut->right[i].Area();
    }
    EXPECT_NEAR(leftArea, 30.0, 1e-6);
    EXPECT_NEAR(rightArea, 30.0, 1e-6);
}

TEST(ShapesPathopsTest, BuildsSeparatePolygonsFromClosedLines)
{
    const SCMultiPolyline2d closedLines{
        SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{2.0, 0.0}, SCPoint2d{2.0, 2.0}, SCPoint2d{0.0, 2.0}},
                   SCPolylineClosure::Closed),
        SCPolyline2d({SCPoint2d{5.0, 5.0}, SCPoint2d{6.0, 5.0}, SCPoint2d{6.0, 6.0}, SCPoint2d{5.0, 6.0}},
                   SCPolylineClosure::Closed)};
    const auto closedMulti = BuildMultiPolygonByLines(closedLines);
    ASSERT_EQ(closedMulti.Size(), 2);
}

TEST(ShapesPathopsTest, BuildsSquareFromOpenBoundaryLines)
{
    const SCMultiPolyline2d openSquareLines{SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}}, SCPolylineClosure::Open),
                                          SCPolyline2d({SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}}, SCPolylineClosure::Open),
                                          SCPolyline2d({SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}}, SCPolylineClosure::Open),
                                          SCPolyline2d({SCPoint2d{0.0, 4.0}, SCPoint2d{0.0, 0.0}}, SCPolylineClosure::Open)};
    const auto openSquare = BuildMultiPolygonByLines(openSquareLines);
    ASSERT_EQ(openSquare.Size(), 1);
    ASSERT_EQ(openSquare[0].HoleCount(), 0);
    EXPECT_NEAR(openSquare[0].Area(), 16.0, 1e-9);
}

TEST(ShapesPathopsTest, BuildsHoleFromNestedOpenLines)
{
    const SCMultiPolyline2d nestedOpenLines{SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{6.0, 0.0}}, SCPolylineClosure::Open),
                                          SCPolyline2d({SCPoint2d{6.0, 0.0}, SCPoint2d{6.0, 6.0}}, SCPolylineClosure::Open),
                                          SCPolyline2d({SCPoint2d{6.0, 6.0}, SCPoint2d{0.0, 6.0}}, SCPolylineClosure::Open),
                                          SCPolyline2d({SCPoint2d{0.0, 6.0}, SCPoint2d{0.0, 0.0}}, SCPolylineClosure::Open),
                                          SCPolyline2d({SCPoint2d{2.0, 2.0}, SCPoint2d{4.0, 2.0}}, SCPolylineClosure::Open),
                                          SCPolyline2d({SCPoint2d{4.0, 2.0}, SCPoint2d{4.0, 4.0}}, SCPolylineClosure::Open),
                                          SCPolyline2d({SCPoint2d{4.0, 4.0}, SCPoint2d{2.0, 4.0}}, SCPolylineClosure::Open),
                                          SCPolyline2d({SCPoint2d{2.0, 4.0}, SCPoint2d{2.0, 2.0}}, SCPolylineClosure::Open)};
    const auto nested = BuildMultiPolygonByLines(nestedOpenLines);
    ASSERT_EQ(nested.Size(), 1);
    ASSERT_EQ(nested[0].HoleCount(), 1);
    EXPECT_NEAR(nested[0].Area(), 32.0, 1e-9);
}

TEST(ShapesPathopsTest, RebuildsBoundaryWithCrossingBranch)
{
    const SCMultiPolyline2d branchedLines{SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}}, SCPolylineClosure::Open),
                                        SCPolyline2d({SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}}, SCPolylineClosure::Open),
                                        SCPolyline2d({SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}}, SCPolylineClosure::Open),
                                        SCPolyline2d({SCPoint2d{0.0, 4.0}, SCPoint2d{0.0, 0.0}}, SCPolylineClosure::Open),
                                        SCPolyline2d({SCPoint2d{2.0, -1.0}, SCPoint2d{2.0, 2.0}}, SCPolylineClosure::Open)};
    const auto branched = BuildMultiPolygonByLines(branchedLines);
    ASSERT_EQ(branched.Size(), 1);
    EXPECT_NEAR(branched[0].Area(), 16.0, 1e-9);
}

TEST(ShapesPathopsTest, RepairsNearClosedBoundaryWithDuplicateEdge)
{
    const SCMultiPolyline2d dirtyNearClosedLines{
        SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{0.0, 4.0}, SCPoint2d{0.0, 0.12}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{2.0, 4.0}, SCPoint2d{2.0, 5.0}}, SCPolylineClosure::Open)};
    const auto dirtyNearClosed = BuildMultiPolygonByLines(dirtyNearClosedLines);
    ASSERT_EQ(dirtyNearClosed.Size(), 1);
    EXPECT_NEAR(dirtyNearClosed[0].Area(), 16.0, 1e-6);
}

TEST(ShapesPathopsTest, ExtendsBoundaryToCloseGap)
{
    const SCMultiPolyline2d autoExtendLines{
        SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{0.0, 4.0}, SCPoint2d{0.0, 1.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{-0.15, 1.0}, SCPoint2d{-0.15, 0.25}}, SCPolylineClosure::Open)};
    const auto autoExtended = BuildMultiPolygonByLines(autoExtendLines);
    ASSERT_EQ(autoExtended.Size(), 1);
    ASSERT_GE(autoExtended[0].Area(), 15.5);
    ASSERT_LE(autoExtended[0].Area(), 16.5);
}

TEST(ShapesPathopsTest, SelectsBoundaryAcrossAmbiguousSyntheticEdges)
{
    const SCMultiPolyline2d ambiguousFakeLines{
        SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{0.0, 4.0}, SCPoint2d{0.0, 0.25}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{-0.15, 0.25}, SCPoint2d{-0.15, 0.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{-0.15, 0.0}, SCPoint2d{0.15, 0.0}}, SCPolylineClosure::Open)};
    const auto ambiguousFake = BuildMultiPolygonByLines(ambiguousFakeLines);
    ASSERT_EQ(ambiguousFake.Size(), 1);
    ASSERT_EQ(ambiguousFake[0].HoleCount(), 0);
    ASSERT_GE(ambiguousFake[0].Area(), 15.5);
    ASSERT_LE(ambiguousFake[0].Area(), 16.5);
}

TEST(ShapesPathopsTest, RebuildsBranchHeavyAmbiguousBoundary)
{
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
    ASSERT_EQ(branchHeavyAmbiguous.Size(), 1);
    ASSERT_EQ(branchHeavyAmbiguous[0].HoleCount(), 0);
    ASSERT_GE(branchHeavyAmbiguous[0].Area(), 35.0);
    ASSERT_LE(branchHeavyAmbiguous[0].Area(), 37.0);
}

TEST(ShapesPathopsTest, ScoresAmbiguousBoundaryBranches)
{
    const SCMultiPolyline2d branchScoredAmbiguousLines{
        SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{0.0, 4.0}, SCPoint2d{0.0, 0.4}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{-0.2, 0.4}, SCPoint2d{-0.2, -0.2}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{-0.2, -0.2}, SCPoint2d{1.0, -0.2}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{2.0, -1.0}, SCPoint2d{2.0, 2.0}}, SCPolylineClosure::Open)};
    const auto branchScoredAmbiguous = BuildMultiPolygonByLines(branchScoredAmbiguousLines);
    ASSERT_EQ(branchScoredAmbiguous.Size(), 1);
    ASSERT_EQ(branchScoredAmbiguous[0].HoleCount(), 0);
    EXPECT_NEAR(branchScoredAmbiguous[0].Area(), 16.0, 1e-6);
}

TEST(ShapesPathopsTest, NormalizesNearDuplicateBoundaryVertices)
{
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
    EXPECT_NEAR(normalizedByLines.Area(), 16.0, 1e-6);
}

TEST(ShapesPathopsTest, SmallCallerToleranceKeepsNearEndpointLineCrossing)
{
    constexpr double eps = 1e-15;
    constexpr double crossingX = 5e-13;
    SCMultiPolyline2d boundaries;
    boundaries.Add(SCPolyline2d({{0.0, 0.0}, {1.0, 0.0}}, SCPolylineClosure::Open));
    boundaries.Add(SCPolyline2d({{crossingX, -0.5}, {crossingX, 1.0}}, SCPolylineClosure::Open));
    boundaries.Add(SCPolyline2d({{crossingX, 1.0}, {1.0, 1.0}}, SCPolylineClosure::Open));
    boundaries.Add(SCPolyline2d({{1.0, 1.0}, {1.0, 0.0}}, SCPolylineClosure::Open));

    Geometry::SCPolygonBuildOptions2d options;
    options.allowFakeEdges = false;
    const auto rebuilt = Geometry::BuildMultiPolygonByLinesWithDiagnostics(boundaries, eps, options);
    ASSERT_EQ(rebuilt.polygons.Size(), 1U);
    const auto& polygon = rebuilt.polygons[0];
    EXPECT_NEAR(polygon.Area(), 1.0 - crossingX, 1e-14);
    for (std::size_t i = 0; i < polygon.OuterRing().PointCount(); ++i)
    {
        EXPECT_GE(polygon.OuterRing().VertexAt(i).x, crossingX - eps);
    }
}

TEST(ShapesPathopsTest, NormalizesThinValidPolygonWithCallerTolerance)
{
    constexpr double eps = 1e-15;
    const SCPolygon2d thin(SCPolyline2d(
        {{0.0, 0.0}, {1000.0, 0.0}, {500.0, 5e-13}}, SCPolylineClosure::Closed));
    ASSERT_TRUE(thin.IsValid());

    const SCPolygon2d normalized = NormalizePolygonByLines(thin, eps);
    ASSERT_TRUE(normalized.IsValid());
    EXPECT_NEAR(normalized.Area(), thin.Area(), 1e-12);
}

TEST(ShapesPathopsTest, NormalizesSubDefaultSquareWithCallerTolerance)
{
    constexpr double width = 1e-10;
    constexpr double eps = 1e-15;
    const SCPolygon2d square(SCPolyline2d(
        {{0.0, 0.0}, {width, 0.0}, {width, width}, {0.0, width}},
        SCPolylineClosure::Closed));
    ASSERT_FALSE(square.IsValid());
    ASSERT_TRUE(square.IsValid(eps));
    SCMultiPolyline2d boundaries;
    boundaries.Add(square.OuterRing());
    const auto rebuilt = Geometry::BuildMultiPolygonByLinesWithDiagnostics(boundaries, eps);
    EXPECT_EQ(rebuilt.diagnostics.inputSegmentCount, 4U);
    EXPECT_EQ(rebuilt.diagnostics.uniqueVertexCount, 4U);
    EXPECT_EQ(rebuilt.diagnostics.danglingEndpointCount, 0U);
    ASSERT_EQ(rebuilt.polygons.Size(), 1U);

    const SCPolygon2d normalized = NormalizePolygonByLines(square, eps);
    ASSERT_TRUE(normalized.IsValid(eps));
    EXPECT_NEAR(normalized.Area(eps), width * width, 1e-25);
    EXPECT_EQ(normalized.OuterRing().PointCount(), 4U);
}

TEST(ShapesPathopsTest, SmallNestedRingsRetainInteriorHole)
{
    constexpr double eps = 1e-12;
    constexpr double width = 1e-6;
    constexpr double margin = 5e-10;
    Geometry::SCMultiPolyline2d boundaries;
    boundaries.Add(SCPolyline2d(
        {{0.0, 0.0}, {width, 0.0}, {width, width}, {0.0, width}},
        SCPolylineClosure::Closed));
    boundaries.Add(SCPolyline2d(
        {{margin, margin}, {width - margin, margin},
         {width - margin, width - margin}, {margin, width - margin}},
        SCPolylineClosure::Closed));

    const auto rebuilt = Geometry::BuildMultiPolygonByLines(boundaries, eps);
    ASSERT_EQ(rebuilt.Size(), 1U);
    ASSERT_EQ(rebuilt[0].HoleCount(), 1U);
    EXPECT_NEAR(rebuilt[0].Area(eps), width * width - (width - 2.0 * margin) * (width - 2.0 * margin),
                1e-17);
}

TEST(ShapesPathopsTest, RejectsInvalidNormalizationTolerance)
{
    const SCPolygon2d square(SCPolyline2d(
        {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}}, SCPolylineClosure::Closed));
    ASSERT_TRUE(square.IsValid());
    for (double eps : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN()})
    {
        EXPECT_FALSE(NormalizePolygonByLines(square, eps).IsValid());
    }
}

TEST(ShapesPathopsTest, RepairedLoopDoesNotEraseSeparateThinOriginalLoop)
{
    constexpr double eps = 1e-15;
    SCMultiPolyline2d boundaries;
    boundaries.Add(SCPolyline2d(
        {{0.0, 0.0}, {10.0, 0.0}, {5.0, 5e-11}}, SCPolylineClosure::Closed));
    boundaries.Add(SCPolyline2d(
        {{1e6, 0.0}, {1e6 + 1.0, 0.0}, {1e6 + 1.0, 1.0}, {1e6, 1.0}, {1e6, 0.01}},
        SCPolylineClosure::Open));

    const auto rebuilt = Geometry::BuildMultiPolygonByLinesWithDiagnostics(boundaries, eps);
    ASSERT_TRUE(rebuilt.usedAutoClose);
    EXPECT_EQ(rebuilt.polygons.Size(), 2U);
    bool foundThin = false;
    bool foundRepaired = false;
    for (std::size_t i = 0; i < rebuilt.polygons.Size(); ++i)
    {
        foundThin |= std::abs(rebuilt.polygons[i].Area() - 2.5e-10) < 1e-12;
        foundRepaired |= std::abs(rebuilt.polygons[i].Area() - 1.0) < 1e-9;
    }
    EXPECT_TRUE(foundThin);
    EXPECT_TRUE(foundRepaired);
}

TEST(ShapesPathopsTest, CutPolygonUsesCallerToleranceForBothBoundaryPaths)
{
    const SCPolygon2d square(SCPolyline2d(
        {{0.0, 0.0}, {4.0, 0.0}, {4.0, 4.0}, {0.0, 4.0}}, SCPolylineClosure::Closed));
    const SCPolygon2d donut(SCPolyline2d(
        {{0.0, 0.0}, {4.0, 0.0}, {4.0, 4.0}, {0.0, 4.0}}, SCPolylineClosure::Closed),
        {SCPolyline2d({{1.0, 1.0}, {1.0, 3.0}, {3.0, 3.0}, {3.0, 1.0}}, SCPolylineClosure::Closed)});
    const SCLineSegment2d shortCutter({2.0, -1.0}, {2.0, -1.0 + 1e-5});
    for (const SCPolygon2d* polygon : {&square, &donut})
    {
        EXPECT_FALSE(Geometry::CutPolygonParts(*polygon, shortCutter, 1e-4).has_value());
        EXPECT_FALSE(Geometry::CutPolygonParts(*polygon,
            SCLineSegment2d({2.0, 0.0}, {2.0, 0.0}), 1e-9).has_value());
        const auto smallCutter = Geometry::CutPolygonParts(*polygon,
            SCLineSegment2d({2.0, -1e-10}, {2.0, 1e-10}), 1e-12);
        ASSERT_TRUE(smallCutter.has_value());
        ASSERT_EQ(smallCutter->left.Size(), 1U);
        ASSERT_EQ(smallCutter->right.Size(), 1U);
        const double expectedArea = polygon == &square ? 8.0 : 6.0;
        EXPECT_NEAR(smallCutter->left[0].Area(), expectedArea, 1e-8);
        EXPECT_NEAR(smallCutter->right[0].Area(), expectedArea, 1e-8);
    }
}

TEST(ShapesPathopsTest, CutConcavePolygonKeepsDisconnectedPartsWithDistantCutter)
{
    const SCPolygon2d concave(SCPolyline2d(
        {{0.0, 0.0}, {4.0, 0.0}, {4.0, 1.0}, {1.0, 1.0},
         {1.0, 3.0}, {4.0, 3.0}, {4.0, 4.0}, {0.0, 4.0}},
        SCPolylineClosure::Closed));
    ASSERT_TRUE(concave.IsValid());
    const auto parts = Geometry::CutPolygonParts(
        concave, SCLineSegment2d({2.0, -1000000.0}, {2.0, -999999.0}));
    ASSERT_TRUE(parts.has_value());
    ASSERT_EQ(parts->left.Size(), 1U);
    ASSERT_EQ(parts->right.Size(), 2U);
    double rightArea = 0.0;
    for (std::size_t i = 0; i < parts->right.Size(); ++i) rightArea += parts->right[i].Area();
    EXPECT_NEAR(parts->left[0].Area(), 6.0, 1e-8);
    EXPECT_NEAR(rightArea, 4.0, 1e-8);
}

TEST(ShapesPathopsTest, CutPolygonAcceptsFiniteLongCutter)
{
    const SCPolygon2d square(SCPolyline2d(
        {{0.0, 0.0}, {4.0, 0.0}, {4.0, 4.0}, {0.0, 4.0}},
        SCPolylineClosure::Closed));
    const auto parts = Geometry::CutPolygonParts(
        square, SCLineSegment2d({2.0, -1e160}, {2.0, 1e160}));
    ASSERT_TRUE(parts.has_value());
    ASSERT_EQ(parts->left.Size(), 1U);
    ASSERT_EQ(parts->right.Size(), 1U);
    EXPECT_NEAR(parts->left[0].Area(), 8.0, 1e-9);
    EXPECT_NEAR(parts->right[0].Area(), 8.0, 1e-9);
}

TEST(ShapesPathopsTest, NarrowNotchRepairPreservesCallerScale)
{
    for (const double scale : {1.0, 1e-4, 1e-8})
    {
        for (const double relativeEps : {1e-12, 1e-9})
        {
            SCOPED_TRACE(scale);
            constexpr double notch = 1e-9;
            SCOPED_TRACE(relativeEps);
            const double eps = relativeEps * scale;
            std::vector<SCPoint2d> points{{0, 0}, {6, 0}, {6, 1}, {4, 1},
                {4, 1 + notch}, {6, 1 + notch}, {6, 4}, {0, 4}};
            for (auto& point : points)
            {
                point.x *= scale;
                point.y *= scale;
            }
            const SCPolygon2d polygon(SCPolyline2d(std::move(points), SCPolylineClosure::Closed));
            if (relativeEps < notch)
            {
                ASSERT_TRUE(polygon.IsValid(eps));
                const auto normalized = NormalizePolygonByLines(polygon, eps);
                ASSERT_TRUE(normalized.IsValid(eps));
                EXPECT_NEAR(normalized.Area(eps) / (scale * scale), 24.0 - 2.0 * notch, 1e-10);
                EXPECT_EQ(normalized.HoleCount(), 0U);
            }
            else
            {
                const SCPolygon2d clip(SCPolyline2d({{2 * scale, -scale}, {8 * scale, -scale},
                    {8 * scale, 3 * scale}, {2 * scale, 3 * scale}}, SCPolylineClosure::Closed));
                const auto area = [&](const Geometry::SCMultiPolygon2d& polygons) {
                    double sum = 0.0;
                    for (std::size_t i = 0; i < polygons.Size(); ++i) sum += polygons[i].Area(eps);
                    return sum / (scale * scale);
                };
                EXPECT_NEAR(area(Geometry::Union(polygon, clip, eps)), 36.0, 1e-7);
                EXPECT_NEAR(area(Geometry::Difference(polygon, clip, eps)), 12.0, 1e-7);
            }
        }
    }
}
