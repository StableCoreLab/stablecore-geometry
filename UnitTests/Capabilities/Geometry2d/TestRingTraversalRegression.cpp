#include <algorithm>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "../../../Source/Detail/ArrangementVertices2d.h"
#include "../../../Source/Detail/DirectedEdgeFans2d.h"
#include "../../../Source/Detail/PolygonNesting2d.h"
#include "../../../Source/Detail/RingVertices2d.h"
#include "../../../Source/Detail/SegmentParameters2d.h"
#include "../../../Source/Detail/SegmentSubdivision2d.h"
#include "Geometry.h"

using Geometry::BuildMultiPolygonByLines;
using Geometry::Difference;
using Geometry::SCLineSegment2d;
using Geometry::SCMultiPolygon2d;
using Geometry::SCMultiPolyline2d;
using Geometry::SCPoint2d;
using Geometry::SCPolygon2d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;
using Geometry::SearchPolygons;

namespace
{
    using PolygonSignature = std::pair<double, std::size_t>;

    std::vector<PolygonSignature> MakeSignature(const SCMultiPolygon2d& polygons)
    {
        std::vector<PolygonSignature> signature;
        signature.reserve(polygons.Count());
        for (std::size_t index = 0; index < polygons.Count(); ++index)
        {
            signature.emplace_back(polygons[index].Area(), polygons[index].HoleCount());
        }
        std::sort(signature.begin(), signature.end());
        return signature;
    }

    void ExpectEquivalentPolygons(const SCMultiPolygon2d& actual, const SCMultiPolygon2d& expected)
    {
        const std::vector<PolygonSignature> actualSignature = MakeSignature(actual);
        const std::vector<PolygonSignature> expectedSignature = MakeSignature(expected);
        ASSERT_EQ(actualSignature.size(), expectedSignature.size());
        for (std::size_t index = 0; index < actualSignature.size(); ++index)
        {
            EXPECT_NEAR(actualSignature[index].first, expectedSignature[index].first, 1e-9);
            EXPECT_EQ(actualSignature[index].second, expectedSignature[index].second);
        }
    }

    SCMultiPolyline2d BuildNestedSquareEdges(bool shuffled)
    {
        const SCPolyline2d outerBottom({SCPoint2d{0.0, 0.0}, SCPoint2d{8.0, 0.0}}, SCPolylineClosure::Open);
        const SCPolyline2d outerRight({SCPoint2d{8.0, 0.0}, SCPoint2d{8.0, 8.0}}, SCPolylineClosure::Open);
        const SCPolyline2d outerTop({SCPoint2d{8.0, 8.0}, SCPoint2d{0.0, 8.0}}, SCPolylineClosure::Open);
        const SCPolyline2d outerLeft({SCPoint2d{0.0, 8.0}, SCPoint2d{0.0, 0.0}}, SCPolylineClosure::Open);
        const SCPolyline2d innerBottom({SCPoint2d{2.0, 2.0}, SCPoint2d{6.0, 2.0}}, SCPolylineClosure::Open);
        const SCPolyline2d innerRight({SCPoint2d{6.0, 2.0}, SCPoint2d{6.0, 6.0}}, SCPolylineClosure::Open);
        const SCPolyline2d innerTop({SCPoint2d{6.0, 6.0}, SCPoint2d{2.0, 6.0}}, SCPolylineClosure::Open);
        const SCPolyline2d innerLeft({SCPoint2d{2.0, 6.0}, SCPoint2d{2.0, 2.0}}, SCPolylineClosure::Open);

        if (!shuffled)
        {
            return {outerBottom, outerRight, outerTop, outerLeft, innerBottom, innerRight, innerTop, innerLeft};
        }

        return {innerTop, outerLeft, innerBottom, outerTop, innerLeft, outerRight, innerRight, outerBottom};
    }

    double TotalArea(const SCMultiPolygon2d& polygons)
    {
        double total = 0.0;
        for (std::size_t index = 0; index < polygons.Count(); ++index)
        {
            total += polygons[index].Area();
        }
        return total;
    }

    void ExpectSearchPolyMatchesRebuild(const SCMultiPolyline2d& lines)
    {
        const SCMultiPolygon2d rebuilt = BuildMultiPolygonByLines(lines);
        const auto searchResult = SearchPolygons(lines);
        ASSERT_TRUE(searchResult.IsSuccess());
        ExpectEquivalentPolygons(searchResult.polygons, rebuilt);
    }

    TEST(RingTraversalRegressionTest, RebuildIsInvariantToIndependentEdgeOrderAndSearchPolyMatchesIt)
    {
        const SCMultiPolyline2d orderedEdges = BuildNestedSquareEdges(false);
        const SCMultiPolyline2d shuffledEdges = BuildNestedSquareEdges(true);

        const SCMultiPolygon2d orderedRebuild = BuildMultiPolygonByLines(orderedEdges);
        const SCMultiPolygon2d shuffledRebuild = BuildMultiPolygonByLines(shuffledEdges);
        ASSERT_EQ(orderedRebuild.Count(), 1U);
        ASSERT_EQ(orderedRebuild[0].HoleCount(), 1U);
        EXPECT_NEAR(orderedRebuild[0].Area(), 48.0, 1e-9);
        ExpectEquivalentPolygons(shuffledRebuild, orderedRebuild);

        const auto searchResult = SearchPolygons(shuffledEdges);
        ASSERT_TRUE(searchResult.IsSuccess());
        ExpectEquivalentPolygons(searchResult.polygons, orderedRebuild);
    }

    TEST(RingTraversalRegressionTest, BooleanDifferenceMatchesIndependentBoundaryRebuild)
    {
        const SCPolygon2d outer(
            SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{8.0, 0.0}, SCPoint2d{8.0, 8.0}, SCPoint2d{0.0, 8.0}},
                         SCPolylineClosure::Closed));
        const SCPolygon2d inner(
            SCPolyline2d({SCPoint2d{2.0, 2.0}, SCPoint2d{6.0, 2.0}, SCPoint2d{6.0, 6.0}, SCPoint2d{2.0, 6.0}},
                         SCPolylineClosure::Closed));

        ASSERT_TRUE(outer.IsValid());
        ASSERT_TRUE(inner.IsValid());
        const SCMultiPolygon2d difference = Difference(outer, inner);
        const SCMultiPolygon2d rebuilt = BuildMultiPolygonByLines(BuildNestedSquareEdges(true));

        ASSERT_EQ(difference.Count(), 1U);
        ASSERT_EQ(difference[0].HoleCount(), 1U);
        EXPECT_NEAR(difference[0].Area(), 48.0, 1e-9);
        ExpectEquivalentPolygons(difference, rebuilt);
    }

    TEST(RingTraversalRegressionTest, CrossedEdgesKeepRebuildAndSearchPolyResultsAligned)
    {
        const SCMultiPolyline2d crossedEdges{
            SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{8.0, 0.0}}, SCPolylineClosure::Open),
            SCPolyline2d({SCPoint2d{8.0, 0.0}, SCPoint2d{8.0, 8.0}}, SCPolylineClosure::Open),
            SCPolyline2d({SCPoint2d{8.0, 8.0}, SCPoint2d{0.0, 8.0}}, SCPolylineClosure::Open),
            SCPolyline2d({SCPoint2d{0.0, 8.0}, SCPoint2d{0.0, 0.0}}, SCPolylineClosure::Open),
            SCPolyline2d({SCPoint2d{0.0, 4.0}, SCPoint2d{8.0, 4.0}}, SCPolylineClosure::Open),
            SCPolyline2d({SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 8.0}}, SCPolylineClosure::Open),
        };

        const SCMultiPolygon2d rebuilt = BuildMultiPolygonByLines(crossedEdges);
        ASSERT_EQ(rebuilt.Count(), 4U);
        EXPECT_NEAR(TotalArea(rebuilt), 64.0, 1e-9);
        ExpectSearchPolyMatchesRebuild(crossedEdges);
    }

    TEST(RingTraversalRegressionTest, CollinearOverlapKeepsRebuildAndSearchPolyResultsAligned)
    {
        const SCMultiPolyline2d collinearEdges{
            SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{6.0, 0.0}}, SCPolylineClosure::Open),
            SCPolyline2d({SCPoint2d{2.0, 0.0}, SCPoint2d{8.0, 0.0}}, SCPolylineClosure::Open),
            SCPolyline2d({SCPoint2d{8.0, 0.0}, SCPoint2d{8.0, 8.0}}, SCPolylineClosure::Open),
            SCPolyline2d({SCPoint2d{8.0, 8.0}, SCPoint2d{0.0, 8.0}}, SCPolylineClosure::Open),
            SCPolyline2d({SCPoint2d{0.0, 8.0}, SCPoint2d{0.0, 0.0}}, SCPolylineClosure::Open),
        };

        const SCMultiPolygon2d rebuilt = BuildMultiPolygonByLines(collinearEdges);
        ASSERT_EQ(rebuilt.Count(), 1U);
        EXPECT_NEAR(TotalArea(rebuilt), 64.0, 1e-9);
        ExpectSearchPolyMatchesRebuild(collinearEdges);
    }

    TEST(RingTraversalRegressionTest, NarrowGapDifferenceMatchesIndependentBoundaryRebuild)
    {
        const SCPolygon2d outer(
            SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{8.0, 0.0}, SCPoint2d{8.0, 8.0}, SCPoint2d{0.0, 8.0}},
                         SCPolylineClosure::Closed));
        const SCPolygon2d narrowInner(
            SCPolyline2d({SCPoint2d{0.02, 2.0}, SCPoint2d{7.98, 2.0}, SCPoint2d{7.98, 6.0}, SCPoint2d{0.02, 6.0}},
                         SCPolylineClosure::Closed));
        const SCMultiPolyline2d boundaries{outer.OuterRing(), narrowInner.OuterRing()};

        ASSERT_TRUE(outer.IsValid());
        ASSERT_TRUE(narrowInner.IsValid());
        const SCMultiPolygon2d difference = Difference(outer, narrowInner);
        const SCMultiPolygon2d rebuilt = BuildMultiPolygonByLines(boundaries);

        ASSERT_EQ(difference.Count(), 1U);
        ASSERT_EQ(difference[0].HoleCount(), 1U);
        EXPECT_NEAR(difference[0].Area(), 32.16, 1e-9);
        ExpectEquivalentPolygons(difference, rebuilt);
        ExpectSearchPolyMatchesRebuild(boundaries);
    }

    TEST(RingTraversalRegressionTest, DirectedEdgeFanUsesDeterministicTieBreaks)
    {
        using Geometry::Detail::DirectedEdge2d;

        const std::vector<DirectedEdge2d> edges{
            DirectedEdge2d{0, 9, 0, 0.0},
            DirectedEdge2d{0, 5, 0, 0.0},
            DirectedEdge2d{0, 5, 0, 0.0},
            DirectedEdge2d{1, 0, 1, Geometry::kPi},
        };
        std::vector<std::vector<std::size_t>> outgoing{{0, 2, 1}};

        Geometry::Detail::SortOutgoingFans(edges, outgoing);
        ASSERT_EQ(outgoing[0], (std::vector<std::size_t>{1, 2, 0}));
        EXPECT_EQ(Geometry::Detail::NextFaceEdge(edges, outgoing, 3), 0U);
    }

    TEST(RingTraversalRegressionTest, SegmentParametersClampDeduplicateAndCompactNearValues)
    {
        std::vector<double> parameters;
        for (const double value : {1.2, 0.5000004, 0.5, -0.1, 0.25, 0.2500003})
        {
            Geometry::Detail::AddClampedParameter(parameters, value, 1e-6);
        }

        const std::vector<double> compacted = Geometry::Detail::CompactSortedParameters(parameters, 1e-6);
        ASSERT_EQ(compacted.size(), 4U);
        EXPECT_DOUBLE_EQ(compacted[0], 0.0);
        EXPECT_NEAR(compacted[1], 0.25, 1e-6);
        EXPECT_NEAR(compacted[2], 0.5000004, 1e-6);
        EXPECT_DOUBLE_EQ(compacted[3], 1.0);

        std::vector<double> permuted;
        for (const double value : {-0.1, 0.25, 0.5, 0.5000004, 0.2500003, 1.2})
        {
            Geometry::Detail::AddClampedParameter(permuted, value, 1e-6);
        }
        const std::vector<double> permutedCompacted = Geometry::Detail::CompactSortedParameters(permuted, 1e-6);
        ASSERT_EQ(permutedCompacted.size(), compacted.size());
        for (std::size_t index = 0; index < compacted.size(); ++index)
        {
            EXPECT_NEAR(permutedCompacted[index], compacted[index], 1e-6);
        }
    }

    TEST(RingTraversalRegressionTest, ArrangementVertexToolsMergeByToleranceAndNormalizeEdgeOrder)
    {
        std::vector<SCPoint2d> vertices;
        const std::size_t first = Geometry::Detail::FindOrAddVertex2d(vertices, SCPoint2d{1.0, 2.0}, 1e-6);
        const std::size_t matching = Geometry::Detail::FindOrAddVertex2d(vertices, SCPoint2d{1.0 + 5e-7, 2.0}, 1e-6);
        const std::size_t distinct = Geometry::Detail::FindOrAddVertex2d(vertices, SCPoint2d{1.0 + 2e-6, 2.0}, 1e-6);

        EXPECT_EQ(first, 0U);
        EXPECT_EQ(matching, first);
        EXPECT_EQ(distinct, 1U);
        EXPECT_EQ(Geometry::Detail::FindVertexIndex2d(vertices, SCPoint2d{1.0, 2.0}, 1e-6), first);
        EXPECT_EQ(Geometry::Detail::FindVertexIndex2d(vertices, SCPoint2d{9.0, 9.0}, 1e-6),
                  Geometry::Detail::kInvalidVertexIndex2d);
        EXPECT_EQ(Geometry::Detail::MakeUndirectedEdgeKey2d(3U, 9U), Geometry::Detail::MakeUndirectedEdgeKey2d(9U, 3U));
        EXPECT_NE(Geometry::Detail::MakeUndirectedEdgeKey2d(3U, 9U),
                  Geometry::Detail::MakeUndirectedEdgeKey2d(3U, 10U));
    }

    TEST(RingTraversalRegressionTest, RingNormalizationAndPolygonNestingKeepSmallestContainer)
    {
        const std::vector<SCPoint2d> normalized = Geometry::Detail::NormalizeRingVertices2d(
            {SCPoint2d{0.0, 0.0}, SCPoint2d{1.0, 0.0}, SCPoint2d{1.0, 0.0}, SCPoint2d{0.0, 1.0}, SCPoint2d{0.0, 0.0}},
            1e-9);
        ASSERT_EQ(normalized.size(), 3U);
        EXPECT_TRUE(normalized.front().AlmostEquals(SCPoint2d{0.0, 0.0}, 1e-9));
        EXPECT_TRUE(normalized.back().AlmostEquals(SCPoint2d{0.0, 1.0}, 1e-9));

        const SCPolygon2d outer(
            SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{10.0, 0.0}, SCPoint2d{10.0, 10.0}, SCPoint2d{0.0, 10.0}},
                         SCPolylineClosure::Closed));
        const SCPolygon2d middle(
            SCPolyline2d({SCPoint2d{2.0, 2.0}, SCPoint2d{8.0, 2.0}, SCPoint2d{8.0, 8.0}, SCPoint2d{2.0, 8.0}},
                         SCPolylineClosure::Closed));
        const SCPolygon2d inner(
            SCPolyline2d({SCPoint2d{3.0, 3.0}, SCPoint2d{4.0, 3.0}, SCPoint2d{4.0, 4.0}, SCPoint2d{3.0, 4.0}},
                         SCPolylineClosure::Closed));
        const std::vector<std::size_t> parents = Geometry::Detail::BuildPolygonParents2d({outer, middle, inner}, 1e-9);

        ASSERT_EQ(parents.size(), 3U);
        EXPECT_EQ(parents[0], Geometry::Detail::kNoContainingPolygon2d);
        EXPECT_EQ(parents[1], 0U);
        EXPECT_EQ(parents[2], 1U);
    }

    TEST(RingTraversalRegressionTest, SegmentSubdivisionPreservesSourceIndicesForCrossingAndOverlap)
    {
        const auto parameterTolerance = [](const SCLineSegment2d&) { return 1e-9; };
        const std::vector<SCLineSegment2d> crossing{
            SCLineSegment2d(SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}),
            SCLineSegment2d(SCPoint2d{2.0, -1.0}, SCPoint2d{2.0, 1.0}),
        };
        const std::vector<Geometry::Detail::SubdividedLineSegment2d> crossingResult =
            Geometry::Detail::SubdivideLineSegments(crossing, 1e-9, parameterTolerance);

        ASSERT_EQ(crossingResult.size(), 4U);
        EXPECT_EQ(crossingResult[0].sourceIndex, 0U);
        EXPECT_EQ(crossingResult[1].sourceIndex, 0U);
        EXPECT_EQ(crossingResult[2].sourceIndex, 1U);
        EXPECT_EQ(crossingResult[3].sourceIndex, 1U);
        EXPECT_TRUE(crossingResult[0].end.AlmostEquals(SCPoint2d{2.0, 0.0}, 1e-9));
        EXPECT_TRUE(crossingResult[1].start.AlmostEquals(SCPoint2d{2.0, 0.0}, 1e-9));
        EXPECT_TRUE(crossingResult[2].end.AlmostEquals(SCPoint2d{2.0, 0.0}, 1e-9));
        EXPECT_TRUE(crossingResult[3].start.AlmostEquals(SCPoint2d{2.0, 0.0}, 1e-9));

        const std::vector<SCLineSegment2d> overlap{
            SCLineSegment2d(SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}),
            SCLineSegment2d(SCPoint2d{2.0, 0.0}, SCPoint2d{6.0, 0.0}),
        };
        const std::vector<Geometry::Detail::SubdividedLineSegment2d> overlapResult =
            Geometry::Detail::SubdivideLineSegments(overlap, 1e-9, parameterTolerance);

        ASSERT_EQ(overlapResult.size(), 4U);
        EXPECT_EQ(overlapResult[0].sourceIndex, 0U);
        EXPECT_EQ(overlapResult[1].sourceIndex, 0U);
        EXPECT_EQ(overlapResult[2].sourceIndex, 1U);
        EXPECT_EQ(overlapResult[3].sourceIndex, 1U);
        EXPECT_TRUE(overlapResult[0].end.AlmostEquals(SCPoint2d{2.0, 0.0}, 1e-9));
        EXPECT_TRUE(overlapResult[1].end.AlmostEquals(SCPoint2d{4.0, 0.0}, 1e-9));
        EXPECT_TRUE(overlapResult[2].start.AlmostEquals(SCPoint2d{2.0, 0.0}, 1e-9));
        EXPECT_TRUE(overlapResult[3].start.AlmostEquals(SCPoint2d{4.0, 0.0}, 1e-9));
    }
}  // namespace
