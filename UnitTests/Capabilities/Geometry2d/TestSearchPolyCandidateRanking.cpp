#include <gtest/gtest.h>

#include "Core/SearchPoly.h"

using Geometry::SCMultiPolyline2d;
using Geometry::SCPoint2d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;
using Geometry::SearchPolygons;
using SearchPolyOptions2d = Geometry::SCSearchPolyOptions2d;
using SearchPolyPenaltyKind2d = Geometry::SCSearchPolyPenaltyKind2d;
using SearchPolySyntheticEdgeKind2d = Geometry::SCSearchPolySyntheticEdgeKind2d;
using SearchPolySyntheticEdgeSource2d = Geometry::SCSearchPolySyntheticEdgeSource2d;

TEST(SearchPolyCapabilityTest, SearchPolygonsRanksCleanCandidateAheadOfSyntheticPeer)
{
    const SCMultiPolyline2d lines{SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{0.0, 4.0}, SCPoint2d{0.0, 0.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{10.0, 0.0}, SCPoint2d{14.0, 0.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{14.0, 0.0}, SCPoint2d{14.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{14.0, 4.0}, SCPoint2d{10.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{10.0, 4.0}, SCPoint2d{10.0, 0.3}}, SCPolylineClosure::Open)};

    const auto result = SearchPolygons(lines);

    ASSERT_TRUE(result.IsSuccess());
    ASSERT_EQ(result.candidates.size(), 2U);
    EXPECT_TRUE(result.usedAutoClose);
    EXPECT_TRUE(result.usedSyntheticEdges);
    EXPECT_TRUE(result.usedBranchScoring);

    const auto& first = result.candidates[0];
    const auto& second = result.candidates[1];
    EXPECT_LT(first.polygon.Bounds().MinPoint().x, 1.0);
    EXPECT_GT(second.polygon.Bounds().MinPoint().x, 9.0);
    EXPECT_EQ(first.inferredSyntheticEdgeCount, 0U);
    EXPECT_GE(second.inferredSyntheticEdgeCount, 1U);
    EXPECT_TRUE(first.inferredSyntheticEdges.empty());
    EXPECT_TRUE(first.inferredSyntheticEdgeKinds.empty());
    EXPECT_TRUE(first.inferredSyntheticEdgeSources.empty());
    EXPECT_TRUE(first.inferredSyntheticEdgeStartVertexIndices.empty());
    EXPECT_TRUE(first.inferredSyntheticEdgeEndVertexIndices.empty());
    EXPECT_TRUE(first.inferredSyntheticEdgeStartDegrees.empty());
    EXPECT_TRUE(first.inferredSyntheticEdgeEndDegrees.empty());
    EXPECT_TRUE(first.inferredSyntheticEdgeDanglingTouchCounts.empty());
    EXPECT_TRUE(first.inferredSyntheticEdgeBranchTouchCounts.empty());
    ASSERT_EQ(second.inferredSyntheticEdges.size(), second.inferredSyntheticEdgeCount);
    ASSERT_EQ(second.inferredSyntheticEdgeKinds.size(), second.inferredSyntheticEdgeCount);
    ASSERT_EQ(second.inferredSyntheticEdgeSources.size(), second.inferredSyntheticEdgeCount);
    ASSERT_EQ(second.inferredSyntheticEdgeStartVertexIndices.size(), second.inferredSyntheticEdgeCount);
    ASSERT_EQ(second.inferredSyntheticEdgeEndVertexIndices.size(), second.inferredSyntheticEdgeCount);
    ASSERT_EQ(second.inferredSyntheticEdgeStartDegrees.size(), second.inferredSyntheticEdgeCount);
    ASSERT_EQ(second.inferredSyntheticEdgeEndDegrees.size(), second.inferredSyntheticEdgeCount);
    ASSERT_EQ(second.inferredSyntheticEdgeDanglingTouchCounts.size(), second.inferredSyntheticEdgeCount);
    ASSERT_EQ(second.inferredSyntheticEdgeBranchTouchCounts.size(), second.inferredSyntheticEdgeCount);
    EXPECT_GT(first.branchScore, second.branchScore);
    EXPECT_EQ(first.rank, 0U);
    EXPECT_EQ(second.rank, 1U);
    EXPECT_DOUBLE_EQ(result.bestCandidateSyntheticPerimeter, 0.0);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeCount, 0U);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::Unknown);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::Unknown);
    EXPECT_GT(result.runnerUpSyntheticPerimeter, 0.0);
    EXPECT_GE(result.runnerUpSyntheticEdgeCount, 1U);
    EXPECT_EQ(result.runnerUpSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::GapClosure);
    EXPECT_EQ(result.runnerUpSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::SingleGapClose);
    EXPECT_EQ(result.runnerUpBranchVertexCount, 0U);
    EXPECT_EQ(result.bestCandidatePenaltyKind, SearchPolyPenaltyKind2d::None);
    EXPECT_EQ(result.runnerUpPenaltyKind, SearchPolyPenaltyKind2d::SyntheticClosure);
    EXPECT_EQ(result.ambiguousTopPenaltyKind, SearchPolyPenaltyKind2d::None);
    EXPECT_EQ(result.ambiguousTopSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::Unknown);
    EXPECT_EQ(result.ambiguousTopSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::Unknown);
    EXPECT_EQ(result.ambiguousTopCandidateCountWithSyntheticEdges, 0U);
    EXPECT_EQ(result.ambiguousTopCandidateCountWithBranchPenalty, 0U);
    EXPECT_TRUE(result.bestCandidateBeatsSyntheticRunnerUp);
    EXPECT_FALSE(result.bestCandidateBeatsBranchRunnerUp);
    EXPECT_EQ(result.candidateCountWithSyntheticEdges, 1U);
    EXPECT_EQ(result.candidateCountWithBranchPenalty, 0U);
    EXPECT_EQ(result.ambiguousTopCandidateCount, 1U);
    EXPECT_GT(result.bestCandidateScoreMargin, 0.0);
}

TEST(SearchPolyCapabilityTest, SearchPolygonsReportsFakeEdgeDiagnosticsForAmbiguousCandidate)
{
    const SCMultiPolyline2d ambiguousFakeLines{
        SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{0.0, 4.0}, SCPoint2d{0.0, 0.25}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{-0.15, 0.25}, SCPoint2d{-0.15, 0.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{-0.15, 0.0}, SCPoint2d{0.15, 0.0}}, SCPolylineClosure::Open)};

    const auto result = SearchPolygons(ambiguousFakeLines);

    ASSERT_TRUE(result.IsSuccess());
    ASSERT_EQ(result.candidates.size(), 1U);
    EXPECT_TRUE(result.usedSyntheticEdges);
    EXPECT_TRUE(result.usedBranchScoring);
    EXPECT_GE(result.candidates.front().inferredSyntheticEdgeCount, 1U);
    EXPECT_GT(result.candidates.front().inferredSyntheticPerimeter, 0.0);
    ASSERT_EQ(result.candidates.front().inferredSyntheticEdges.size(),
              result.candidates.front().inferredSyntheticEdgeCount);
    ASSERT_EQ(result.candidates.front().inferredSyntheticEdgeKinds.size(),
              result.candidates.front().inferredSyntheticEdgeCount);
    ASSERT_EQ(result.candidates.front().inferredSyntheticEdgeSources.size(),
              result.candidates.front().inferredSyntheticEdgeCount);
    ASSERT_EQ(result.candidates.front().inferredSyntheticEdgeStartVertexIndices.size(),
              result.candidates.front().inferredSyntheticEdgeCount);
    ASSERT_EQ(result.candidates.front().inferredSyntheticEdgeEndVertexIndices.size(),
              result.candidates.front().inferredSyntheticEdgeCount);
    ASSERT_EQ(result.candidates.front().inferredSyntheticEdgeStartDegrees.size(),
              result.candidates.front().inferredSyntheticEdgeCount);
    ASSERT_EQ(result.candidates.front().inferredSyntheticEdgeEndDegrees.size(),
              result.candidates.front().inferredSyntheticEdgeCount);
    ASSERT_EQ(result.candidates.front().inferredSyntheticEdgeDanglingTouchCounts.size(),
              result.candidates.front().inferredSyntheticEdgeCount);
    ASSERT_EQ(result.candidates.front().inferredSyntheticEdgeBranchTouchCounts.size(),
              result.candidates.front().inferredSyntheticEdgeCount);
    for (const auto& syntheticEdge : result.candidates.front().inferredSyntheticEdges)
    {
        EXPECT_TRUE(syntheticEdge.IsValid());
        EXPECT_GT(syntheticEdge.Length(), 0.0);
    }
    for (const auto syntheticKind : result.candidates.front().inferredSyntheticEdgeKinds)
    {
        EXPECT_EQ(syntheticKind, SearchPolySyntheticEdgeKind2d::GapClosure);
    }
    for (const auto syntheticSource : result.candidates.front().inferredSyntheticEdgeSources)
    {
        EXPECT_EQ(syntheticSource, SearchPolySyntheticEdgeSource2d::SingleGapClose);
    }
    for (std::size_t index = 0; index < result.candidates.front().inferredSyntheticEdgeCount; ++index)
    {
        EXPECT_LT(result.candidates.front().inferredSyntheticEdgeStartVertexIndices[index],
                  result.diagnostics.uniqueVertexCount);
        EXPECT_LT(result.candidates.front().inferredSyntheticEdgeEndVertexIndices[index],
                  result.diagnostics.uniqueVertexCount);
        EXPECT_EQ(result.candidates.front().inferredSyntheticEdgeDanglingTouchCounts[index], 2U);
        EXPECT_EQ(result.candidates.front().inferredSyntheticEdgeBranchTouchCounts[index], 0U);
    }
    EXPECT_LT(result.candidates.front().branchScore, result.candidates.front().absoluteArea);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeCount, result.candidates.front().inferredSyntheticEdgeCount);
    EXPECT_DOUBLE_EQ(result.bestCandidateSyntheticPerimeter, result.candidates.front().inferredSyntheticPerimeter);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::GapClosure);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::SingleGapClose);
    EXPECT_DOUBLE_EQ(result.runnerUpSyntheticPerimeter, 0.0);
    EXPECT_EQ(result.runnerUpSyntheticEdgeCount, 0U);
    EXPECT_EQ(result.runnerUpSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::Unknown);
    EXPECT_EQ(result.runnerUpSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::Unknown);
    EXPECT_EQ(result.runnerUpBranchVertexCount, 0U);
    EXPECT_EQ(result.bestCandidatePenaltyKind, SearchPolyPenaltyKind2d::SyntheticClosure);
    EXPECT_EQ(result.runnerUpPenaltyKind, SearchPolyPenaltyKind2d::None);
    EXPECT_EQ(result.ambiguousTopPenaltyKind, SearchPolyPenaltyKind2d::SyntheticClosure);
    EXPECT_EQ(result.ambiguousTopSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::GapClosure);
    EXPECT_EQ(result.ambiguousTopSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::SingleGapClose);
    EXPECT_EQ(result.ambiguousTopCandidateCountWithSyntheticEdges, 1U);
    EXPECT_EQ(result.ambiguousTopCandidateCountWithBranchPenalty, 0U);
    EXPECT_FALSE(result.bestCandidateBeatsSyntheticRunnerUp);
    EXPECT_FALSE(result.bestCandidateBeatsBranchRunnerUp);
    EXPECT_EQ(result.candidateCountWithSyntheticEdges, 1U);
    EXPECT_EQ(result.ambiguousTopCandidateCount, 1U);
}

TEST(SearchPolyCapabilityTest, SearchPolygonsAppliesBranchPenaltyAtExplicitBranchVertex)
{
    const SCMultiPolyline2d lines{SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{0.0, 4.0}, SCPoint2d{0.0, 2.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{0.0, 2.0}, SCPoint2d{0.0, 0.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{0.0, 2.0}, SCPoint2d{-1.0, 2.0}}, SCPolylineClosure::Open)};

    const auto result = SearchPolygons(lines);

    ASSERT_TRUE(result.IsSuccess());
    ASSERT_EQ(result.candidates.size(), 1U);
    EXPECT_EQ(result.diagnostics.branchVertexCount, 1U);
    EXPECT_TRUE(result.usedBranchScoring);
    EXPECT_FALSE(result.usedAutoExtend);
    EXPECT_EQ(result.candidates.front().inferredSyntheticEdgeCount, 0U);
    EXPECT_TRUE(result.candidates.front().inferredSyntheticEdges.empty());
    EXPECT_TRUE(result.candidates.front().inferredSyntheticEdgeKinds.empty());
    EXPECT_TRUE(result.candidates.front().inferredSyntheticEdgeSources.empty());
    EXPECT_TRUE(result.candidates.front().inferredSyntheticEdgeStartVertexIndices.empty());
    EXPECT_TRUE(result.candidates.front().inferredSyntheticEdgeEndVertexIndices.empty());
    EXPECT_TRUE(result.candidates.front().inferredSyntheticEdgeStartDegrees.empty());
    EXPECT_TRUE(result.candidates.front().inferredSyntheticEdgeEndDegrees.empty());
    EXPECT_TRUE(result.candidates.front().inferredSyntheticEdgeDanglingTouchCounts.empty());
    EXPECT_TRUE(result.candidates.front().inferredSyntheticEdgeBranchTouchCounts.empty());
    EXPECT_GE(result.candidates.front().branchVertexCount, 1U);
    EXPECT_TRUE(result.candidates.front().inferredSyntheticEdgeLengths.empty());
    EXPECT_EQ(result.candidates.front().dominantPenaltyKind, SearchPolyPenaltyKind2d::BranchPenalty);
    EXPECT_EQ(result.candidates.front().dominantSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::Unknown);
    EXPECT_EQ(result.candidates.front().dominantSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::Unknown);
    EXPECT_LT(result.candidates.front().branchScore, result.candidates.front().absoluteArea);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeCount, 0U);
    EXPECT_DOUBLE_EQ(result.bestCandidateSyntheticPerimeter, 0.0);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::Unknown);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::Unknown);
    EXPECT_DOUBLE_EQ(result.runnerUpSyntheticPerimeter, 0.0);
    EXPECT_EQ(result.runnerUpSyntheticEdgeCount, 0U);
    EXPECT_EQ(result.runnerUpSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::Unknown);
    EXPECT_EQ(result.runnerUpSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::Unknown);
    EXPECT_EQ(result.runnerUpBranchVertexCount, 0U);
    EXPECT_EQ(result.bestCandidatePenaltyKind, SearchPolyPenaltyKind2d::BranchPenalty);
    EXPECT_EQ(result.runnerUpPenaltyKind, SearchPolyPenaltyKind2d::None);
    EXPECT_EQ(result.ambiguousTopPenaltyKind, SearchPolyPenaltyKind2d::BranchPenalty);
    EXPECT_EQ(result.ambiguousTopSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::Unknown);
    EXPECT_EQ(result.ambiguousTopSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::Unknown);
    EXPECT_EQ(result.ambiguousTopCandidateCountWithSyntheticEdges, 0U);
    EXPECT_EQ(result.ambiguousTopCandidateCountWithBranchPenalty, 1U);
    EXPECT_FALSE(result.bestCandidateBeatsSyntheticRunnerUp);
    EXPECT_FALSE(result.bestCandidateBeatsBranchRunnerUp);
    EXPECT_EQ(result.candidateCountWithSyntheticEdges, 0U);
    EXPECT_EQ(result.candidateCountWithBranchPenalty, 1U);
    EXPECT_EQ(result.ambiguousTopCandidateCount, 1U);
}

TEST(SearchPolyCapabilityTest, BranchPenaltyScalesWithCandidateArea)
{
    const auto search = [](double scale) {
        const SCMultiPolyline2d lines{
            SCPolyline2d({{0.0, 0.0}, {4.0 * scale, 0.0}}, SCPolylineClosure::Open),
            SCPolyline2d({{4.0 * scale, 0.0}, {4.0 * scale, 4.0 * scale}}, SCPolylineClosure::Open),
            SCPolyline2d({{4.0 * scale, 4.0 * scale}, {0.0, 4.0 * scale}}, SCPolylineClosure::Open),
            SCPolyline2d({{0.0, 4.0 * scale}, {0.0, 2.0 * scale}}, SCPolylineClosure::Open),
            SCPolyline2d({{0.0, 2.0 * scale}, {0.0, 0.0}}, SCPolylineClosure::Open),
            SCPolyline2d({{0.0, 2.0 * scale}, {-1.0 * scale, 2.0 * scale}}, SCPolylineClosure::Open)};
        SearchPolyOptions2d options;
        options.epsilon = 1e-9 * scale;
        return SearchPolygons(lines, options);
    };

    const auto original = search(1.0);
    const auto reduced = search(1e-3);
    ASSERT_EQ(original.candidates.size(), 1U);
    ASSERT_EQ(reduced.candidates.size(), 1U);
    ASSERT_EQ(original.candidates[0].branchVertexCount, reduced.candidates[0].branchVertexCount);
    const double originalPenalty = original.candidates[0].absoluteArea - original.candidates[0].branchScore;
    const double reducedPenalty = reduced.candidates[0].absoluteArea - reduced.candidates[0].branchScore;
    ASSERT_GT(originalPenalty, 0.0);
    EXPECT_NEAR(reducedPenalty, originalPenalty * 1e-6, 1e-12);
}

TEST(SearchPolyCapabilityTest, SmallCallerTolerancePreservesBranchAndBoundaryEvidence)
{
    constexpr double scale = 1e-10;
    const SCMultiPolyline2d lines{
        SCPolyline2d({{0.0, 0.0}, {4.0 * scale, 0.0}}, SCPolylineClosure::Open),
        SCPolyline2d({{4.0 * scale, 0.0}, {4.0 * scale, 4.0 * scale}}, SCPolylineClosure::Open),
        SCPolyline2d({{4.0 * scale, 4.0 * scale}, {0.0, 4.0 * scale}}, SCPolylineClosure::Open),
        SCPolyline2d({{0.0, 4.0 * scale}, {0.0, 2.0 * scale}}, SCPolylineClosure::Open),
        SCPolyline2d({{0.0, 2.0 * scale}, {0.0, 0.0}}, SCPolylineClosure::Open),
        SCPolyline2d({{0.0, 2.0 * scale}, {-1.0 * scale, 2.0 * scale}}, SCPolylineClosure::Open)};
    SearchPolyOptions2d options;
    options.epsilon = 1e-15;

    const auto result = SearchPolygons(lines, options);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_EQ(result.candidates.size(), 1U);
    EXPECT_EQ(result.diagnostics.branchVertexCount, 1U);
    EXPECT_GE(result.candidates[0].branchVertexCount, 1U);
    EXPECT_EQ(result.candidates[0].inferredSyntheticEdgeCount, 0U);
    EXPECT_LT(result.candidates[0].branchScore, result.candidates[0].absoluteArea);
}
