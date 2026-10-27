#include <gtest/gtest.h>

#include <cstddef>

#include "Core/SearchPoly.h"

using Geometry::SCMultiPolyline2d;
using Geometry::SCPoint2d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;
using Geometry::SearchPolygonContainingPoint;
using Geometry::SearchPolygons;
using SearchPolyIssue2d = Geometry::SCSearchPolyIssue2d;
using SearchPolyOptions2d = Geometry::SCSearchPolyOptions2d;
using SearchPolyPenaltyKind2d = Geometry::SCSearchPolyPenaltyKind2d;
using SearchPolySyntheticEdgeKind2d = Geometry::SCSearchPolySyntheticEdgeKind2d;
using SearchPolySyntheticEdgeSource2d = Geometry::SCSearchPolySyntheticEdgeSource2d;

TEST(SearchPolyCapabilityTest, SearchPolygonsReportsTopCandidateExplanationAgainstRunnerUp)
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
    EXPECT_EQ(result.ambiguousTopCandidateCount, 1U);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeCount, 0U);
    EXPECT_DOUBLE_EQ(result.bestCandidateSyntheticPerimeter, 0.0);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::Unknown);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::Unknown);
    EXPECT_GT(result.runnerUpSyntheticPerimeter, 0.0);
    EXPECT_GE(result.runnerUpSyntheticEdgeCount, 1U);
    EXPECT_EQ(result.runnerUpSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::GapClosure);
    EXPECT_EQ(result.runnerUpSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::SingleGapClose);
    EXPECT_EQ(result.runnerUpBranchVertexCount, 0U);
    EXPECT_TRUE(result.bestCandidateBeatsSyntheticRunnerUp);
    EXPECT_FALSE(result.bestCandidateBeatsBranchRunnerUp);
    EXPECT_EQ(result.candidateCountWithSyntheticEdges, 1U);
    EXPECT_GT(result.bestCandidateScoreMargin, 0.0);
    EXPECT_DOUBLE_EQ(result.bestCandidateScoreMargin,
                     result.candidates[0].branchScore - result.candidates[1].branchScore);
}

TEST(SearchPolyCapabilityTest, SearchPolygonsExplainsWhySyntheticRunnerUpLostToCleanWinner)
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

    const auto& winner = result.candidates[0];
    const auto& runnerUp = result.candidates[1];
    EXPECT_EQ(winner.rank, 0U);
    EXPECT_EQ(runnerUp.rank, 1U);
    EXPECT_EQ(winner.dominantPenaltyKind, SearchPolyPenaltyKind2d::None);
    EXPECT_EQ(winner.dominantSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::Unknown);
    EXPECT_EQ(winner.dominantSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::Unknown);
    EXPECT_EQ(runnerUp.dominantPenaltyKind, SearchPolyPenaltyKind2d::SyntheticClosure);
    EXPECT_EQ(runnerUp.dominantSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::GapClosure);
    EXPECT_EQ(runnerUp.dominantSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::SingleGapClose);
    EXPECT_GT(result.bestCandidateScoreMargin, 0.0);
    EXPECT_TRUE(result.bestCandidateBeatsSyntheticRunnerUp);
    EXPECT_FALSE(result.bestCandidateBeatsBranchRunnerUp);
    EXPECT_EQ(result.bestCandidatePenaltyKind, SearchPolyPenaltyKind2d::None);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeCount, 0U);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::Unknown);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::Unknown);
    EXPECT_EQ(result.runnerUpPenaltyKind, SearchPolyPenaltyKind2d::SyntheticClosure);
    EXPECT_EQ(result.runnerUpSyntheticEdgeCount, runnerUp.inferredSyntheticEdgeCount);
    EXPECT_EQ(result.runnerUpSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::GapClosure);
    EXPECT_EQ(result.runnerUpSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::SingleGapClose);
    EXPECT_GT(result.runnerUpSyntheticPerimeter, 0.0);
    EXPECT_EQ(result.runnerUpBranchVertexCount, 0U);
    EXPECT_EQ(result.candidateCountWithSyntheticEdges, 1U);
    EXPECT_EQ(result.candidateCountWithBranchPenalty, 0U);
    EXPECT_EQ(result.ambiguousTopCandidateCount, 1U);
    EXPECT_EQ(result.ambiguousTopPenaltyKind, SearchPolyPenaltyKind2d::None);
    EXPECT_EQ(result.ambiguousTopSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::Unknown);
    EXPECT_EQ(result.ambiguousTopSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::Unknown);
    EXPECT_EQ(result.ambiguousTopCandidateCountWithSyntheticEdges, 0U);
    EXPECT_EQ(result.ambiguousTopCandidateCountWithBranchPenalty, 0U);
}

TEST(SearchPolyCapabilityTest, SearchPolygonsReportsRunnerUpBranchPenaltyExplanation)
{
    const SCMultiPolyline2d lines{SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{0.0, 4.0}, SCPoint2d{0.0, 0.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{10.0, 0.0}, SCPoint2d{14.0, 0.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{14.0, 0.0}, SCPoint2d{14.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{14.0, 4.0}, SCPoint2d{10.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{10.0, 4.0}, SCPoint2d{10.0, 2.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{10.0, 2.0}, SCPoint2d{10.0, 0.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{10.0, 2.0}, SCPoint2d{9.0, 2.0}}, SCPolylineClosure::Open)};

    const auto result = SearchPolygons(lines);

    ASSERT_TRUE(result.IsSuccess());
    ASSERT_EQ(result.candidates.size(), 2U);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeCount, 0U);
    EXPECT_DOUBLE_EQ(result.bestCandidateSyntheticPerimeter, 0.0);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::Unknown);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::Unknown);
    EXPECT_DOUBLE_EQ(result.runnerUpSyntheticPerimeter, 0.0);
    EXPECT_EQ(result.runnerUpSyntheticEdgeCount, 0U);
    EXPECT_EQ(result.runnerUpSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::Unknown);
    EXPECT_EQ(result.runnerUpSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::Unknown);
    EXPECT_GE(result.runnerUpBranchVertexCount, 1U);
    EXPECT_EQ(result.bestCandidatePenaltyKind, SearchPolyPenaltyKind2d::None);
    EXPECT_EQ(result.runnerUpPenaltyKind, SearchPolyPenaltyKind2d::BranchPenalty);
    EXPECT_EQ(result.ambiguousTopPenaltyKind, SearchPolyPenaltyKind2d::None);
    EXPECT_EQ(result.ambiguousTopSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::Unknown);
    EXPECT_EQ(result.ambiguousTopSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::Unknown);
    EXPECT_EQ(result.ambiguousTopCandidateCountWithSyntheticEdges, 0U);
    EXPECT_EQ(result.ambiguousTopCandidateCountWithBranchPenalty, 0U);
    EXPECT_FALSE(result.bestCandidateBeatsSyntheticRunnerUp);
    EXPECT_TRUE(result.bestCandidateBeatsBranchRunnerUp);
    EXPECT_EQ(result.candidateCountWithSyntheticEdges, 0U);
    EXPECT_EQ(result.candidateCountWithBranchPenalty, 1U);
}

TEST(SearchPolyCapabilityTest, SearchPolygonsSummarizesAmbiguousTopKindsForTiedCandidates)
{
    const SCMultiPolyline2d lines{SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{0.0, 4.0}, SCPoint2d{0.0, 0.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{10.0, 0.0}, SCPoint2d{14.0, 0.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{14.0, 0.0}, SCPoint2d{14.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{14.0, 4.0}, SCPoint2d{10.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{10.0, 4.0}, SCPoint2d{10.0, 0.0}}, SCPolylineClosure::Open)};

    const auto result = SearchPolygons(lines);

    ASSERT_TRUE(result.IsSuccess());
    ASSERT_EQ(result.candidates.size(), 2U);
    EXPECT_EQ(result.ambiguousTopCandidateCount, 2U);
    EXPECT_DOUBLE_EQ(result.bestCandidateScoreMargin, 0.0);
    EXPECT_EQ(result.ambiguousTopPenaltyKind, SearchPolyPenaltyKind2d::None);
    EXPECT_EQ(result.ambiguousTopSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::Unknown);
    EXPECT_EQ(result.ambiguousTopSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::Unknown);
    EXPECT_EQ(result.ambiguousTopCandidateCountWithSyntheticEdges, 0U);
    EXPECT_EQ(result.ambiguousTopCandidateCountWithBranchPenalty, 0U);
    EXPECT_EQ(result.bestCandidatePenaltyKind, SearchPolyPenaltyKind2d::None);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::Unknown);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::Unknown);
}

TEST(SearchPolyCapabilityTest, SearchPolygonsSummarizesAmbiguousTopSyntheticSourcesForTiedGapClosures)
{
    const SCMultiPolyline2d lines{SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{0.0, 4.0}, SCPoint2d{0.0, 0.3}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{10.0, 0.0}, SCPoint2d{14.0, 0.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{14.0, 0.0}, SCPoint2d{14.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{14.0, 4.0}, SCPoint2d{10.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{10.0, 4.0}, SCPoint2d{10.0, 0.3}}, SCPolylineClosure::Open)};

    const auto result = SearchPolygons(lines);

    ASSERT_TRUE(result.IsSuccess());
    ASSERT_EQ(result.candidates.size(), 2U);
    EXPECT_EQ(result.ambiguousTopCandidateCount, 2U);
    EXPECT_DOUBLE_EQ(result.bestCandidateScoreMargin, 0.0);
    EXPECT_EQ(result.bestCandidatePenaltyKind, SearchPolyPenaltyKind2d::SyntheticClosure);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::GapClosure);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::SingleGapClose);
    EXPECT_EQ(result.runnerUpPenaltyKind, SearchPolyPenaltyKind2d::SyntheticClosure);
    EXPECT_EQ(result.runnerUpSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::GapClosure);
    EXPECT_EQ(result.runnerUpSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::SingleGapClose);
    EXPECT_EQ(result.ambiguousTopPenaltyKind, SearchPolyPenaltyKind2d::SyntheticClosure);
    EXPECT_EQ(result.ambiguousTopSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::GapClosure);
    EXPECT_EQ(result.ambiguousTopSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::SingleGapClose);
    EXPECT_EQ(result.ambiguousTopCandidateCountWithSyntheticEdges, 2U);
    EXPECT_EQ(result.ambiguousTopCandidateCountWithBranchPenalty, 0U);
    EXPECT_EQ(result.candidateCountWithSyntheticEdges, 2U);
}

TEST(SearchPolyCapabilityTest, SearchPolygonsClassifiesBranchCleanupSyntheticEdges)
{
    const SCMultiPolyline2d lines{SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{0.0, 4.0}, SCPoint2d{-1.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{0.0, 4.0}, SCPoint2d{0.0, 5.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{-1.0, 0.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{0.0, -1.0}}, SCPolylineClosure::Open)};

    const auto result = SearchPolygons(lines);

    ASSERT_TRUE(result.IsSuccess());
    EXPECT_TRUE(result.usedBranchCleanup);
    ASSERT_EQ(result.candidates.size(), 1U);
    const auto& candidate = result.candidates.front();
    ASSERT_EQ(candidate.inferredSyntheticEdgeKinds.size(), candidate.inferredSyntheticEdgeCount);
    ASSERT_EQ(candidate.inferredSyntheticEdgeSources.size(), candidate.inferredSyntheticEdgeCount);
    ASSERT_EQ(candidate.inferredSyntheticEdgeStartVertexIndices.size(), candidate.inferredSyntheticEdgeCount);
    ASSERT_EQ(candidate.inferredSyntheticEdgeEndVertexIndices.size(), candidate.inferredSyntheticEdgeCount);
    ASSERT_EQ(candidate.inferredSyntheticEdgeStartDegrees.size(), candidate.inferredSyntheticEdgeCount);
    ASSERT_EQ(candidate.inferredSyntheticEdgeEndDegrees.size(), candidate.inferredSyntheticEdgeCount);
    ASSERT_EQ(candidate.inferredSyntheticEdgeDanglingTouchCounts.size(), candidate.inferredSyntheticEdgeCount);
    ASSERT_EQ(candidate.inferredSyntheticEdgeBranchTouchCounts.size(), candidate.inferredSyntheticEdgeCount);
    ASSERT_GE(candidate.inferredSyntheticEdgeKinds.size(), 1U);
    EXPECT_EQ(candidate.dominantSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::BranchCleanup);
    EXPECT_EQ(candidate.dominantSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::BranchCleanup);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::BranchCleanup);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::BranchCleanup);
    for (const auto syntheticKind : candidate.inferredSyntheticEdgeKinds)
    {
        EXPECT_EQ(syntheticKind, SearchPolySyntheticEdgeKind2d::BranchCleanup);
    }
    for (const auto syntheticSource : candidate.inferredSyntheticEdgeSources)
    {
        EXPECT_EQ(syntheticSource, SearchPolySyntheticEdgeSource2d::BranchCleanup);
    }
    for (std::size_t index = 0; index < candidate.inferredSyntheticEdgeCount; ++index)
    {
        EXPECT_LT(candidate.inferredSyntheticEdgeStartVertexIndices[index], result.diagnostics.uniqueVertexCount);
        EXPECT_LT(candidate.inferredSyntheticEdgeEndVertexIndices[index], result.diagnostics.uniqueVertexCount);
        EXPECT_NE(candidate.inferredSyntheticEdgeStartVertexIndices[index],
                  candidate.inferredSyntheticEdgeEndVertexIndices[index]);
        EXPECT_EQ(candidate.inferredSyntheticEdgeDanglingTouchCounts[index], 0U);
        EXPECT_EQ(candidate.inferredSyntheticEdgeBranchTouchCounts[index], 2U);
        EXPECT_GT(candidate.inferredSyntheticEdgeStartDegrees[index], 2U);
        EXPECT_GT(candidate.inferredSyntheticEdgeEndDegrees[index], 2U);
    }
}

TEST(SearchPolyCapabilityTest, SearchPolygonsClassifiesMixedSyntheticEdges)
{
    const SCMultiPolyline2d lines{SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{0.0, 4.0}, SCPoint2d{-1.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{0.0, 4.0}, SCPoint2d{0.0, 5.0}}, SCPolylineClosure::Open)};

    const auto result = SearchPolygons(lines);

    ASSERT_TRUE(result.IsSuccess());
    ASSERT_EQ(result.candidates.size(), 1U);
    const auto& candidate = result.candidates.front();
    ASSERT_EQ(candidate.inferredSyntheticEdgeKinds.size(), candidate.inferredSyntheticEdgeCount);
    ASSERT_EQ(candidate.inferredSyntheticEdgeSources.size(), candidate.inferredSyntheticEdgeCount);
    ASSERT_EQ(candidate.inferredSyntheticEdgeStartVertexIndices.size(), candidate.inferredSyntheticEdgeCount);
    ASSERT_EQ(candidate.inferredSyntheticEdgeEndVertexIndices.size(), candidate.inferredSyntheticEdgeCount);
    ASSERT_EQ(candidate.inferredSyntheticEdgeStartDegrees.size(), candidate.inferredSyntheticEdgeCount);
    ASSERT_EQ(candidate.inferredSyntheticEdgeEndDegrees.size(), candidate.inferredSyntheticEdgeCount);
    ASSERT_EQ(candidate.inferredSyntheticEdgeDanglingTouchCounts.size(), candidate.inferredSyntheticEdgeCount);
    ASSERT_EQ(candidate.inferredSyntheticEdgeBranchTouchCounts.size(), candidate.inferredSyntheticEdgeCount);
    ASSERT_GE(candidate.inferredSyntheticEdgeKinds.size(), 1U);
    EXPECT_EQ(candidate.dominantSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::Mixed);
    EXPECT_EQ(candidate.dominantSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::MixedBridge);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::Mixed);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::MixedBridge);
    for (const auto syntheticKind : candidate.inferredSyntheticEdgeKinds)
    {
        EXPECT_EQ(syntheticKind, SearchPolySyntheticEdgeKind2d::Mixed);
    }
    for (const auto syntheticSource : candidate.inferredSyntheticEdgeSources)
    {
        EXPECT_EQ(syntheticSource, SearchPolySyntheticEdgeSource2d::MixedBridge);
    }
    for (std::size_t index = 0; index < candidate.inferredSyntheticEdgeCount; ++index)
    {
        EXPECT_LT(candidate.inferredSyntheticEdgeStartVertexIndices[index], result.diagnostics.uniqueVertexCount);
        EXPECT_LT(candidate.inferredSyntheticEdgeEndVertexIndices[index], result.diagnostics.uniqueVertexCount);
        EXPECT_NE(candidate.inferredSyntheticEdgeStartVertexIndices[index],
                  candidate.inferredSyntheticEdgeEndVertexIndices[index]);
        EXPECT_EQ(candidate.inferredSyntheticEdgeDanglingTouchCounts[index], 1U);
        EXPECT_EQ(candidate.inferredSyntheticEdgeBranchTouchCounts[index], 1U);
    }
}

TEST(SearchPolyCapabilityTest, SearchPolygonContainingPointReturnsSmallestContainingCandidate)
{
    const SCMultiPolyline2d lines{SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{10.0, 0.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{10.0, 0.0}, SCPoint2d{10.0, 10.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{10.0, 10.0}, SCPoint2d{0.0, 10.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{0.0, 10.0}, SCPoint2d{0.0, 0.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{20.0, 0.0}, SCPoint2d{24.0, 0.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{24.0, 0.0}, SCPoint2d{24.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{24.0, 4.0}, SCPoint2d{20.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{20.0, 4.0}, SCPoint2d{20.0, 0.0}}, SCPolylineClosure::Open)};

    const auto candidate = SearchPolygonContainingPoint(lines, SCPoint2d{22.0, 2.0});

    ASSERT_TRUE(candidate.has_value());
    EXPECT_TRUE(candidate->IsValid());
    EXPECT_DOUBLE_EQ(candidate->absoluteArea, 16.0);
    EXPECT_EQ(candidate->rank, 1U);
}

TEST(SearchPolyCapabilityTest, SearchPolygonContainingPointPreservesSyntheticExplanation)
{
    const SCMultiPolyline2d lines{SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{8.0, 0.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{8.0, 0.0}, SCPoint2d{8.0, 8.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{8.0, 8.0}, SCPoint2d{0.0, 8.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{0.0, 8.0}, SCPoint2d{0.0, 0.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{20.0, 0.0}, SCPoint2d{24.0, 0.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{24.0, 0.0}, SCPoint2d{24.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{24.0, 4.0}, SCPoint2d{20.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{20.0, 4.0}, SCPoint2d{20.0, 0.3}}, SCPolylineClosure::Open)};

    const auto candidate = SearchPolygonContainingPoint(lines, SCPoint2d{22.0, 2.0});

    ASSERT_TRUE(candidate.has_value());
    EXPECT_TRUE(candidate->IsValid());
    EXPECT_EQ(candidate->rank, 1U);
    EXPECT_EQ(candidate->dominantPenaltyKind, SearchPolyPenaltyKind2d::SyntheticClosure);
    EXPECT_EQ(candidate->dominantSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::GapClosure);
    EXPECT_EQ(candidate->dominantSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::SingleGapClose);
    EXPECT_GE(candidate->inferredSyntheticEdgeCount, 1U);
    ASSERT_EQ(candidate->inferredSyntheticEdges.size(), candidate->inferredSyntheticEdgeCount);
    ASSERT_EQ(candidate->inferredSyntheticEdgeKinds.size(), candidate->inferredSyntheticEdgeCount);
    ASSERT_EQ(candidate->inferredSyntheticEdgeSources.size(), candidate->inferredSyntheticEdgeCount);
    ASSERT_EQ(candidate->inferredSyntheticEdgeStartVertexIndices.size(), candidate->inferredSyntheticEdgeCount);
    ASSERT_EQ(candidate->inferredSyntheticEdgeEndVertexIndices.size(), candidate->inferredSyntheticEdgeCount);
    ASSERT_EQ(candidate->inferredSyntheticEdgeDanglingTouchCounts.size(), candidate->inferredSyntheticEdgeCount);
    ASSERT_EQ(candidate->inferredSyntheticEdgeBranchTouchCounts.size(), candidate->inferredSyntheticEdgeCount);
    for (std::size_t index = 0; index < candidate->inferredSyntheticEdgeCount; ++index)
    {
        EXPECT_EQ(candidate->inferredSyntheticEdgeKinds[index], SearchPolySyntheticEdgeKind2d::GapClosure);
        EXPECT_EQ(candidate->inferredSyntheticEdgeSources[index], SearchPolySyntheticEdgeSource2d::SingleGapClose);
        EXPECT_EQ(candidate->inferredSyntheticEdgeDanglingTouchCounts[index], 2U);
        EXPECT_EQ(candidate->inferredSyntheticEdgeBranchTouchCounts[index], 0U);
    }
}
