#include <gtest/gtest.h>

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

TEST(SearchPolyCapabilityTest, InvalidInputContractRejectsEmptyLineCollection)
{
    const SCMultiPolyline2d lines{};
    const SearchPolyOptions2d options{};
    const auto result = SearchPolygons(lines, options);

    EXPECT_EQ(result.issue, SearchPolyIssue2d::InvalidInput);
    EXPECT_FALSE(result.IsSuccess());
    EXPECT_EQ(result.polygons.Count(), 0U);
    EXPECT_TRUE(result.candidates.empty());
    EXPECT_EQ(result.diagnostics.inputPolylineCount, 0U);
    EXPECT_EQ(result.diagnostics.inputSegmentCount, 0U);
    EXPECT_EQ(result.diagnostics.uniqueVertexCount, 0U);
    EXPECT_EQ(result.diagnostics.danglingEndpointCount, 0U);
    EXPECT_EQ(result.diagnostics.branchVertexCount, 0U);
    EXPECT_EQ(result.diagnostics.inferredSyntheticEdgeCount, 0U);
    EXPECT_DOUBLE_EQ(result.bestCandidateScoreMargin, 0.0);
    EXPECT_DOUBLE_EQ(result.bestCandidateSyntheticPerimeter, 0.0);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeCount, 0U);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::Unknown);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::Unknown);
    EXPECT_DOUBLE_EQ(result.runnerUpSyntheticPerimeter, 0.0);
    EXPECT_EQ(result.runnerUpSyntheticEdgeCount, 0U);
    EXPECT_EQ(result.runnerUpSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::Unknown);
    EXPECT_EQ(result.runnerUpSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::Unknown);
    EXPECT_EQ(result.runnerUpBranchVertexCount, 0U);
    EXPECT_EQ(result.bestCandidatePenaltyKind, SearchPolyPenaltyKind2d::None);
    EXPECT_EQ(result.runnerUpPenaltyKind, SearchPolyPenaltyKind2d::None);
    EXPECT_EQ(result.ambiguousTopPenaltyKind, SearchPolyPenaltyKind2d::None);
    EXPECT_EQ(result.ambiguousTopSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::Unknown);
    EXPECT_EQ(result.ambiguousTopSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::Unknown);
    EXPECT_EQ(result.ambiguousTopCandidateCountWithSyntheticEdges, 0U);
    EXPECT_EQ(result.ambiguousTopCandidateCountWithBranchPenalty, 0U);
    EXPECT_FALSE(result.bestCandidateBeatsSyntheticRunnerUp);
    EXPECT_FALSE(result.bestCandidateBeatsBranchRunnerUp);
    EXPECT_EQ(result.candidateCountWithSyntheticEdges, 0U);
    EXPECT_EQ(result.candidateCountWithBranchPenalty, 0U);
    EXPECT_EQ(result.ambiguousTopCandidateCount, 0U);
    EXPECT_FALSE(result.usedAutoClose);
    EXPECT_FALSE(result.usedAutoExtend);
    EXPECT_FALSE(result.usedSyntheticEdges);
    EXPECT_FALSE(result.usedBranchScoring);
}

TEST(SearchPolyCapabilityTest, NoClosedPolygonFoundKeepsDiagnosticsAndCandidateSetConsistent)
{
    const SCMultiPolyline2d lines{SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{2.0, 0.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{2.0, 0.0}, SCPoint2d{2.0, 1.0}}, SCPolylineClosure::Open)};

    const auto result = SearchPolygons(lines);

    EXPECT_EQ(result.issue, SearchPolyIssue2d::NoClosedPolygonFound);
    EXPECT_FALSE(result.IsSuccess());
    EXPECT_EQ(result.polygons.Count(), 0U);
    EXPECT_TRUE(result.candidates.empty());
    EXPECT_EQ(result.diagnostics.inputPolylineCount, 2U);
    EXPECT_EQ(result.diagnostics.inputSegmentCount, 2U);
    EXPECT_EQ(result.diagnostics.uniqueVertexCount, 3U);
    EXPECT_EQ(result.diagnostics.danglingEndpointCount, 2U);
    EXPECT_EQ(result.diagnostics.branchVertexCount, 0U);
    EXPECT_EQ(result.diagnostics.inferredSyntheticEdgeCount, 1U);
    EXPECT_DOUBLE_EQ(result.bestCandidateScoreMargin, 0.0);
    EXPECT_DOUBLE_EQ(result.bestCandidateSyntheticPerimeter, 0.0);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeCount, 0U);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::Unknown);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::Unknown);
    EXPECT_DOUBLE_EQ(result.runnerUpSyntheticPerimeter, 0.0);
    EXPECT_EQ(result.runnerUpSyntheticEdgeCount, 0U);
    EXPECT_EQ(result.runnerUpSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::Unknown);
    EXPECT_EQ(result.runnerUpSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::Unknown);
    EXPECT_EQ(result.runnerUpBranchVertexCount, 0U);
    EXPECT_EQ(result.bestCandidatePenaltyKind, SearchPolyPenaltyKind2d::None);
    EXPECT_EQ(result.runnerUpPenaltyKind, SearchPolyPenaltyKind2d::None);
    EXPECT_EQ(result.ambiguousTopPenaltyKind, SearchPolyPenaltyKind2d::None);
    EXPECT_EQ(result.ambiguousTopSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::Unknown);
    EXPECT_EQ(result.ambiguousTopSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::Unknown);
    EXPECT_EQ(result.ambiguousTopCandidateCountWithSyntheticEdges, 0U);
    EXPECT_EQ(result.ambiguousTopCandidateCountWithBranchPenalty, 0U);
    EXPECT_FALSE(result.bestCandidateBeatsSyntheticRunnerUp);
    EXPECT_FALSE(result.bestCandidateBeatsBranchRunnerUp);
    EXPECT_EQ(result.candidateCountWithSyntheticEdges, 0U);
    EXPECT_EQ(result.candidateCountWithBranchPenalty, 0U);
    EXPECT_EQ(result.ambiguousTopCandidateCount, 0U);
    EXPECT_FALSE(result.usedAutoClose);
    EXPECT_FALSE(result.usedAutoExtend);
    EXPECT_FALSE(result.usedSyntheticEdges);
    EXPECT_FALSE(result.usedBranchScoring);
}

TEST(SearchPolyCapabilityTest, SearchPolygonsBuildsRepresentativeClosedCandidate)
{
    const SCMultiPolyline2d lines{SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{0.0, 4.0}, SCPoint2d{0.0, 0.0}}, SCPolylineClosure::Open)};

    const SearchPolyOptions2d options{};
    const auto result = SearchPolygons(lines, options);

    ASSERT_EQ(result.issue, SearchPolyIssue2d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_EQ(result.polygons.Count(), 1U);
    ASSERT_EQ(result.candidates.size(), 1U);
    EXPECT_EQ(result.diagnostics.inputPolylineCount, 4U);
    EXPECT_EQ(result.diagnostics.inputSegmentCount, 4U);
    EXPECT_EQ(result.diagnostics.danglingEndpointCount, 0U);
    EXPECT_EQ(result.diagnostics.branchVertexCount, 0U);
    EXPECT_FALSE(result.usedAutoClose);
    EXPECT_FALSE(result.usedAutoExtend);
    EXPECT_FALSE(result.usedSyntheticEdges);
    EXPECT_FALSE(result.usedBranchScoring);
    EXPECT_EQ(result.candidates.front().rank, 0U);
    EXPECT_DOUBLE_EQ(result.bestCandidateScoreMargin, 0.0);
    EXPECT_DOUBLE_EQ(result.bestCandidateSyntheticPerimeter, 0.0);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeCount, 0U);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::Unknown);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::Unknown);
    EXPECT_DOUBLE_EQ(result.runnerUpSyntheticPerimeter, 0.0);
    EXPECT_EQ(result.runnerUpSyntheticEdgeCount, 0U);
    EXPECT_EQ(result.runnerUpSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::Unknown);
    EXPECT_EQ(result.runnerUpSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::Unknown);
    EXPECT_EQ(result.runnerUpBranchVertexCount, 0U);
    EXPECT_EQ(result.bestCandidatePenaltyKind, SearchPolyPenaltyKind2d::None);
    EXPECT_EQ(result.runnerUpPenaltyKind, SearchPolyPenaltyKind2d::None);
    EXPECT_EQ(result.ambiguousTopPenaltyKind, SearchPolyPenaltyKind2d::None);
    EXPECT_EQ(result.ambiguousTopSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::Unknown);
    EXPECT_EQ(result.ambiguousTopSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::Unknown);
    EXPECT_EQ(result.ambiguousTopCandidateCountWithSyntheticEdges, 0U);
    EXPECT_EQ(result.ambiguousTopCandidateCountWithBranchPenalty, 0U);
    EXPECT_FALSE(result.bestCandidateBeatsSyntheticRunnerUp);
    EXPECT_FALSE(result.bestCandidateBeatsBranchRunnerUp);
    EXPECT_EQ(result.candidateCountWithSyntheticEdges, 0U);
    EXPECT_EQ(result.candidateCountWithBranchPenalty, 0U);
    EXPECT_EQ(result.ambiguousTopCandidateCount, 1U);
    EXPECT_TRUE(result.candidates.front().polygon.IsValid());
    EXPECT_DOUBLE_EQ(result.candidates.front().absoluteArea, 16.0);
    EXPECT_DOUBLE_EQ(result.candidates.front().branchScore, 16.0);
    EXPECT_EQ(result.candidates.front().holeCount, 0U);
    EXPECT_EQ(result.candidates.front().inferredSyntheticEdgeCount, 0U);
    EXPECT_DOUBLE_EQ(result.candidates.front().inferredSyntheticPerimeter, 0.0);
    EXPECT_TRUE(result.candidates.front().inferredSyntheticEdges.empty());
    EXPECT_TRUE(result.candidates.front().inferredSyntheticEdgeKinds.empty());
    EXPECT_TRUE(result.candidates.front().inferredSyntheticEdgeSources.empty());
    EXPECT_TRUE(result.candidates.front().inferredSyntheticEdgeStartVertexIndices.empty());
    EXPECT_TRUE(result.candidates.front().inferredSyntheticEdgeEndVertexIndices.empty());
    EXPECT_TRUE(result.candidates.front().inferredSyntheticEdgeStartDegrees.empty());
    EXPECT_TRUE(result.candidates.front().inferredSyntheticEdgeEndDegrees.empty());
    EXPECT_TRUE(result.candidates.front().inferredSyntheticEdgeDanglingTouchCounts.empty());
    EXPECT_TRUE(result.candidates.front().inferredSyntheticEdgeBranchTouchCounts.empty());
    EXPECT_TRUE(result.candidates.front().inferredSyntheticEdgeLengths.empty());
    EXPECT_EQ(result.candidates.front().branchVertexCount, 0U);
    EXPECT_EQ(result.candidates.front().syntheticBranchVertexCount, 0U);
    EXPECT_EQ(result.candidates.front().dominantPenaltyKind, SearchPolyPenaltyKind2d::None);
    EXPECT_EQ(result.candidates.front().dominantSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::Unknown);
    EXPECT_EQ(result.candidates.front().dominantSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::Unknown);
}

TEST(SearchPolyCapabilityTest, SearchPolygonsRanksLargerCandidateFirst)
{
    const SCMultiPolyline2d lines{SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{8.0, 0.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{8.0, 0.0}, SCPoint2d{8.0, 8.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{8.0, 8.0}, SCPoint2d{0.0, 8.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{0.0, 8.0}, SCPoint2d{0.0, 0.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{12.0, 0.0}, SCPoint2d{14.0, 0.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{14.0, 0.0}, SCPoint2d{14.0, 2.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{14.0, 2.0}, SCPoint2d{12.0, 2.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{12.0, 2.0}, SCPoint2d{12.0, 0.0}}, SCPolylineClosure::Open)};

    const auto result = SearchPolygons(lines);

    ASSERT_TRUE(result.IsSuccess());
    ASSERT_EQ(result.candidates.size(), 2U);
    EXPECT_DOUBLE_EQ(result.candidates[0].absoluteArea, 64.0);
    EXPECT_DOUBLE_EQ(result.candidates[1].absoluteArea, 4.0);
    EXPECT_EQ(result.candidates[0].rank, 0U);
    EXPECT_EQ(result.candidates[1].rank, 1U);
}

TEST(SearchPolyCapabilityTest, SearchPolygonsReportsRepairDiagnosticsForNearClosedLoop)
{
    const SCMultiPolyline2d lines{SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{0.0, 4.0}, SCPoint2d{0.0, 0.3}}, SCPolylineClosure::Open)};

    const auto result = SearchPolygons(lines);

    ASSERT_TRUE(result.IsSuccess());
    EXPECT_EQ(result.diagnostics.danglingEndpointCount, 2U);
    EXPECT_GE(result.diagnostics.inferredSyntheticEdgeCount, 1U);
    EXPECT_TRUE(result.usedAutoClose);
    EXPECT_TRUE(result.usedSyntheticEdges);
    EXPECT_TRUE(result.usedBranchScoring);
    ASSERT_EQ(result.candidates.size(), 1U);
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
        EXPECT_NE(result.candidates.front().inferredSyntheticEdgeStartVertexIndices[index],
                  result.candidates.front().inferredSyntheticEdgeEndVertexIndices[index]);
        EXPECT_EQ(result.candidates.front().inferredSyntheticEdgeDanglingTouchCounts[index], 2U);
        EXPECT_EQ(result.candidates.front().inferredSyntheticEdgeBranchTouchCounts[index], 0U);
        EXPECT_EQ(result.candidates.front().inferredSyntheticEdgeStartDegrees[index], 1U);
        EXPECT_EQ(result.candidates.front().inferredSyntheticEdgeEndDegrees[index], 1U);
    }
    ASSERT_EQ(result.candidates.front().inferredSyntheticEdgeLengths.size(),
              result.candidates.front().inferredSyntheticEdgeCount);
    for (double syntheticLength : result.candidates.front().inferredSyntheticEdgeLengths)
    {
        EXPECT_GT(syntheticLength, 0.0);
    }
    EXPECT_EQ(result.candidates.front().dominantPenaltyKind, SearchPolyPenaltyKind2d::SyntheticClosure);
    EXPECT_EQ(result.candidates.front().dominantSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::GapClosure);
    EXPECT_EQ(result.candidates.front().dominantSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::SingleGapClose);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeKind, SearchPolySyntheticEdgeKind2d::GapClosure);
    EXPECT_EQ(result.bestCandidateSyntheticEdgeSource, SearchPolySyntheticEdgeSource2d::SingleGapClose);
    EXPECT_LT(result.candidates.front().branchScore, result.candidates.front().absoluteArea);
}

TEST(SearchPolyCapabilityTest, AutoFlagsRespectOptionsWhileDiagnosticsRemainStable)
{
    const SCMultiPolyline2d syntheticLines{SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}}, SCPolylineClosure::Open),
                                         SCPolyline2d({SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}}, SCPolylineClosure::Open),
                                         SCPolyline2d({SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}}, SCPolylineClosure::Open),
                                         SCPolyline2d({SCPoint2d{0.0, 4.0}, SCPoint2d{0.0, 0.3}}, SCPolylineClosure::Open)};

    SearchPolyOptions2d autoCloseDisabled{};
    autoCloseDisabled.autoClose = false;
    const auto syntheticResult = SearchPolygons(syntheticLines, autoCloseDisabled);

    ASSERT_TRUE(syntheticResult.IsSuccess());
    EXPECT_EQ(syntheticResult.diagnostics.danglingEndpointCount, 2U);
    EXPECT_TRUE(syntheticResult.usedSyntheticEdges);
    EXPECT_TRUE(syntheticResult.usedBranchScoring);
    EXPECT_FALSE(syntheticResult.usedAutoClose);
    EXPECT_FALSE(syntheticResult.usedAutoExtend);

    const SCMultiPolyline2d branchingLines{SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}}, SCPolylineClosure::Open),
                                         SCPolyline2d({SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}}, SCPolylineClosure::Open),
                                         SCPolyline2d({SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}}, SCPolylineClosure::Open),
                                         SCPolyline2d({SCPoint2d{0.0, 4.0}, SCPoint2d{0.0, 2.0}}, SCPolylineClosure::Open),
                                         SCPolyline2d({SCPoint2d{0.0, 2.0}, SCPoint2d{0.0, 0.0}}, SCPolylineClosure::Open),
                                         SCPolyline2d({SCPoint2d{0.0, 2.0}, SCPoint2d{-1.0, 2.0}}, SCPolylineClosure::Open)};

    SearchPolyOptions2d autoExtendDisabled{};
    autoExtendDisabled.autoExtend = false;
    const auto branchingResult = SearchPolygons(branchingLines, autoExtendDisabled);

    ASSERT_TRUE(branchingResult.IsSuccess());
    EXPECT_EQ(branchingResult.diagnostics.branchVertexCount, 1U);
    EXPECT_TRUE(branchingResult.usedBranchScoring);
    EXPECT_TRUE(branchingResult.usedAutoClose);
    EXPECT_FALSE(branchingResult.usedAutoExtend);
}

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
    EXPECT_TRUE(result.usedAutoExtend);
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
