#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "Core/SearchPoly.h"

using Geometry::SCMultiPolyline2d;
using Geometry::SCPoint2d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;
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
    EXPECT_EQ(result.polygons.Size(), 0U);
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
    EXPECT_EQ(result.polygons.Size(), 0U);
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
    ASSERT_EQ(result.polygons.Size(), 1U);
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

TEST(SearchPolyCapabilityTest, SmallCallerToleranceRetainsNarrowInsetCandidate)
{
    constexpr double eps = 1e-15;
    constexpr double gap = 1e-9;
    SCMultiPolyline2d lines;
    lines.Add(SCPolyline2d(
        {{0.0, 0.0}, {6.0, 0.0}, {6.0, 1.0}, {4.0, 1.0},
         {4.0, 1.0 + gap}, {6.0, 1.0 + gap}, {6.0, 4.0}, {0.0, 4.0}},
        SCPolylineClosure::Closed));
    SearchPolyOptions2d options;
    options.epsilon = eps;

    const auto result = SearchPolygons(lines, options);
    ASSERT_EQ(result.issue, SearchPolyIssue2d::None);
    ASSERT_EQ(result.candidates.size(), 1U);
    EXPECT_TRUE(result.candidates[0].polygon.IsValid(eps));
    EXPECT_TRUE(result.candidates[0].IsValid(eps));
    EXPECT_NEAR(result.candidates[0].absoluteArea, 24.0 - 2.0 * gap, 1e-11);
}

TEST(SearchPolyCapabilityTest, CandidateValidityRejectsNonFiniteMetrics)
{
    Geometry::SCSearchPolyCandidate2d candidate;
    candidate.polygon = Geometry::SCPolygon2d(SCPolyline2d(
        {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}}, SCPolylineClosure::Closed));
    candidate.absoluteArea = 4.0;
    ASSERT_TRUE(candidate.IsValid());
    ASSERT_TRUE(candidate.IsValid(1e-12));

    candidate.absoluteArea = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(candidate.IsValid());
    EXPECT_FALSE(candidate.IsValid(1e-12));
    candidate.absoluteArea = 4.0;
    candidate.branchScore = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(candidate.IsValid());
    EXPECT_FALSE(candidate.IsValid(1e-12));
    candidate.branchScore = 4.0;
    candidate.inferredSyntheticPerimeter = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(candidate.IsValid());
    EXPECT_FALSE(candidate.IsValid(1e-12));
}

TEST(SearchPolyCapabilityTest, SmallDistinctAreasRankLargerCandidateFirst)
{
    constexpr double eps = 1e-15;
    constexpr double small = 1e-7;
    constexpr double large = 1.2e-7;
    SCMultiPolyline2d lines;
    lines.Add(SCPolyline2d(
        {{1.0, 0.0}, {1.0 + large, 0.0}, {1.0 + large, large}, {1.0, large}},
        SCPolylineClosure::Closed));
    lines.Add(SCPolyline2d(
        {{0.0, 0.0}, {small, 0.0}, {small, small}, {0.0, small}}, SCPolylineClosure::Closed));
    SearchPolyOptions2d options;
    options.epsilon = eps;

    const auto result = SearchPolygons(lines, options);
    ASSERT_EQ(result.issue, SearchPolyIssue2d::None);
    ASSERT_EQ(result.candidates.size(), 2U);
    EXPECT_LT(std::abs(result.candidates[0].branchScore - result.candidates[1].branchScore), 1e-12);
    EXPECT_GT(result.candidates[0].absoluteArea, result.candidates[1].absoluteArea);
    EXPECT_EQ(result.ambiguousTopCandidateCount, 1U);
    EXPECT_NEAR(result.candidates[0].absoluteArea, large * large, 1e-17);
    EXPECT_NEAR(result.candidates[1].absoluteArea, small * small, 1e-17);
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
