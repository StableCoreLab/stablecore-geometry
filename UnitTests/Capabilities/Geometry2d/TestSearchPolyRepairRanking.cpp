#include <gtest/gtest.h>

#include <limits>

#include "Core/SearchPoly.h"
#include "Geometry2d/SCPathOps.h"

using Geometry::SCMultiPolyline2d;
using Geometry::SCPoint2d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;
using Geometry::SearchPolygons;
using SearchPolyOptions2d = Geometry::SCSearchPolyOptions2d;
using SearchPolyPenaltyKind2d = Geometry::SCSearchPolyPenaltyKind2d;
using SearchPolySyntheticEdgeKind2d = Geometry::SCSearchPolySyntheticEdgeKind2d;
using SearchPolySyntheticEdgeSource2d = Geometry::SCSearchPolySyntheticEdgeSource2d;

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

    EXPECT_FALSE(syntheticResult.IsSuccess());
    EXPECT_EQ(syntheticResult.diagnostics.danglingEndpointCount, 2U);
    EXPECT_FALSE(syntheticResult.usedSyntheticEdges);
    EXPECT_FALSE(syntheticResult.usedBranchScoring);
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
    EXPECT_FALSE(branchingResult.usedAutoClose);
    EXPECT_FALSE(branchingResult.usedAutoExtend);
}

TEST(SearchPolyCapabilityTest, SearchAndPathOpsShareRepairPolicyAndInputDiagnostics)
{
    const SCMultiPolyline2d lines{
        SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}}, SCPolylineClosure::Open),
        SCPolyline2d({SCPoint2d{0.0, 4.0}, SCPoint2d{0.0, 0.3}}, SCPolylineClosure::Open)};

    const auto built = Geometry::BuildMultiPolygonByLinesWithDiagnostics(lines);
    const auto searched = SearchPolygons(lines);
    ASSERT_TRUE(searched.IsSuccess());
    EXPECT_EQ(built.polygons.Count(), searched.polygons.Count());
    EXPECT_EQ(built.diagnostics.danglingEndpointCount, searched.diagnostics.danglingEndpointCount);
    EXPECT_EQ(built.diagnostics.branchVertexCount, searched.diagnostics.branchVertexCount);
    EXPECT_EQ(built.usedAutoClose, searched.usedAutoClose);
    EXPECT_EQ(built.usedAutoExtend, searched.usedAutoExtend);
    EXPECT_EQ(built.usedBranchCleanup, searched.usedBranchCleanup);

    SearchPolyOptions2d noRepair{};
    noRepair.allowFakeEdges = false;
    const auto noRepairSearch = SearchPolygons(lines, noRepair);
    const auto noRepairBuild = Geometry::BuildMultiPolygonByLinesWithDiagnostics(
        lines, noRepair.epsilon, {noRepair.autoClose, noRepair.autoExtend,
                                  noRepair.allowFakeEdges, noRepair.removeBranches});
    EXPECT_FALSE(noRepairSearch.IsSuccess());
    EXPECT_TRUE(noRepairBuild.polygons.IsEmpty());
    EXPECT_EQ(noRepairSearch.diagnostics.danglingEndpointCount,
              noRepairBuild.diagnostics.danglingEndpointCount);
    EXPECT_FALSE(noRepairSearch.usedAutoClose);
    EXPECT_FALSE(noRepairSearch.usedAutoExtend);
}

TEST(SearchPolyCapabilityTest, RejectsNonFiniteRebuildTolerance)
{
    const SCMultiPolyline2d lines{SCPolyline2d(
        {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}}, SCPolylineClosure::Closed)};
    SearchPolyOptions2d options{};
    options.epsilon = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(SearchPolygons(lines, options).issue, Geometry::SCSearchPolyIssue2d::InvalidInput);
    EXPECT_TRUE(Geometry::BuildMultiPolygonByLinesWithDiagnostics(lines, options.epsilon).polygons.IsEmpty());
}
