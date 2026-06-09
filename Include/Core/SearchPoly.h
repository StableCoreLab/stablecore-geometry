#pragma once
#include <optional>
#include <vector>
#include "Core/GeometryTypes.h"
#include "Export/GeometryExport.h"
#include "Geometry2d/SCPolygon2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Geometry2d/SCMultiPolygon2d.h"
#include "Geometry2d/SCMultiPolyline2d.h"

namespace Geometry
{
    enum class SCSearchPolyIssue2d
    {
        None,
        InvalidInput,
        NoClosedPolygonFound
    };

    enum class SCSearchPolyPenaltyKind2d
    {
        None,
        SyntheticClosure,
        BranchPenalty,
        SyntheticBranchPenalty,
        Mixed
    };

    enum class SCSearchPolySyntheticEdgeKind2d
    {
        Unknown,
        GapClosure,
        BranchCleanup,
        Mixed
    };

    enum class SCSearchPolySyntheticEdgeSource2d
    {
        Unknown,
        SingleGapClose,
        BranchCleanup,
        MixedBridge
    };

    struct GEOMETRY_API SCSearchPolyDiagnostics2d
    {
        std::size_t inputPolylineCount{0};
        std::size_t inputSegmentCount{0};
        std::size_t uniqueVertexCount{0};
        std::size_t danglingEndpointCount{0};
        std::size_t branchVertexCount{0};
        std::size_t inferredSyntheticEdgeCount{0};

        [[nodiscard]] bool RequiresRepair() const
        {
            return danglingEndpointCount > 0 || branchVertexCount > 0;
        }
    };

    struct GEOMETRY_API SCSearchPolyOptions2d
    {
        double epsilon{1e-9};
        bool autoClose{true};
        bool autoExtend{true};
        bool allowFakeEdges{true};
        bool removeBranches{true};
    };

    struct GEOMETRY_API SCSearchPolyCandidate2d
    {
        SCPolygon2d polygon{};
        double absoluteArea{0.0};
        double branchScore{0.0};
        double inferredSyntheticPerimeter{0.0};
        std::size_t holeCount{0};
        std::size_t inferredSyntheticEdgeCount{0};
        std::size_t branchVertexCount{0};
        std::size_t syntheticBranchVertexCount{0};
        SCSearchPolyPenaltyKind2d dominantPenaltyKind{SCSearchPolyPenaltyKind2d::None};
        SCSearchPolySyntheticEdgeKind2d dominantSyntheticEdgeKind{SCSearchPolySyntheticEdgeKind2d::Unknown};
        SCSearchPolySyntheticEdgeSource2d dominantSyntheticEdgeSource{SCSearchPolySyntheticEdgeSource2d::Unknown};
        std::vector<SCLineSegment2d> inferredSyntheticEdges{};
        std::vector<SCSearchPolySyntheticEdgeKind2d> inferredSyntheticEdgeKinds{};
        std::vector<SCSearchPolySyntheticEdgeSource2d> inferredSyntheticEdgeSources{};
        std::vector<std::size_t> inferredSyntheticEdgeStartVertexIndices{};
        std::vector<std::size_t> inferredSyntheticEdgeEndVertexIndices{};
        std::vector<std::size_t> inferredSyntheticEdgeStartDegrees{};
        std::vector<std::size_t> inferredSyntheticEdgeEndDegrees{};
        std::vector<std::size_t> inferredSyntheticEdgeDanglingTouchCounts{};
        std::vector<std::size_t> inferredSyntheticEdgeBranchTouchCounts{};
        std::vector<double> inferredSyntheticEdgeLengths{};
        std::size_t rank{0};

        [[nodiscard]] bool IsValid() const
        {
            return polygon.IsValid() && absoluteArea >= 0.0 && inferredSyntheticPerimeter >= 0.0;
        }
    };

    struct GEOMETRY_API SCSearchPolyResult2d
    {
        SCSearchPolyIssue2d issue{SCSearchPolyIssue2d::None};
        SCMultiPolygon2d polygons{};
        std::vector<SCSearchPolyCandidate2d> candidates{};
        SCSearchPolyDiagnostics2d diagnostics{};
        double bestCandidateScoreMargin{0.0};
        double bestCandidateSyntheticPerimeter{0.0};
        std::size_t bestCandidateSyntheticEdgeCount{0};
        SCSearchPolySyntheticEdgeKind2d bestCandidateSyntheticEdgeKind{SCSearchPolySyntheticEdgeKind2d::Unknown};
        SCSearchPolySyntheticEdgeSource2d bestCandidateSyntheticEdgeSource{SCSearchPolySyntheticEdgeSource2d::Unknown};
        double runnerUpSyntheticPerimeter{0.0};
        std::size_t runnerUpSyntheticEdgeCount{0};
        SCSearchPolySyntheticEdgeKind2d runnerUpSyntheticEdgeKind{SCSearchPolySyntheticEdgeKind2d::Unknown};
        SCSearchPolySyntheticEdgeSource2d runnerUpSyntheticEdgeSource{SCSearchPolySyntheticEdgeSource2d::Unknown};
        std::size_t runnerUpBranchVertexCount{0};
        SCSearchPolyPenaltyKind2d bestCandidatePenaltyKind{SCSearchPolyPenaltyKind2d::None};
        SCSearchPolyPenaltyKind2d runnerUpPenaltyKind{SCSearchPolyPenaltyKind2d::None};
        SCSearchPolyPenaltyKind2d ambiguousTopPenaltyKind{SCSearchPolyPenaltyKind2d::None};
        SCSearchPolySyntheticEdgeKind2d ambiguousTopSyntheticEdgeKind{SCSearchPolySyntheticEdgeKind2d::Unknown};
        SCSearchPolySyntheticEdgeSource2d ambiguousTopSyntheticEdgeSource{SCSearchPolySyntheticEdgeSource2d::Unknown};
        std::size_t ambiguousTopCandidateCountWithSyntheticEdges{0};
        std::size_t ambiguousTopCandidateCountWithBranchPenalty{0};
        bool bestCandidateBeatsSyntheticRunnerUp{false};
        bool bestCandidateBeatsBranchRunnerUp{false};
        std::size_t candidateCountWithSyntheticEdges{0};
        std::size_t candidateCountWithBranchPenalty{0};
        std::size_t ambiguousTopCandidateCount{0};
        bool usedAutoClose{false};
        bool usedAutoExtend{false};
        bool usedSyntheticEdges{false};
        bool usedBranchScoring{false};

        [[nodiscard]] bool IsSuccess() const
        {
            return issue == SCSearchPolyIssue2d::None && polygons.Count() > 0;
        }
    };

    [[nodiscard]] GEOMETRY_API SCSearchPolyResult2d SearchPolygons(const SCMultiPolyline2d& lines,
                                                                 SCSearchPolyOptions2d options = {});

    [[nodiscard]] GEOMETRY_API std::optional<SCSearchPolyCandidate2d> SearchPolygonContainingPoint(
        const SCMultiPolyline2d& lines, const SCPoint2d& point, SCSearchPolyOptions2d options = {});
}  // namespace Geometry

