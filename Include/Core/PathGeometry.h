#pragma once

#include <cstddef>
#include <optional>
#include <span>
#include <variant>
#include <vector>

#include "Export/GeometryExport.h"
#include "Geometry2d/ISCSegment2d.h"
#include "Geometry2d/SCPolyline2d.h"
#include "Types/Geometry3d/SCArcSegment3d.h"
#include "Types/Geometry3d/SCLineSegment3d.h"
#include "Types/Geometry3d/SCPlane.h"
#include "Types/Geometry3d/SCPlaneFrame3d.h"

namespace Geometry
{
    struct GEOMETRY_API SCPathGeometryTolerance
    {
        double distanceTolerance{0.0};
        double angleToleranceRad{0.0};
        double relativeTolerance{0.0};

        [[nodiscard]] bool IsValid() const;
    };

    enum class SCPathGeometryFailure
    {
        None, InvalidTolerance, InvalidOptions, InvalidClosure, NonFiniteInput,
        InsufficientPoints, CoincidentPoints, RankDeficient, AmbiguousPlane,
        NonCollinear, InvalidPlane, DegenerateDirection, InvalidFrame,
        InvalidRadius, InvalidSweep, InvalidNormal, EndpointMismatch,
        IncompatiblePlanes, DiscontinuousPath, InsufficientRemainingPoints,
        UnsupportedSegment, Unrepresentable2d, DegenerateOffset,
        JoinNotFound, AmbiguousJoin, MiterLimitExceeded, SelfIntersection,
        NonFiniteResult, NumericalFailure
    };

    using SCPathSegment3d = std::variant<SCLineSegment3d, SCArcSegment3d>;

    enum class SCPolylineSimplificationInputKind { PointSequence, SegmentSequence };
    enum class SCCornerTurn2d { Straight, Left, Right, Reversal };

    struct GEOMETRY_API SCSourceSegmentMapping3d
    {
        std::size_t sourceSegmentIndex{0};
        std::optional<std::size_t> simplifiedSegmentIndex{};
        bool exactZeroLength{false};
    };

    struct GEOMETRY_API SCSimplifiedSegmentCoverage3d
    {
        std::size_t simplifiedSegmentIndex{0};
        std::size_t firstSourceSegmentIndex{0};
        std::size_t lastSourceSegmentIndex{0};
        double sourceStartLength{0.0};
        double sourceEndLength{0.0};
        double coveredTrueLength{0.0};
        double simplifiedGeometryLength{0.0};
    };

    struct GEOMETRY_API SCPlaneFitResult
    {
        bool success{false};
        SCPathGeometryFailure failure{SCPathGeometryFailure::None};
        std::optional<std::size_t> inputIndex{};
        std::optional<std::size_t> relatedIndex{};
        SCPlane plane{};
        std::vector<double> signedDistances{};
        double maxAbsoluteDeviation{0.0};
    };

    struct GEOMETRY_API SCPlaneFrameResult
    {
        bool success{false};
        SCPathGeometryFailure failure{SCPathGeometryFailure::None};
        std::optional<std::size_t> inputIndex{};
        std::optional<std::size_t> relatedIndex{};
        SCPlaneFrame3d frame{};
    };

    struct GEOMETRY_API SCFrameProjectionResult
    {
        bool success{false};
        SCPathGeometryFailure failure{SCPathGeometryFailure::None};
        std::optional<std::size_t> inputIndex{};
        std::optional<std::size_t> relatedIndex{};
        std::vector<SCPoint2d> points{};
        std::vector<double> signedDistances{};
    };

    struct GEOMETRY_API SCFrameLiftResult
    {
        bool success{false};
        SCPathGeometryFailure failure{SCPathGeometryFailure::None};
        std::optional<std::size_t> inputIndex{};
        std::optional<std::size_t> relatedIndex{};
        std::vector<SCPoint3d> points{};
    };

    struct GEOMETRY_API SCPolylineLengthResult
    {
        bool success{false};
        SCPathGeometryFailure failure{SCPathGeometryFailure::None};
        std::optional<std::size_t> inputIndex{};
        std::optional<std::size_t> relatedIndex{};
        std::vector<double> segmentLengths{};
        std::vector<double> cumulativeLengths{};
        double totalLength{0.0};
    };

    struct GEOMETRY_API SCPolylineSimplificationResult
    {
        bool success{false};
        SCPathGeometryFailure failure{SCPathGeometryFailure::None};
        std::optional<std::size_t> inputIndex{};
        std::optional<std::size_t> relatedIndex{};
        SCPolylineSimplificationInputKind inputKind{SCPolylineSimplificationInputKind::PointSequence};
        SCPolylineClosure closure{SCPolylineClosure::Open};
        std::vector<SCPoint3d> simplifiedPoints{};
        std::vector<SCPathSegment3d> simplifiedSegments{};
        std::vector<SCSourceSegmentMapping3d> sourceMappings{};
        std::vector<SCSimplifiedSegmentCoverage3d> coverages{};
        std::vector<double> sourceSegmentLengths{};
        std::vector<double> sourceCumulativeLengths{};
        double sourceTotalLength{0.0};
    };

    struct GEOMETRY_API SCCornerGeometryResult2d
    {
        bool success{false};
        SCPathGeometryFailure failure{SCPathGeometryFailure::None};
        std::optional<std::size_t> inputIndex{};
        std::optional<std::size_t> relatedIndex{};
        double interiorAngleRad{0.0};
        double signedTurnRad{0.0};
        SCCornerTurn2d turn{SCCornerTurn2d::Straight};
    };

    [[nodiscard]] GEOMETRY_API SCPlaneFitResult FitPlane(
        std::span<const SCPoint3d> orderedPoints, const SCPathGeometryTolerance& tolerance);
    [[nodiscard]] GEOMETRY_API SCPlaneFrameResult MakePlaneFrame(
        const SCPlane& plane, std::span<const SCPoint3d> orderedPoints,
        const SCPathGeometryTolerance& tolerance);
    [[nodiscard]] GEOMETRY_API SCPlaneFrameResult MakeCollinearFrame(
        std::span<const SCPoint3d> orderedPoints, const SCPathGeometryTolerance& tolerance);
    [[nodiscard]] GEOMETRY_API SCFrameProjectionResult ProjectToFrame(
        const SCPlaneFrame3d& frame, std::span<const SCPoint3d> points,
        const SCPathGeometryTolerance& tolerance);
    [[nodiscard]] GEOMETRY_API SCFrameLiftResult LiftFromFrame(
        const SCPlaneFrame3d& frame, std::span<const SCPoint2d> points,
        const SCPathGeometryTolerance& tolerance);
    [[nodiscard]] GEOMETRY_API SCPolylineLengthResult MeasurePolyline3d(
        std::span<const SCPoint3d> points, SCPolylineClosure closure,
        const SCPathGeometryTolerance& tolerance);
    [[nodiscard]] GEOMETRY_API SCPolylineSimplificationResult SimplifyPolylineWithMapping(
        std::span<const SCPoint3d> points, const SCPathGeometryTolerance& tolerance,
        SCPolylineClosure closure);
    [[nodiscard]] GEOMETRY_API SCPolylineSimplificationResult SimplifyPolylineWithMapping(
        std::span<const SCPathSegment3d> segments, const SCPathGeometryTolerance& tolerance,
        SCPolylineClosure closure);
    [[nodiscard]] GEOMETRY_API SCCornerGeometryResult2d EvaluateCornerGeometry(
        const ISCSegment2d& incoming, const ISCSegment2d& outgoing,
        const SCPathGeometryTolerance& tolerance);
}
