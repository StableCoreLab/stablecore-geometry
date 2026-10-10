#include "Core/ArcOps3d.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

#include "../Detail/PathNumerics3d.h"

namespace Geometry
{
    SCArcConstruction3dResult CreateArcSegment3d(
        const SCPoint3d& start, const SCPoint3d& end, const SCPoint3d& center,
        const SCVector3d& unitNormal, double radius, double signedSweepRad,
        const SCPathGeometryTolerance& tolerance)
    {
        SCArcConstruction3dResult result;
        if (!tolerance.IsValid())
        { result.failure = SCPathGeometryFailure::InvalidTolerance; return result; }
        const std::array<SCPoint3d, 3> points{start, end, center};
        for (std::size_t i = 0; i < points.size(); ++i)
        {
            if (!points[i].IsValid())
            { result.failure = SCPathGeometryFailure::NonFiniteInput; result.inputIndex = i; return result; }
        }
        if (!unitNormal.IsValid() || !std::isfinite(radius) || !std::isfinite(signedSweepRad))
        { result.failure = SCPathGeometryFailure::NonFiniteInput; return result; }
        if (radius <= tolerance.distanceTolerance)
        { result.failure = SCPathGeometryFailure::InvalidRadius; return result; }
        if (std::abs(signedSweepRad) <= tolerance.angleToleranceRad ||
            std::abs(signedSweepRad) > 2.0 * std::numbers::pi_v<double>)
        { result.failure = SCPathGeometryFailure::InvalidSweep; return result; }
        const double normalLength = PathDetail::Norm(unitNormal);
        if (!std::isfinite(normalLength) || normalLength == 0.0 ||
            std::abs(normalLength - 1.0) > tolerance.relativeTolerance)
        { result.failure = SCPathGeometryFailure::InvalidNormal; return result; }
        const SCVector3d n = unitNormal / normalLength;
        const SCVector3d radial = start - center;
        const SCVector3d endRadial = end - center;
        const double radialLength = PathDetail::Norm(radial);
        const double endLength = PathDetail::Norm(endRadial);
        if (!std::isfinite(radialLength) || !std::isfinite(endLength))
        { result.failure = SCPathGeometryFailure::NonFiniteResult; return result; }
        const double startPlaneDistance = Dot(radial, n);
        const double endPlaneDistance = Dot(endRadial, n);
        if (!std::isfinite(startPlaneDistance) || !std::isfinite(endPlaneDistance))
        { result.failure = SCPathGeometryFailure::NonFiniteResult; return result; }
        if (std::abs(radialLength - radius) > tolerance.distanceTolerance ||
            std::abs(startPlaneDistance) > tolerance.distanceTolerance)
        { result.failure = SCPathGeometryFailure::EndpointMismatch; result.inputIndex = 0; return result; }
        if (std::abs(endLength - radius) > tolerance.distanceTolerance ||
            std::abs(endPlaneDistance) > tolerance.distanceTolerance)
        { result.failure = SCPathGeometryFailure::EndpointMismatch; result.inputIndex = 1; return result; }
        const SCVector3d projected = radial - n * startPlaneDistance;
        const double projectedLength = PathDetail::Norm(projected);
        if (!std::isfinite(projectedLength) || projectedLength == 0.0)
        { result.failure = SCPathGeometryFailure::DegenerateDirection; return result; }
        SCVector3d e = projected / projectedLength;
        e = e - n * Dot(e, n);
        const double correctedLength = PathDetail::Norm(e);
        if (!std::isfinite(correctedLength) || correctedLength == 0.0)
        { result.failure = SCPathGeometryFailure::NumericalFailure; return result; }
        e = e / correctedLength;
        const SCArcSegment3d arc{center, n, e, radius, signedSweepRad};
        const auto arcFailure = PathDetail::ValidateArc(arc, tolerance);
        if (arcFailure != SCPathGeometryFailure::None)
        {
            result.failure = arcFailure == SCPathGeometryFailure::NonFiniteResult
                ? arcFailure : SCPathGeometryFailure::NumericalFailure;
            return result;
        }
        const SCPoint3d actualStart = PathDetail::ArcPoint(arc, 0.0);
        const SCPoint3d actualEnd = PathDetail::ArcPoint(arc, 1.0);
        const double startError = PathDetail::Norm(actualStart - start);
        const double endError = PathDetail::Norm(actualEnd - end);
        if (!std::isfinite(startError) || !std::isfinite(endError))
        { result.failure = SCPathGeometryFailure::NonFiniteResult; return result; }
        if (startError > tolerance.distanceTolerance)
        { result.failure = SCPathGeometryFailure::EndpointMismatch; result.inputIndex = 0; return result; }
        if (endError > tolerance.distanceTolerance)
        { result.failure = SCPathGeometryFailure::EndpointMismatch; result.inputIndex = 1; return result; }
        for (std::size_t i = 0; i < result.fitSamples.size(); ++i)
        {
            result.fitSamples[i] = PathDetail::ArcPoint(arc, static_cast<double>(i) / 4.0);
            if (!result.fitSamples[i].IsValid())
            { result = {}; result.failure = SCPathGeometryFailure::NonFiniteResult; return result; }
        }
        result.samples = {result.fitSamples[0], result.fitSamples[2], result.fitSamples[4]};
        result.trueLength = radius * std::abs(signedSweepRad);
        if (!std::isfinite(result.trueLength))
        { result = {}; result.failure = SCPathGeometryFailure::NonFiniteResult; return result; }
        result.arc = arc;
        result.success = true;
        return result;
    }

    SCArcProjectionResult ProjectArcToFrame(const SCArcSegment3d& arc,
                                             const SCPlaneFrame3d& frame,
                                             const SCPathGeometryTolerance& tolerance)
    {
        SCArcProjectionResult result;
        if (!tolerance.IsValid())
        { result.failure = SCPathGeometryFailure::InvalidTolerance; return result; }
        if (!arc.center.IsValid() || !arc.unitNormal.IsValid() ||
            !arc.startRadialUnit.IsValid() || !std::isfinite(arc.radius) ||
            !std::isfinite(arc.signedSweepRad))
        { result.failure = SCPathGeometryFailure::NonFiniteInput; return result; }
        if (!frame.origin.IsValid() || !frame.u.IsValid() || !frame.v.IsValid() ||
            !frame.n.IsValid())
        { result.failure = SCPathGeometryFailure::InvalidFrame; return result; }
        const auto arcFailure = PathDetail::ValidateArc(arc, tolerance);
        if (arcFailure != SCPathGeometryFailure::None)
        { result.failure = arcFailure; return result; }
        if (!PathDetail::ValidFrame(frame, tolerance))
        { result.failure = SCPathGeometryFailure::InvalidFrame; return result; }
        const double alignment = Dot(arc.unitNormal, frame.n);
        if (std::abs(alignment) <= 0.0 ||
            PathDetail::Norm(Cross(arc.unitNormal, frame.n)) >
                std::sin(tolerance.angleToleranceRad))
        { result.failure = SCPathGeometryFailure::IncompatiblePlanes; return result; }
        const SCVector3d tangent = Cross(arc.unitNormal, arc.startRadialUnit);
        const double a = Dot(arc.center - frame.origin, frame.n);
        const double b = arc.radius * Dot(arc.startRadialUnit, frame.n);
        const double c = arc.radius * Dot(tangent, frame.n);
        if (!std::isfinite(a) || !std::isfinite(b) || !std::isfinite(c))
        { result.failure = SCPathGeometryFailure::NonFiniteResult; return result; }
        const double low = std::min(0.0, arc.signedSweepRad);
        const double high = std::max(0.0, arc.signedSweepRad);
        double deviation = std::max(std::abs(a + b * std::cos(low) + c * std::sin(low)),
                                    std::abs(a + b * std::cos(high) + c * std::sin(high)));
        const double stationary = std::atan2(c, b);
        for (int turn = -4; turn <= 4; ++turn)
        {
            const double angle = stationary + turn * std::numbers::pi_v<double>;
            if (angle >= low && angle <= high)
            { deviation = std::max(deviation, std::abs(a + b * std::cos(angle) + c * std::sin(angle))); }
        }
        if (!std::isfinite(deviation))
        { result.failure = SCPathGeometryFailure::NonFiniteResult; return result; }
        if (deviation > tolerance.distanceTolerance)
        { result.failure = SCPathGeometryFailure::IncompatiblePlanes; return result; }
        const std::array<SCPoint3d, 2> anchors{arc.center, PathDetail::ArcPoint(arc, 0.0)};
        const auto anchorProjection = ProjectToFrame(frame, anchors, tolerance);
        if (!anchorProjection.success)
        { result.failure = anchorProjection.failure; return result; }
        const SCPoint2d center = anchorProjection.points[0];
        const SCVector2d radial = anchorProjection.points[1] - center;
        const double startAngle = std::atan2(radial.y, radial.x);
        const double sweep = alignment > 0.0 ? arc.signedSweepRad : -arc.signedSweepRad;
        SCArcSegment2d projected = SCArcSegment2d::FromCenterRadiusStartSweep(
            center, arc.radius, startAngle, sweep);
        if (!projected.IsValid())
        { result.failure = SCPathGeometryFailure::Unrepresentable2d; return result; }
        std::array<SCPoint3d, 5> samples3d{};
        for (std::size_t i = 0; i < samples3d.size(); ++i)
        { samples3d[i] = PathDetail::ArcPoint(arc, static_cast<double>(i) / 4.0); }
        const auto samples2d = ProjectToFrame(frame, samples3d, tolerance);
        if (!samples2d.success)
        { result.failure = samples2d.failure; return result; }
        for (std::size_t i = 0; i < samples3d.size(); ++i)
        {
            const SCPoint2d sample = projected.PointAt(static_cast<double>(i) / 4.0);
            if (!sample.IsValid())
            { result.failure = SCPathGeometryFailure::NonFiniteResult; return result; }
            const double sampleError = PathDetail::Norm(sample - samples2d.points[i]);
            if (!std::isfinite(sampleError))
            { result.failure = SCPathGeometryFailure::NonFiniteResult; return result; }
            if (sampleError > tolerance.distanceTolerance)
            { result.failure = SCPathGeometryFailure::IncompatiblePlanes; return result; }
        }
        const double trueLength3d = arc.radius * std::abs(arc.signedSweepRad);
        const double projectedLength2d = projected.Length();
        if (!std::isfinite(trueLength3d) || !std::isfinite(projectedLength2d))
        { result.failure = SCPathGeometryFailure::NonFiniteResult; return result; }
        result.samples = {projected.PointAt(0.0), projected.PointAt(0.5), projected.PointAt(1.0)};
        if (!result.samples[0].IsValid() || !result.samples[1].IsValid() ||
            !result.samples[2].IsValid())
        { result = {}; result.failure = SCPathGeometryFailure::NonFiniteResult; return result; }
        result.trueLength3d = trueLength3d;
        result.projectedLength2d = projectedLength2d;
        result.arc = projected;
        result.success = true;
        return result;
    }
}
