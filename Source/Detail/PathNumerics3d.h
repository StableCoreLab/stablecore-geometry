#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

#include "Core/PathGeometry.h"
#include "Types/Geometry2d/SCVector2.h"

namespace Geometry::PathDetail
{
    inline constexpr double kPathRepresentationTolerance = 64.0 * std::numeric_limits<double>::epsilon();

    [[nodiscard]] inline double Norm(const SCVector3d& value)
    {
        return std::hypot(value.x, value.y, value.z);
    }

    [[nodiscard]] inline double Norm(const SCVector2d& value)
    {
        return std::hypot(value.x, value.y);
    }

    [[nodiscard]] inline bool ValidClosure(SCPolylineClosure closure)
    {
        return closure == SCPolylineClosure::Open || closure == SCPolylineClosure::Closed;
    }

    [[nodiscard]] inline bool Unit(const SCVector3d& value, const SCPathGeometryTolerance& tolerance)
    {
        const double length = Norm(value);
        return std::isfinite(length) &&
               std::abs(length - 1.0) <= std::min(tolerance.relativeTolerance, kPathRepresentationTolerance);
    }

    [[nodiscard]] inline bool Orthogonal(const SCVector3d& first, const SCVector3d& second,
                                         const SCPathGeometryTolerance& tolerance)
    {
        return std::abs(Dot(first, second)) <=
               std::min(std::sin(tolerance.angleToleranceRad), kPathRepresentationTolerance);
    }

    [[nodiscard]] inline bool ValidFrame(const SCPlaneFrame3d& frame,
                                         const SCPathGeometryTolerance& tolerance)
    {
        return frame.origin.IsValid() && frame.u.IsValid() && frame.v.IsValid() && frame.n.IsValid() &&
               Unit(frame.u, tolerance) && Unit(frame.v, tolerance) && Unit(frame.n, tolerance) &&
               Orthogonal(frame.u, frame.v, tolerance) && Orthogonal(frame.u, frame.n, tolerance) &&
               Orthogonal(frame.v, frame.n, tolerance) &&
               Norm(Cross(frame.u, frame.v) - frame.n) <= 4.0 * kPathRepresentationTolerance;
    }

    [[nodiscard]] inline SCPoint3d ArcPoint(const SCArcSegment3d& arc, double parameter)
    {
        const double angle = parameter * arc.signedSweepRad;
        const SCVector3d tangent = Cross(arc.unitNormal, arc.startRadialUnit);
        return arc.center + arc.radius *
            (arc.startRadialUnit * std::cos(angle) + tangent * std::sin(angle));
    }

    [[nodiscard]] inline SCPathGeometryFailure ValidateArc(const SCArcSegment3d& arc,
                                                             const SCPathGeometryTolerance& tolerance)
    {
        if (!arc.center.IsValid() || !arc.unitNormal.IsValid() ||
            !arc.startRadialUnit.IsValid() || !std::isfinite(arc.radius) ||
            !std::isfinite(arc.signedSweepRad))
        {
            return SCPathGeometryFailure::NonFiniteInput;
        }
        if (!Unit(arc.unitNormal, tolerance))
        {
            return SCPathGeometryFailure::InvalidNormal;
        }
        if (!Unit(arc.startRadialUnit, tolerance) ||
            !Orthogonal(arc.unitNormal, arc.startRadialUnit, tolerance))
        {
            return SCPathGeometryFailure::DegenerateDirection;
        }
        if (arc.radius <= tolerance.distanceTolerance)
        {
            return SCPathGeometryFailure::InvalidRadius;
        }
        if (std::abs(arc.signedSweepRad) <= tolerance.angleToleranceRad ||
            std::abs(arc.signedSweepRad) > 2.0 * std::numbers::pi_v<double>)
        {
            return SCPathGeometryFailure::InvalidSweep;
        }
        const double length = arc.radius * std::abs(arc.signedSweepRad);
        if (!std::isfinite(length))
        { return SCPathGeometryFailure::NonFiniteResult; }
        if (length == 0.0)
        { return SCPathGeometryFailure::NumericalFailure; }
        for (int sample = 0; sample <= 4; ++sample)
        {
            if (!ArcPoint(arc, static_cast<double>(sample) / 4.0).IsValid())
            { return SCPathGeometryFailure::NonFiniteResult; }
        }
        return SCPathGeometryFailure::None;
    }

    struct CompensatedSum
    {
        double sum{0.0};
        double correction{0.0};

        void Add(double value)
        {
            const double updated = sum + value;
            correction += std::abs(sum) >= std::abs(value) ? (sum - updated) + value
                                                            : (value - updated) + sum;
            sum = updated;
        }

        [[nodiscard]] double Value() const { return sum + correction; }
    };
}
