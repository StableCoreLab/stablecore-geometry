#include "sdk/GeometryIntersection.h"

#include <algorithm>
#include <array>
#include <cmath>

#include "sdk/PlaneSurface.h"

namespace geometry::sdk
{
namespace
{
[[nodiscard]] double SurfaceResidualSquared(
    const Line3d& line,
    double t,
    const Surface& surface,
    double u,
    double v)
{
    return (line.PointAt(t) - surface.PointAt(u, v)).LengthSquared();
}

[[nodiscard]] LineSurfaceIntersection3d RefineLineSurfaceIntersection(
    const Line3d& line,
    const Surface& surface,
    double initialT,
    double initialU,
    double initialV,
    const GeometryTolerance3d& tolerance)
{
    const Intervald uRange = surface.URange();
    const Intervald vRange = surface.VRange();
    double bestT = initialT;
    double bestU = std::clamp(initialU, uRange.min, uRange.max);
    double bestV = std::clamp(initialV, vRange.min, vRange.max);
    double bestResidualSquared = SurfaceResidualSquared(line, bestT, surface, bestU, bestV);

    double stepT = std::max(1.0, tolerance.parameterEpsilon * 8.0);
    double stepU = std::max(uRange.Length() * 0.25, tolerance.parameterEpsilon);
    double stepV = std::max(vRange.Length() * 0.25, tolerance.parameterEpsilon);

    for (int iteration = 0; iteration < 20; ++iteration)
    {
        bool improved = false;
        for (const double dt : std::array<double, 3>{-stepT, 0.0, stepT})
        {
            for (const double du : std::array<double, 3>{-stepU, 0.0, stepU})
            {
                for (const double dv : std::array<double, 3>{-stepV, 0.0, stepV})
                {
                    const double candidateT = bestT + dt;
                    const double candidateU = std::clamp(bestU + du, uRange.min, uRange.max);
                    const double candidateV = std::clamp(bestV + dv, vRange.min, vRange.max);
                    const double candidateResidualSquared =
                        SurfaceResidualSquared(line, candidateT, surface, candidateU, candidateV);
                    if (candidateResidualSquared +
                            tolerance.distanceEpsilon * tolerance.distanceEpsilon <
                        bestResidualSquared)
                    {
                        bestT = candidateT;
                        bestU = candidateU;
                        bestV = candidateV;
                        bestResidualSquared = candidateResidualSquared;
                        improved = true;
                    }
                }
            }
        }

        if (!improved)
        {
            stepT *= 0.5;
            stepU *= 0.5;
            stepV *= 0.5;
        }

        if (stepT <= tolerance.parameterEpsilon &&
            stepU <= tolerance.parameterEpsilon &&
            stepV <= tolerance.parameterEpsilon)
        {
            break;
        }
    }

    const Point3d pointOnLine = line.PointAt(bestT);
    const Point3d pointOnSurface = surface.PointAt(bestU, bestV);
    if ((pointOnLine - pointOnSurface).LengthSquared() >
        tolerance.distanceEpsilon * tolerance.distanceEpsilon)
    {
        return {};
    }

    return {true, false, false, bestT, bestU, bestV, pointOnSurface};
}
} // namespace

LinePlaneIntersection3d Intersect(
    const Line3d& line,
    const Plane& plane,
    const GeometryTolerance3d& tolerance)
{
    LinePlaneIntersection3d result{};
    if (!line.IsValid(tolerance.distanceEpsilon) || !plane.IsValid(tolerance.distanceEpsilon))
    {
        return result;
    }

    const Vector3d unitNormal = plane.UnitNormal(tolerance.distanceEpsilon);
    const double denominator = Dot(unitNormal, line.direction);
    const double numerator = Dot(unitNormal, plane.origin - line.origin);

    if (std::abs(denominator) <= tolerance.distanceEpsilon)
    {
        result.isParallel = true;
        result.liesOnPlane = std::abs(numerator) <= tolerance.distanceEpsilon;
        result.intersects = result.liesOnPlane;
        result.point = line.origin;
        return result;
    }

    result.intersects = true;
    result.parameter = numerator / denominator;
    result.point = line.PointAt(result.parameter);
    return result;
}

LineSurfaceIntersection3d Intersect(
    const Line3d& line,
    const Surface& surface,
    const GeometryTolerance3d& tolerance)
{
    if (!line.IsValid(tolerance.distanceEpsilon) || !surface.IsValid(tolerance))
    {
        return {};
    }

    if (const auto* planeSurface = dynamic_cast<const PlaneSurface*>(&surface))
    {
        const LinePlaneIntersection3d planeIntersection =
            Intersect(line, planeSurface->SupportPlane(), tolerance);
        if (!planeIntersection.intersects)
        {
            return {false, planeIntersection.isParallel, planeIntersection.liesOnPlane, 0.0, 0.0, 0.0, {}};
        }

        const Vector3d delta = planeIntersection.point - planeSurface->SupportPlane().origin;
        const Vector3d uAxis = planeSurface->UAxis();
        const Vector3d vAxis = planeSurface->VAxis();
        const double uDenominator = std::max(uAxis.LengthSquared(), tolerance.parameterEpsilon);
        const double vDenominator = std::max(vAxis.LengthSquared(), tolerance.parameterEpsilon);
        const double u = Dot(delta, uAxis) / uDenominator;
        const double v = Dot(delta, vAxis) / vDenominator;
        const bool insideDomain = planeSurface->URange().Contains(u, tolerance.parameterEpsilon) &&
                                  planeSurface->VRange().Contains(v, tolerance.parameterEpsilon);
        if (!insideDomain)
        {
            return {};
        }

        return {true, false, planeIntersection.liesOnPlane, planeIntersection.parameter, u, v, planeIntersection.point};
    }

    const Intervald uRange = surface.URange();
    const Intervald vRange = surface.VRange();
    if (!uRange.IsValid() || !vRange.IsValid())
    {
        return {};
    }

    constexpr std::size_t sampleCountT = 17;
    constexpr std::size_t sampleCountU = 9;
    constexpr std::size_t sampleCountV = 9;
    const double lineStep = 8.0;

    double bestT = -lineStep * static_cast<double>(sampleCountT / 2);
    double bestU = uRange.min;
    double bestV = vRange.min;
    double bestResidualSquared = SurfaceResidualSquared(line, bestT, surface, bestU, bestV);
    for (std::size_t ti = 0; ti < sampleCountT; ++ti)
    {
        const double t = (static_cast<double>(ti) - static_cast<double>(sampleCountT / 2)) * lineStep;
        for (std::size_t ui = 0; ui < sampleCountU; ++ui)
        {
            const double u = ui + 1 == sampleCountU
                                 ? uRange.max
                                 : uRange.min + uRange.Length() * static_cast<double>(ui) /
                                                    static_cast<double>(sampleCountU - 1);
            for (std::size_t vi = 0; vi < sampleCountV; ++vi)
            {
                const double v = vi + 1 == sampleCountV
                                     ? vRange.max
                                     : vRange.min + vRange.Length() * static_cast<double>(vi) /
                                                        static_cast<double>(sampleCountV - 1);
                const double residualSquared = SurfaceResidualSquared(line, t, surface, u, v);
                if (residualSquared < bestResidualSquared)
                {
                    bestT = t;
                    bestU = u;
                    bestV = v;
                    bestResidualSquared = residualSquared;
                }
            }
        }
    }

    return RefineLineSurfaceIntersection(line, surface, bestT, bestU, bestV, tolerance);
}

PlanePlaneIntersection3d Intersect(
    const Plane& first,
    const Plane& second,
    const GeometryTolerance3d& tolerance)
{
    PlanePlaneIntersection3d result{};
    if (!first.IsValid(tolerance.distanceEpsilon) || !second.IsValid(tolerance.distanceEpsilon))
    {
        return result;
    }

    const Vector3d n1 = first.UnitNormal(tolerance.distanceEpsilon);
    const Vector3d n2 = second.UnitNormal(tolerance.distanceEpsilon);
    const Vector3d direction = Cross(n1, n2);
    const double directionLengthSquared = direction.LengthSquared();

    if (directionLengthSquared <= tolerance.distanceEpsilon * tolerance.distanceEpsilon)
    {
        result.isParallel = true;
        result.isCoincident = std::abs(second.SignedDistanceTo(first.origin, tolerance.distanceEpsilon)) <=
                              tolerance.distanceEpsilon;
        result.intersects = result.isCoincident;
        return result;
    }

    const double d1 = Dot(n1, first.origin - Point3d{});
    const double d2 = Dot(n2, second.origin - Point3d{});
    const Vector3d pointVector = Cross((d1 * n2) - (d2 * n1), direction) / directionLengthSquared;

    result.intersects = true;
    result.line = Line3d::FromOriginAndDirection(Point3d{pointVector.x, pointVector.y, pointVector.z}, direction);
    return result;
}
} // namespace geometry::sdk
