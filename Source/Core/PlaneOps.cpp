#include "Core/PlaneOps.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Geometry
{
    namespace
    {
        [[nodiscard]] bool IsFinite(const SCPoint3d& point)
        {
            return point.IsValid();
        }

        [[nodiscard]] bool IsFinite(const SCPoint2d& point)
        {
            return point.IsValid();
        }

        [[nodiscard]] bool IsFinite(const SCVector2d& vector)
        {
            return vector.IsValid();
        }

        [[nodiscard]] long double DistanceSquared(const SCPoint3d& first, const SCPoint3d& second)
        {
            const long double dx = static_cast<long double>(first.x) - static_cast<long double>(second.x);
            const long double dy = static_cast<long double>(first.y) - static_cast<long double>(second.y);
            const long double dz = static_cast<long double>(first.z) - static_cast<long double>(second.z);
            return dx * dx + dy * dy + dz * dz;
        }

        [[nodiscard]] bool IsFinite(long double value)
        {
            return std::isfinite(value);
        }

        [[nodiscard]] SCPlaneConstructionResult MakePlaneResult(const SCPoint3d& origin,
                                                                long double nx,
                                                                long double ny,
                                                                long double nz)
        {
            const long double scale = std::max({std::abs(nx), std::abs(ny), std::abs(nz)});
            if (!(scale > 0.0L) || !IsFinite(scale))
            {
                return {false, {}, SCPlaneConstructionFailure::NonFiniteResult};
            }

            const long double sx = nx / scale;
            const long double sy = ny / scale;
            const long double sz = nz / scale;
            const long double length = std::sqrt(sx * sx + sy * sy + sz * sz);
            if (!(length > 0.0L) || !IsFinite(length))
            {
                return {false, {}, SCPlaneConstructionFailure::NonFiniteResult};
            }

            const SCVector3d normal{static_cast<double>(sx / length),
                                    static_cast<double>(sy / length),
                                    static_cast<double>(sz / length)};
            const SCPlane plane = SCPlane::FromPointAndNormal(origin, normal);
            if (!plane.IsValid() || !normal.IsValid())
            {
                return {false, {}, SCPlaneConstructionFailure::NonFiniteResult};
            }
            return {true, plane, SCPlaneConstructionFailure::None};
        }
    }  // namespace

    SCPlaneConstructionResult CreatePlaneThroughThreePoints(const SCPoint3d& first,
                                                             const SCPoint3d& second,
                                                             const SCPoint3d& third,
                                                             const SCPlaneTolerance& tolerance)
    {
        if (!tolerance.IsValid())
        {
            return {false, {}, SCPlaneConstructionFailure::InvalidTolerance};
        }
        if (!IsFinite(first) || !IsFinite(second) || !IsFinite(third))
        {
            return {false, {}, SCPlaneConstructionFailure::NonFiniteInput};
        }

        const long double pointToleranceSquared = static_cast<long double>(tolerance.pointTolerance) *
                                                  static_cast<long double>(tolerance.pointTolerance);
        if (DistanceSquared(first, second) <= pointToleranceSquared ||
            DistanceSquared(first, third) <= pointToleranceSquared ||
            DistanceSquared(second, third) <= pointToleranceSquared)
        {
            return {false, {}, SCPlaneConstructionFailure::CoincidentPoints};
        }

        const long double ux = static_cast<long double>(second.x) - static_cast<long double>(first.x);
        const long double uy = static_cast<long double>(second.y) - static_cast<long double>(first.y);
        const long double uz = static_cast<long double>(second.z) - static_cast<long double>(first.z);
        const long double vx = static_cast<long double>(third.x) - static_cast<long double>(first.x);
        const long double vy = static_cast<long double>(third.y) - static_cast<long double>(first.y);
        const long double vz = static_cast<long double>(third.z) - static_cast<long double>(first.z);
        const long double nx = uy * vz - uz * vy;
        const long double ny = uz * vx - ux * vz;
        const long double nz = ux * vy - uy * vx;
        const long double crossScale = std::sqrt((ux * ux + uy * uy + uz * uz) * (vx * vx + vy * vy + vz * vz));
        const long double crossLength = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (!IsFinite(crossScale) || !IsFinite(crossLength))
        {
            return {false, {}, SCPlaneConstructionFailure::NonFiniteResult};
        }
        if (crossLength <= static_cast<long double>(tolerance.relativeAngularTolerance) * crossScale)
        {
            return {false, {}, SCPlaneConstructionFailure::CollinearPoints};
        }

        return MakePlaneResult(first, nx, ny, nz);
    }

    SCPlaneConstructionResult CreatePlaneThroughThreePoints(const SCPoint3d& first,
                                                             const SCPoint3d& second,
                                                             const SCPoint3d& third,
                                                             double tolerance)
    {
        return CreatePlaneThroughThreePoints(first, second, third, SCPlaneTolerance{tolerance, tolerance});
    }

    SCPlaneConstructionResult CreatePlaneFromPointAndXYGradient(const SCPoint3d& referencePoint,
                                                                const SCVector2d& gradientDirection,
                                                                double risePerRun,
                                                                const SCPlaneTolerance& tolerance)
    {
        if (!tolerance.IsValid())
        {
            return {false, {}, SCPlaneConstructionFailure::InvalidTolerance};
        }
        if (!IsFinite(referencePoint) || !IsFinite(gradientDirection) || !std::isfinite(risePerRun))
        {
            return {false, {}, SCPlaneConstructionFailure::NonFiniteInput};
        }

        const long double directionLength = std::hypotl(static_cast<long double>(gradientDirection.x),
                                                         static_cast<long double>(gradientDirection.y));
        if (!(directionLength > static_cast<long double>(tolerance.pointTolerance)))
        {
            return {false, {}, SCPlaneConstructionFailure::ZeroGradientDirection};
        }

        const long double dx = static_cast<long double>(gradientDirection.x) / directionLength;
        const long double dy = static_cast<long double>(gradientDirection.y) / directionLength;
        return MakePlaneResult(referencePoint,
                               -static_cast<long double>(risePerRun) * dx,
                               -static_cast<long double>(risePerRun) * dy,
                               1.0L);
    }

    SCPlaneConstructionResult CreatePlaneFromPointAndXYGradient(const SCPoint3d& referencePoint,
                                                                const SCVector2d& gradientDirection,
                                                                double risePerRun,
                                                                double tolerance)
    {
        return CreatePlaneFromPointAndXYGradient(
            referencePoint, gradientDirection, risePerRun, SCPlaneTolerance{tolerance, tolerance});
    }

    SCPlaneElevationResult EvaluatePlaneElevationAtXY(const SCPlane& plane,
                                                       const SCPoint2d& point,
                                                       const SCPlaneTolerance& tolerance)
    {
        if (!tolerance.IsValid())
        {
            return {false, 0.0, SCPlaneElevationFailure::InvalidTolerance};
        }
        if (!plane.origin.IsValid() || !plane.normal.IsValid() || !IsFinite(point))
        {
            return {false, 0.0, SCPlaneElevationFailure::NonFiniteInput};
        }

        const long double nx = static_cast<long double>(plane.normal.x);
        const long double ny = static_cast<long double>(plane.normal.y);
        const long double nz = static_cast<long double>(plane.normal.z);
        const long double scale = std::max({std::abs(nx), std::abs(ny), std::abs(nz)});
        if (!(scale > 0.0L) || !IsFinite(scale))
        {
            return {false, 0.0, SCPlaneElevationFailure::InvalidPlane};
        }
        if (std::abs(nz) <= static_cast<long double>(tolerance.relativeAngularTolerance) * scale)
        {
            return {false, 0.0, SCPlaneElevationFailure::VerticalOrNearVerticalPlane};
        }

        const long double dx = static_cast<long double>(point.x) - static_cast<long double>(plane.origin.x);
        const long double dy = static_cast<long double>(point.y) - static_cast<long double>(plane.origin.y);
        const long double elevation = static_cast<long double>(plane.origin.z) - (nx * dx + ny * dy) / nz;
        if (!IsFinite(dx) || !IsFinite(dy) || !IsFinite(elevation) ||
            elevation > static_cast<long double>(std::numeric_limits<double>::max()) ||
            elevation < -static_cast<long double>(std::numeric_limits<double>::max()))
        {
            return {false, 0.0, SCPlaneElevationFailure::NonFiniteResult};
        }
        return {true, static_cast<double>(elevation), SCPlaneElevationFailure::None};
    }

    SCPlaneElevationResult EvaluatePlaneElevationAtXY(const SCPlane& plane,
                                                       const SCPoint2d& point,
                                                       double tolerance)
    {
        return EvaluatePlaneElevationAtXY(plane, point, SCPlaneTolerance{tolerance, tolerance});
    }
}  // namespace Geometry
