#pragma once

#include <cmath>

#include "Export/GeometryExport.h"
#include "Types/Geometry2d/SCPoint2.h"
#include "Types/Geometry2d/SCVector2.h"
#include "Types/Geometry3d/SCPlane.h"

namespace Geometry
{
    struct GEOMETRY_API SCPlaneTolerance
    {
        double pointTolerance{kDefaultEpsilon};
        double relativeAngularTolerance{kDefaultEpsilon};

        [[nodiscard]] bool IsValid() const
        {
            return std::isfinite(pointTolerance) && pointTolerance > 0.0 &&
                   std::isfinite(relativeAngularTolerance) && relativeAngularTolerance > 0.0;
        }
    };

    enum class SCPlaneConstructionFailure
    {
        None,
        InvalidTolerance,
        NonFiniteInput,
        CoincidentPoints,
        CollinearPoints,
        ZeroGradientDirection,
        NonFiniteResult
    };

    enum class SCPlaneElevationFailure
    {
        None,
        InvalidTolerance,
        NonFiniteInput,
        InvalidPlane,
        VerticalOrNearVerticalPlane,
        NonFiniteResult
    };

    struct GEOMETRY_API SCPlaneConstructionResult
    {
        bool success{false};
        SCPlane plane{};
        SCPlaneConstructionFailure failure{SCPlaneConstructionFailure::None};
    };

    struct GEOMETRY_API SCPlaneElevationResult
    {
        bool success{false};
        double elevation{0.0};
        SCPlaneElevationFailure failure{SCPlaneElevationFailure::None};
    };

    [[nodiscard]] GEOMETRY_API SCPlaneConstructionResult CreatePlaneThroughThreePoints(
        const SCPoint3d& first,
        const SCPoint3d& second,
        const SCPoint3d& third,
        const SCPlaneTolerance& tolerance);

    [[nodiscard]] GEOMETRY_API SCPlaneConstructionResult CreatePlaneThroughThreePoints(
        const SCPoint3d& first,
        const SCPoint3d& second,
        const SCPoint3d& third,
        double tolerance = kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API SCPlaneConstructionResult CreatePlaneFromPointAndXYGradient(
        const SCPoint3d& referencePoint,
        const SCVector2d& gradientDirection,
        double risePerRun,
        const SCPlaneTolerance& tolerance);

    [[nodiscard]] GEOMETRY_API SCPlaneConstructionResult CreatePlaneFromPointAndXYGradient(
        const SCPoint3d& referencePoint,
        const SCVector2d& gradientDirection,
        double risePerRun,
        double tolerance = kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API SCPlaneElevationResult EvaluatePlaneElevationAtXY(
        const SCPlane& plane,
        const SCPoint2d& point,
        const SCPlaneTolerance& tolerance);

    [[nodiscard]] GEOMETRY_API SCPlaneElevationResult EvaluatePlaneElevationAtXY(
        const SCPlane& plane,
        const SCPoint2d& point,
        double tolerance = kDefaultEpsilon);
}  // namespace Geometry
