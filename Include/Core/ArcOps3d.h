#pragma once

#include <array>
#include <cstddef>
#include <optional>

#include "Core/PathGeometry.h"
#include "Geometry2d/SCArcSegment2d.h"

namespace Geometry
{
    struct GEOMETRY_API SCArcConstruction3dResult
    {
        bool success{false};
        SCPathGeometryFailure failure{SCPathGeometryFailure::None};
        std::optional<std::size_t> inputIndex{};
        std::optional<std::size_t> relatedIndex{};
        SCArcSegment3d arc{};
        double trueLength{0.0};
        std::array<SCPoint3d, 3> samples{};
        std::array<SCPoint3d, 5> fitSamples{};
    };

    struct GEOMETRY_API SCArcProjectionResult
    {
        bool success{false};
        SCPathGeometryFailure failure{SCPathGeometryFailure::None};
        std::optional<std::size_t> inputIndex{};
        std::optional<std::size_t> relatedIndex{};
        SCArcSegment2d arc{};
        double trueLength3d{0.0};
        double projectedLength2d{0.0};
        std::array<SCPoint2d, 3> samples{};
    };

    [[nodiscard]] GEOMETRY_API SCArcConstruction3dResult CreateArcSegment3d(
        const SCPoint3d& start, const SCPoint3d& end, const SCPoint3d& center,
        const SCVector3d& unitNormal, double radius, double signedSweepRad,
        const SCPathGeometryTolerance& tolerance);
    [[nodiscard]] GEOMETRY_API SCArcProjectionResult ProjectArcToFrame(
        const SCArcSegment3d& arc, const SCPlaneFrame3d& frame,
        const SCPathGeometryTolerance& tolerance);
}
