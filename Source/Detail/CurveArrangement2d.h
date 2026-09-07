#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include "Geometry2d/ISCSegment2d.h"
#include "Geometry2d/SCPolyline2d.h"

namespace Geometry::Detail
{
    struct ArrangementPiece2d
    {
        std::unique_ptr<ISCSegment2d> segment{};
        std::size_t sourceRing{0};
        std::size_t sourceSegment{0};
        double sourceStart{0.0};
        double sourceEnd{1.0};
    };

    struct CurveArrangementResult2d
    {
        bool success{false};
        bool hasOverlap{false};
        bool hasAmbiguousEvent{false};
        std::vector<ArrangementPiece2d> pieces{};
        std::vector<SCPolyline2d> faceRings{};
    };

    [[nodiscard]] CurveArrangementResult2d BuildCurveArrangement2d(
        const std::vector<std::vector<std::unique_ptr<ISCSegment2d>>>& rings,
        double tolerance);
}  // namespace Geometry::Detail
