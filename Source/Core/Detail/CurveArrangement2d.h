#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include "Geometry2d/ISCSegment2d.h"
#include "Geometry2d/SCPolyline2d.h"

namespace Geometry::Detail
{
    // 仅供正面积专用 arrangement 传播 checked 失败原因；既有通用入口不依赖该字段。
    enum class CurveArrangementFailure2d
    {
        None,
        InvalidInput,
        NumericalIndeterminate,
        EventInsertionFailure,
        EventOrderingFailure,
        PieceConstructionFailure,
        ReverseEdgeFailure,
        FaceClosureFailure
    };

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
        CurveArrangementFailure2d failure{CurveArrangementFailure2d::InvalidInput};
        std::vector<ArrangementPiece2d> pieces{};
        std::vector<SCPolyline2d> faceRings{};
    };

    [[nodiscard]] CurveArrangementResult2d BuildCurveArrangement2d(
        const std::vector<std::vector<std::unique_ptr<ISCSegment2d>>>& rings,
        double tolerance);

    [[nodiscard]] CurveArrangementResult2d BuildPositiveAreaCurveArrangement2d(
        const std::vector<std::vector<std::unique_ptr<ISCSegment2d>>>& rings,
        double tolerance);
}  // namespace Geometry::Detail
