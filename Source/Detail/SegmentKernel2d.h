#pragma once

#include <memory>
#include <span>
#include <vector>

#include "Core/Intersection.h"
#include "Geometry2d/ISCSegment2d.h"

namespace Geometry::Detail
{
    enum class SegmentKernelFailure
    {
        None,
        UnsupportedSegmentType,
        InvalidParameters,
        NonFiniteResult
    };

    struct SegmentSplitPiece2d
    {
        std::unique_ptr<ISCSegment2d> segment{};
        double sourceStart{0.0};
        double sourceEnd{0.0};
    };

    struct SegmentSplitResult2d
    {
        bool success{false};
        std::vector<SegmentSplitPiece2d> pieces{};
        SegmentKernelFailure failure{SegmentKernelFailure::None};
    };

    [[nodiscard]] bool IsKernelSegment(const ISCSegment2d& segment);
    [[nodiscard]] std::unique_ptr<ISCSegment2d> ReverseKernelSegment(const ISCSegment2d& segment);
    [[nodiscard]] SCSegmentIntersection2d IntersectKernelSegments(const ISCSegment2d& first,
                                                                   const ISCSegment2d& second,
                                                                   double tolerance);
    [[nodiscard]] SCClosestPoints2d ClosestKernelSegments(const ISCSegment2d& first,
                                                           const ISCSegment2d& second);
    [[nodiscard]] SegmentSplitResult2d SplitAtParameters(const ISCSegment2d& segment,
                                                         std::span<const double> parameters,
                                                         double tolerance);
}  // namespace Geometry::Detail
