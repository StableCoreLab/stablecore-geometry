#include "SegmentKernel2d.h"

#include <algorithm>
#include <cmath>

#include "Core/ShapeOps.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"

namespace Geometry::Detail
{
    namespace
    {
        [[nodiscard]] bool Finite(double value)
        {
            return std::isfinite(value);
        }

        [[nodiscard]] bool ValidParameters(std::span<const double> parameters, double tolerance)
        {
            if (parameters.size() < 2 || !Finite(tolerance) || tolerance <= 0.0 ||
                !Finite(parameters.front()) || !Finite(parameters.back()) ||
                std::abs(parameters.front()) > tolerance || std::abs(parameters.back() - 1.0) > tolerance)
            {
                return false;
            }
            for (std::size_t i = 0; i < parameters.size(); ++i)
            {
                if (!Finite(parameters[i]) || parameters[i] < -tolerance || parameters[i] > 1.0 + tolerance ||
                    (i > 0 && parameters[i] <= parameters[i - 1]))
                {
                    return false;
                }
            }
            return true;
        }

        [[nodiscard]] std::unique_ptr<ISCSegment2d> MakePiece(const ISCSegment2d& source,
                                                               double start,
                                                               double end)
        {
            if (const auto* line = dynamic_cast<const SCLineSegment2d*>(&source))
            {
                return std::make_unique<SCLineSegment2d>(line->PointAt(start), line->PointAt(end));
            }
            if (const auto* arc = dynamic_cast<const SCArcSegment2d*>(&source))
            {
                return std::make_unique<SCArcSegment2d>(arc->center,
                                                        arc->radius,
                                                        arc->startAngle + arc->sweepAngle * start,
                                                        arc->sweepAngle * (end - start));
            }
            return nullptr;
        }
    }  // namespace

    bool IsKernelSegment(const ISCSegment2d& segment)
    {
        return (segment.Kind() == SCSegmentKind2::Line && dynamic_cast<const SCLineSegment2d*>(&segment) != nullptr) ||
               (segment.Kind() == SCSegmentKind2::Arc && dynamic_cast<const SCArcSegment2d*>(&segment) != nullptr);
    }

    std::unique_ptr<ISCSegment2d> ReverseKernelSegment(const ISCSegment2d& segment)
    {
        if (const auto* line = dynamic_cast<const SCLineSegment2d*>(&segment))
        {
            return std::make_unique<SCLineSegment2d>(line->endPoint, line->startPoint);
        }
        if (const auto* arc = dynamic_cast<const SCArcSegment2d*>(&segment))
        {
            return std::make_unique<SCArcSegment2d>(arc->center,
                                                    arc->radius,
                                                    arc->startAngle + arc->sweepAngle,
                                                    -arc->sweepAngle);
        }
        return nullptr;
    }

    SCSegmentIntersection2d IntersectKernelSegments(const ISCSegment2d& first,
                                                    const ISCSegment2d& second,
                                                    double tolerance)
    {
        if (!first.IsValid() || !second.IsValid() || !IsKernelSegment(first) || !IsKernelSegment(second))
        {
            return {};
        }
        if (const auto* firstLine = dynamic_cast<const SCLineSegment2d*>(&first))
        {
            if (const auto* secondLine = dynamic_cast<const SCLineSegment2d*>(&second))
                return Intersect(*firstLine, *secondLine, tolerance);
            if (const auto* secondArc = dynamic_cast<const SCArcSegment2d*>(&second))
                return Intersect(*firstLine, *secondArc, tolerance);
        }
        if (const auto* firstArc = dynamic_cast<const SCArcSegment2d*>(&first))
        {
            if (const auto* secondLine = dynamic_cast<const SCLineSegment2d*>(&second))
            {
                const SCSegmentIntersection2d swapped = Intersect(*secondLine, *firstArc, tolerance);
                SCSegmentIntersection2d result = swapped;
                for (std::size_t i = 0; i < result.pointCount; ++i)
                    std::swap(result.points[i].parameterOnFirst, result.points[i].parameterOnSecond);
                return result;
            }
            if (const auto* secondArc = dynamic_cast<const SCArcSegment2d*>(&second))
                return Intersect(*firstArc, *secondArc, tolerance);
        }
        return {};
    }

    SCClosestPoints2d ClosestKernelSegments(const ISCSegment2d& first, const ISCSegment2d& second)
    {
        if (!first.IsValid() || !second.IsValid() || !IsKernelSegment(first) || !IsKernelSegment(second))
        {
            return {};
        }
        if (const auto* firstLine = dynamic_cast<const SCLineSegment2d*>(&first))
        {
            if (const auto* secondLine = dynamic_cast<const SCLineSegment2d*>(&second))
                return ClosestPoints(*firstLine, *secondLine);
            if (const auto* secondArc = dynamic_cast<const SCArcSegment2d*>(&second))
                return ClosestPoints(*firstLine, *secondArc);
        }
        if (const auto* firstArc = dynamic_cast<const SCArcSegment2d*>(&first))
        {
            if (const auto* secondLine = dynamic_cast<const SCLineSegment2d*>(&second))
            {
                SCClosestPoints2d result = ClosestPoints(*secondLine, *firstArc);
                std::swap(result.firstPoint, result.secondPoint);
                std::swap(result.parameterOnFirst, result.parameterOnSecond);
                return result;
            }
            if (const auto* secondArc = dynamic_cast<const SCArcSegment2d*>(&second))
                return ClosestPoints(*firstArc, *secondArc);
        }
        return {};
    }

    SegmentSplitResult2d SplitAtParameters(const ISCSegment2d& segment,
                                           std::span<const double> parameters,
                                           double tolerance)
    {
        if (!IsKernelSegment(segment))
        {
            return {false, {}, SegmentKernelFailure::UnsupportedSegmentType};
        }
        if (!ValidParameters(parameters, tolerance))
        {
            return {false, {}, SegmentKernelFailure::InvalidParameters};
        }

        SegmentSplitResult2d result;
        result.pieces.reserve(parameters.size() - 1);
        for (std::size_t i = 0; i + 1 < parameters.size(); ++i)
        {
            const double start = std::clamp(parameters[i], 0.0, 1.0);
            const double end = std::clamp(parameters[i + 1], 0.0, 1.0);
            std::unique_ptr<ISCSegment2d> piece = MakePiece(segment, start, end);
            if (piece == nullptr || !piece->IsValid() || !Finite(piece->Length()))
            {
                return {false, {}, SegmentKernelFailure::NonFiniteResult};
            }
            result.pieces.push_back({std::move(piece), start, end});
        }
        result.success = true;
        return result;
    }
}  // namespace Geometry::Detail
