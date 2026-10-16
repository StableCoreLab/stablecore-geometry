#pragma once

#include <cstdint>
#include <cstddef>

#include "Export/GeometryExport.h"

namespace Geometry
{
    enum class SCGeometryEvidenceState : std::uint8_t
    {
        NotApplicable = 0,
        Verified = 1,
        Unknown = 2
    };

    enum class SCGeometryOperationState : std::uint8_t
    {
        InvalidInput = 0,
        HasResult = 1,
        NoResult = 2,
        NonUnique = 3,
        Unknown = 4
    };

    enum class SCGeometryFailureReason : std::uint32_t
    {
        None = 0,
        InvalidInput = 1u << 0,
        NonFinite = 1u << 1,
        EvaluationUnavailable = 1u << 2,
        RegionUnavailable = 1u << 3,
        TopologyInvalid = 1u << 4,
        NotClosed = 1u << 5,
        SearchIncomplete = 1u << 6,
        Degenerate = 1u << 7,
        NoProof = 1u << 8,
        NonUnique = 1u << 9,
        NotImplemented = 1u << 10,
        Unrepresentable = 1u << 11
    };

    [[nodiscard]] constexpr SCGeometryFailureReason operator|(SCGeometryFailureReason lhs,
                                                             SCGeometryFailureReason rhs) noexcept
    {
        return static_cast<SCGeometryFailureReason>(static_cast<std::uint32_t>(lhs) |
                                                    static_cast<std::uint32_t>(rhs));
    }

    constexpr SCGeometryFailureReason& operator|=(SCGeometryFailureReason& lhs,
                                                 SCGeometryFailureReason rhs) noexcept
    {
        lhs = lhs | rhs;
        return lhs;
    }

    [[nodiscard]] constexpr bool HasGeometryFailureReason(SCGeometryFailureReason reasons,
                                                          SCGeometryFailureReason reason) noexcept
    {
        return (static_cast<std::uint32_t>(reasons) & static_cast<std::uint32_t>(reason)) != 0;
    }

    struct GEOMETRY_API SCGeometryEvidenceStatus
    {
        SCGeometryEvidenceState region{SCGeometryEvidenceState::NotApplicable};
        SCGeometryEvidenceState hit{SCGeometryEvidenceState::NotApplicable};
        SCGeometryEvidenceState optimality{SCGeometryEvidenceState::NotApplicable};
        SCGeometryEvidenceState negativeCompleteness{SCGeometryEvidenceState::NotApplicable};
        SCGeometryFailureReason reasons{SCGeometryFailureReason::InvalidInput};
    };

    struct SCGeometrySearchStatistics
    {
        std::size_t positionEvaluations{0};
        std::size_t derivativeEvaluations{0};
        std::size_t seedCount{0};
        bool searchIncomplete{false};
    };

}
