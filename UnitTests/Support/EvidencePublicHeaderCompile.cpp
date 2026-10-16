#include SCGEOMETRY_HEADER_UNDER_TEST

#include <type_traits>
#include <cstdint>

using namespace Geometry;
static_assert(std::is_same_v<std::underlying_type_t<SCGeometryEvidenceState>, std::uint8_t>);
static_assert(std::is_same_v<std::underlying_type_t<SCGeometryOperationState>, std::uint8_t>);
static_assert(std::is_same_v<std::underlying_type_t<SCGeometryFailureReason>, std::uint32_t>);
static_assert(static_cast<unsigned>(SCGeometryEvidenceState::Unknown) == 2);
static_assert(static_cast<unsigned>(SCGeometryOperationState::Unknown) == 4);
static_assert(static_cast<unsigned>(SCGeometryFailureReason::Unrepresentable) == (1u << 11));
constexpr SCGeometryEvidenceStatus defaultEvidence{};
static_assert(HasGeometryFailureReason(defaultEvidence.reasons, SCGeometryFailureReason::InvalidInput));
