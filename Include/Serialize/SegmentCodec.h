#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "Export/GeometryExport.h"
#include "Geometry2d/ISCSegment2d.h"

namespace Geometry
{
    enum class SCSegmentCodecFailure
    {
        None,
        UnsupportedSegmentType,
        UnknownTypeId,
        UnknownDefinitionVersion,
        MalformedPayload,
        NonFiniteDefinition,
        InvalidDecodedSegment,
        UnsupportedPlatform
    };

    struct GEOMETRY_API SCSegmentCodecResult
    {
        bool success{false};
        std::vector<std::byte> serializedRecord{};
        SCSegmentCodecFailure failure{SCSegmentCodecFailure::None};
    };

    struct GEOMETRY_API SCSegmentDecodeResult
    {
        bool success{false};
        std::unique_ptr<ISCSegment2d> segment{};
        SCSegmentCodecFailure failure{SCSegmentCodecFailure::None};
    };

    [[nodiscard]] GEOMETRY_API SCSegmentCodecResult SerializeSegment(const ISCSegment2d& segment);

    [[nodiscard]] GEOMETRY_API SCSegmentDecodeResult DeserializeSegment(
        std::span<const std::byte> serializedRecord);
}  // namespace Geometry
