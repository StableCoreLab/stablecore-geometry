#include "Serialize/SegmentCodec.h"

#include <array>
#include <bit>
#include <cmath>
#include <cstring>
#include <limits>

#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"

namespace Geometry
{
    namespace
    {
        constexpr std::string_view kLineTypeId{"SC.LineSegment2d"};
        constexpr std::string_view kArcTypeId{"SC.ArcSegment2d"};
        constexpr std::uint32_t kDefinitionVersion{1};

        [[nodiscard]] bool IsPortableBinary64()
        {
            return std::numeric_limits<double>::is_iec559 &&
                   std::numeric_limits<double>::digits == 53 && sizeof(double) == sizeof(std::uint64_t);
        }

        void AppendUint64(std::vector<std::byte>& output, std::uint64_t value)
        {
            for (std::size_t i = 0; i < sizeof(value); ++i)
            {
                output.push_back(static_cast<std::byte>((value >> (i * 8)) & 0xffU));
            }
        }

        void AppendDouble(std::vector<std::byte>& output, double value)
        {
            std::uint64_t bits = 0;
            std::memcpy(&bits, &value, sizeof(bits));
            AppendUint64(output, bits);
        }

        [[nodiscard]] bool ReadUint64(std::span<const std::byte> input, std::size_t& offset, std::uint64_t& value)
        {
            if (input.size() - offset < sizeof(value))
            {
                return false;
            }
            value = 0;
            for (std::size_t i = 0; i < sizeof(value); ++i)
            {
                value |= static_cast<std::uint64_t>(std::to_integer<unsigned int>(input[offset + i])) << (i * 8);
            }
            offset += sizeof(value);
            return true;
        }

        [[nodiscard]] bool ReadDouble(std::span<const std::byte> input, std::size_t& offset, double& value)
        {
            std::uint64_t bits = 0;
            if (!ReadUint64(input, offset, bits))
            {
                return false;
            }
            std::memcpy(&value, &bits, sizeof(value));
            return std::isfinite(value);
        }

        [[nodiscard]] SCSegmentCodecResult MakeFailure(SCSegmentCodecFailure failure)
        {
            return {false, {}, failure};
        }
    }  // namespace

    SCSegmentCodecResult SerializeSegment(const ISCSegment2d& segment)
    {
        if (!IsPortableBinary64())
        {
            return MakeFailure(SCSegmentCodecFailure::UnsupportedPlatform);
        }
        SCSegmentDefinition definition;
        definition.definitionVersion = kDefinitionVersion;
        if (const auto* line = dynamic_cast<const SCLineSegment2d*>(&segment); line != nullptr)
        {
            if (!line->IsValid())
            {
                return MakeFailure(SCSegmentCodecFailure::InvalidDecodedSegment);
            }
            definition.typeId = std::string(kLineTypeId);
            definition.definitionPayload.reserve(4 * sizeof(double));
            AppendDouble(definition.definitionPayload, line->startPoint.x);
            AppendDouble(definition.definitionPayload, line->startPoint.y);
            AppendDouble(definition.definitionPayload, line->endPoint.x);
            AppendDouble(definition.definitionPayload, line->endPoint.y);
        }
        else if (const auto* arc = dynamic_cast<const SCArcSegment2d*>(&segment); arc != nullptr)
        {
            if (!arc->IsValid())
            {
                return MakeFailure(SCSegmentCodecFailure::InvalidDecodedSegment);
            }
            definition.typeId = std::string(kArcTypeId);
            definition.definitionPayload.reserve(5 * sizeof(double));
            AppendDouble(definition.definitionPayload, arc->center.x);
            AppendDouble(definition.definitionPayload, arc->center.y);
            AppendDouble(definition.definitionPayload, arc->radius);
            AppendDouble(definition.definitionPayload, arc->startAngle);
            AppendDouble(definition.definitionPayload, arc->sweepAngle);
        }
        else
        {
            return MakeFailure(SCSegmentCodecFailure::UnsupportedSegmentType);
        }
        return {true, std::move(definition), SCSegmentCodecFailure::None};
    }

    SCSegmentDecodeResult DeserializeSegment(std::string_view typeId,
                                             std::uint32_t definitionVersion,
                                             std::span<const std::byte> definitionPayload)
    {
        if (!IsPortableBinary64())
        {
            return {false, nullptr, SCSegmentCodecFailure::UnsupportedPlatform};
        }
        if (typeId != kLineTypeId && typeId != kArcTypeId)
        {
            return {false, nullptr, SCSegmentCodecFailure::UnknownTypeId};
        }
        if (definitionVersion != kDefinitionVersion)
        {
            return {false, nullptr, SCSegmentCodecFailure::UnknownDefinitionVersion};
        }

        const std::size_t valueCount = typeId == kLineTypeId ? 4 : 5;
        if (definitionPayload.size() != valueCount * sizeof(double))
        {
            return {false, nullptr, SCSegmentCodecFailure::MalformedPayload};
        }
        std::array<double, 5> values{};
        std::size_t offset = 0;
        for (std::size_t i = 0; i < valueCount; ++i)
        {
            if (!ReadDouble(definitionPayload, offset, values[i]))
            {
                return {false, nullptr, SCSegmentCodecFailure::NonFiniteDefinition};
            }
        }

        std::unique_ptr<ISCSegment2d> segment;
        if (typeId == kLineTypeId)
        {
            segment = std::make_unique<SCLineSegment2d>(SCPoint2d{values[0], values[1]}, SCPoint2d{values[2], values[3]});
        }
        else
        {
            segment = std::make_unique<SCArcSegment2d>(SCPoint2d{values[0], values[1]}, values[2], values[3], values[4]);
        }
        if (segment == nullptr || !segment->IsValid())
        {
            return {false, nullptr, SCSegmentCodecFailure::InvalidDecodedSegment};
        }
        return {true, std::move(segment), SCSegmentCodecFailure::None};
    }
}  // namespace Geometry
