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
            if (offset > input.size() || input.size() - offset < sizeof(value))
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

        [[nodiscard]] bool ReadUint32(std::span<const std::byte> input, std::size_t& offset, std::uint32_t& value)
        {
            if (offset > input.size() || input.size() - offset < sizeof(value))
            {
                return false;
            }
            value = 0;
            for (std::size_t i = 0; i < sizeof(value); ++i)
            {
                value |= static_cast<std::uint32_t>(std::to_integer<unsigned int>(input[offset + i])) << (i * 8);
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

        void AppendUint32(std::vector<std::byte>& output, std::uint32_t value)
        {
            for (std::size_t i = 0; i < sizeof(value); ++i)
            {
                output.push_back(static_cast<std::byte>((value >> (i * 8)) & 0xffU));
            }
        }

        [[nodiscard]] SCSegmentDecodeResult DeserializeSegmentDefinition(
            std::string_view typeId,
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
    }  // namespace

    SCSegmentCodecResult SerializeSegment(const ISCSegment2d& segment)
    {
        if (!IsPortableBinary64())
        {
            return MakeFailure(SCSegmentCodecFailure::UnsupportedPlatform);
        }
        std::vector<std::byte> definitionPayload;
        std::string_view typeId;
        if (const auto* line = dynamic_cast<const SCLineSegment2d*>(&segment); line != nullptr)
        {
            if (!line->IsValid())
            {
                return MakeFailure(SCSegmentCodecFailure::InvalidDecodedSegment);
            }
            typeId = kLineTypeId;
            definitionPayload.reserve(4 * sizeof(double));
            AppendDouble(definitionPayload, line->startPoint.x);
            AppendDouble(definitionPayload, line->startPoint.y);
            AppendDouble(definitionPayload, line->endPoint.x);
            AppendDouble(definitionPayload, line->endPoint.y);
        }
        else if (const auto* arc = dynamic_cast<const SCArcSegment2d*>(&segment); arc != nullptr)
        {
            if (!arc->IsValid())
            {
                return MakeFailure(SCSegmentCodecFailure::InvalidDecodedSegment);
            }
            typeId = kArcTypeId;
            definitionPayload.reserve(5 * sizeof(double));
            AppendDouble(definitionPayload, arc->center.x);
            AppendDouble(definitionPayload, arc->center.y);
            AppendDouble(definitionPayload, arc->radius);
            AppendDouble(definitionPayload, arc->startAngle);
            AppendDouble(definitionPayload, arc->sweepAngle);
        }
        else
        {
            return MakeFailure(SCSegmentCodecFailure::UnsupportedSegmentType);
        }
        std::vector<std::byte> record;
        record.reserve(sizeof(std::uint32_t) + typeId.size() + sizeof(std::uint32_t) +
                       sizeof(std::uint64_t) + definitionPayload.size());
        AppendUint32(record, static_cast<std::uint32_t>(typeId.size()));
        for (const char character : typeId)
        {
            record.push_back(static_cast<std::byte>(static_cast<unsigned char>(character)));
        }
        AppendUint32(record, kDefinitionVersion);
        AppendUint64(record, static_cast<std::uint64_t>(definitionPayload.size()));
        record.insert(record.end(), definitionPayload.begin(), definitionPayload.end());
        return {true, std::move(record), SCSegmentCodecFailure::None};
    }

    SCSegmentDecodeResult DeserializeSegment(std::span<const std::byte> serializedRecord)
    {
        constexpr std::size_t kMaxTypeIdLength = 256;
        constexpr std::uint64_t kMaxPayloadLength = 4096;

        std::size_t offset = 0;
        std::uint32_t typeIdLength = 0;
        if (!ReadUint32(serializedRecord, offset, typeIdLength) || typeIdLength == 0 || typeIdLength > kMaxTypeIdLength)
        {
            return {false, nullptr, SCSegmentCodecFailure::MalformedPayload};
        }
        if (offset > serializedRecord.size() || typeIdLength > serializedRecord.size() - offset)
        {
            return {false, nullptr, SCSegmentCodecFailure::MalformedPayload};
        }
        const auto typeIdBytes = serializedRecord.subspan(offset, typeIdLength);
        const std::string_view typeId(reinterpret_cast<const char*>(typeIdBytes.data()), typeIdBytes.size());
        offset += typeIdLength;

        std::uint32_t definitionVersion = 0;
        std::uint64_t payloadLength = 0;
        if (!ReadUint32(serializedRecord, offset, definitionVersion) ||
            !ReadUint64(serializedRecord, offset, payloadLength) ||
            payloadLength > kMaxPayloadLength ||
            payloadLength > serializedRecord.size() - offset)
        {
            return {false, nullptr, SCSegmentCodecFailure::MalformedPayload};
        }

        const auto definitionPayload = serializedRecord.subspan(offset, static_cast<std::size_t>(payloadLength));
        offset += static_cast<std::size_t>(payloadLength);
        if (offset != serializedRecord.size())
        {
            return {false, nullptr, SCSegmentCodecFailure::MalformedPayload};
        }
        return DeserializeSegmentDefinition(typeId, definitionVersion, definitionPayload);
    }
}  // namespace Geometry
