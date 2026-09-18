#include <gtest/gtest.h>

#include "Serialize/SegmentCodec.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"

using namespace Geometry;

namespace
{
    void AppendUint32(std::vector<std::byte>& output, std::uint32_t value)
    {
        for (std::size_t i = 0; i < sizeof(value); ++i)
        {
            output.push_back(static_cast<std::byte>((value >> (i * 8)) & 0xffU));
        }
    }

    void AppendUint64(std::vector<std::byte>& output, std::uint64_t value)
    {
        for (std::size_t i = 0; i < sizeof(value); ++i)
        {
            output.push_back(static_cast<std::byte>((value >> (i * 8)) & 0xffU));
        }
    }

    std::vector<std::byte> MakeSerializedRecord(std::string_view typeId,
                                                std::uint32_t definitionVersion,
                                                std::span<const std::byte> payload)
    {
        std::vector<std::byte> record;
        AppendUint32(record, static_cast<std::uint32_t>(typeId.size()));
        for (const char character : typeId)
        {
            record.push_back(static_cast<std::byte>(static_cast<unsigned char>(character)));
        }
        AppendUint32(record, definitionVersion);
        AppendUint64(record, static_cast<std::uint64_t>(payload.size()));
        record.insert(record.end(), payload.begin(), payload.end());
        return record;
    }

}

TEST(SegmentCodecTest, RoundTripsLineAndArc)
{
    const SCLineSegment2d line{{1.0, 2.0}, {3.0, 4.0}};
    const auto lineEncoded = SerializeSegment(line);
    ASSERT_TRUE(lineEncoded.success);
    const auto lineDecoded = DeserializeSegment(lineEncoded.serializedRecord);
    ASSERT_TRUE(lineDecoded.success);
    EXPECT_NEAR(lineDecoded.segment->StartPoint().x, 1.0, 1e-12);
    EXPECT_NEAR(lineDecoded.segment->EndPoint().y, 4.0, 1e-12);

    const SCArcSegment2d arc{{0.0, 0.0}, 2.0, 0.0, kPi * 0.5};
    const auto arcEncoded = SerializeSegment(arc);
    ASSERT_TRUE(arcEncoded.success);
    const auto arcDecoded = DeserializeSegment(arcEncoded.serializedRecord);
    ASSERT_TRUE(arcDecoded.success);
    EXPECT_NEAR(arcDecoded.segment->Length(), arc.Length(), 1e-12);
}

TEST(SegmentCodecTest, RejectsUnknownVersionAndCorruptPayload)
{
    std::vector<std::byte> payload(32);
    EXPECT_EQ(DeserializeSegment(MakeSerializedRecord("SC.LineSegment2d", 99, payload)).failure,
              SCSegmentCodecFailure::UnknownDefinitionVersion);
    EXPECT_EQ(DeserializeSegment(MakeSerializedRecord("SC.LineSegment2d", 1, {})).failure,
              SCSegmentCodecFailure::MalformedPayload);
    EXPECT_EQ(DeserializeSegment(MakeSerializedRecord("SC.Unknown", 1, payload)).failure,
              SCSegmentCodecFailure::UnknownTypeId);
}

TEST(SegmentCodecTest, RejectsTrailingAndNonFiniteDefinitions)
{
    const SCLineSegment2d line{{1.0, 2.0}, {3.0, 4.0}};
    const auto encoded = SerializeSegment(line);
    ASSERT_TRUE(encoded.success);

    auto trailingRecord = std::vector<std::byte>(encoded.serializedRecord.begin(), encoded.serializedRecord.end());
    trailingRecord.push_back(std::byte{0});
    EXPECT_EQ(DeserializeSegment(trailingRecord).failure,
              SCSegmentCodecFailure::MalformedPayload);

    auto nonFinitePayload = std::vector<std::byte>(encoded.serializedRecord.end() - 32, encoded.serializedRecord.end());
    for (std::size_t i = 0; i < sizeof(double); ++i)
    {
        nonFinitePayload[i] = std::byte{0};
    }
    nonFinitePayload[sizeof(double) - 2] = std::byte{0xf0};
    nonFinitePayload[sizeof(double) - 1] = std::byte{0x7f};
    EXPECT_EQ(DeserializeSegment(MakeSerializedRecord("SC.LineSegment2d", 1, nonFinitePayload)).failure,
              SCSegmentCodecFailure::NonFiniteDefinition);
}
