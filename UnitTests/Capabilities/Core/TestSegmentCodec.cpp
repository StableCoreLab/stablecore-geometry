#include <gtest/gtest.h>

#include "Serialize/SegmentCodec.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"

using namespace Geometry;

TEST(SegmentCodecTest, RoundTripsLineAndArc)
{
    const SCLineSegment2d line{{1.0, 2.0}, {3.0, 4.0}};
    const auto lineEncoded = SerializeSegment(line);
    ASSERT_TRUE(lineEncoded.success);
    EXPECT_EQ(lineEncoded.definition.typeId, "SC.LineSegment2d");
    const auto lineDecoded = DeserializeSegment(lineEncoded.definition.typeId,
                                                lineEncoded.definition.definitionVersion,
                                                lineEncoded.definition.definitionPayload);
    ASSERT_TRUE(lineDecoded.success);
    EXPECT_NEAR(lineDecoded.segment->StartPoint().x, 1.0, 1e-12);
    EXPECT_NEAR(lineDecoded.segment->EndPoint().y, 4.0, 1e-12);

    const SCArcSegment2d arc{{0.0, 0.0}, 2.0, 0.0, kPi * 0.5};
    const auto arcEncoded = SerializeSegment(arc);
    ASSERT_TRUE(arcEncoded.success);
    const auto arcDecoded = DeserializeSegment(arcEncoded.definition.typeId,
                                               arcEncoded.definition.definitionVersion,
                                               arcEncoded.definition.definitionPayload);
    ASSERT_TRUE(arcDecoded.success);
    EXPECT_NEAR(arcDecoded.segment->Length(), arc.Length(), 1e-12);
}

TEST(SegmentCodecTest, RejectsUnknownVersionAndCorruptPayload)
{
    const std::vector<std::byte> payload(32);
    EXPECT_EQ(DeserializeSegment("SC.LineSegment2d", 99, payload).failure,
              SCSegmentCodecFailure::UnknownDefinitionVersion);
    EXPECT_EQ(DeserializeSegment("SC.LineSegment2d", 1, std::span<const std::byte>{}).failure,
              SCSegmentCodecFailure::MalformedPayload);
    EXPECT_EQ(DeserializeSegment("SC.Unknown", 1, payload).failure,
              SCSegmentCodecFailure::UnknownTypeId);
}

TEST(SegmentCodecTest, RejectsTrailingAndNonFiniteDefinitions)
{
    const SCLineSegment2d line{{1.0, 2.0}, {3.0, 4.0}};
    const auto encoded = SerializeSegment(line);
    ASSERT_TRUE(encoded.success);

    auto trailingPayload = encoded.definition.definitionPayload;
    trailingPayload.push_back(std::byte{0});
    EXPECT_EQ(DeserializeSegment(encoded.definition.typeId, encoded.definition.definitionVersion, trailingPayload).failure,
              SCSegmentCodecFailure::MalformedPayload);

    auto nonFinitePayload = encoded.definition.definitionPayload;
    for (std::size_t i = 0; i < sizeof(double); ++i)
    {
        nonFinitePayload[i] = std::byte{0};
    }
    nonFinitePayload[sizeof(double) - 2] = std::byte{0xf0};
    nonFinitePayload[sizeof(double) - 1] = std::byte{0x7f};
    EXPECT_EQ(DeserializeSegment(encoded.definition.typeId, encoded.definition.definitionVersion, nonFinitePayload).failure,
              SCSegmentCodecFailure::NonFiniteDefinition);
}
