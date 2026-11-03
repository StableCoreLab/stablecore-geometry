#include <algorithm>

#include <gtest/gtest.h>

#include "../../../Source/Core/Detail/SegmentSubdivision2d.h"

TEST(SegmentSubdivision2dTest, LongSegmentsKeepGeometricallySeparatedNearEndpointCrossings)
{
    std::vector<Geometry::SCLineSegment2d> segments{
        {{0.0, 0.0}, {1e6, 0.0}},
        {{0.1, -1.0}, {0.1, 1.0}}};
    const double epsilon = 1e-6;
    for (const bool reverseOrder : {false, true})
    {
        SCOPED_TRACE(reverseOrder);
        if (reverseOrder)
        {
            std::reverse(segments.begin(), segments.end());
        }
        const auto pieces = Geometry::Detail::SubdivideLineSegments(segments, epsilon, [&](const auto& segment) {
            return Geometry::Detail::ComputeSubdivisionParameterTolerance2d(segment.Length(), epsilon);
        });
        ASSERT_EQ(pieces.size(), 4U);
        const std::size_t horizontal = reverseOrder ? 2U : 0U;
        const std::size_t vertical = reverseOrder ? 0U : 2U;
        EXPECT_EQ(pieces[horizontal].sourceIndex, reverseOrder ? 1U : 0U);
        EXPECT_NEAR(pieces[horizontal].end.x, 0.1, 1e-12);
        EXPECT_NEAR(pieces[horizontal + 1].start.x, 0.1, 1e-12);
        EXPECT_EQ(pieces[vertical].sourceIndex, reverseOrder ? 0U : 1U);
        EXPECT_NEAR(pieces[vertical].end.y, 0.0, 1e-12);
    }
}
