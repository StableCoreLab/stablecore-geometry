#include <gtest/gtest.h>

TEST(DISABLED_MissingAlgorithmsGapTest, TangentPointsBoundaryClassificationRemainsOpen)
{
    GTEST_SKIP() << "Known gap: tangent classification at circle/arc tolerance boundaries.";
}

TEST(DISABLED_MissingAlgorithmsGapTest, IntersectExtendedDegenerateOverlapClassificationRemainsOpen)
{
    GTEST_SKIP() << "Known gap: degenerate extended-overlap classification.";
}

TEST(DISABLED_MissingAlgorithmsGapTest, PolylineIntersectionEndpointDeduplicationRemainsOpen)
{
    GTEST_SKIP() << "Known gap: endpoint-hit de-duplication and ordering.";
}
