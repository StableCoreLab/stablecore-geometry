#include <gtest/gtest.h>

#include "Geometry.h"

TEST(MissingAlgorithmsGapTest, TangentPointsBoundaryClassificationRemainsOpen)
{
    GTEST_SKIP() << "Known gap: tangent-point boundary classification near the circle/arc limit "
                    "still depends on epsilon-sensitive angle filtering, especially when the "
                    "query point is numerically close to the limiting tangent circle.";
}

TEST(MissingAlgorithmsGapTest, IntersectExtendedDegenerateOverlapClassificationRemainsOpen)
{
    GTEST_SKIP() << "Known gap: IntersectExtended overlap semantics for degenerate or nearly "
                    "degenerate extended configurations remain open, including when support "
                    "geometries are coincident but finite overlap endpoints are unstable.";
}

TEST(MissingAlgorithmsGapTest, PolylineIntersectionEndpointDeduplicationRemainsOpen)
{
    GTEST_SKIP() << "Known gap: polyline-vs-polyline intersection still has boundary-sensitive "
                    "endpoint de-duplication and ordering cases when multiple adjacent segment "
                    "pairs report the same geometric hit.";
}
