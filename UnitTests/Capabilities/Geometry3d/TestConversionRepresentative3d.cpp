#include <cmath>
#include <ostream>
#include <string>

#include <gtest/gtest.h>

#include "Geometry.h"
#include "Support/ConversionFixturesRepresentative3d.h"

using Geometry::BrepConversionIssue3d;
using Geometry::ConvertToBrepBody;
using Geometry::SCPoint3d;
using Geometry::PolyhedronBody;
using Geometry::PolyhedronBrepBodyConversion3d;

using namespace Geometry::Test::ConversionFixtures;

// Demonstrates representative-id global target averaging participates in final
// shared-vertex placement: near-equal shared-edge inputs no longer pin to the
// first encountered face point.
TEST(Conversion3dCapabilityTest, NearEqualSharedEdgeVerticesUseRepresentativeAverageTarget)
{
    const PolyhedronBody body = BuildNearEqualSharedEdgeQuadPairBody();
    ASSERT_TRUE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 2);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    if (!result.success)
    {
        FAIL() << "issue=" << static_cast<int>(result.issue);
    }
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 2);
    ASSERT_EQ(result.body.ShellCount(), 1);
    ASSERT_FALSE(result.body.ShellAt(0).IsClosed());
    ASSERT_EQ(result.body.VertexCount(), 6);
    ASSERT_EQ(result.body.EdgeCount(), 7);

    const double s = 1e-5;
    const double dx = 2e-7;
    const double expectedSharedX = s + 0.5 * dx;

    std::size_t sharedBLikeCount = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const SCPoint3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12 && point.x > 0.5 * s && point.x < 1.5 * s)
        {
            ++sharedBLikeCount;
            ASSERT_LT(std::abs(point.x - expectedSharedX), 1e-12);
            ASSERT_GT(std::abs(point.x - s), 5e-8);
        }
    }

    ASSERT_EQ(sharedBLikeCount, 1);
}

// Demonstrates representative-average shared-vertex placement also holds when
// conversion first performs support-plane refit repair.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualSharedEdgeRepairsWithRepresentativeAverageTarget)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualSharedEdgeQuadPairBody();
    ASSERT_FALSE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 2);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    if (!result.success)
    {
        FAIL() << "issue=" << static_cast<int>(result.issue);
    }
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 2);
    ASSERT_EQ(result.body.ShellCount(), 1);
    ASSERT_FALSE(result.body.ShellAt(0).IsClosed());
    ASSERT_EQ(result.body.VertexCount(), 6);
    ASSERT_EQ(result.body.EdgeCount(), 7);

    const double s = 1e-5;
    const double dx = 2e-7;
    const double expectedSharedX = s + 0.5 * dx;

    std::size_t sharedBLikeCount = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const SCPoint3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12 && point.x > 0.5 * s && point.x < 1.5 * s)
        {
            ++sharedBLikeCount;
            ASSERT_LT(std::abs(point.x - expectedSharedX), 1e-12);
            ASSERT_GT(std::abs(point.x - s), 5e-8);
        }
    }

    ASSERT_EQ(sharedBLikeCount, 1);
}

// Demonstrates representative-id global averaging with support-plane refit on
// a multi-face shared-apex fan: near-equal apex variants merge to one shared
// Brep vertex at the expected average position.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualApexFanRepairsWithRepresentativeAverageTarget)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualApexTriangularFanBody();
    ASSERT_FALSE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 4);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    if (!result.success)
    {
        FAIL() << "issue=" << static_cast<int>(result.issue);
    }
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 4);
    ASSERT_EQ(result.body.ShellCount(), 1);
    ASSERT_FALSE(result.body.ShellAt(0).IsClosed());
    ASSERT_EQ(result.body.VertexCount(), 5);
    ASSERT_EQ(result.body.EdgeCount(), 8);

    const double s = 1e-5;
    const double expectedApexX = 5e-8;

    std::size_t apexLikeCount = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const SCPoint3d point = result.body.VertexAt(i).Point();
        if (point.z > 0.5 * s)
        {
            ++apexLikeCount;
            ASSERT_LT(std::abs(point.x - expectedApexX), 1e-12);
            ASSERT_LT(std::abs(point.y), 1e-12);
        }
    }

    ASSERT_EQ(apexLikeCount, 1);
}
// Demonstrates representative-average shared-vertex placement also works for
// a shared-corner fan where faces only meet at a single near-equal corner.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualSharedCornerFanRepairsWithRepresentativeAverageTarget)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualSharedCornerQuadFanBody();
    ASSERT_FALSE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 3);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 3);
    ASSERT_EQ(result.body.ShellCount(), 1);
    ASSERT_FALSE(result.body.ShellAt(0).IsClosed());
    ASSERT_EQ(result.body.VertexCount(), 8);
    ASSERT_EQ(result.body.EdgeCount(), 10);

    std::size_t sharedCornerCount = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const SCPoint3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.x) < 1e-12 && std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12)
        {
            ++sharedCornerCount;
        }
    }

    ASSERT_EQ(sharedCornerCount, 1);
}

// Demonstrates representative-average shared-vertex placement remains stable
// on a support-mismatch closed-shell input (tetrahedron).
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualClosedTetrahedronRepairsWithRepresentativeAverageTarget)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualClosedTetrahedronBody();
    ASSERT_FALSE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 4);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 4);
    ASSERT_EQ(result.body.ShellCount(), 1);
    ASSERT_TRUE(result.body.ShellAt(0).IsClosed());
    ASSERT_EQ(result.body.VertexCount(), 4);
    ASSERT_EQ(result.body.EdgeCount(), 6);

    std::size_t sharedV0Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const SCPoint3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.x) < 1e-12 && std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12)
        {
            ++sharedV0Count;
        }
    }

    ASSERT_EQ(sharedV0Count, 1);
}

struct SupportMismatchSharedChainCase
{
    const char* name;
    PolyhedronBody (*build)();
    std::size_t vertexCount;
    std::size_t edgeCount;
};

void PrintTo(const SupportMismatchSharedChainCase& scenario, std::ostream* stream)
{
    *stream << scenario.name;
}

class SupportMismatchSharedChainTest : public ::testing::TestWithParam<SupportMismatchSharedChainCase>
{
};

TEST_P(SupportMismatchSharedChainTest, PreservesRepresentativeAverageTargets)
{
    const SupportMismatchSharedChainCase& scenario = GetParam();
    const PolyhedronBody body = scenario.build();
    ASSERT_FALSE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 3);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 3);
    ASSERT_EQ(result.body.ShellCount(), 1);
    ASSERT_FALSE(result.body.ShellAt(0).IsClosed());
    ASSERT_EQ(result.body.VertexCount(), scenario.vertexCount);
    ASSERT_EQ(result.body.EdgeCount(), scenario.edgeCount);

    const double s = 1e-5;
    const double dx = 2e-7;
    const double expectedXLeft = s + 0.5 * dx;
    const double expectedXRight = 2.0 * s + 0.5 * dx;

    std::size_t leftCount = 0;
    std::size_t rightCount = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const SCPoint3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.y) < 1e-12 || std::abs(point.y - s) < 1e-12)
        {
            if (std::abs(point.x - expectedXLeft) < 1e-12)
            {
                ++leftCount;
            }
            if (std::abs(point.x - expectedXRight) < 1e-12)
            {
                ++rightCount;
            }
        }
    }

    ASSERT_EQ(leftCount, 2);
    ASSERT_EQ(rightCount, 2);
}

INSTANTIATE_TEST_SUITE_P(
    Conversion3d, SupportMismatchSharedChainTest,
    ::testing::Values(
        SupportMismatchSharedChainCase{"Base", &BuildSupportMismatchNearEqualSharedEdgeChainBody, 8, 10},
        SupportMismatchSharedChainCase{"DuplicateLoop", &BuildSupportMismatchNearEqualSharedEdgeChainWithDuplicateBody, 8, 10},
        SupportMismatchSharedChainCase{"DuplicateHole", &BuildSupportMismatchNearEqualSharedChainMixedContentWithDuplicateHoleBody, 12, 14},
        SupportMismatchSharedChainCase{"FullComposition", &BuildSupportMismatchNearEqualSharedChainFullCompositionBody, 13, 15},
        SupportMismatchSharedChainCase{"DualDuplicateFullComposition", &BuildSupportMismatchNearEqualSharedChainDualDuplicateFullCompositionBody, 13, 15}),
    [](const ::testing::TestParamInfo<SupportMismatchSharedChainCase>& info) {
        return std::string(info.param.name);
    });
