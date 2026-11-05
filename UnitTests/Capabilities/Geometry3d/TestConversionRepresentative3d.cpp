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

// 同一共享边代表点判据覆盖有效输入和先执行支撑面修复的输入。
struct SharedEdgeRepresentativeCase
{
    const char* name;
    PolyhedronBody (*build)();
    bool inputValid;
};

void PrintTo(const SharedEdgeRepresentativeCase& scenario, std::ostream* stream)
{
    *stream << scenario.name;
}

class SharedEdgeRepresentative3dTest : public ::testing::TestWithParam<SharedEdgeRepresentativeCase>
{
};

TEST_P(SharedEdgeRepresentative3dTest, PreservesRepresentativeAverageTarget)
{
    const PolyhedronBody body = GetParam().build();
    ASSERT_EQ(body.IsValid(), GetParam().inputValid);
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

INSTANTIATE_TEST_SUITE_P(
    Conversion3d, SharedEdgeRepresentative3dTest,
    ::testing::Values(
        SharedEdgeRepresentativeCase{"ValidInput", &BuildNearEqualSharedEdgeQuadPairBody, true},
        SharedEdgeRepresentativeCase{"SupportMismatch", &BuildSupportMismatchNearEqualSharedEdgeQuadPairBody, false}),
    [](const ::testing::TestParamInfo<SharedEdgeRepresentativeCase>& info) {
        return std::string(info.param.name);
    });

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
// 原点代表点合并在开放共享角扇形与封闭四面体中使用同一坐标判据。
struct OriginRepresentativeCase
{
    const char* name;
    PolyhedronBody (*build)();
    std::size_t faceCount;
    std::size_t vertexCount;
    std::size_t edgeCount;
    bool closed;
};

void PrintTo(const OriginRepresentativeCase& scenario, std::ostream* stream)
{
    *stream << scenario.name;
}

class OriginRepresentative3dTest : public ::testing::TestWithParam<OriginRepresentativeCase>
{
};

TEST_P(OriginRepresentative3dTest, MergesOriginWithExpectedTopology)
{
    const PolyhedronBody body = GetParam().build();
    ASSERT_FALSE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), GetParam().faceCount);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), GetParam().faceCount);
    ASSERT_EQ(result.body.ShellCount(), 1);
    ASSERT_EQ(result.body.ShellAt(0).IsClosed(), GetParam().closed);
    ASSERT_EQ(result.body.VertexCount(), GetParam().vertexCount);
    ASSERT_EQ(result.body.EdgeCount(), GetParam().edgeCount);

    std::size_t originCount = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const SCPoint3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.x) < 1e-12 && std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12)
        {
            ++originCount;
        }
    }

    ASSERT_EQ(originCount, 1);
}

INSTANTIATE_TEST_SUITE_P(
    Conversion3d, OriginRepresentative3dTest,
    ::testing::Values(
        OriginRepresentativeCase{"SharedCornerFan", &BuildSupportMismatchNearEqualSharedCornerQuadFanBody, 3, 8, 10, false},
        OriginRepresentativeCase{"ClosedTetrahedron", &BuildSupportMismatchNearEqualClosedTetrahedronBody, 4, 4, 6, true}),
    [](const ::testing::TestParamInfo<OriginRepresentativeCase>& info) {
        return std::string(info.param.name);
    });

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

class SupportMismatchSharedChain3dTest : public ::testing::TestWithParam<SupportMismatchSharedChainCase>
{
};

TEST_P(SupportMismatchSharedChain3dTest, PreservesRepresentativeAverageTargets)
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
    Conversion3d, SupportMismatchSharedChain3dTest,
    ::testing::Values(
        SupportMismatchSharedChainCase{"Base", &BuildSupportMismatchNearEqualSharedEdgeChainBody, 8, 10},
        SupportMismatchSharedChainCase{"DuplicateLoop", &BuildSupportMismatchNearEqualSharedEdgeChainWithDuplicateBody, 8, 10},
        SupportMismatchSharedChainCase{"DuplicateHole", &BuildSupportMismatchNearEqualSharedChainMixedContentWithDuplicateHoleBody, 12, 14},
        SupportMismatchSharedChainCase{"FullComposition", &BuildSupportMismatchNearEqualSharedChainFullCompositionBody, 13, 15},
        SupportMismatchSharedChainCase{"DualDuplicateFullComposition", &BuildSupportMismatchNearEqualSharedChainDualDuplicateFullCompositionBody, 13, 15}),
    [](const ::testing::TestParamInfo<SupportMismatchSharedChainCase>& info) {
        return std::string(info.param.name);
    });
