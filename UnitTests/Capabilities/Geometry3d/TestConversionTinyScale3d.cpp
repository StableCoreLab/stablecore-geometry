#include <cstddef>
#include <ostream>
#include <string>

#include <gtest/gtest.h>

#include "Geometry.h"
#include "Support/ConversionFixturesTinyScale3d.h"

using Geometry::BrepConversionIssue3d;
using Geometry::ConvertToBrepBody;
using Geometry::PolyhedronBody;
using Geometry::PolyhedronBrepBodyConversion3d;

using namespace Geometry::Test::ConversionFixtures;

struct TinyScaleRepairCase
{
    const char* name;
    PolyhedronBody (*build)();
    std::size_t faceCount;
    std::size_t vertexCount;
    std::size_t edgeCount;
    bool expectOpenShell{false};
};

void PrintTo(const TinyScaleRepairCase& scenario, std::ostream* stream)
{
    *stream << scenario.name;
}

class TinyScaleRepair3dTest : public ::testing::TestWithParam<TinyScaleRepairCase>
{
};

TEST_P(TinyScaleRepair3dTest, ConvertsToBrepWithExpectedTopology)
{
    const TinyScaleRepairCase& scenario = GetParam();
    const PolyhedronBody body = scenario.build();
    ASSERT_FALSE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), scenario.faceCount);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), scenario.faceCount);
    ASSERT_EQ(result.body.VertexCount(), scenario.vertexCount);
    ASSERT_EQ(result.body.EdgeCount(), scenario.edgeCount);
    if (scenario.expectOpenShell)
    {
        ASSERT_EQ(result.body.ShellCount(), 1);
        ASSERT_FALSE(result.body.ShellAt(0).IsClosed());
    }
}

INSTANTIATE_TEST_SUITE_P(
    Conversion3d, TinyScaleRepair3dTest,
    ::testing::Values(
        TinyScaleRepairCase{"NonPlanarFace", &BuildTinyScaleNonPlanarRepairBody, 1, 4, 4},
        TinyScaleRepairCase{"NonPlanarHoledFace", &BuildTinyScaleNonPlanarHoledRepairBody, 1, 8, 8},
        TinyScaleRepairCase{"NonPlanarMultiFace", &BuildTinyScaleNonPlanarMultiFaceRepairBody, 2, 8, 8},
        TinyScaleRepairCase{"NonPlanarMixedContent", &BuildTinyScaleNonPlanarMixedContentBody, 2, 12, 12},
        TinyScaleRepairCase{"NonPlanarSharedEdgeFaces", &BuildTinyScaleNonPlanarSharedEdgeBody, 2, 6, 7},
        TinyScaleRepairCase{"NonPlanarSharedEdgeChain", &BuildTinyScaleNonPlanarSharedEdgeChainBody, 3, 8, 10},
        TinyScaleRepairCase{"SupportMismatchChain", &BuildTinyScaleSupportMismatchSharedEdgeChainBody, 3, 8, 10, true},
        TinyScaleRepairCase{"SupportMismatchChainWithDuplicate", &BuildTinyScaleSupportMismatchSharedEdgeChainWithDuplicateBody, 3, 8, 10, true},
        TinyScaleRepairCase{"NonPlanarChainMixedContent", &BuildTinyScaleNonPlanarSharedEdgeChainMixedContentBody, 3, 12, 14},
        TinyScaleRepairCase{"ChainWithDuplicateLoop", &BuildTinyScaleNonPlanarSharedEdgeChainWithDuplicateLoopBody, 3, 8, 10},
        TinyScaleRepairCase{"ChainMixedContentDuplicateHole", &BuildTinyScaleSharedEdgeChainMixedContentWithDuplicateHoleBody, 3, 12, 14},
        TinyScaleRepairCase{"ChainMixedContentCollinearLeading", &BuildTinyScaleSharedChainMixedContentWithCollinearLeadingBody, 3, 13, 15},
        TinyScaleRepairCase{"ChainMixedContentSupportPlaneMismatch", &BuildTinyScaleSharedChainMixedContentSupportPlaneMismatchBody, 3, 12, 14},
        TinyScaleRepairCase{"ChainMixedContentSupportMismatchDuplicateHole", &BuildTinyScaleSharedChainMixedContentSupportMismatchWithDuplicateHoleBody, 3, 12, 14, true},
        TinyScaleRepairCase{"ChainSupportMismatchCollinear", &BuildTinyScaleSharedChainSupportMismatchAndCollinearBody, 3, 13, 15},
        TinyScaleRepairCase{"ChainFullComposition", &BuildTinyScaleSharedChainFullCompositionBody, 3, 13, 15},
        TinyScaleRepairCase{"ChainDualDuplicateFullComposition", &BuildTinyScaleSharedChainDualDuplicateFullCompositionBody, 3, 13, 15},
        TinyScaleRepairCase{"TriangularFaceChain", &BuildTinyScaleTriangularFaceChainBody, 3, 5, 7},
        TinyScaleRepairCase{"TriangularFan", &BuildTinyScaleTriangularFanBody, 4, 5, 8}),
    [](const ::testing::TestParamInfo<TinyScaleRepairCase>& info) {
        return std::string(info.param.name);
    });

// 验证小尺度封闭四面体的支撑面修复与共享拓扑，转换后仍为封闭壳。
TEST(Conversion3dCapabilityTest, TinyScaleClosedTetrahedronConvertsToBrepBodyWithClosedShell)
{
    const PolyhedronBody body = BuildTinyScaleClosedTetrahedronBody();
    ASSERT_FALSE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 4);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 4);
    ASSERT_EQ(result.body.VertexCount(), 4);
    ASSERT_EQ(result.body.EdgeCount(), 6);
    ASSERT_EQ(result.body.ShellCount(), 1);
    ASSERT_TRUE(result.body.ShellAt(0).IsClosed());
}
