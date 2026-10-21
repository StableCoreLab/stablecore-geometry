#include <cmath>
#include <memory>
#include <ostream>
#include <string>

#include <gtest/gtest.h>

#include "Geometry.h"
#include "Support/ConversionFixtures3d.h"
#include "Support/Fixtures3d.h"

using Geometry::SCBrepBody;
using Geometry::SCBrepCoedge;
using Geometry::BrepConversionIssue3d;
using Geometry::SCBrepEdge;
using Geometry::SCBrepFace;
using Geometry::SCBrepLoop;
using Geometry::SCBrepShell;
using Geometry::SCBrepVertex;
using Geometry::ComputeTriangleConnectedComponents;
using Geometry::ConvertToBrepBody;
using Geometry::ConvertToTriangleMesh;
using Geometry::SCCurveOnSurface;
using Geometry::SCIntervald;
using Geometry::SCLine3d;
using Geometry::SCLineCurve3d;
using Geometry::MeshConversionIssue3d;
using Geometry::SCPlane;
using Geometry::SCPlaneSurface;
using Geometry::SCPoint2d;
using Geometry::SCPoint3d;
using Geometry::PolyhedronBody;
using Geometry::PolyhedronBrepBodyConversion3d;
using Geometry::PolyhedronMeshConversion3d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;
using Geometry::ISCSurface;
using Geometry::SCVector3d;

using namespace Geometry::Test::ConversionFixtures;

// Demonstrates that a closed PolyhedronBody (unit cube, 6 quad faces) converts
// to a TriangleMesh with the correct triangle count and total ISCSurface area.
// High-fidelity feature-preserving conversion and non-planar polyhedron闂備焦鍓氶崑鍛櫠閻氱p
// repair remain open gaps.
TEST(Conversion3dCapabilityTest, UnitCubePolyhedronBodyConvertsToTriangleMesh)
{
    const PolyhedronBody cubeBody = Geometry::Test::BuildUnitCubeBody();
    ASSERT_TRUE(cubeBody.IsValid());
    ASSERT_EQ(cubeBody.FaceCount(), 6);

    const PolyhedronMeshConversion3d result = ConvertToTriangleMesh(cubeBody);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, MeshConversionIssue3d::None);
    ASSERT_TRUE(result.mesh.IsValid());
    // Each of the 6 quad faces is split into 2 triangles 闂?12 total
    ASSERT_EQ(result.mesh.TriangleCount(), 12);
    // Total ISCSurface area of a unit cube = 6 闂?(1闂?) = 6.0
    ASSERT_LT(std::abs(result.mesh.SurfaceArea() - 6.0), 1e-10);
}

// Demonstrates that a valid PolyhedronBody can be lifted into a planar-line
// SCBrepBody representation while preserving top-level face cardinality.
TEST(Conversion3dCapabilityTest, UnitCubePolyhedronBodyConvertsToBrepBody)
{
    const PolyhedronBody cubeBody = Geometry::Test::BuildUnitCubeBody();
    ASSERT_TRUE(cubeBody.IsValid());
    ASSERT_EQ(cubeBody.FaceCount(), 6);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(cubeBody);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 6);
    ASSERT_EQ(result.body.ShellCount(), 1);
    ASSERT_TRUE(result.body.ShellAt(0).IsClosed());
    ASSERT_EQ(result.body.VertexCount(), 8);
    ASSERT_EQ(result.body.EdgeCount(), 12);
}

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
// Demonstrates representative-average shared-vertex placement scales to a
// support-mismatch shared-edge chain (three faces) with near-equal middle-face
// perturbations on both shared edges.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualSharedEdgeChainRepairsWithRepresentativeAverageTargets)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualSharedEdgeChainBody();
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

// Demonstrates representative-average placement remains deterministic when
// support-mismatch shared-edge chain also requires duplicate-loop normalization
// on the middle face.
TEST(Conversion3dCapabilityTest,
     SupportMismatchNearEqualSharedEdgeChainWithDuplicateRepairsWithRepresentativeAverageTargets)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualSharedEdgeChainWithDuplicateBody();
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
        SupportMismatchSharedChainCase{"DuplicateHole", &BuildSupportMismatchNearEqualSharedChainMixedContentWithDuplicateHoleBody, 12, 14},
        SupportMismatchSharedChainCase{"FullComposition", &BuildSupportMismatchNearEqualSharedChainFullCompositionBody, 13, 15},
        SupportMismatchSharedChainCase{"DualDuplicateFullComposition", &BuildSupportMismatchNearEqualSharedChainDualDuplicateFullCompositionBody, 13, 15}),
    [](const ::testing::TestParamInfo<SupportMismatchSharedChainCase>& info) {
        return std::string(info.param.name);
    });

// Closed tetrahedron and prism representative-average repair variants.
struct SupportMismatchRepresentativeCase
{
    const char* name;
    PolyhedronBody (*build)();
};

void PrintTo(const SupportMismatchRepresentativeCase& scenario, std::ostream* stream)
{
    *stream << scenario.name;
}

class ClosedTetrahedronDualVerticesTest : public ::testing::TestWithParam<SupportMismatchRepresentativeCase>
{
};

TEST_P(ClosedTetrahedronDualVerticesTest, PreservesRepresentativeAverageTarget)
{
    const PolyhedronBody body = GetParam().build();
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

    const double s = 1e-5;
    const double expectedV1X = s + (2e-7 - 1e-7 + 0.0) / 3.0;

    std::size_t sharedV0Count = 0;
    std::size_t sharedV1Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const SCPoint3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.x) < 1e-12 && std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12)
        {
            ++sharedV0Count;
        }
        if (std::abs(point.y) < 1e-12 && std::abs(point.z - 1.2e-6) < 1e-12 && point.x > 0.5 * s && point.x < 1.5 * s)
        {
            ++sharedV1Count;
            ASSERT_LT(std::abs(point.x - expectedV1X), 1e-10);
        }
    }

    ASSERT_EQ(sharedV0Count, 1);
    ASSERT_EQ(sharedV1Count, 1);
}

INSTANTIATE_TEST_SUITE_P(
    Conversion3d, ClosedTetrahedronDualVerticesTest,
    ::testing::Values(
        SupportMismatchRepresentativeCase{"Base", &BuildSupportMismatchNearEqualClosedTetrahedronDualVerticesBody},
        SupportMismatchRepresentativeCase{"DuplicateLoop", &BuildSupportMismatchNearEqualClosedTetrahedronDualVerticesWithDuplicateLoopBody},
        SupportMismatchRepresentativeCase{"DualDuplicateLoop", &BuildSupportMismatchNearEqualClosedTetrahedronDualVerticesWithDualDuplicateLoopBody}),
    [](const ::testing::TestParamInfo<SupportMismatchRepresentativeCase>& info) {
        return std::string(info.param.name);
    });

class ClosedTetrahedronAllVerticesTest : public ::testing::TestWithParam<SupportMismatchRepresentativeCase>
{
};

TEST_P(ClosedTetrahedronAllVerticesTest, PreservesRepresentativeAverageTarget)
{
    const PolyhedronBody body = GetParam().build();
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

    const double s = 1e-5;
    const double expectedV1X = s + 1.0e-7 / 3.0;
    const double expectedV3X = 0.5 * s + 1.0e-7 / 3.0;

    std::size_t sharedV1Count = 0;
    std::size_t sharedV3Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const SCPoint3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.y) < 1e-12 && point.z > 0.5e-6 && point.z < 2.0e-6 && point.x > 0.5 * s)
        {
            ++sharedV1Count;
            ASSERT_LT(std::abs(point.x - expectedV1X), 1e-10);
        }
        if (point.z > 0.7 * s && point.x > 0.3 * s && point.x < 0.7 * s)
        {
            ++sharedV3Count;
            ASSERT_LT(std::abs(point.x - expectedV3X), 1e-10);
        }
    }

    ASSERT_EQ(sharedV1Count, 1);
    ASSERT_EQ(sharedV3Count, 1);
}

INSTANTIATE_TEST_SUITE_P(
    Conversion3d, ClosedTetrahedronAllVerticesTest,
    ::testing::Values(
        SupportMismatchRepresentativeCase{"Base", &BuildSupportMismatchNearEqualClosedTetrahedronAllVerticesBody},
        SupportMismatchRepresentativeCase{"DuplicateLoop", &BuildSupportMismatchNearEqualClosedTetrahedronAllVerticesWithDuplicateLoopBody},
        SupportMismatchRepresentativeCase{"DualDuplicateLoop", &BuildSupportMismatchNearEqualClosedTetrahedronAllVerticesWithDualDuplicateLoopBody}),
    [](const ::testing::TestParamInfo<SupportMismatchRepresentativeCase>& info) {
        return std::string(info.param.name);
    });

class ClosedPrismDualVerticesTest : public ::testing::TestWithParam<SupportMismatchRepresentativeCase>
{
};

TEST_P(ClosedPrismDualVerticesTest, PreservesRepresentativeAverageTarget)
{
    const PolyhedronBody body = GetParam().build();
    ASSERT_FALSE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 5);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 5);
    ASSERT_EQ(result.body.ShellCount(), 1);
    ASSERT_TRUE(result.body.ShellAt(0).IsClosed());
    ASSERT_EQ(result.body.VertexCount(), 6);
    ASSERT_EQ(result.body.EdgeCount(), 9);

    const double s = 1e-5;
    const double expectedV4X = s + 1.0e-7 / 3.0;

    std::size_t sharedV0Count = 0;
    std::size_t sharedV4Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const SCPoint3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.x) < 1e-12 && std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12)
        {
            ++sharedV0Count;
        }
        if (std::abs(point.y) < 1e-12 && point.x > 0.5 * s && point.x < 1.5 * s && point.z > 0.5 * s &&
            point.z < 1.5 * s)
        {
            ++sharedV4Count;
            ASSERT_LT(std::abs(point.x - expectedV4X), 1e-10);
        }
    }

    ASSERT_EQ(sharedV0Count, 1);
    ASSERT_EQ(sharedV4Count, 1);
}

INSTANTIATE_TEST_SUITE_P(
    Conversion3d, ClosedPrismDualVerticesTest,
    ::testing::Values(
        SupportMismatchRepresentativeCase{"Base", &BuildSupportMismatchNearEqualClosedPrismDualVerticesBody},
        SupportMismatchRepresentativeCase{"DuplicateLoop", &BuildSupportMismatchNearEqualClosedPrismDualVerticesWithDuplicateLoopBody},
        SupportMismatchRepresentativeCase{"DualDuplicateLoop", &BuildSupportMismatchNearEqualClosedPrismDualVerticesWithDualDuplicateLoopBody}),
    [](const ::testing::TestParamInfo<SupportMismatchRepresentativeCase>& info) {
        return std::string(info.param.name);
    });

class ClosedPrismAllVerticesTest : public ::testing::TestWithParam<SupportMismatchRepresentativeCase>
{
};

TEST_P(ClosedPrismAllVerticesTest, PreservesRepresentativeAverageTarget)
{
    const PolyhedronBody body = GetParam().build();
    ASSERT_FALSE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 5);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 5);
    ASSERT_EQ(result.body.ShellCount(), 1);
    ASSERT_TRUE(result.body.ShellAt(0).IsClosed());
    ASSERT_EQ(result.body.VertexCount(), 6);
    ASSERT_EQ(result.body.EdgeCount(), 9);

    const double s = 1e-5;
    const double expectedV1X = s + 1.0e-7 / 3.0;
    const double expectedV4X = s + 2.0e-7 / 3.0;

    std::size_t sharedV1Count = 0;
    std::size_t sharedV4Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const SCPoint3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12 && point.x > 0.5 * s && point.x < 1.5 * s)
        {
            ++sharedV1Count;
            ASSERT_LT(std::abs(point.x - expectedV1X), 1e-10);
        }
        if (std::abs(point.y) < 1e-12 && point.x > 0.5 * s && point.x < 1.5 * s && point.z > 0.5 * s &&
            point.z < 1.5 * s)
        {
            ++sharedV4Count;
            ASSERT_LT(std::abs(point.x - expectedV4X), 1e-10);
        }
    }

    ASSERT_EQ(sharedV1Count, 1);
    ASSERT_EQ(sharedV4Count, 1);
}

INSTANTIATE_TEST_SUITE_P(
    Conversion3d, ClosedPrismAllVerticesTest,
    ::testing::Values(
        SupportMismatchRepresentativeCase{"Base", &BuildSupportMismatchNearEqualClosedPrismAllVerticesBody},
        SupportMismatchRepresentativeCase{"DuplicateLoop", &BuildSupportMismatchNearEqualClosedPrismAllVerticesWithDuplicateLoopBody},
        SupportMismatchRepresentativeCase{"DualDuplicateLoop", &BuildSupportMismatchNearEqualClosedPrismAllVerticesWithDualDuplicateLoopBody}),
    [](const ::testing::TestParamInfo<SupportMismatchRepresentativeCase>& info) {
        return std::string(info.param.name);
    });


// Demonstrates a non-axis-aligned (affine-skewed) polyhedron subset can be
// converted to SCBrepBody without requiring robust non-planar repair.
TEST(Conversion3dCapabilityTest, SkewedCubePolyhedronBodyConvertsToBrepBody)
{
    PolyhedronBody skewedBody;
    ASSERT_NO_THROW(skewedBody = BuildSkewedUnitCubeBody());
    ASSERT_TRUE(skewedBody.IsValid());
    ASSERT_EQ(skewedBody.FaceCount(), 6);

    PolyhedronBrepBodyConversion3d result{};
    ASSERT_NO_THROW(result = ConvertToBrepBody(skewedBody));
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);

    bool bodyValid = false;
    ASSERT_NO_THROW(bodyValid = result.body.IsValid());
    ASSERT_TRUE(bodyValid);

    ASSERT_EQ(result.body.FaceCount(), 6);
    ASSERT_EQ(result.body.VertexCount(), 8);
    ASSERT_EQ(result.body.EdgeCount(), 12);

    std::size_t shellCount = 0;
    ASSERT_NO_THROW(shellCount = result.body.ShellCount());
    ASSERT_EQ(shellCount, 1u);
}

// Demonstrates the conversion can repair face support-plane mismatch and still
// produce a valid SCBrepBody for a recoverable polyhedron input.
TEST(Conversion3dCapabilityTest, SupportPlaneMismatchedCubeCanBeRepairedToBrepBody)
{
    const PolyhedronBody mismatchedBody = BuildSupportPlaneMismatchedCubeBody();
    ASSERT_FALSE(mismatchedBody.IsValid());

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(mismatchedBody);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 6);
    ASSERT_EQ(result.body.VertexCount(), 8);
    ASSERT_EQ(result.body.EdgeCount(), 12);
    ASSERT_EQ(result.body.ShellCount(), 1);
    ASSERT_TRUE(result.body.ShellAt(0).IsClosed());
}

// Demonstrates conversion can repair mildly non-planar face loops by
// projecting vertices onto a refit support plane.
TEST(Conversion3dCapabilityTest, MildlyNonPlanarCubeFaceCanBeRepairedToBrepBody)
{
    const PolyhedronBody nonPlanarBody = BuildMildlyNonPlanarCubeFaceBody();
    ASSERT_FALSE(nonPlanarBody.IsValid());

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(nonPlanarBody);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 6);
    ASSERT_EQ(result.body.VertexCount(), 8);
    ASSERT_EQ(result.body.EdgeCount(), 12);
    ASSERT_EQ(result.body.ShellCount(), 1);
    ASSERT_TRUE(result.body.ShellAt(0).IsClosed());
}

// Non-planar cube repair variants with displaced vertices and duplicate loops.
struct DeformedCubeRepairCase
{
    const char* name;
    PolyhedronBody (*build)();
};

void PrintTo(const DeformedCubeRepairCase& scenario, std::ostream* stream)
{
    *stream << scenario.name;
}

class DeformedCubeRepairTest : public ::testing::TestWithParam<DeformedCubeRepairCase>
{
};

TEST_P(DeformedCubeRepairTest, ConvertsToClosedSharedTopologyBrepBody)
{
    const PolyhedronBody deformedBody = GetParam().build();
    ASSERT_FALSE(deformedBody.IsValid());

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(deformedBody);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 6);
    ASSERT_EQ(result.body.VertexCount(), 8);
    ASSERT_EQ(result.body.EdgeCount(), 12);
    ASSERT_EQ(result.body.ShellCount(), 1);
    ASSERT_TRUE(result.body.ShellAt(0).IsClosed());
}

INSTANTIATE_TEST_SUITE_P(
    Conversion3d, DeformedCubeRepairTest,
    ::testing::Values(
        DeformedCubeRepairCase{"DisplacedVertex", &BuildDeformedUnitCubeBody},
        DeformedCubeRepairCase{"DisplacedVertexDuplicateLoop", &BuildDeformedUnitCubeWithDuplicateLoopBody},
        DeformedCubeRepairCase{"DisplacedVertexDualDuplicateLoop", &BuildDeformedUnitCubeWithDualDuplicateLoopBody},
        DeformedCubeRepairCase{"DualDisplacedVertices", &BuildDualDeformedUnitCubeBody},
        DeformedCubeRepairCase{"DualDisplacedVerticesDuplicateLoop", &BuildDualDeformedUnitCubeWithDuplicateLoopBody},
        DeformedCubeRepairCase{"DualDisplacedVerticesDualDuplicateLoop", &BuildDualDeformedUnitCubeWithDualDuplicateLoopBody}),
    [](const ::testing::TestParamInfo<DeformedCubeRepairCase>& info) {
        return std::string(info.param.name);
    });


// Demonstrates refit-plane projection repair also handles mildly non-planar
// hole loops in a planar-holed face.
TEST(Conversion3dCapabilityTest, MildlyNonPlanarHoleLoopCanBeRepairedToBrepBody)
{
    const PolyhedronBody nonPlanarHoledBody = BuildMildlyNonPlanarHoledFaceBody();
    ASSERT_FALSE(nonPlanarHoledBody.IsValid());

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(nonPlanarHoledBody);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 1);
    ASSERT_EQ(result.body.VertexCount(), 8);
    ASSERT_EQ(result.body.EdgeCount(), 8);
}

// Demonstrates refit support-plane selection now scores all loop vertices, so
// a holed face can converge to the lower-error shared plane even when the outer
// loop alone would suggest a worse tilted support plane.
TEST(Conversion3dCapabilityTest, HoleDominatedNonPlanarHoledFaceRepairsToPlanarBrepBody)
{
    const PolyhedronBody body = BuildHoleDominatedNonPlanarHoledFaceBody();
    ASSERT_FALSE(body.IsValid());

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 1);
    ASSERT_EQ(result.body.VertexCount(), 8);
    ASSERT_EQ(result.body.EdgeCount(), 8);

    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const SCPoint3d point = result.body.VertexAt(i).Point();
        ASSERT_LT(std::abs(point.z), 1e-10);
    }
}

// Demonstrates the all-loop support-plane scoring also composes with
// shared-edge mixed-content repair: a holed face can choose the lower-error
// plane dictated by its hole loop while still preserving shared topology with
// its adjacent plain face.
TEST(Conversion3dCapabilityTest, SharedEdgeHoleDominatedMixedContentRepairsToPlanarSharedTopologyBrepBody)
{
    const PolyhedronBody body = BuildSharedEdgeHoleDominatedMixedContentBody();
    ASSERT_FALSE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 2);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 2);
    ASSERT_EQ(result.body.VertexCount(), 10);
    ASSERT_EQ(result.body.EdgeCount(), 11);
    ASSERT_EQ(result.body.ShellCount(), 1);
    ASSERT_FALSE(result.body.ShellAt(0).IsClosed());

    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const SCPoint3d point = result.body.VertexAt(i).Point();
        ASSERT_LT(std::abs(point.z), 1e-10);
    }
}

// Demonstrates the all-loop support-plane scoring scales from one shared edge
// to a minimal three-face chain: the middle holed face still chooses the
// lower-error plane dictated by its hole loop while preserving shared topology
// with both adjacent plain faces.
TEST(Conversion3dCapabilityTest, SharedChainHoleDominatedMixedContentRepairsToPlanarSharedTopologyBrepBody)
{
    const PolyhedronBody body = BuildSharedChainHoleDominatedMixedContentBody();
    ASSERT_FALSE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 3);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 3);
    ASSERT_EQ(result.body.VertexCount(), 12);
    ASSERT_EQ(result.body.EdgeCount(), 14);
    ASSERT_EQ(result.body.ShellCount(), 1);
    ASSERT_FALSE(result.body.ShellAt(0).IsClosed());

    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const SCPoint3d point = result.body.VertexAt(i).Point();
        ASSERT_LT(std::abs(point.z), 1e-10);
    }
}

// Demonstrates all-loop support-plane scoring also composes with
// representative-average placement on a minimal three-face shared chain: the
// middle holed face picks the lower-error plane dictated by its hole loop
// while near-equal shared-edge vertices still converge to deterministic
// averaged targets across both adjacent plain faces.
TEST(Conversion3dCapabilityTest,
     SupportMismatchNearEqualSharedChainHoleDominatedMixedContentRepairsWithRepresentativeAverageTarget)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualSharedChainHoleDominatedMixedContentBody();
    ASSERT_FALSE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 3);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 3);
    ASSERT_EQ(result.body.VertexCount(), 12);
    ASSERT_EQ(result.body.EdgeCount(), 14);
    ASSERT_EQ(result.body.ShellCount(), 1);
    ASSERT_FALSE(result.body.ShellAt(0).IsClosed());

    const double expectedLeftX = 2.0 + 0.5 * 2.0e-7;
    const double expectedRightX = 6.0 + 0.5 * 2.0e-7;
    std::size_t leftCount = 0;
    std::size_t rightCount = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const SCPoint3d point = result.body.VertexAt(i).Point();
        ASSERT_LT(std::abs(point.z), 1e-10);

        if (std::abs(point.y) < 1e-12 || std::abs(point.y - 2.0) < 1e-12)
        {
            if (std::abs(point.x - expectedLeftX) < 1e-10)
            {
                ++leftCount;
            }
            if (std::abs(point.x - expectedRightX) < 1e-10)
            {
                ++rightCount;
            }
        }
    }

    ASSERT_EQ(leftCount, 2);
    ASSERT_EQ(rightCount, 2);
}

// Demonstrates duplicate-hole normalization composes with the
// hole-dominated shared-chain representative-average path: the middle holed
// face still picks the lower-error plane while near-equal shared-edge
// vertices converge to deterministic average targets.
TEST(
    Conversion3dCapabilityTest,
    SupportMismatchNearEqualSharedChainHoleDominatedMixedContentWithDuplicateHoleRepairsWithRepresentativeAverageTarget)
{
    const PolyhedronBody body =
        BuildSupportMismatchNearEqualSharedChainHoleDominatedMixedContentWithDuplicateHoleBody();
    ASSERT_FALSE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 3);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 3);
    ASSERT_EQ(result.body.VertexCount(), 12);
    ASSERT_EQ(result.body.EdgeCount(), 14);
    ASSERT_EQ(result.body.ShellCount(), 1);
    ASSERT_FALSE(result.body.ShellAt(0).IsClosed());

    const double expectedLeftX = 2.0 + 0.5 * 2.0e-7;
    const double expectedRightX = 6.0 + 0.5 * 2.0e-7;
    std::size_t leftCount = 0;
    std::size_t rightCount = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const SCPoint3d point = result.body.VertexAt(i).Point();
        ASSERT_LT(std::abs(point.z), 1e-10);

        if (std::abs(point.y) < 1e-12 || std::abs(point.y - 2.0) < 1e-12)
        {
            if (std::abs(point.x - expectedLeftX) < 1e-10)
            {
                ++leftCount;
            }
            if (std::abs(point.x - expectedRightX) < 1e-10)
            {
                ++rightCount;
            }
        }
    }

    ASSERT_EQ(leftCount, 2);
    ASSERT_EQ(rightCount, 2);
}

// Demonstrates collinear-leading fallback also composes with the
// hole-dominated shared-chain representative-average path after duplicate-hole
// normalization: the middle holed face still picks the lower-error plane while
// shared-edge targets remain deterministic.
TEST(Conversion3dCapabilityTest,
     SupportMismatchNearEqualSharedChainHoleDominatedFullCompositionRepairsWithRepresentativeAverageTarget)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualSharedChainHoleDominatedFullCompositionBody();
    ASSERT_FALSE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 3);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 3);
    ASSERT_EQ(result.body.VertexCount(), 13);
    ASSERT_EQ(result.body.EdgeCount(), 15);
    ASSERT_EQ(result.body.ShellCount(), 1);
    ASSERT_FALSE(result.body.ShellAt(0).IsClosed());

    const double expectedLeftX = 2.0 + 0.5 * 2.0e-7;
    const double expectedRightX = 6.0 + 0.5 * 2.0e-7;
    std::size_t leftCount = 0;
    std::size_t rightCount = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const SCPoint3d point = result.body.VertexAt(i).Point();
        ASSERT_LT(std::abs(point.z), 1e-10);

        if (std::abs(point.y) < 1e-12 || std::abs(point.y - 2.0) < 1e-12)
        {
            if (std::abs(point.x - expectedLeftX) < 1e-10)
            {
                ++leftCount;
            }
            if (std::abs(point.x - expectedRightX) < 1e-10)
            {
                ++rightCount;
            }
        }
    }

    ASSERT_EQ(leftCount, 2);
    ASSERT_EQ(rightCount, 2);
}

// Demonstrates refit-plane repair remains robust when leading loop vertices
// are collinear and cannot directly define a support normal.
TEST(Conversion3dCapabilityTest, CollinearLeadingLoopStillRepairsToBrepBody)
{
    const PolyhedronBody body = BuildSupportPlaneMismatchedCollinearLeadingLoopBody();
    ASSERT_FALSE(body.IsValid());

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 1);
    ASSERT_EQ(result.body.VertexCount(), 5);
    ASSERT_EQ(result.body.EdgeCount(), 5);
}

// Demonstrates conversion repair can normalize consecutive duplicate loop
// vertices before support-plane refit and Brep reconstruction.
TEST(Conversion3dCapabilityTest, DuplicateVertexLoopStillRepairsToBrepBody)
{
    const PolyhedronBody body = BuildDuplicateVertexLoopBody();
    ASSERT_FALSE(body.IsValid());

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 1);
    ASSERT_EQ(result.body.VertexCount(), 4);
    ASSERT_EQ(result.body.EdgeCount(), 4);
}

// Demonstrates duplicate vertices in a hole loop are normalized before
// refit-plane conversion to SCBrepBody.
TEST(Conversion3dCapabilityTest, DuplicateVertexHoleLoopStillRepairsToBrepBody)
{
    const PolyhedronBody body = BuildDuplicateVertexHoleLoopBody();
    ASSERT_FALSE(body.IsValid());

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 1);
    ASSERT_EQ(result.body.VertexCount(), 8);
    ASSERT_EQ(result.body.EdgeCount(), 8);
}

// Demonstrates repair sub-strategies compose deterministically on one face:
// support-plane mismatch + collinear leading outer points + duplicate points +
// mild non-planar disturbances on outer and hole loops.
TEST(Conversion3dCapabilityTest, CompositeRepairStressFaceStillConvertsToBrepBody)
{
    const PolyhedronBody body = BuildCompositeRepairStressFaceBody();
    ASSERT_FALSE(body.IsValid());

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 1);
    ASSERT_EQ(result.body.VertexCount(), 10);
    ASSERT_EQ(result.body.EdgeCount(), 10);
}

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

class TinyScaleRepairTest : public ::testing::TestWithParam<TinyScaleRepairCase>
{
};

TEST_P(TinyScaleRepairTest, ConvertsToBrepWithExpectedTopology)
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
    Conversion3d, TinyScaleRepairTest,
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
        TinyScaleRepairCase{"ChainDualDuplicateFullComposition", &BuildTinyScaleSharedChainDualDuplicateFullCompositionBody, 3, 13, 15}),
    [](const ::testing::TestParamInfo<TinyScaleRepairCase>& info) {
        return std::string(info.param.name);
    });

// Demonstrates that a tiny-scale closed tetrahedron polyhedron (4 triangular
// faces, all 6 edges shared by exactly two faces each) with identical
// mismatched support planes can be fully repaired and converted to a valid
// closed SCBrepBody with globally shared topology (VertexCount=4, EdgeCount=6,
// IsClosed=true). This is the representative closed-shell shared-topology
// conversion sub-scenario for the non-planar repair capability.
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

// Demonstrates that for a strip of triangular faces sharing edges (all with
// mismatched support planes), independent per-face refit achieves provably
// correct shared-edge vertex consistency.  Each shared vertex is one of the
// 3 defining points of every triangle that contains it, so its distance to
// that face's refit-plane is exactly 0 闂?no projection divergence.  The
// result is VertexCount=5 and EdgeCount=7 (2 shared edges properly reused).
TEST(Conversion3dCapabilityTest, TinyScaleTriangularFaceChainRepairsToBrepBodyWithSharedEdgeConsistency)
{
    const PolyhedronBody body = BuildTinyScaleTriangularFaceChainBody();
    ASSERT_FALSE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 3);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 3);
    ASSERT_EQ(result.body.VertexCount(), 5);
    ASSERT_EQ(result.body.EdgeCount(), 7);
}

// Demonstrates planar holed SCBrepBody conversion keeps representative area by
// honoring hole trims in mesh triangulation.
// Demonstrates that four triangular faces sharing a common apex vertex (square
// pyramid cap without base), all with mismatched support planes, produce a
// SCBrepBody with exactly VertexCount=5 and EdgeCount=8. The shared apex is a
// defining vertex of all four triangles 闂?distance 0 from each refit-plane
// 闂?// never projected 闂?exact merge.  Four radial edges are each used twice;
// four outer edges are each used once 闂?the open boundary is deterministic.
TEST(Conversion3dCapabilityTest, TinyScaleTriangularFanRepairsToBrepBodyWithSharedApex)
{
    const PolyhedronBody body = BuildTinyScaleTriangularFanBody();
    ASSERT_FALSE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 4);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 4);
    ASSERT_EQ(result.body.VertexCount(), 5);
    ASSERT_EQ(result.body.EdgeCount(), 8);
}

// Demonstrates planar holed SCBrepBody conversion keeps representative area by
// honoring hole trims in mesh triangulation.
TEST(Conversion3dCapabilityTest, PlanarHoledBrepBodyConvertsToMeshWithExpectedArea)
{
    std::vector<SCBrepVertex> vertices{SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{4.0, 0.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{4.0, 4.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{0.0, 4.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{3.0, 1.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{3.0, 3.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{1.0, 3.0, 0.0})};

    std::vector<SCBrepEdge> edges;
    auto addEdge = [&](std::size_t start, std::size_t end) {
        const SCPoint3d first = vertices[start].Point();
        const SCPoint3d second = vertices[end].Point();
        edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                               SCLine3d::FromOriginAndDirection(first, second - first), SCIntervald{0.0, 1.0})),
                           start,
                           end);
    };
    addEdge(0, 1);
    addEdge(1, 2);
    addEdge(2, 3);
    addEdge(3, 0);
    addEdge(4, 5);
    addEdge(5, 6);
    addEdge(6, 7);
    addEdge(7, 4);

    const SCBrepLoop outerLoop({SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop holeLoop({SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(7, false)});

    const SCPlaneSurface planeSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    auto supportSurface = std::shared_ptr<ISCSurface>(planeSurface.Clone().release());
    const SCCurveOnSurface outerTrim(supportSurface,
                                   SCPolyline2d(
                                       {
                                           SCPoint2d{0.0, 0.0},
                                           SCPoint2d{4.0, 0.0},
                                           SCPoint2d{4.0, 4.0},
                                           SCPoint2d{0.0, 4.0},
                                       },
                                       SCPolylineClosure::Closed));
    const SCCurveOnSurface holeTrim(supportSurface,
                                  SCPolyline2d(
                                      {
                                          SCPoint2d{1.0, 1.0},
                                          SCPoint2d{3.0, 1.0},
                                          SCPoint2d{3.0, 3.0},
                                          SCPoint2d{1.0, 3.0},
                                      },
                                      SCPolylineClosure::Closed));

    const SCBrepFace face(
        std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), outerLoop, {holeLoop}, outerTrim, {holeTrim});
    const SCBrepBody body(vertices, edges, {SCBrepShell({face}, false)});
    ASSERT_TRUE(body.IsValid());

    const PolyhedronMeshConversion3d mesh = ConvertToTriangleMesh(body);
    ASSERT_TRUE(mesh.success);
    ASSERT_TRUE(mesh.mesh.IsValid());
    // Expected area: 4x4 outer minus 2x2 hole = 12.
    ASSERT_LT(std::abs(mesh.mesh.SurfaceArea() - 12.0), 1e-8);
}

// Demonstrates planar multi-face SCBrepBody conversion keeps representative
// aggregate area across disconnected faces.
TEST(Conversion3dCapabilityTest, PlanarMultiFaceBrepBodyConvertsToMeshWithExpectedArea)
{
    std::vector<SCBrepVertex> vertices{// Face A square: area 4
                                     SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{2.0, 0.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{2.0, 2.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{0.0, 2.0, 0.0}),
                                     // Face B rectangle: area 2
                                     SCBrepVertex(SCPoint3d{3.0, 0.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{4.0, 0.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{4.0, 2.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{3.0, 2.0, 0.0})};

    std::vector<SCBrepEdge> edges;
    auto addEdge = [&](std::size_t start, std::size_t end) {
        const SCPoint3d first = vertices[start].Point();
        const SCPoint3d second = vertices[end].Point();
        edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                               SCLine3d::FromOriginAndDirection(first, second - first), SCIntervald{0.0, 1.0})),
                           start,
                           end);
    };

    // Face A
    addEdge(0, 1);
    addEdge(1, 2);
    addEdge(2, 3);
    addEdge(3, 0);
    // Face B
    addEdge(4, 5);
    addEdge(5, 6);
    addEdge(6, 7);
    addEdge(7, 4);

    const SCBrepLoop outerA({SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop outerB({SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(7, false)});

    const SCPlaneSurface planeSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    auto supportA = std::shared_ptr<ISCSurface>(planeSurface.Clone().release());
    auto supportB = std::shared_ptr<ISCSurface>(planeSurface.Clone().release());

    const SCCurveOnSurface trimA(supportA,
                               SCPolyline2d(
                                   {
                                       SCPoint2d{0.0, 0.0},
                                       SCPoint2d{2.0, 0.0},
                                       SCPoint2d{2.0, 2.0},
                                       SCPoint2d{0.0, 2.0},
                                   },
                                   SCPolylineClosure::Closed));
    const SCCurveOnSurface trimB(supportB,
                               SCPolyline2d(
                                   {
                                       SCPoint2d{3.0, 0.0},
                                       SCPoint2d{4.0, 0.0},
                                       SCPoint2d{4.0, 2.0},
                                       SCPoint2d{3.0, 2.0},
                                   },
                                   SCPolylineClosure::Closed));

    const SCBrepFace faceA(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), outerA, {}, trimA, {});
    const SCBrepFace faceB(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), outerB, {}, trimB, {});
    const SCBrepBody body(vertices, edges, {SCBrepShell({faceA, faceB}, false)});
    ASSERT_TRUE(body.IsValid());

    const PolyhedronMeshConversion3d mesh = ConvertToTriangleMesh(body);
    ASSERT_TRUE(mesh.success);
    ASSERT_TRUE(mesh.mesh.IsValid());
    // Expected area: 4 + 2 = 6.
    ASSERT_LT(std::abs(mesh.mesh.SurfaceArea() - 6.0), 1e-8);
}

// Demonstrates representative shared-edge Brep->mesh conversion can preserve
// shared geometric vertices globally instead of duplicating each face mesh.
TEST(Conversion3dCapabilityTest, PlanarSharedEdgeBrepBodyConvertsToMeshWithSharedVertices)
{
    std::vector<SCBrepVertex> vertices{SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{1.0, 0.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{2.0, 0.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{2.0, 1.0, 0.0})};

    std::vector<SCBrepEdge> edges;
    auto addEdge = [&](std::size_t start, std::size_t end) {
        const SCPoint3d first = vertices[start].Point();
        const SCPoint3d second = vertices[end].Point();
        edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                               SCLine3d::FromOriginAndDirection(first, second - first), SCIntervald{0.0, 1.0})),
                           start,
                           end);
    };

    addEdge(0, 1);
    addEdge(1, 2);
    addEdge(2, 3);
    addEdge(3, 0);
    addEdge(1, 4);
    addEdge(4, 5);
    addEdge(5, 2);

    const SCBrepLoop outerA({SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop outerB({SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(1, true)});

    const SCPlaneSurface planeSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));

    auto makeTrim = [&](const std::vector<SCPoint2d>& points) {
        return SCCurveOnSurface(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()),
                              SCPolyline2d(points, SCPolylineClosure::Closed));
    };

    const SCBrepFace faceA(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()),
                         outerA,
                         {},
                         makeTrim({
                             SCPoint2d{0.0, 0.0},
                             SCPoint2d{1.0, 0.0},
                             SCPoint2d{1.0, 1.0},
                             SCPoint2d{0.0, 1.0},
                         }),
                         {});
    const SCBrepFace faceB(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()),
                         outerB,
                         {},
                         makeTrim({
                             SCPoint2d{1.0, 0.0},
                             SCPoint2d{2.0, 0.0},
                             SCPoint2d{2.0, 1.0},
                             SCPoint2d{1.0, 1.0},
                         }),
                         {});

    const SCBrepBody body(vertices, edges, {SCBrepShell({faceA, faceB}, false)});
    ASSERT_TRUE(body.IsValid());

    const PolyhedronMeshConversion3d mesh = ConvertToTriangleMesh(body);
    ASSERT_TRUE(mesh.success);
    ASSERT_TRUE(mesh.mesh.IsValid());
    ASSERT_EQ(mesh.mesh.TriangleCount(), 4);
    ASSERT_EQ(mesh.mesh.VertexCount(), 6);
    ASSERT_LT(std::abs(mesh.mesh.SurfaceArea() - 2.0), 1e-8);
}

// Demonstrates planar mixed content conversion: one holed face plus one
// disjoint face preserve aggregate representative area.
TEST(Conversion3dCapabilityTest, PlanarHoledAndMultiFaceBrepBodyConvertsToMeshWithExpectedArea)
{
    std::vector<SCBrepVertex> vertices{// Face A outer square (area 16)
                                     SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{4.0, 0.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{4.0, 4.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{0.0, 4.0, 0.0}),
                                     // Face A hole square (area 4)
                                     SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{3.0, 1.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{3.0, 3.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{1.0, 3.0, 0.0}),
                                     // Face B rectangle (area 2)
                                     SCBrepVertex(SCPoint3d{5.0, 0.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{6.0, 0.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{6.0, 2.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{5.0, 2.0, 0.0})};

    std::vector<SCBrepEdge> edges;
    auto addEdge = [&](std::size_t start, std::size_t end) {
        const SCPoint3d first = vertices[start].Point();
        const SCPoint3d second = vertices[end].Point();
        edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                               SCLine3d::FromOriginAndDirection(first, second - first), SCIntervald{0.0, 1.0})),
                           start,
                           end);
    };

    // Face A outer
    addEdge(0, 1);
    addEdge(1, 2);
    addEdge(2, 3);
    addEdge(3, 0);
    // Face A hole
    addEdge(4, 5);
    addEdge(5, 6);
    addEdge(6, 7);
    addEdge(7, 4);
    // Face B
    addEdge(8, 9);
    addEdge(9, 10);
    addEdge(10, 11);
    addEdge(11, 8);

    const SCBrepLoop outerA({SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop holeA({SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(7, false)});
    const SCBrepLoop outerB({SCBrepCoedge(8, false), SCBrepCoedge(9, false), SCBrepCoedge(10, false), SCBrepCoedge(11, false)});

    const SCPlaneSurface planeSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));

    auto makeTrim = [&](const std::vector<SCPoint2d>& points) {
        return SCCurveOnSurface(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()),
                              SCPolyline2d(points, SCPolylineClosure::Closed));
    };

    const SCCurveOnSurface outerTrimA = makeTrim({
        SCPoint2d{0.0, 0.0},
        SCPoint2d{4.0, 0.0},
        SCPoint2d{4.0, 4.0},
        SCPoint2d{0.0, 4.0},
    });
    const SCCurveOnSurface holeTrimA = makeTrim({
        SCPoint2d{1.0, 1.0},
        SCPoint2d{3.0, 1.0},
        SCPoint2d{3.0, 3.0},
        SCPoint2d{1.0, 3.0},
    });
    const SCCurveOnSurface outerTrimB = makeTrim({
        SCPoint2d{5.0, 0.0},
        SCPoint2d{6.0, 0.0},
        SCPoint2d{6.0, 2.0},
        SCPoint2d{5.0, 2.0},
    });

    const SCBrepFace faceA(
        std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), outerA, {holeA}, outerTrimA, {holeTrimA});
    const SCBrepFace faceB(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), outerB, {}, outerTrimB, {});
    const SCBrepBody body(vertices, edges, {SCBrepShell({faceA, faceB}, false)});
    ASSERT_TRUE(body.IsValid());

    const PolyhedronMeshConversion3d mesh = ConvertToTriangleMesh(body);
    ASSERT_TRUE(mesh.success);
    ASSERT_TRUE(mesh.mesh.IsValid());
    // Expected area: (16 - 4) + 2 = 14.
    ASSERT_LT(std::abs(mesh.mesh.SurfaceArea() - 14.0), 1e-8);
}

// Demonstrates disconnected closed-shell Brep->mesh conversion preserves
// component separation (feature-level connectivity) in addition to geometry.
TEST(Conversion3dCapabilityTest, TwoSeparatedCubeBrepBodyConvertsToMeshWithTwoComponents)
{
    const PolyhedronBody twoCubes = BuildTwoSeparatedUnitCubeBody();
    ASSERT_TRUE(twoCubes.IsValid());
    ASSERT_EQ(twoCubes.FaceCount(), 12);

    const PolyhedronBrepBodyConversion3d brep = ConvertToBrepBody(twoCubes);
    ASSERT_TRUE(brep.success);
    ASSERT_EQ(brep.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(brep.body.IsValid());
    ASSERT_EQ(brep.body.FaceCount(), 12);

    const PolyhedronMeshConversion3d mesh = ConvertToTriangleMesh(brep.body);
    ASSERT_TRUE(mesh.success);
    ASSERT_TRUE(mesh.mesh.IsValid());
    ASSERT_EQ(mesh.mesh.TriangleCount(), 24);
    ASSERT_LT(std::abs(mesh.mesh.SurfaceArea() - 12.0), 1e-8);

    const auto components = ComputeTriangleConnectedComponents(mesh.mesh);
    ASSERT_EQ(components.size(), 2);
    ASSERT_EQ(components[0].size(), 12);
    ASSERT_EQ(components[1].size(), 12);
}

struct ClosedCuboidRepresentativeCase
{
    const char* name;
    PolyhedronBody (*build)();
};

void PrintTo(const ClosedCuboidRepresentativeCase& scenario, std::ostream* stream)
{
    *stream << scenario.name;
}

class ClosedCuboidAllVerticesTest : public ::testing::TestWithParam<ClosedCuboidRepresentativeCase>
{
};

TEST_P(ClosedCuboidAllVerticesTest, PreservesRepresentativeAverageTarget)
{
    const PolyhedronBody body = GetParam().build();
    ASSERT_FALSE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 6);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 6);
    ASSERT_EQ(result.body.ShellCount(), 1);
    ASSERT_TRUE(result.body.ShellAt(0).IsClosed());
    ASSERT_EQ(result.body.VertexCount(), 8);
    ASSERT_EQ(result.body.EdgeCount(), 12);
    AssertClosedCuboidAllVerticesRepresentativeTargets(result.body);
}

INSTANTIATE_TEST_SUITE_P(
    Conversion3d, ClosedCuboidAllVerticesTest,
    ::testing::Values(
        ClosedCuboidRepresentativeCase{"Base", &BuildSupportMismatchNearEqualClosedCuboidAllVerticesBody},
        ClosedCuboidRepresentativeCase{"DuplicateLoop", &BuildSupportMismatchNearEqualClosedCuboidAllVerticesWithDuplicateLoopBody},
        ClosedCuboidRepresentativeCase{"DualDuplicateLoop", &BuildSupportMismatchNearEqualClosedCuboidAllVerticesWithDualDuplicateLoopBody}),
    [](const ::testing::TestParamInfo<ClosedCuboidRepresentativeCase>& info) {
        return std::string(info.param.name);
    });

class ClosedCuboidDualVerticesTest : public ::testing::TestWithParam<ClosedCuboidRepresentativeCase>
{
};

TEST_P(ClosedCuboidDualVerticesTest, PreservesRepresentativeAverageTarget)
{
    const PolyhedronBody body = GetParam().build();
    ASSERT_FALSE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 6);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 6);
    ASSERT_EQ(result.body.ShellCount(), 1);
    ASSERT_TRUE(result.body.ShellAt(0).IsClosed());
    ASSERT_EQ(result.body.VertexCount(), 8);
    ASSERT_EQ(result.body.EdgeCount(), 12);

    const double s = 1e-5;
    const double w = 2.0 * s;
    const double expectedV6X = w + 1.0e-7 / 3.0;

    std::size_t sharedV0Count = 0;
    std::size_t sharedV6Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const SCPoint3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.x) < 1e-12 && std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12)
        {
            ++sharedV0Count;
        }
        if (point.x > 1.5 * s && point.x < 2.5 * s && std::abs(point.y - s) < 1e-12 && std::abs(point.z - s) < 1e-12)
        {
            ++sharedV6Count;
            ASSERT_LT(std::abs(point.x - expectedV6X), 1e-10);
        }
    }

    ASSERT_EQ(sharedV0Count, 1);
    ASSERT_EQ(sharedV6Count, 1);
}

INSTANTIATE_TEST_SUITE_P(
    Conversion3d, ClosedCuboidDualVerticesTest,
    ::testing::Values(
        ClosedCuboidRepresentativeCase{"Base", &BuildSupportMismatchNearEqualClosedCuboidDualVerticesBody},
        ClosedCuboidRepresentativeCase{"DuplicateLoop", &BuildSupportMismatchNearEqualClosedCuboidDualVerticesWithDuplicateLoopBody},
        ClosedCuboidRepresentativeCase{"DualDuplicateLoop", &BuildSupportMismatchNearEqualClosedCuboidDualVerticesWithDualDuplicateLoopBody}),
    [](const ::testing::TestParamInfo<ClosedCuboidRepresentativeCase>& info) {
        return std::string(info.param.name);
    });

class ClosedCuboidTripleVerticesTest : public ::testing::TestWithParam<ClosedCuboidRepresentativeCase>
{
};

TEST_P(ClosedCuboidTripleVerticesTest, PreservesRepresentativeAverageTarget)
{
    const PolyhedronBody body = GetParam().build();
    ASSERT_FALSE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 6);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 6);
    ASSERT_EQ(result.body.ShellCount(), 1);
    ASSERT_TRUE(result.body.ShellAt(0).IsClosed());
    ASSERT_EQ(result.body.VertexCount(), 8);
    ASSERT_EQ(result.body.EdgeCount(), 12);

    const double s = 1e-5;
    const double w = 2.0 * s;
    const double expectedV1X = w;
    const double expectedV6X = w + 1.0e-7 / 3.0;

    std::size_t sharedV0Count = 0;
    std::size_t sharedV1Count = 0;
    std::size_t sharedV6Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const SCPoint3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.x) < 1e-10 && std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12)
        {
            ++sharedV0Count;
        }
        if (point.x > 1.5 * s && point.x < 2.5 * s && std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12)
        {
            ++sharedV1Count;
            ASSERT_LT(std::abs(point.x - expectedV1X), 1e-10);
        }
        if (point.x > 1.5 * s && point.x < 2.5 * s && std::abs(point.y - s) < 1e-12 && std::abs(point.z - s) < 1e-12)
        {
            ++sharedV6Count;
            ASSERT_LT(std::abs(point.x - expectedV6X), 1e-10);
        }
    }

    ASSERT_EQ(sharedV0Count, 1);
    ASSERT_EQ(sharedV1Count, 1);
    ASSERT_EQ(sharedV6Count, 1);
}

INSTANTIATE_TEST_SUITE_P(
    Conversion3d, ClosedCuboidTripleVerticesTest,
    ::testing::Values(
        ClosedCuboidRepresentativeCase{"Base", &BuildSupportMismatchNearEqualClosedCuboidTripleVerticesBody},
        ClosedCuboidRepresentativeCase{"DuplicateLoop", &BuildSupportMismatchNearEqualClosedCuboidTripleVerticesWithDuplicateLoopBody},
        ClosedCuboidRepresentativeCase{"DualDuplicateLoop", &BuildSupportMismatchNearEqualClosedCuboidTripleVerticesWithDualDuplicateLoopBody}),
    [](const ::testing::TestParamInfo<ClosedCuboidRepresentativeCase>& info) {
        return std::string(info.param.name);
    });

\n