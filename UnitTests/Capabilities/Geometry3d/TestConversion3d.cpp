#include <cmath>
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
using Geometry::ConvertToBrepBody;
using Geometry::SCCurveOnSurface;
using Geometry::SCIntervald;
using Geometry::SCLine3d;
using Geometry::SCLineCurve3d;
using Geometry::SCPlane;
using Geometry::SCPlaneSurface;
using Geometry::SCPoint2d;
using Geometry::SCPoint3d;
using Geometry::PolyhedronBody;
using Geometry::PolyhedronBrepBodyConversion3d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;
using Geometry::ISCSurface;
using Geometry::SCVector3d;

using namespace Geometry::Test::ConversionFixtures;

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
