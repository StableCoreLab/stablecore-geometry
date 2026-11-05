#include <gtest/gtest.h>

#include "Geometry.h"

using namespace Geometry;

TEST(BrepResultTolerance3dTest, InvalidToleranceIsRejectedForFailedAndEmptyResults)
{
    SCGeometryTolerance3d invalidTolerance{};
    invalidTolerance.parameterEpsilon = -1.0;

    EXPECT_FALSE(BrepFaceEdit3d{}.IsValid(invalidTolerance));
    EXPECT_FALSE(BrepShellEdit3d{}.IsValid(invalidTolerance));
    EXPECT_FALSE(BrepBodyEdit3d{}.IsValid(invalidTolerance));
    EXPECT_FALSE(PolyhedronBrepBodyConversion3d{}.IsValid(invalidTolerance));
    EXPECT_FALSE(SCSectionBrepFaceRebuildResult3d{}.IsValid(invalidTolerance));
    EXPECT_FALSE(SCSectionBrepBodyRebuildResult3d{}.IsValid(invalidTolerance));
    EXPECT_FALSE(SCSectionBrepBodySetRebuildResult3d{}.IsValid(invalidTolerance));

    SCSectionBrepFaceRebuildResult3d emptyFaces{};
    emptyFaces.success = true;
    EXPECT_FALSE(emptyFaces.IsValid(invalidTolerance));
    SCSectionBrepBodyRebuildResult3d emptyBody{};
    emptyBody.success = true;
    EXPECT_FALSE(emptyBody.IsValid(invalidTolerance));
    SCSectionBrepBodySetRebuildResult3d emptyBodies{};
    emptyBodies.success = true;
    EXPECT_FALSE(emptyBodies.IsValid(invalidTolerance));

    constexpr double invalidDistance = -1.0;
    EXPECT_FALSE(BrepFaceConversion3d{}.IsValid(invalidDistance));
    EXPECT_FALSE(BrepBodyConversion3d{}.IsValid(invalidDistance));
    EXPECT_FALSE(SCSectionFaceRebuildResult3d{}.IsValid(invalidDistance));
    EXPECT_FALSE(SCSectionBodyRebuildResult3d{}.IsValid(invalidDistance));
    EXPECT_FALSE(SCSectionMeshConversionResult3d{}.IsValid(invalidDistance));
    EXPECT_FALSE(SCSectionMeshSetConversionResult3d{}.IsValid(invalidDistance));
    EXPECT_FALSE(TriangleMeshRepair3d{}.IsValid(invalidDistance));
}
