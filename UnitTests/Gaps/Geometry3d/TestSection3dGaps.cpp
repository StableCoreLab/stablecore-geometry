#include <gtest/gtest.h>

TEST(DISABLED_Section3dGapTest, NonPlanarDominantSectionGraphRemainsOpen)
{
    GTEST_SKIP() << "Known gap: non-planar dominant section graph reconstruction.";
}

TEST(DISABLED_Section3dGapTest, FaceMergeSemanticsAfterSectionRemainsOpen)
{
    GTEST_SKIP() << "Known gap: ambiguous coplanar face-merge semantics.";
}

TEST(DISABLED_Section3dGapTest, MixedMergedAreaWithInteriorHoleAndDualBoundaryAttachedOpenContoursStillNeedArbitration)
{
    GTEST_SKIP() << "Known gap: merged area-with-hole and dual attached contour arbitration.";
}
