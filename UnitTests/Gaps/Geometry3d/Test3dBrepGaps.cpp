#include <gtest/gtest.h>

TEST(DISABLED_Brep3dGapTest, CoedgeLoopOwnershipEditingWorkflowRemainsOpen)
{
    GTEST_SKIP() << "Known gap: coedge-loop ownership and shell-consistent editing.";
}

TEST(DISABLED_Brep3dGapTest, NonPlanarTrimmedFaceTopologyRepairRemainsOpen)
{
    GTEST_SKIP() << "Known gap: non-planar trim repair and non-manifold shell stitching.";
}
