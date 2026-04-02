#include <gtest/gtest.h>

#include "sdk/Geometry.h"

using geometry::sdk::BrepBody;
using geometry::sdk::BrepEdge;
using geometry::sdk::BrepFace;
using geometry::sdk::BrepLoop;
using geometry::sdk::BrepShell;
using geometry::sdk::BrepVertex;
using geometry::sdk::Point3d;

TEST(Brep3dGapTest, CoedgeLoopOwnershipEditingWorkflowRemainsOpen)
{
    const BrepVertex v0(Point3d{0.0, 0.0, 0.0});
    const BrepVertex v1(Point3d{1.0, 0.0, 0.0});
    const BrepEdge edge;
    (void)v0;
    (void)v1;
    (void)edge;
    GTEST_SKIP() << "Known 3D gap: Brep coedge-loop ownership/shell-consistency editing workflow beyond minimal loop->face->shell->body replacement subset is still open.";
}

TEST(Brep3dGapTest, NonPlanarTrimmedFaceTopologyRepairRemainsOpen)
{
    const BrepBody body({});
    (void)body;
    GTEST_SKIP() << "Known 3D gap: non-planar trimmed face topology repair and robust shell stitching are still open.";
}
