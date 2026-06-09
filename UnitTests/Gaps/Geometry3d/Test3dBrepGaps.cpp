#include <gtest/gtest.h>

#include "Geometry.h"

using Geometry::SCBrepBody;
using Geometry::SCBrepEdge;
using Geometry::SCBrepFace;
using Geometry::SCBrepLoop;
using Geometry::SCBrepShell;
using Geometry::SCBrepVertex;
using Geometry::SCPoint3d;

TEST(Brep3dGapTest, CoedgeLoopOwnershipEditingWorkflowRemainsOpen)
{
    const SCBrepVertex v0(SCPoint3d{0.0, 0.0, 0.0});
    const SCBrepVertex v1(SCPoint3d{1.0, 0.0, 0.0});
    const SCBrepEdge edge;
    (void)v0;
    (void)v1;
    (void)edge;
    GTEST_SKIP() << "Known 3D gap: Brep coedge-loop "
                    "ownership/shell-consistency editing workflow beyond "
                    "currently covered replacement subsets (single-face "
                    "rebuilt body + multi-face "
                    "closed-shell no-op replacement) is still open.";
}

TEST(Brep3dGapTest, NonPlanarTrimmedFaceTopologyRepairRemainsOpen)
{
    const SCBrepBody body({});
    (void)body;
    GTEST_SKIP() << "Known 3D gap: non-planar trimmed face topology repair and "
                    "robust shell stitching are still "
                    "open. "
                    "Covered subsets: single planar face missing-trim "
                    "conservative backfill (horizontal z=0 "
                    "plane), "
                    "single planar face missing-trim backfill with "
                    "non-horizontal (y=0 vertical plane, +y "
                    "normal), "
                    "single planar face missing-trim backfill with x-normal "
                    "(x=0 vertical plane, +x normal), "
                    "single planar face missing-trim backfill on oblique plane "
                    "(x+y+z=0, normal (1,1,1)). "
                    "Remaining open: curved-ISCSurface trim recompute and "
                    "non-manifold shell stitching.";
}

