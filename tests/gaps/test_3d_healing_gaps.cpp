#include <gtest/gtest.h>

#include "sdk/Geometry.h"

using geometry::sdk::BrepBody;
using geometry::sdk::Heal;
using geometry::sdk::MeshHealing3d;
using geometry::sdk::TriangleMesh;

TEST(Healing3dGapTest, AggressiveShellRepairPolicyRemainsOpen)
{
    const BrepBody body({});
    const auto healing = Heal(body);
    (void)healing;
    GTEST_SKIP() << "Known 3D gap: general topology-changing aggressive shell repair policy beyond single-face planar closure subset is not implemented.";
}

TEST(Healing3dGapTest, MultiStepMeshBodyJointHealingRemainsOpen)
{
    const TriangleMesh mesh({}, {});
    const MeshHealing3d meshHealing = Heal(mesh);
    (void)meshHealing;
    GTEST_SKIP() << "Known 3D gap: multi-step mesh/body joint healing workflow is still open.";
}
