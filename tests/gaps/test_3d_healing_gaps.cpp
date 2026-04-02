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
    GTEST_SKIP() << "Known 3D gap: general topology-changing aggressive shell repair policy beyond recoverable planar open-shell subsets (single/multi-face, holed, multi-shell, mixed closed/open-shell, partial mixed-shell repair, shared-edge open-sheet, three-shell deterministic mixed behavior, three-shell+trim-backfill interplay, three-shell with eligible multi-face open-sheet, three-shell with eligible holed-shell missing trims, three-shell with eligible multi-face holed+missing-trims, three-shell with eligible multi-face both-trims-missing, three-shell with eligible support-plane mismatch composition, support-mismatch eligible + ineligible multiface coexistence, support-mismatch+trim-backfill+ineligible-multiface composition, support-mismatch holed-eligible + ineligible-multiface composition, support-mismatch eligible-multiface-missing-trims + ineligible-multiface composition) is not implemented.";
}

TEST(Healing3dGapTest, MultiStepMeshBodyJointHealingRemainsOpen)
{
    const TriangleMesh mesh({}, {});
    const MeshHealing3d meshHealing = Heal(mesh);
    (void)meshHealing;
    GTEST_SKIP() << "Known 3D gap: multi-step mesh/body joint healing workflow is still open.";
}
