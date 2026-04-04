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
    GTEST_SKIP() << "Known 3D gap: general topology-changing aggressive shell repair policy remains open beyond the current deterministic planar subsets: single/multi-face open shells, holed shells, multi-shell closure, mixed closed/open-shell repair, partial mixed-shell repair, standalone coplanar shared-edge boundary-cap closure (including support-mismatch + trim-backfill + holed representative subset), three-shell deterministic mixed behavior, three-shell+trim-backfill interplay, three-shell with eligible multi-face open-sheet, three-shell with eligible holed-shell missing trims, three-shell with eligible multi-face holed+missing-trims, three-shell with eligible multi-face both-trims-missing, three-shell with eligible support-plane mismatch composition, support-mismatch eligible + ineligible multiface coexistence, support-mismatch+trim-backfill+ineligible-multiface composition, support-mismatch holed-eligible + ineligible-multiface composition, support-mismatch eligible-multiface-missing-trims + ineligible-multiface composition, and four-shell two-eligible-plus-one-ineligible deterministic behavior. Non-horizontal trim backfill now covers z=0 (horizontal), y=0 (+y normal), x=0 (+x normal), and oblique planar orientations.";
}

TEST(Healing3dGapTest, MultiStepMeshBodyJointHealingRemainsOpen)
{
    const TriangleMesh mesh({}, {});
    const MeshHealing3d meshHealing = Heal(mesh);
    (void)meshHealing;
    GTEST_SKIP() << "Known 3D gap: multi-step mesh/body joint healing workflow is still open.";
}
