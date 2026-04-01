#include <cassert>

#include <gtest/gtest.h>

#include "sdk/Geometry.h"
#include "support/Fixtures3d.h"

using geometry::sdk::Heal;
using geometry::sdk::HealingIssue3d;
using geometry::sdk::PolyhedronBody;
using geometry::sdk::PolyhedronHealing3d;

// Demonstrates that the conservative healing pass preserves an already-valid
// PolyhedronBody without altering face count or validity.
// The "aggressive shell repair policy" (topology-changing repair) and
// "multi-step mesh/body joint healing" remain open gaps.
TEST(Healing3dCapabilityTest, UnitCubePolyhedronBodyHealingPreservesAllSixFaces)
{
    const PolyhedronBody cubeBody = geometry::test::BuildUnitCubeBody();
    assert(cubeBody.IsValid());
    assert(cubeBody.FaceCount() == 6);

    const PolyhedronHealing3d healed = Heal(cubeBody);
    assert(healed.success);
    assert(healed.issue == HealingIssue3d::None);
    assert(healed.body.IsValid());
    assert(healed.body.FaceCount() == 6);
}
