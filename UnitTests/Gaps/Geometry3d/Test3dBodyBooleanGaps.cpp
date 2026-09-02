#include <gtest/gtest.h>

TEST(DISABLED_BodyBoolean3dGapTest, BodyAndShellBooleanRemainOpen)
{
    GTEST_SKIP() << "Known gap: general non-box, non-axis-aligned body/shell Boolean operations.";
}

TEST(DISABLED_BodyBoolean3dGapTest, ContainedShellPolicyOptionStillHasNoEffectAndStaysGap)
{
    GTEST_SKIP() << "Known gap: deterministic public semantics for operateOnShells.";
}
