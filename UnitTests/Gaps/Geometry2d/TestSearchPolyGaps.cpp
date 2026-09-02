#include <gtest/gtest.h>

TEST(DISABLED_SearchPolyGapTest, AmbiguousBranchScoringRemainsOpen)
{
    GTEST_SKIP() << "Known gap: full ambiguous-branch recovery and smart-search alignment.";
}

TEST(DISABLED_SearchPolyGapTest, SearchPolygonsReportsAmbiguousRecoveryWhenTwoCandidatesTieAfterSyntheticPenaltyNormalization)
{
    GTEST_SKIP() << "Known gap: per-candidate recovery explanation for tied synthetic penalties.";
}
