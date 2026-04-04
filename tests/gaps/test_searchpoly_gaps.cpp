#include <gtest/gtest.h>

#include "sdk/GeometrySearchPoly.h"

TEST(SearchPolyGapTest, DelphiGradeAmbiguousBranchScoringRemainsOpen)
{
    GTEST_SKIP() << "Known 2D gap: GeometrySearchPoly now exposes stable SDK entry points, "
                    "candidate ranking, branch scoring, and candidate-level fake-edge diagnostics, "
                    "but Delphi-grade ambiguous recovery, richer fake-edge explanation, and full "
                    "smart-search parity remain open.";
}
