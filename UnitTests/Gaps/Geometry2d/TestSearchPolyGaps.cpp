#include <gtest/gtest.h>

#include "Core/SearchPoly.h"

TEST(SearchPolyGapTest, AmbiguousBranchScoringRemainsOpen)
{
    GTEST_SKIP() << "已知 2D 差距：SearchPoly 已具备稳定发布 API "
                    "入口、候选排序、分支评分、候选级 "
                    "fake-edge 诊断、边级 synthetic 解释、top-candidate / runner-up / "
                    "ambiguous-top "
                    "源摘要、ambiguous-top "
                    "摘要说明、clean-winner-vs-synthetic-runner-up "
                    "因果说明、result/diagnostics 一致性和 auto-flag gating，但更丰富的 "
                    "ambiguous "
                    "recovery、fake-edge 解释和完整 smart-search 对齐仍未闭合。";
}

TEST(SearchPolyGapTest, SearchPolygonsReportsAmbiguousRecoveryWhenTwoCandidatesTieAfterSyntheticPenaltyNormalization)
{
    GTEST_SKIP() << "已知 2D 差距：在 synthetic penalty 归一化后，两个候选分数打平时的 "
                    "ambiguous recovery "
                    "仍未形成完整契约。代表性场景是两个 top-ranked "
                    "候选在归一化分支分数上打平，且都需要 "
                    "synthetic edges，但它们的 dominant synthetic source 不同（例如 "
                    "SingleGapClose 与 "
                    "MixedBridge / BranchCleanup）。当前稳定摘要已经可以报告 "
                    "ambiguous-top 数量和混合 "
                    "synthetic-source "
                    "摘要，但还不能直接指出每个打平候选各自承担了哪一种竞争性恢复画像。"
                    "期望的后续能力是返回稳定的"
                    " ambiguous-recovery 解释，直接命名打平候选并可读地展示它们的竞争性 "
                    "synthetic-source 画像。";
}
