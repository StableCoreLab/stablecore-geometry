#include <gtest/gtest.h>

#include "sdk/GeometryBodyBoolean.h"
#include "sdk/PolyhedronBody.h"

using geometry::sdk::BodyBooleanIssue3d;
using geometry::sdk::IntersectBodies;
using geometry::sdk::PolyhedronBody;

TEST(BodyBoolean3dGapTest, DelphiGradeBodyAndShellBooleanRemainOpen)
{
    const PolyhedronBody first;
    const PolyhedronBody second;
    const auto result = IntersectBodies(first, second);
    (void)result;

    GTEST_SKIP() << "Known 3D gap: Delphi-grade body/shell boolean workflow remains open. "
                    "Current fast-track now covers only deterministic identical/disjoint closed-body "
                    "subsets plus invalid-input contract; robust overlap boolean, shell-policy "
                    "variants, and topology-preserving healing integration are still pending.";
}
