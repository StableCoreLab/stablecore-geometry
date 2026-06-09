#include <gtest/gtest.h>

#include "Brep/BodyBoolean.h"
#include "Brep/PolyhedronBody.h"

using Geometry::BodyBooleanIssue3d;
using Geometry::IntersectBodies;
using Geometry::PolyhedronBody;

TEST(BodyBoolean3dGapTest, BodyAndShellBooleanRemainOpen)
{
    const PolyhedronBody first;
    const PolyhedronBody second;
    const auto result = IntersectBodies(first, second);
    (void)result;

    GTEST_SKIP() << "已知 3D 差距：body/shell boolean "
                    "工作流仍未闭合。当前已覆盖无效输入契约、确定性的相同/"
                    "相离闭体子集、相同差集为空、轴对齐包含体的相交/"
                    "并集子集、轴对齐单盒重叠子集、face-touching "
                    "轴对齐盒并集且结果仍为单个闭体、face-touching "
                    "外差保持原闭体、轴对齐包含差集为空、轴对齐边/"
                    "点接触的有序多体并集、轴对齐边/"
                    "点接触外差、face-touching L "
                    "形非盒并集的显式不支持契约、旋转盒正体积相交的显式不支持契"
                    "约，以及确定性的相离有序多体并集 "
                    "/ 相离 / "
                    "轴对齐面/边/"
                    "点接触空相交子集。更一般的非盒重叠、非轴对齐或更丰富的接触"
                    "相交语义、shell-policy "
                    "变体和拓扑保持的 healing 集成仍未完成。";
}

TEST(BodyBoolean3dGapTest, ContainedShellPolicyOptionStillHasNoEffectAndStaysGap)
{
    GTEST_SKIP() << "Known 3D gap: shell-policy options are still not part of "
                    "the public boolean contract. "
                    "Representative unresolved scenario: a contained or "
                    "face-touching axis-aligned pair "
                    "evaluated with operateOnShells=true still follows the "
                    "closed-body fast path, "
                    "so the option does not yet advertise or prove any "
                    "deterministic shell-level semantic "
                    "effect. "
                    "Expected future capability: either prove the option "
                    "changes result selection/diagnostics "
                    "deterministically or keep it explicitly unsupported.";
}
