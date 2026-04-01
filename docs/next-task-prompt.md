# 下一步任务提示

你是一个 AI 编程助手，正在恢复之前中断的工作。请直接读取本文件内容，定位当前任务状态，然后继续写代码，不要重新分析、不要询问确认、不要等待输入。

---

## 一、仓库位置

工作区：`D:\code\stablecore-geometry`
语言：C++20；构建：MSBuild + CMake；测试：GoogleTest

**不要编译、不要跑构建。只做源码级实现、静态检查、测试代码补充、文档更新。**

---

## 二、当前状态快照（2026-04-01）

### 2D：已全部稳定，无需继续

所有 2D gap 已转正为 capability tests，`tests/gaps/` 中 2D 条目已清空。
覆盖范围：Boolean / Offset / Topology / SearchPoly / Relation，包括 `OffsetToMultiPolygon` narrow bridge split。

### 3D：第一阶段已完成，进入第二阶段

**已落地的 capability tests（上次会话写完，用户将手工编译）：**

| 文件 | 测试 | 验证内容 |
| --- | --- | --- |
| tests/support/Fixtures3d.h |  | 共享 BuildUnitCubeBody() 单位立方体 |
| tests/capabilities/test_3d_section.cpp | Section3dCapabilityTest | Section+Topology+Components 倾斜截面 |
| tests/capabilities/test_3d_brep.cpp | Brep3dCapabilityTest | RebuildSectionBrepBody 单面 BrepBody |
| tests/capabilities/test_3d_healing.cpp | Healing3dCapabilityTest | Heal(PolyhedronBody) 幂等 6 faces |
| tests/capabilities/test_3d_conversion.cpp | Conversion3dCapabilityTest | ConvertToTriangleMesh 12 triangles area6.0 |

**仍为 gap 的 3D 场景（tests/gaps/ 中已用 GTEST_SKIP 标记）：**

| Gap 测试名 | 剩余能力点 |
| --- | --- |
| Section3dGapTest::NonPlanarDominantSectionGraphRemainsOpen | 非平面主导下的歧义 contour stitching |
| Section3dGapTest::FaceMergeSemanticsAfterSectionRemainsOpen | coplanar fragment merge 语义 |
| Brep3dGapTest::CoedgeLoopOwnershipEditingWorkflowRemainsOpen | coedge-loop 编辑 API |
| Brep3dGapTest::NonPlanarTrimmedFaceTopologyRepairRemainsOpen | non-planar trimmed face shell repair |
| Healing3dGapTest::AggressiveShellRepairPolicyRemainsOpen | 激进 shell 修复策略 |
| Healing3dGapTest::MultiStepMeshBodyJointHealingRemainsOpen | mesh/body 联合多阶段修复 |
| Conversion3dGapTest::HighFidelityBrepToMeshFeaturePreservationRemainsOpen | 高保真 Brep->mesh 特征保持 |
| Conversion3dGapTest::GeneralNonPlanarPolyhedronToBrepRepairRemainsOpen | 通用非平面 polyhedron->Brep 转换 |

---

## 三、本次任务：立即开始，按优先级执行

### P1-A：扩展 Heal(BrepBody) 并转正为 capability test

在 `tests/capabilities/test_3d_healing.cpp` 中新增测试：
- 构造一个 plane-surface + line-edge 主导但缺失 trim 的 BrepBody
- 调用 `Heal(BrepBody)` → 验证 `success == true`，`body.IsValid()`，trim 已补齐
- 相关实现在 `src/sdk/GeometryHealing.cpp` 的 `Heal(BrepBody, ...)`

### P1-B：ConvertToBrepBody(PolyhedronBody) 转正为 capability test

在 `tests/capabilities/test_3d_conversion.cpp` 中新增测试：
- 调用 `ConvertToBrepBody(cubeBody)` → 验证 `success == true`，`body.IsValid()`，`FaceCount() == 6`
- 相关实现在 `src/sdk/GeometryBrepConversion.cpp`

### P1-C：多组件 section → RebuildSectionBrepBodies

在 `tests/capabilities/test_3d_brep.cpp` 中新增测试：
- 构造两个分离的立方体（或一个立方体 + 一个足够远的独立面），截面产生 2 个独立 area component
- 调用 `RebuildSectionBrepBodies(section)` → 验证返回 bodies 数量 == 2
- 如果独立 fixture 难以构造，也可以先只验证 `ClassifySectionContent` 对 Mixed/Multi 场景正确分类

### P2（若 P1 完成有余力）
- 扩展 non-planar section graph：verifying contour count for a non-axis-aligned multi-face body
- coedge-loop editing：若 API 已存在，转正一个最小 non-trivial coedge 编辑用例

---

## 四、主要相关文件

源码：
- `src/sdk/GeometryHealing.cpp` / `include/sdk/GeometryHealing.h`
- `src/sdk/GeometryBrepConversion.cpp` / `include/sdk/GeometryBrepConversion.h`
- `src/sdk/GeometrySection.cpp` / `include/sdk/GeometrySection.h`

测试：
- `tests/capabilities/test_3d_healing.cpp`
- `tests/capabilities/test_3d_conversion.cpp`
- `tests/capabilities/test_3d_brep.cpp`
- `tests/support/Fixtures3d.h`
- `tests/gaps/test_3d_healing_gaps.cpp`
- `tests/gaps/test_3d_conversion_gaps.cpp`
- `tests/gaps/test_3d_brep_gaps.cpp`

---

## 五、修改约束

- 不要回退已有改动，不要删除用户已有变化
- 保持现有 API 风格（自由函数 + 结果类型）
- 只有能稳定通过的测试才放入 tests/capabilities/
- 完成后更新 docs/test-capability-coverage.md 与 docs/design-doc-sync-tracker.md
- 完成后更新本文件（next-task-prompt.md）与 docs/session-handoff.md
- 不要编译，不要跑构建

完成后输出：
1. 这次实现了哪些能力
2. 还剩哪些没有补
3. 建议提交信息 Msg
