# 下一步任务提示

你是一个 AI 编程助手，正在恢复之前中断的工作。请直接读取本文件内容，定位当前任务状态，然后继续写代码，不要重新分析、不要询问确认、不要等待输入。

---

## 一、仓库位置

工作区：`D:\code\stablecore-geometry`
语言：C++20；构建：MSBuild + CMake；测试：GoogleTest

**不要编译、不要跑构建。只做源码级实现、静态检查、测试代码补充、文档更新。**

---

## 二、当前状态快照（2026-04-02）

### 2D：已全部稳定，无需继续

所有 2D gap 已转正为 capability tests，`tests/gaps/` 中 2D 条目已清空。
覆盖范围：Boolean / Offset / Topology / SearchPoly / Relation，包括 `OffsetToMultiPolygon` narrow bridge split。

### 3D：第一阶段已完成，进入第二阶段

**已落地的 capability tests（截至本次会话）：**

| 文件 | 测试 | 验证内容 |
| --- | --- | --- |
| tests/support/Fixtures3d.h |  | 共享 BuildUnitCubeBody() 单位立方体 |
| tests/capabilities/test_3d_section.cpp | Section3dCapabilityTest | Section+Topology+Components 倾斜截面 + non-axis-aligned contour count |
| tests/capabilities/test_3d_brep.cpp | Brep3dCapabilityTest | RebuildSectionBrepBody 单面 BrepBody + RebuildSectionBrepBodies 双组件重建 + coedge-loop 最小编辑链路 |
| tests/capabilities/test_3d_healing.cpp | Healing3dCapabilityTest | Heal(PolyhedronBody) 幂等 6 faces + Heal(BrepBody) trim 回填（含 holed face） + Aggressive 平面 open-sheet/holed-shell 闭壳子策略 + Aggressive+trim-backfill 组合子场景 |
| tests/capabilities/test_3d_conversion.cpp | Conversion3dCapabilityTest | ConvertToTriangleMesh 12 triangles area6.0 + ConvertToBrepBody FaceCount=6 + affine-skew 子类 + support-plane refit + mild outer/hole loop-projection + collinear-leading-loop + duplicate outer/hole loop-normalization + 组合病理输入可叠加修复 + planar holed/multi-face/holed+multi-face Brep->mesh 面积保持 |

**仍为 gap 的 3D 场景（tests/gaps/ 中已用 GTEST_SKIP 标记）：**

| Gap 测试名 | 剩余能力点 |
| --- | --- |
| Section3dGapTest::NonPlanarDominantSectionGraphRemainsOpen | 非平面主导下的歧义 contour stitching |
| Section3dGapTest::FaceMergeSemanticsAfterSectionRemainsOpen | coplanar fragment merge 语义 |
| Brep3dGapTest::CoedgeLoopOwnershipEditingWorkflowRemainsOpen | coedge-loop ownership/关联拓扑级编辑语义 |
| Brep3dGapTest::NonPlanarTrimmedFaceTopologyRepairRemainsOpen | non-planar trimmed face shell repair |
| Healing3dGapTest::AggressiveShellRepairPolicyRemainsOpen | 激进 shell 修复策略（超出 planar open-sheet/holed-shell closure 子策略） |
| Healing3dGapTest::MultiStepMeshBodyJointHealingRemainsOpen | mesh/body 联合多阶段修复 |
| Conversion3dGapTest::HighFidelityBrepToMeshFeaturePreservationRemainsOpen | 高保真 Brep->mesh 特征保持（超出 planar holed/multi-face/holed+multi-face area-preserving 子集） |
| Conversion3dGapTest::GeneralNonPlanarPolyhedronToBrepRepairRemainsOpen | 通用非平面 polyhedron->Brep 转换（超出 affine-skew + support-plane-refit + mild outer/hole loop-projection + collinear-leading-loop + duplicate outer/hole loop-normalization 子集） |

---

## 三、下一次任务：立即开始，按优先级执行

### P1 已完成（本次）

- P1-A：`Heal(BrepBody)` 缺失 trim 回填 capability 已落地
- P1-B：`ConvertToBrepBody(PolyhedronBody)` capability 已落地
- P1-C：多组件 section -> `RebuildSectionBrepBodies` capability 已落地

### P2 已完成（本次）

- P2-A：non-axis-aligned multi-face section contour count capability 已落地
- P2-B：coedge-loop editing 最小 non-trivial capability 已落地

### P3 已完成（本次）

- P3-A：`ConvertToBrepBody` 已转正 affine-skew 非轴对齐子类
- P3-B：`Heal(BrepBody)` 已转正 holed-face 的 outer/hole trims 同步回填子场景
- P3-C：`ConvertToBrepBody` 已转正 support-plane mismatch 的可解释 repair 子场景（support-plane refit）
- P3-D：`Heal(..., policy=Aggressive)` 已转正 open planar single/multi-face sheet 与 holed shell 的确定性闭壳子策略
- P3-E：`ConvertToBrepBody` 已转正 mild non-planar loop 的 refit-plane 投影修复子场景

### P4-A：鲁棒 non-planar repair（优先）
- 针对 `GeneralNonPlanarPolyhedronToBrepRepairRemainsOpen`，尝试引入最小 non-planar 失配样例与可解释 repair 策略
- 已新增 tiny-scale non-planar loop 的 scale-aware 法向回退 capability 子样例；下一步聚焦更一般 non-planar 拓扑失配场景
- 已扩展 tiny-scale holed non-planar face capability 子样例；下一步聚焦跨 face 拓扑关联的 non-planar 修复策略
- 已扩展 tiny-scale non-planar multi-face capability 子样例；下一步聚焦跨 face 拓扑耦合（共享边/邻接约束）下的修复策略
- 已扩展 tiny-scale non-planar mixed-content（holed+plain）multi-face capability 子样例；下一步聚焦共享边一致性与全局约束驱动的修复策略
- 已扩展 tiny-scale non-planar shared-edge 邻接面 capability 子样例；下一步聚焦共享边一致性约束真正参与修复决策
- 已扩展 tiny-scale non-planar shared-edge 邻接链 capability 子样例；下一步聚焦共享边一致性约束驱动的跨面联合修复策略
- 已扩展 tiny-scale non-planar shared-edge 邻接链 mixed-content capability 子样例；下一步聚焦共享边一致性约束驱动的跨面联合修复策略
- 已扩展 shared-edge 邻接链下 duplicate-loop-normalization + refit/projection 组合子样例；下一步聚焦共享边一致性约束真正参与修复决策
- 已扩展 shared-edge 邻接链 mixed-content 下 duplicate hole-loop-normalization + refit/projection 组合子样例；下一步聚焦共享边一致性约束真正参与修复决策
- 已扩展 shared-edge 邻接链 mixed-content 下 collinear-leading + duplicate/hole normalization 组合子样例；下一步聚焦共享边一致性约束真正参与修复决策

### P4-B：aggressive shell policy 分层落地（已完成最小子集）
- 已覆盖 single-face / multi-face / holed / multi-shell open-shell 的最小 deterministic closure 子策略
- 已覆盖 mixed closed/open-shell 输入下“保持闭壳稳定 + 闭合开壳”的最小 deterministic 子策略
- 已覆盖 mixed open-shell 输入下“可恢复壳闭合 + 不可恢复壳保持原状”的最小 deterministic 部分修复子策略
- 已覆盖三壳 mixed 输入下“closed 保持 + eligible 闭壳 + ineligible 保持”的 deterministic 子策略
- 已覆盖三壳 mixed 输入下“eligible 先 trim-backfill 再闭壳 + ineligible 保持”的 deterministic 组合子策略
- 已覆盖三壳 mixed 输入下 eligible multi-face open-sheet 的 deterministic 闭壳子策略
- 已覆盖三壳 mixed 输入下 eligible holed shell（缺失 outer/hole trims）回填后 deterministic 闭壳子策略
- 已覆盖三壳 mixed 输入下 eligible multi-face holed shell（缺失 trims）回填后 deterministic 闭壳子策略
- 下一步转向更一般 topology-changing aggressive policy（保留为 gap）

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
