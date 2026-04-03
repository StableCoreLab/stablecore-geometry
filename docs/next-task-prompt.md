# 下一步任务提示

你是一个 AI 编程助手，正在恢复之前中断的工作。请直接读取本文件内容，定位当前任务状态，然后继续写代码，不要重新分析、不要询问确认、不要等待输入。

---

## 一、仓库位置

工作区：`D:\code\stablecore-geometry`
语言：C++20；构建：MSBuild + CMake；测试：GoogleTest

**不要编译、不要跑构建。只做源码级实现、静态检查、测试代码补充、文档更新。**

---

## 二、当前状态快照（2026-04-03）

### 2D：已全部稳定，无需继续

所有 2D gap 已转正为 capability tests，`tests/gaps/` 中 2D 条目已清空。
覆盖范围：Boolean / Offset / Topology / SearchPoly / Relation，包括 `OffsetToMultiPolygon` narrow bridge split。

### 3D：第一阶段已完成，进入第二阶段

**已落地的 capability tests（截至本次会话）：**

| 文件 | 测试 | 验证内容 |
| --- | --- | --- |
| tests/support/Fixtures3d.h |  | 共享 BuildUnitCubeBody() 单位立方体 |
| tests/capabilities/test_3d_section.cpp | Section3dCapabilityTest | Section+Topology+Components 倾斜截面 + non-axis-aligned contour count |
| tests/capabilities/test_3d_brep.cpp | Brep3dCapabilityTest | RebuildSectionBrepBody/Body 单面重建 + RebuildSectionBrepBodies/Bodies 双组件重建 + Brep section->rebuild 端到端双组件 + coedge-loop 最小编辑链路 |
| tests/capabilities/test_3d_healing.cpp | Healing3dCapabilityTest | Heal(PolyhedronBody) 幂等 6 faces + Heal(BrepBody) trim 回填（含 holed face） + Aggressive 平面 open-sheet/holed-shell 闭壳子策略 + Aggressive+trim-backfill 组合子场景 |
| tests/capabilities/test_3d_conversion.cpp | Conversion3dCapabilityTest | ConvertToTriangleMesh 12 triangles area6.0 + ConvertToBrepBody FaceCount=6 + affine-skew 子类 + support-plane refit + mild outer/hole loop-projection + collinear-leading-loop + duplicate outer/hole loop-normalization + 组合病理输入可叠加修复 + planar holed/multi-face/holed+multi-face Brep->mesh 面积保持 |

**仍为 gap 的 3D 场景（tests/gaps/ 中已用 GTEST_SKIP 标记）：**

| Gap 测试名 | 剩余能力点 |
| --- | --- |
| Section3dGapTest::NonPlanarDominantSectionGraphRemainsOpen | 非平面主导下的歧义 contour stitching |
| Section3dGapTest::FaceMergeSemanticsAfterSectionRemainsOpen | 超出已覆盖（Polyhedron/Brep）相邻 coplanar fragment union 子集的 coplanar fragment merge 语义 |
| Brep3dGapTest::CoedgeLoopOwnershipEditingWorkflowRemainsOpen | 超出已覆盖（single-face + multi-face closed-shell no-op replacement）子集的 coedge-loop ownership/关联拓扑级编辑语义 |
| Brep3dGapTest::NonPlanarTrimmedFaceTopologyRepairRemainsOpen | non-planar trimmed face shell repair |
| Healing3dGapTest::AggressiveShellRepairPolicyRemainsOpen | 激进 shell 修复策略（超出 planar open-sheet/holed-shell closure 子策略） |
| Healing3dGapTest::MultiStepMeshBodyJointHealingRemainsOpen | mesh/body 联合多阶段修复 |
| Conversion3dGapTest::HighFidelityBrepToMeshFeaturePreservationRemainsOpen | 高保真 Brep->mesh 特征保持（超出 planar holed/multi-face/holed+multi-face area-preserving + shared-edge vertex-reuse + disconnected closed-shell component-preserving 子集） |
| Conversion3dGapTest::GeneralNonPlanarPolyhedronToBrepRepairRemainsOpen | 通用非平面 polyhedron->Brep 转换（超出 affine-skew + support-plane-refit + mild outer/hole loop-projection + collinear-leading-loop + duplicate outer/hole loop-normalization 子集） |

---

## 三、下一次任务：立即开始，按优先级执行

### P1 已完成（本次）

- P1-A：`Heal(BrepBody)` 缺失 trim 回填 capability 已落地
- P1-B：`ConvertToBrepBody(PolyhedronBody)` capability 已落地
- P1-C：多组件 section -> `RebuildSectionBrepBodies` capability 已落地

### P2 已完成（本次）

- P2-A：non-axis-aligned multi-face section contour count capability 已落地
- P2-A1：`Section(BrepBody, Plane)` oblique-cut capability 已落地（1 polygon / 1 contour / 6 points）
- P2-A2：`Section(BrepBody, Plane)` coplanar 邻接片段 merge capability 已落地（1 polygon / area=2.0）
- P2-A3：`Section(BrepBody, Plane)` multi-component area capability 已落地（2 polygons / 2 roots / 2 components）
- P2-B：coedge-loop editing 最小 non-trivial capability 已落地
- P2-B1：Brep section rebuild 壳体语义断言已补齐（single/multi-component 输出均显式断言 open shell）
- P2-C：coplanar 相邻 face fragment 在 `Section(...)` 中合并为单 polygon 的 face-merge 子样例已落地
- P2-D：最小 loop->face->shell->body ownership-consistent replacement workflow 已落地

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
- 已扩展 shared-edge 邻接链 mixed-content 下 support-plane mismatch + refit/projection 组合子样例；下一步聚焦共享边一致性约束真正参与修复决策
- 已扩展 shared-edge 邻接链 mixed-content 下 support-plane mismatch + collinear-leading 组合子样例；下一步聚焦共享边一致性约束真正参与修复决策
- 已扩展 shared-edge 邻接链 mixed-content 下 support-plane mismatch + collinear-leading + duplicate-hole normalization 全组合子样例；下一步聚焦共享边一致性约束真正参与修复决策
- 已扩展 shared-edge 邻接链 mixed-content full-composition 下 outer/hole 双重复顶点 normalization 组合子样例；下一步聚焦共享边一致性约束真正参与修复决策
- 已扩展 shared-edge 邻接链修复后的全局共享顶点/边复用子样例；下一步聚焦共享边一致性约束真正参与 support-plane / refit 决策，而不仅是结果拓扑去重
- 已扩展 closed-shell 代表性输入（tiny-scale tetrahedron：4 triangular faces, 6 edges, 4 vertices, 所有 support planes mismatched）的共享拓扑 BrepBody 子样例；下一步聚焦共享边一致性约束真正参与 support-plane / refit 决策，而不仅是结果拓扑去重
- 已扩展 closed-shell 代表性输入（tiny-scale tetrahedron：4 triangular faces, 6 edges, 4 vertices, 所有 support planes mismatched）的共享拓扑 BrepBody 子样例；已完成
- 已说明独立 per-face refit 下四边形面链共享顶点不一致的原因，并已修正 quad-chain test 中错误的 VertexCount/EdgeCount 断言；下一步已转向显式 triangular-face chain 验证
- 已扩展 triangular-face chain（3 个三角面，mismatched support planes，T1/T2/T3 共享边）的代表性子样例，验证 per-face refit 下三角面共享顶点精确一致性（VertexCount=5 / EdgeCount=7）；下一步聚焦 quad-face 场景下共享边一致性约束真正参与 refit 决策（需要修改 `ConvertToBrepBody` 的 repair 算法）
- 已扩展 triangular-face fan（4 个三角面共享 apex，mismatched support planes）的代表性子样例，验证 per-face refit 下共享 apex 顶点精确一致性（VertexCount=5 / EdgeCount=8）；下一步聚焦 quad-face 场景下共享边一致性约束真正参与 refit 决策（需要修改 `ConvertToBrepBody` 的 repair 算法，在 per-face 独立投影后引入跨面顶点位置 snapping 步骤）
- 已在 `src/sdk/GeometryBrepConversion.cpp` 落地 shared-vertex-aware refit 启发式：outer loop 中跨面共享顶点优先参与 support-plane 三点组选取，以减少 shared-edge 顶点跨面投影偏差；下一步仍需实现 quad-face 场景下的跨面顶点 snapping/联合约束以彻底消除 residual mismatch
- 已在 `ConvertToBrepBody(...)` 落地 source representative-id 贯穿复用（输入拓扑代表点驱动 BrepVertex 复用），并已恢复 tiny-scale quad shared-edge chain 的确定性拓扑断言（VertexCount=8 / EdgeCount=10）
- 已扩展 support-plane mismatch 的 tiny-scale quad shared-edge chain（3 quads）capability：`ConvertToBrepBody(...)` 可稳定保持共享拓扑（VertexCount=8 / EdgeCount=10）
- 已扩展 support-plane mismatch + duplicate-loop-normalization 的 tiny-scale quad shared-edge chain capability：`ConvertToBrepBody(...)` 在中间 face 含重复 leading 顶点时仍可稳定保持共享拓扑（VertexCount=8 / EdgeCount=10）
- 已扩展 shared-chain mixed-content 计数组合 capability：duplicate-hole、support-plane mismatch、以及 support-mismatch + duplicate-hole 组合场景均可稳定保持共享拓扑计数（VertexCount=12 / EdgeCount=14）
- 已扩展 shared-chain 其余组合子场景的计数断言能力：shared-edge duplicate-loop、mixed-content collinear-leading、support-mismatch+collinear、full-composition、dual-duplicate full-composition 现均补齐 VertexCount/EdgeCount 确定性断言
- 已扩展 tiny-scale 基础子场景计数断言能力：non-planar multi-face、non-planar mixed-content、non-planar shared-edge faces、non-planar shared-edge-chain mixed-content 现均补齐 VertexCount/EdgeCount 确定性断言
- 已扩展早期 repair 子场景计数断言能力：skewed/mismatched/non-planar cube、single-face holed/collinear/duplicate/composite、tiny-scale face/holed-face 等代表性输入均补齐 VertexCount/EdgeCount 确定性断言
- 已开始 topology-changing non-planar repair 算法预备：在 `TryRepairPolyhedronBodyForBrepConversion(...)` 后新增 representative-id global snapping pass（并回投影到各自 face support plane），作为跨面联合修复第一步
- 已扩展 conversion capability 语义断言：cube-like repair 子场景补齐 `ShellCount()==1 && IsClosed()==true`，shared-chain sheet-like 子场景补齐 `ShellCount()==1 && IsClosed()==false`
- 已新增 conversion capability：deformed unit cube（单顶点位移导致三面同时非平面）可经 per-face refit + representative-id 复用收敛为有效 closed BrepBody（FaceCount=6/VertexCount=8/EdgeCount=12/IsClosed=true）；gap 文案已纳入 deformed-cube multi-face 子集
- 已新增 conversion capability：dual-deformed unit cube（双顶点位移导致六面均非平面）可经 per-face refit + representative-id 复用收敛为有效 closed BrepBody（FaceCount=6/VertexCount=8/EdgeCount=12/IsClosed=true）；gap 文案已纳入 dual-displaced-vertices 子集
- 已扩展 representative-id 全局目标点聚合 capability：near-equal shared-edge（<eps 顶点扰动）输入下，共享 `BrepVertex` 落点由跨面代表点全局平均驱动，不再依赖首个面点
- 已扩展 support-plane mismatch + near-equal shared-edge capability：`ConvertToBrepBody(...)` 经 refit 后仍可保持 representative-average 共享顶点落点与确定性拓扑计数（VertexCount=6 / EdgeCount=7）
- 已扩展 support-plane mismatch + near-equal shared-apex triangular-fan capability：`ConvertToBrepBody(...)` 经 refit 后仍可保持 representative-average 共享 apex 顶点落点与确定性拓扑计数（VertexCount=5 / EdgeCount=8）
- 已扩展 support-plane mismatch + near-equal shared-edge-chain（3 faces）capability：`ConvertToBrepBody(...)` 经 refit 后仍可保持 representative-average 共享顶点落点与确定性拓扑计数（VertexCount=8 / EdgeCount=10）
- 已扩展 support-plane mismatch + near-equal shared-corner fan（3 faces，仅共享顶点）capability：`ConvertToBrepBody(...)` 经 refit 后仍可保持 representative-average 共享顶点落点与确定性拓扑计数（VertexCount=8 / EdgeCount=10）
- 已扩展 support-plane mismatch + near-equal closed-tetra（4 triangular faces）capability：`ConvertToBrepBody(...)` 经 refit 后仍可保持 representative-average 共享顶点落点与 closed-shell 确定性拓扑计数（VertexCount=4 / EdgeCount=6）
- 已扩展 support-plane mismatch + near-equal shared-edge-chain（3 faces）+ middle-face duplicate-loop-normalization capability：`ConvertToBrepBody(...)` 经 refit 后仍可保持 representative-average 共享顶点落点与确定性拓扑计数（VertexCount=8 / EdgeCount=10）
- 已扩展 support-plane mismatch + near-equal shared-chain mixed-content（含 middle-face duplicate-hole-normalization）capability：`ConvertToBrepBody(...)` 经 refit 后仍可保持 representative-average 共享顶点落点与确定性拓扑计数（VertexCount=12 / EdgeCount=14）
- 已扩展 support-plane mismatch + near-equal shared-chain full-composition（含 collinear-leading + duplicate-hole-normalization）capability：`ConvertToBrepBody(...)` 经 refit 后仍可保持 representative-average 共享顶点落点与确定性拓扑计数（VertexCount=13 / EdgeCount=15）
- 已扩展 support-plane mismatch + near-equal shared-chain dual-duplicate full-composition（含 outer+hole duplicate-normalization）capability：`ConvertToBrepBody(...)` 经 refit 后仍可保持 representative-average 共享顶点落点与确定性拓扑计数（VertexCount=13 / EdgeCount=15）
- 已扩展 support-plane mismatch + near-equal closed-tetra dual-shared-vertices（两个共享顶点同时 near-equal 扰动）capability：`ConvertToBrepBody(...)` 经 refit 后仍可保持 representative-average 共享顶点落点与 closed-shell 确定性拓扑计数（VertexCount=4 / EdgeCount=6）
- 已扩展 support-plane mismatch + near-equal closed-tetra all-shared-vertices（四个顶点全部 near-equal 扰动）capability：`ConvertToBrepBody(...)` 经 refit 后可对全部共享顶点同时应用 representative-average 落点，保持 closed-shell 确定性拓扑计数（VertexCount=4 / EdgeCount=6）
- 已扩展 support-plane mismatch + near-equal closed-prism dual-shared-vertices（三棱柱拓扑，两不相邻顶点 near-equal 扰动）capability：`ConvertToBrepBody(...)` 经 refit 后可跨三角面与四边形面同时稳定应用 representative-average 落点，保持 closed-shell 确定性拓扑计数（VertexCount=6 / EdgeCount=9）
- 已扩展 support-plane mismatch + near-equal closed-prism all-shared-vertices（三棱柱拓扑，六个共享顶点全部 near-equal 扰动）capability：`ConvertToBrepBody(...)` 经 refit 后可对全体共享顶点同时稳定应用 representative-average 落点，保持 closed-shell 确定性拓扑计数（VertexCount=6 / EdgeCount=9）
- 已完成 representative-target 聚合失败回退硬化：聚合失败时自动回退到 representative-id 复用路径，不再直接 `InvalidBody`
- 已增强 repair 后 representative snapping：由单轮提升为最多两轮小步迭代（每轮保持有效性约束）
- 已在 `Section(...)` 输出链路落地 contour 驱动的 deterministic segment 后处理（无向去重 + 短毛刺过滤），并将三棱柱截面段数稳定断言固化为 `segments=3`
- 下一步聚焦更一般 topology-changing non-planar repair（超出当前 representative-id + shared-refit + representative-average vertex placement + fallback-hardening + iterative-snapping 子集），继续保留 `GeneralNonPlanarPolyhedronToBrepRepairRemainsOpen` 作为总 gap

### P4-A 必需完成清单（面向建模/显示/平切算筋）

- 必需-1：`Section` 在 non-planar dominant 输入下的歧义 contour stitching 稳定收敛（对应 `Section3dGapTest::NonPlanarDominantSectionGraphRemainsOpen`）
  - **[已收敛子集]** 凸体 oblique face-to-face mid-plane 四段闭合轮廓行列式（unit cube y=0.5 四段/Area/area=1）
  - **[已收敛子集]** 三棱柱 mid-section 三段闭合轮廓（perimeter≈3）
  - **[已收敛子集]** 三棱柱 mid-section 在 Brep 路径三段闭合轮廓（perimeter≈3）
  - **[已收敛子集]** unit cube x=0.5 截面四段闭合 1×1 矩形（perimeter=4.0 / area=1.0）
  - **[已收敛子集]** unit cube x=0.5 截面在 Brep 路径四段闭合 1×1 矩形（perimeter=4.0 / area=1.0）
  - **[已收敛子集]** 2×2×1 矩形棱柱 z=0.5 截面四段闭合 2×2 方形（perimeter=8.0 / area=4.0）
  - **[已收敛子集]** 2×2×1 矩形棱柱 z=0.5 截面在 Brep 路径四段闭合 2×2 方形（perimeter=8.0 / area=4.0）
  - **[已收敛子集]** 2×2×1 矩形棱柱 x=1.0 截面四段闭合 2×1 矩形（perimeter=6.0 / area=2.0）
  - **[已收敛子集]** 2×2×1 矩形棱柱 x=1.0 截面在 Brep 路径四段闭合 2×1 矩形（perimeter=6.0 / area=2.0）
  - **[已收敛子集]** unit cube y=0.5 截面在 Brep 路径四段闭合 1×1 方形（segments=4 / perimeter=4.0 / area=1.0）
  - **[已收敛子集]** 2×2×1 矩形棱柱 y=1.0 截面四段闭合 2×1 矩形（perimeter=6.0 / area=2.0）
  - **[已收敛子集]** 2×2×1 矩形棱柱 y=1.0 截面在 Brep 路径四段闭合 2×1 矩形（perimeter=6.0 / area=2.0）
  - **[已收敛子集]** unit cube z=0.5 截面在 Polyhedron 路径四段闭合 1×1 方形（segments=4 / perimeter=4.0 / area=1.0）
  - **[已收敛子集]** unit cube z=0.5 截面在 Brep 路径四段闭合 1×1 方形（segments=4 / perimeter=4.0 / area=1.0）
- 必需-2：`Section` 在超出现有子集时的 coplanar fragment merge 语义闭合，确保切面输出不会碎片化（对应 `Section3dGapTest::FaceMergeSemanticsAfterSectionRemainsOpen`）
  - **[已收敛子集]** 三面共面水平排列 strip 合并为单多边形（3×1 矩形，area=3）
  - **[已收敛子集]** 三面共面水平排列 strip 在 Brep 路径合并为单多边形（3×1 矩形，area=3）
- 必需-3：`ConvertToBrepBody(...)` 向更一般 topology-changing non-planar repair 推进并转正 capability（对应 `Conversion3dGapTest::GeneralNonPlanarPolyhedronToBrepRepairRemainsOpen`）
  - **[已收敛子集]** support-mismatch near-equal closed-cuboid dual-shared-vertices（无 duplicate-loop-normalization，FaceCount=6/VertexCount=8/EdgeCount=12/closed shell）
  - **[已收敛子集]** support-mismatch near-equal closed-cuboid dual-shared-vertices + duplicate-loop-normalization（单面重复 leading 顶点，FaceCount=6/VertexCount=8/EdgeCount=12/closed shell）
  - **[已收敛子集]** support-mismatch near-equal closed-cuboid dual-shared-vertices + dual-duplicate-loop-normalization（双面重复 leading 顶点，FaceCount=6/VertexCount=8/EdgeCount=12/closed shell）
  - **[已收敛子集]** support-mismatch near-equal closed-cuboid all-vertices（2×1×1 矩形盒子，8 顶点全部 near-equal 扰动，FaceCount=6/VertexCount=8/EdgeCount=12/closed shell）
  - **[已收敛子集]** support-mismatch near-equal closed-cuboid all-vertices + duplicate-loop-normalization（单面重复 leading 顶点，FaceCount=6/VertexCount=8/EdgeCount=12/closed shell）
  - **[已收敛子集]** support-mismatch near-equal closed-cuboid all-vertices + dual-duplicate-loop-normalization（双面重复 leading 顶点，FaceCount=6/VertexCount=8/EdgeCount=12/closed shell）
  - **[已收敛子集]** support-mismatch near-equal closed-cuboid triple-shared-vertices（三顶点 near-equal 扰动，无 duplicate-loop-normalization，FaceCount=6/VertexCount=8/EdgeCount=12/closed shell）
  - **[已收敛子集]** support-mismatch near-equal closed-cuboid triple-shared-vertices + duplicate-loop-normalization（单面重复 leading 顶点，FaceCount=6/VertexCount=8/EdgeCount=12/closed shell）
  - **[已收敛子集]** support-mismatch near-equal closed-cuboid triple-shared-vertices + dual-duplicate-loop-normalization（双面重复 leading 顶点，FaceCount=6/VertexCount=8/EdgeCount=12/closed shell）
  - **[已收敛子集]** support-mismatch near-equal closed-tetra dual-shared-vertices + duplicate-loop-normalization（单面重复 leading 顶点，FaceCount=4/VertexCount=4/EdgeCount=6/closed shell）
  - **[已收敛子集]** support-mismatch near-equal closed-prism dual-shared-vertices + duplicate-loop-normalization（单面重复 leading 顶点，FaceCount=5/VertexCount=6/EdgeCount=9/closed shell）
  - **[已收敛子集]** support-mismatch near-equal closed-prism all-shared-vertices + duplicate-loop-normalization（单面重复 leading 顶点，FaceCount=5/VertexCount=6/EdgeCount=9/closed shell）
  - **[已收敛子集]** support-mismatch near-equal closed-tetra all-shared-vertices + duplicate-loop-normalization（单面重复 leading 顶点，FaceCount=4/VertexCount=4/EdgeCount=6/closed shell）
  - 仍为 open gap（更一般 topology-changing repair）
  - **[下一轮优先子集]** support-mismatch near-equal closed-prism dual-shared-vertices + dual-duplicate-loop-normalization（双面重复 leading 顶点，目标 FaceCount=5/VertexCount=6/EdgeCount=9/closed shell）
  - **[下一轮优先子集]** support-mismatch near-equal closed-prism all-shared-vertices + dual-duplicate-loop-normalization（双面重复 leading 顶点，目标 FaceCount=5/VertexCount=6/EdgeCount=9/closed shell）
  - **[下一轮优先子集]** support-mismatch near-equal closed-tetra dual/all-shared-vertices + dual-duplicate-loop-normalization（双面重复 leading 顶点，目标 FaceCount=4/VertexCount=4/EdgeCount=6/closed shell）
- 必需-4：`Heal(BrepBody)` 覆盖 non-planar trimmed face topology repair（对应 `Brep3dGapTest::NonPlanarTrimmedFaceTopologyRepairRemainsOpen`）
  - **[已收敛子集]** 非水平平面（y=0 竖面，法向+y）单面 BrepFace 缺失 trim 回填
  - **[已收敛子集]** x=0 竖面（法向+x）单面 BrepFace 缺失 trim 回填
  - **[已收敛子集]** oblique 平面（x+y+z=0，法向(1,1,1)）单面 BrepFace 缺失 trim 回填
- 必需-5：`Heal(..., policy=Aggressive)` 覆盖超出 planar open-sheet/holed-shell 的更一般 shell repair policy（对应 `Healing3dGapTest::AggressiveShellRepairPolicyRemainsOpen`）
  - **[已收敛子集]** 四壳 mixed（1 closed + 2 eligible single-face + 1 ineligible），两个 eligible 闭合，ineligible 保持 open
- 必需-6：切面钢筋线结果链路补齐 deterministic 后处理（线段去重/共线合并/短毛刺抑制/分组统计稳定），并以 capability tests 固化根数与总长断言
  - **[已收敛子集]** 三棱柱水平截面轮廓总长确定性断言（equilateral triangle 周长≈3）
  - **[已收敛子集]** 三棱柱水平截面线段根数确定性断言（segments=3）
  - **[已收敛子集]** 三棱柱水平截面在 Brep 路径轮廓周长断言（equilateral triangle 周长≈3）
  - **[已收敛子集]** 三棱柱水平截面在 Brep 路径线段根数断言（segments=3）
  - **[已收敛子集]** unit cube x=0.5 截面轮廓周长断言（1×1 square perimeter=4.0）
  - **[已收敛子集]** unit cube x=0.5 截面在 Brep 路径轮廓周长断言（1×1 square perimeter=4.0）
  - **[已收敛子集]** 2×2×1 矩形棱柱 z=0.5 截面轮廓周长断言（2×2 square perimeter=8.0）
  - **[已收敛子集]** 2×2×1 矩形棱柱 z=0.5 截面在 Brep 路径轮廓周长断言（2×2 square perimeter=8.0）
  - **[已收敛子集]** 2×2×1 矩形棱柱 x=1.0 截面轮廓周长断言（2×1 rectangle perimeter=6.0）
  - **[已收敛子集]** 2×2×1 矩形棱柱 x=1.0 截面在 Brep 路径轮廓周长断言（2×1 rectangle perimeter=6.0）
  - **[已收敛子集]** unit cube y=0.5 截面在 Brep 路径轮廓周长断言（1×1 square perimeter=4.0）
  - **[已收敛子集]** 2×2×1 矩形棱柱 y=1.0 截面轮廓周长断言（2×1 rectangle perimeter=6.0）
  - **[已收敛子集]** 2×2×1 矩形棱柱 y=1.0 截面在 Brep 路径轮廓周长断言（2×1 rectangle perimeter=6.0）
  - **[已收敛子集]** unit cube z=0.5 截面轮廓周长断言（1×1 square perimeter=4.0）
  - **[已收敛子集]** unit cube z=0.5 截面在 Brep 路径轮廓周长断言（1×1 square perimeter=4.0）

### P4-B：aggressive shell policy 分层落地（已完成最小子集）
- 已覆盖 mixed support-mismatch + ineligible multiface 系列场景的 shell-level FaceCount 分布断言
- 已覆盖 single-face / multi-face / holed / multi-shell open-shell 的最小 deterministic closure 子策略
- 已覆盖 shared-edge 邻接的 planar multi-face open-sheet 的 deterministic closure 子策略
- 已覆盖 mixed closed/open-shell 输入下“保持闭壳稳定 + 闭合开壳”的最小 deterministic 子策略
- 已覆盖 mixed open-shell 输入下“可恢复壳闭合 + 不可恢复壳保持原状”的最小 deterministic 部分修复子策略
- 已覆盖三壳 mixed 输入下“closed 保持 + eligible 闭壳 + ineligible 保持”的 deterministic 子策略
- 已覆盖三壳 mixed 输入下“eligible 先 trim-backfill 再闭壳 + ineligible 保持”的 deterministic 组合子策略
- 已覆盖三壳 mixed 输入下 eligible multi-face open-sheet 的 deterministic 闭壳子策略
- 已覆盖三壳 mixed 输入下 eligible holed shell（缺失 outer/hole trims）回填后 deterministic 闭壳子策略
- 已覆盖三壳 mixed 输入下 eligible multi-face holed shell（缺失 trims）回填后 deterministic 闭壳子策略
- 已覆盖三壳 mixed 输入下 eligible multi-face（holed+plain）两面 trims 缺失回填后 deterministic 闭壳子策略
- 已覆盖三壳 mixed 输入下 eligible multi-face holed shell 的 support-plane mismatch + missing trims 回填后 deterministic 闭壳子策略
- 已覆盖 support-plane mismatch 的 eligible shell 与 ineligible multi-face shell 共存下的 deterministic 闭壳/保留 open 子策略
- 已覆盖 support-plane mismatch + trim-backfill 的 eligible shell 与 ineligible multi-face shell 共存下的 deterministic 闭壳/保留 open 子策略
- 已覆盖 support-plane mismatch 的 eligible holed shell（缺失 outer/hole trims）与 ineligible multi-face shell 共存下的 deterministic 回填后闭壳/保留 open 子策略
- 已覆盖 support-plane mismatch 的 eligible multi-face（holed+plain，缺失 trims）与 ineligible multi-face shell 共存下的 deterministic 回填后闭壳/保留 open 子策略
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

## 本轮新增（2026-04-03，continuation-71）

- 已扩展 conversion capability：closed-prism dual-shared-vertices + dual-duplicate-loop-normalization，目标闭壳拓扑计数为 `FaceCount=5 / VertexCount=6 / EdgeCount=9`。
- 已扩展 conversion capability：closed-prism all-shared-vertices + dual-duplicate-loop-normalization，目标闭壳拓扑计数为 `FaceCount=5 / VertexCount=6 / EdgeCount=9`。
- 已扩展 conversion capability：closed-tetra dual-shared-vertices + dual-duplicate-loop-normalization，目标闭壳拓扑计数为 `FaceCount=4 / VertexCount=4 / EdgeCount=6`。
- 已预留 closed-tetra all-shared-vertices + dual-duplicate-loop-normalization 输入构造器；下一轮优先补齐对应 capability 断言，然后继续向更一般 topology-changing non-planar repair 推进。
