# 会话交接

## 当前上下文

- 工作区：`D:\code\stablecore-geometry`
- 交接更新时间：`2026-04-02`
- 可用环境：`python`
- 后续会话应聚焦于源码与文档编写
- 编译 / 构建 / 运行由用户手动完成
- 不必担心 `gtest` 环境接入，用户会按需要调整 CMake / 构建侧

## 本轮新增（2026-04-02，continuation）

- 已新增 capability：`Heal(..., policy=Aggressive)` 在同一个 `BrepBody` 内可同时闭合多个可恢复 open shells（`tests/capabilities/test_3d_healing.cpp`）。
- 已同步收敛 `tests/gaps/test_3d_healing_gaps.cpp` 中 aggressive policy gap 文案，明确当前已覆盖 single/multi-face、holed、multi-shell 的 recoverable planar open-shell 子集。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，continuation-2）

- 已新增 conversion capability：`ConvertToBrepBody(...)` 在 tiny-scale non-planar outer loop 输入下可通过 scale-aware 法向回退稳定完成 refit 修复（`tests/capabilities/test_3d_conversion.cpp`）。
- 已同步收敛 `tests/gaps/test_3d_conversion_gaps.cpp` 中 non-planar repair gap 文案，纳入 tiny-loop normal fallback 子集。
- 已更新：`src/sdk/GeometryBrepConversion.cpp`、`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，continuation-3）

- 已新增 conversion capability：tiny-scale non-planar holed face 输入可稳定 `ConvertToBrepBody(...)`（outer/hole loop 投影修复与 scale-aware 法向回退协同生效，见 `tests/capabilities/test_3d_conversion.cpp`）。
- 已同步收敛 `tests/gaps/test_3d_conversion_gaps.cpp` 的 non-planar repair 文案，纳入 tiny-scale holed projection 子集。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，continuation-4）

- 已新增 healing capability：`Heal(..., policy=Aggressive)` 在 mixed closed/open-shell 输入下可保持已闭壳稳定，同时对可恢复 open shell 执行确定性闭壳（`tests/capabilities/test_3d_healing.cpp`）。
- 已同步收敛 `tests/gaps/test_3d_healing_gaps.cpp` 的 aggressive policy 文案，纳入 mixed closed/open-shell 子集。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，continuation-5）

- 已新增 conversion capability：tiny-scale non-planar multi-face 输入可稳定 `ConvertToBrepBody(...)`，逐面 refit 后保持 `FaceCount` 一致（`tests/capabilities/test_3d_conversion.cpp`）。
- 已同步收敛 `tests/gaps/test_3d_conversion_gaps.cpp` 的 non-planar repair 文案，纳入 tiny-scale multi-face 子集。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，continuation-6）

- 已新增 healing capability：mixed open-shell 输入下可执行 deterministic partial repair（eligible shell 闭壳，ineligible shell 保持 open），见 `tests/capabilities/test_3d_healing.cpp`。
- 已同步收敛 `tests/gaps/test_3d_healing_gaps.cpp` 的 aggressive policy 文案，纳入 partial mixed-shell repair 子集。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，continuation-7）

- 已新增 conversion capability：tiny-scale non-planar mixed-content（holed+plain）multi-face 输入可稳定 `ConvertToBrepBody(...)`，逐面修复后保持 `FaceCount` 一致（`tests/capabilities/test_3d_conversion.cpp`）。
- 已同步收敛 `tests/gaps/test_3d_conversion_gaps.cpp` 的 non-planar repair 文案，纳入 tiny-scale mixed-content 子集。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，continuation-8）

- 已新增 conversion capability：tiny-scale non-planar shared-edge 邻接面输入可稳定 `ConvertToBrepBody(...)`，逐面修复后保持 `FaceCount` 一致（`tests/capabilities/test_3d_conversion.cpp`）。
- 已同步收敛 `tests/gaps/test_3d_conversion_gaps.cpp` 的 non-planar repair 文案，纳入 shared-edge 子集。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，continuation-9）

- 已新增 healing capability：三壳 mixed 输入下 `Heal(..., policy=Aggressive)` 可稳定执行 deterministic 行为（closed 保持稳定、eligible open 闭壳、ineligible open 保持 open），见 `tests/capabilities/test_3d_healing.cpp`。
- 已同步收敛 `tests/gaps/test_3d_healing_gaps.cpp` 的 aggressive policy 文案，纳入 three-shell deterministic mixed 子集。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，continuation-10）

- 已新增 conversion capability：tiny-scale non-planar shared-edge 邻接链输入可稳定 `ConvertToBrepBody(...)`，在多邻接面场景下保持 `FaceCount` 一致（`tests/capabilities/test_3d_conversion.cpp`）。
- 已同步收敛 `tests/gaps/test_3d_conversion_gaps.cpp` 的 non-planar repair 文案，纳入 shared-edge chain 子集。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，continuation-11）

- 已新增 healing capability：三壳 mixed 输入下，eligible open shell 可先经 trim-backfill 再执行 aggressive 闭壳，ineligible open shell 保持 open（`tests/capabilities/test_3d_healing.cpp`）。
- 已同步收敛 `tests/gaps/test_3d_healing_gaps.cpp` 的 aggressive policy 文案，纳入 three-shell+trim-backfill 子集。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，continuation-12）

- 已新增 conversion capability：tiny-scale non-planar shared-edge 邻接链 mixed-content 输入可稳定 `ConvertToBrepBody(...)`，并保持 `FaceCount` 一致（`tests/capabilities/test_3d_conversion.cpp`）。
- 已同步收敛 `tests/gaps/test_3d_conversion_gaps.cpp` 的 non-planar repair 文案，纳入 shared-edge chain mixed-content 子集。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，continuation-13）

- 已新增 healing capability：三壳 mixed 输入下可对 eligible multi-face open-sheet 执行 deterministic 闭壳，同时保持 closed shell 稳定与 ineligible shell open（`tests/capabilities/test_3d_healing.cpp`）。
- 已同步收敛 `tests/gaps/test_3d_healing_gaps.cpp` 的 aggressive policy 文案，纳入 three-shell eligible multi-face 子集。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，continuation-14）

- 已新增 conversion 组合 capability：shared-edge 邻接链场景下，duplicate-loop-normalization 与逐面 refit/projection 可稳定叠加（`tests/capabilities/test_3d_conversion.cpp`）。
- 已同步收敛 `tests/gaps/test_3d_conversion_gaps.cpp` 文案，纳入 shared-chain composition 子集。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，continuation-15）

- 已新增 healing capability：三壳 mixed 输入下，eligible holed shell 在缺失 outer/hole trims 时可先回填再 deterministic 闭壳，ineligible shell 保持 open（`tests/capabilities/test_3d_healing.cpp`）。
- 已同步收敛 `tests/gaps/test_3d_healing_gaps.cpp` 文案，纳入 three-shell holed-shell missing-trims 子集。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，continuation-16）

- 已新增 conversion 组合 capability：shared-edge 邻接链 mixed-content 下，duplicate hole-loop normalization 与逐面 refit/projection 可稳定叠加（`tests/capabilities/test_3d_conversion.cpp`）。
- 已同步收敛 `tests/gaps/test_3d_conversion_gaps.cpp` 文案，纳入 duplicate-hole shared-chain composition 子集。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，continuation-17）

- 已新增 healing capability：三壳 mixed 输入下，eligible multi-face holed shell（缺失 trims）可先回填后 deterministic 闭壳，ineligible shell 保持 open（`tests/capabilities/test_3d_healing.cpp`）。
- 已同步收敛 `tests/gaps/test_3d_healing_gaps.cpp` 文案，纳入 three-shell eligible multi-face holed+missing-trims 子集。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，continuation-18）

- 已新增 conversion 组合 capability：shared-edge 邻接链 mixed-content 下，collinear-leading normal fallback 与 duplicate/hole normalization 可稳定叠加（`tests/capabilities/test_3d_conversion.cpp`）。
- 已同步收敛 `tests/gaps/test_3d_conversion_gaps.cpp` 文案，纳入 collinear-leading shared-chain composition 子集。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，continuation-19）

- 已新增 healing 组合 capability：三壳 mixed 输入下，eligible multi-face（holed+plain）两面 trims 缺失可先回填后 deterministic 闭壳，ineligible shell 保持 open（`tests/capabilities/test_3d_healing.cpp`）。
- 已同步收敛 `tests/gaps/test_3d_healing_gaps.cpp` 文案，纳入 three-shell eligible multi-face both-trims-missing 子集。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，continuation-20）

- 已新增 conversion 组合 capability：shared-edge 邻接链 mixed-content 下，support-plane mismatch 的 refit 与 projection/normalization 可稳定叠加（`tests/capabilities/test_3d_conversion.cpp`）。
- 已同步收敛 `tests/gaps/test_3d_conversion_gaps.cpp` 文案，纳入 support-plane mismatch shared-chain composition 子集。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，continuation-21）

- 已新增 healing 组合 capability：三壳 mixed 输入下，eligible multi-face holed shell 的 support-plane mismatch + missing trims 可先回填后 deterministic 闭壳，ineligible shell 保持 open（`tests/capabilities/test_3d_healing.cpp`）。
- 已同步收敛 `tests/gaps/test_3d_healing_gaps.cpp` 文案，纳入 three-shell eligible support-plane-mismatch composition 子集。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，continuation-22）

- 已新增 conversion 组合 capability：shared-edge 邻接链 mixed-content 下，support-plane mismatch refit 与 collinear-leading fallback 可稳定叠加（`tests/capabilities/test_3d_conversion.cpp`）。
- 已同步收敛 `tests/gaps/test_3d_conversion_gaps.cpp` 文案，纳入 support-plane+collinear shared-chain composition 子集。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，continuation-23）

- 已新增 healing 组合 capability：support-plane mismatch 的 eligible shell 与 ineligible multi-face shell 共存时，`Heal(..., policy=Aggressive)` 仍可保持 deterministic（eligible 闭壳、ineligible 保持 open），见 `tests/capabilities/test_3d_healing.cpp`。
- 已同步收敛 `tests/gaps/test_3d_healing_gaps.cpp` 文案，纳入 support-mismatch eligible + ineligible multiface 子集。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，continuation-24）

- 已新增 conversion 组合 capability：shared-edge 邻接链 mixed-content 下，support-plane mismatch + collinear-leading + duplicate-hole normalization 可稳定叠加（`tests/capabilities/test_3d_conversion.cpp`）。
- 已同步收敛 `tests/gaps/test_3d_conversion_gaps.cpp` 文案，纳入 shared-chain full-composition 子集。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，continuation-25）

- 已新增 healing 组合 capability：support-plane mismatch + trim-backfill 的 eligible shell 与 ineligible multi-face shell 共存时，`Heal(..., policy=Aggressive)` 仍可保持 deterministic（eligible 回填后闭壳，ineligible 保持 open），见 `tests/capabilities/test_3d_healing.cpp`。
- 已同步收敛 `tests/gaps/test_3d_healing_gaps.cpp` 文案，纳入 support-mismatch+trim-backfill+ineligible-multiface 子集。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，continuation-26）

- 已新增 healing 组合 capability：support-plane mismatch 的 eligible holed shell（缺失 outer/hole trims）与 ineligible multi-face shell 共存时，`Heal(..., policy=Aggressive)` 仍可保持 deterministic（eligible 回填后闭壳，ineligible 保持 open），见 `tests/capabilities/test_3d_healing.cpp`。
- 已同步收敛 `tests/gaps/test_3d_healing_gaps.cpp` 文案，纳入 support-mismatch holed-eligible + ineligible-multiface 子集。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，continuation-27）

- 已新增 conversion 组合 capability：shared-edge 邻接链 mixed-content full-composition 下，outer/hole 双重复顶点归一化可与 support-plane mismatch + collinear-leading 组合修复稳定叠加，见 `tests/capabilities/test_3d_conversion.cpp`。
- 已同步收敛 `tests/gaps/test_3d_conversion_gaps.cpp` 文案，纳入 dual outer/hole duplicate-normalization full-composition 子集。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，continuation-28）

- 已新增 healing 组合 capability：support-plane mismatch 的 eligible multi-face（holed+plain，缺失 trims）与 ineligible multi-face shell 共存时，`Heal(..., policy=Aggressive)` 仍可保持 deterministic（eligible 回填后闭壳，ineligible 保持 open），见 `tests/capabilities/test_3d_healing.cpp`。
- 已同步收敛 `tests/gaps/test_3d_healing_gaps.cpp` 文案，纳入 support-mismatch eligible-multiface-missing-trims + ineligible-multiface 子集。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，stage-1）

- 已更新 `src/sdk/GeometryBrepConversion.cpp`：`ConvertToBrepBody(...)` 现在在 shared-edge 邻接链修复后可全局复用共享顶点/边，而不是按 face 重复建拓扑。
- 已扩展 conversion capability：`tests/capabilities/test_3d_conversion.cpp` 现在验证 tiny-scale shared-edge chain 修复后得到全局一致的 `VertexCount()==8` 与 `EdgeCount()==10`。
- 已同步收敛 `tests/gaps/test_3d_conversion_gaps.cpp` 文案，纳入 shared-edge chain global vertex/edge consistency 子集。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，stage-2）

- 已更新 `src/sdk/GeometryHealing.cpp`：Aggressive shell closure 现在允许 planar multi-face shell 中存在 shared-edge 邻接，不再要求所有边都只被单面使用。
- 已新增 healing capability：`tests/capabilities/test_3d_healing.cpp` 现在验证 shared-edge open-sheet 在 Aggressive 策略下可确定性闭壳。
- 已同步收敛 `tests/gaps/test_3d_healing_gaps.cpp` 文案，纳入 shared-edge open-sheet 子集。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，stage-3）

- 已更新 `src/sdk/GeometrySection.cpp`：coplanar face section 结果现在会对相邻 polygon fragments 执行 2D union，并重建合并后的 contours / segments。
- 已新增 section capability：`tests/capabilities/test_3d_section.cpp` 现在验证相邻 coplanar faces 经 `Section(...)` 后可合并为单 polygon。
- 已同步收敛 `tests/gaps/test_3d_section_gaps.cpp` 文案，将 face-merge gap 缩小到更高阶歧义 coplanar fragments。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，stage-4）

- 已更新 `include/sdk/GeometryBrepEditing.h` 与 `src/sdk/GeometryBrepEditing.cpp`：新增 `ReplaceOuterLoop(...)` / `ReplaceFace(...)` / `ReplaceShell(...)`，补齐最小 loop->face->shell->body ownership-consistent editing workflow。
- 已新增 brep capability：`tests/capabilities/test_3d_brep.cpp` 现在验证 loop 编辑可通过 replacement workflow 稳定传播回有效 `BrepBody`。
- 已同步收敛 `tests/gaps/test_3d_brep_gaps.cpp` 文案，将 ownership gap 缩小到更高阶关联拓扑级编辑语义。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，continuation-after-stage-4）

- 已更新 `src/sdk/GeometryMeshConversion.cpp`：`ConvertToTriangleMesh(const BrepBody&)` 现在在 face 聚合时可全局复用相同 3D 顶点，不再简单按 face 追加全部顶点。
- 已新增 conversion capability：`tests/capabilities/test_3d_conversion.cpp` 现在验证 planar shared-edge 相邻面转换到 mesh 后可保持 `VertexCount()==6` 的共享顶点子集。
- 已同步收敛 `tests/gaps/test_3d_conversion_gaps.cpp` 文案，纳入 shared-edge vertex-reuse 子集。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，continuation-after-shared-mesh-vertices）

- 已扩展 `tests/capabilities/test_3d_conversion.cpp`：单位立方体 `ConvertToBrepBody(...)` 现在明确验证共享拓扑 Brep 子集（1 shell / closed / 8 vertices / 12 edges）。
- 已同步收敛 `tests/gaps/test_3d_conversion_gaps.cpp` 文案，纳入 closed-shell shared-topology subset。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，continuation-shell-semantics-hardening）

- 已扩展 conversion capability 语义断言：`tests/capabilities/test_3d_conversion.cpp` 为 cube-like repair 子场景补齐 `ShellCount()==1` 与 `IsClosed()==true`，并为 shared-chain sheet-like 子场景补齐 `ShellCount()==1` 与 `IsClosed()==false`。
- 本轮以壳体语义（closed/open）补强现有拓扑计数覆盖，进一步降低“计数正确但壳体语义漂移”的回归风险。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。
## 本轮新增（2026-04-02，deformed-cube multi-face non-planar）

- 已新增 conversion capability：`ConvertToBrepBody(...)` 可处理"单个顶点位移导致多个面同时非平面"的输入（deformed unit cube：V0 从 (0,0,0) 移至 (0.1,0.1,-0.1)，使 bottom/front/left 三面变为非平面）。
- 修复机制：每个非平面面的 per-face refit 选取包含 max-area 三点组，representative-id 驱动跨面 BrepVertex 复用保证拓扑一致性。
- 结果满足 FaceCount=6 / VertexCount=8 / EdgeCount=12 / 1 closed shell 确定性拓扑断言。
- 已同步收敛 `tests/gaps/test_3d_conversion_gaps.cpp` 文案，纳入 deformed-cube multi-face non-planar 子集。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。

## 本轮新增（2026-04-02，section-brep-oblique-capability）

- 已新增 section capability：`tests/capabilities/test_3d_section.cpp` 新增 `BrepBodyObliqueSectionHasSingleHexLikeContour`，验证 `Section(BrepBody, Plane)` 在 oblique cut（`x+y+z=1.5`）下稳定产出单闭合区域。
- 能力路径：先 `ConvertToBrepBody(unit cube)`，再执行 `Section(converted.body, cut)`，并验证 polygon/contour/topology/components 的确定性（1 polygon / 1 contour / 6 points / 1 root）。
- 已更新：`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`、`docs/next-task-prompt.md`。
## 本轮新增（2026-04-02，continuation-closed-shell-tetrahedron）

- 已新增 conversion capability：`ConvertToBrepBody(...)` 在 tiny-scale closed-shell tetrahedron（4 triangular faces, all support planes mismatched）输入上，经 per-face refit 修复后可收敛为合法 closed BrepBody（IsClosed=true / VertexCount=4 / EdgeCount=6）。
- 新增 builder：`BuildTinyScaleClosedTetrahedronBody()` 与 test `TinyScaleClosedTetrahedronConvertsToBrepBodyWithClosedShell` 已加入 `tests/capabilities/test_3d_conversion.cpp`。
- 已更新：`docs/test-capability-coverage.md`、`docs/next-task-prompt.md`、`docs/design-doc-sync-tracker.md`。
- **下一步**：聚焦共享边一致性约束真正参与 support-plane/refit 决策（not just topology deduplication）。
- **下一步**：聚焦共享边一致性约束真正参与 support-plane/refit 决策（not just topology deduplication），具体为 quad-face chain 共享顶点的 refit-plane 锚定机制。
- **下一步**：聚焦 quad-face chain 共享边顶点一致性算法（per-face 独立投影后 snapping pass），triangular-face 变体（chain/fan/tetrahedron）已全部覆盖。

## 本轮新增（2026-04-02，continuation-triangular-fan）

- 已新增 conversion capability：tiny-scale triangular-fan（4 个三角面共享 apex，mismatched support planes），per-face refit 后共享 apex 精确保持（VertexCount=5 / EdgeCount=8，4 条径向共享边 + 4 条外边）。
- Test: `TinyScaleTriangularFanRepairsToBrepBodyWithSharedApex`。
- 已更新：`docs/test-capability-coverage.md`、`docs/next-task-prompt.md`、`docs/design-doc-sync-tracker.md`。

## 本轮新增（2026-04-02，continuation-triangular-chain）

- 发现 `TinyScaleNonPlanarSharedEdgeChainStillRepairsToBrepBody` 中 `VertexCount==8 / EdgeCount==10` 断言对独立 per-face refit 不成立（quad 面的共享顶点投影后偏差 ~1μm >> eps=1e-9），已移除错误断言并补充注释说明限制。
- 已新增 conversion capability：tiny-scale triangular-face chain（T1/T2/T3 共享边，mismatched support planes），per-face refit 后共享顶点精确保持（定义顶点 distance=0），结果满足 VertexCount=5 / EdgeCount=7（test: `TinyScaleTriangularFaceChainRepairsToBrepBodyWithSharedEdgeConsistency`）。
- 已同步收敛 `tests/gaps/test_3d_conversion_gaps.cpp` 文案，明确下一开放前沿：quad-face shared-edge 顶点投影一致性需新的修复策略。
- 已更新：`docs/test-capability-coverage.md`、`docs/next-task-prompt.md`、`docs/design-doc-sync-tracker.md`。

## 本轮新增（2026-04-02，continuation-shared-vertex-aware-refit）

- 已在 `src/sdk/GeometryBrepConversion.cpp` 实现 shared-vertex-aware refit 启发式：在 `TryRepairPolyhedronBodyForBrepConversion(...)` 中先统计 outer loop 跨面共享顶点，再在 `BuildFaceWithRefitSupportPlane(...)` 中优先选择包含更多 shared vertices 的三点组估计 support-plane。
- 目标是减少 per-face 独立 refit 下 shared-edge 顶点的跨面投影偏差；该策略对 triangular-face chain/fan/tetrahedron 子样例保持兼容。
- 仍未完全闭合：quad-face chain 仍可能存在 residual mismatch，下一步需引入跨面顶点 snapping 或联合约束重投影。

## 本轮新增（2026-04-02，continuation-quad-gap-target）

- 历史说明：本节最初引入了显式 quad-chain gap 靶点以驱动算法推进。
- 当前状态：该靶点对应子问题已转正为 capability（含 duplicate-normalization 组合），显式子 gap 已从 `tests/gaps/test_3d_conversion_gaps.cpp` 移除。

## 本轮新增（2026-04-02，continuation-representative-id-reuse）

- 已在 `ConvertToBrepBody(...)` 落地 source representative-id 贯穿复用机制：从输入 `PolyhedronBody` 提取跨面代表点 ID，并在 `AppendSharedBrepLoopFromPolyLoop(...)` 中优先按代表点 ID 复用 `BrepVertex`，不再仅依赖修复后几何点位的 eps 近邻。
- 结果：`TinyScaleNonPlanarSharedEdgeChainStillRepairsToBrepBody` 已恢复确定性拓扑断言（VertexCount=8 / EdgeCount=10）。
- 已新增 capability：`TinyScaleSupportMismatchSharedEdgeChainRepairsWithSharedTopology`，覆盖 support-plane mismatch 的 tiny-scale 3-quad shared-edge chain，验证共享拓扑断言（VertexCount=8 / EdgeCount=10）。
- 已新增 capability：`TinyScaleSupportMismatchSharedEdgeChainWithDuplicateRepairsWithSharedTopology`，覆盖 support-plane mismatch + duplicate-loop-normalization 的 tiny-scale 3-quad shared-edge chain，验证共享拓扑断言（VertexCount=8 / EdgeCount=10）。
- `tests/gaps/test_3d_conversion_gaps.cpp` 已移除显式 quad-chain gap 子项，当前保留 `GeneralNonPlanarPolyhedronToBrepRepairRemainsOpen` 作为总 gap 入口。
- 已补齐 shared-chain mixed-content 的确定性拓扑计数组合能力：`TinyScaleSharedChainMixedContentDuplicateHoleRepairsToBrepBody`、`TinyScaleSharedChainMixedContentSupportPlaneMismatchRepairsToBrepBody`、`TinyScaleSharedChainMixedContentSupportMismatchWithDuplicateHoleRepairsToBrepBody` 现均断言 VertexCount=12 / EdgeCount=14。
- 已进一步补齐 shared-chain 其余组合子场景的确定性拓扑计数断言：`TinyScaleSharedEdgeChainWithDuplicateLoopRepairsToBrepBody`（8/10）、`TinyScaleSharedChainMixedContentCollinearLeadingRepairsToBrepBody`（13/15）、`TinyScaleSharedChainSupportMismatchAndCollinearRepairsToBrepBody`（13/15）、`TinyScaleSharedChainFullCompositionRepairsToBrepBody`（13/15）、`TinyScaleSharedChainDualDuplicateFullCompositionRepairsToBrepBody`（13/15）。
- 已补齐 tiny-scale 基础子场景的确定性拓扑计数断言：`TinyScaleNonPlanarMultiFaceStillRepairsToBrepBody`（8/8）、`TinyScaleNonPlanarMixedContentStillRepairsToBrepBody`（12/12）、`TinyScaleNonPlanarSharedEdgeFacesStillRepairToBrepBody`（6/7）、`TinyScaleNonPlanarSharedEdgeChainMixedContentRepairsToBrepBody`（12/14）。
- 已补齐早期 repair 子场景的确定性拓扑计数断言：`SkewedCubePolyhedronBodyConvertsToBrepBody`、`SupportPlaneMismatchedCubeCanBeRepairedToBrepBody`、`MildlyNonPlanarCubeFaceCanBeRepairedToBrepBody`、`MildlyNonPlanarHoleLoopCanBeRepairedToBrepBody`、`CollinearLeadingLoopStillRepairsToBrepBody`、`DuplicateVertexLoopStillRepairsToBrepBody`、`DuplicateVertexHoleLoopStillRepairsToBrepBody`、`CompositeRepairStressFaceStillConvertsToBrepBody`、`TinyScaleNonPlanarFaceStillRepairsToBrepBody`、`TinyScaleNonPlanarHoledFaceStillRepairsToBrepBody`。
- 已开始 topology-changing non-planar repair 的跨面联合修复预备：`GeometryBrepConversion.cpp` 新增 representative-id global snapping pass（repair 后跨面聚合 representative 点，再按各 face support plane 回投影），当前作为保守启发式增益步骤接入。
## 当前关注优先级

1. **3D robust non-planar repair**：从 closed-shell tetrahedron 子类走向共享边一致性约束驱动的 support-plane/refit 决策
2. **3D aggressive shell policy**：从 single-face planar closure 扩展到 multi-face open shell 策略
3. **3D coedge ownership 深化**：从 loop 级编辑走向 shell/face ownership 一致性
4. 2D SearchPoly 分支评分 / fake-edge 排序（仍有提升空间但不是当前阻塞点）
5. 2D relation 层级细化（当前稳定，不是当前阻塞点）

## 当前实际差距判断

基于当前源码、capability tests、gap tests 与对齐文档，当前仓库与 Delphi / GGP 的实际差距可粗略概括为：

- 2D：
  - 基础几何内核与常规 polygon 工作流，大致已到 Delphi 相关能力的 `70%~80%`
  - 高歧义线网、深退化 boolean、困难 offset 恢复等工程恢复层，大致仍只有 Delphi 相关能力的 `40%~60%`
- 3D：
  - 当前还处在第一阶段底座建设与最小 plane-based workflow 阶段
  - 若对照 Delphi `Geo3DLib` 与 GGP 的工程 3D 目标，大致仍只有目标能力的 `20%~35%`

这里的比例是工程判断，不是跑分结果。

当前最重要的解释是：

- 2D 已全面稳定：所有 2D gap 测试已转正为 capability tests，`tests/gaps/` 中的 2D 条目已清空
- 3D 完成了第一阶段基础能力（section / brep rebuild / healing / conversion 最小闭环），且 P2/P3 与 P4-B 最小子策略已转正；当前进入 robust non-planar repair 深水区

## 当前 2D 状态

### 构建 / 测试状态

- `tests/` 迁移到 gtest 已是当前真实状态
- 当前测试树包含：
  - `tests/capabilities`
  - `tests/gaps`
  - `tests/support`
- `tests/CMakeLists.txt` 已接入 capability 与 gap 两个 gtest 可执行目标
- `stablecore_geometry_capabilities_gtest` 在前一轮修复后已通过

### API / 命名状态

- 旧的 `GetXxx` 风格 2D API 调用点已从代码中清除
- `MultiPolyline2d::Count()`、`MultiPolygon2d::Count()`、`PolygonTopology2d::Count()` 是当前统一形式
- `docs/sdk-type-design-review.md` 仍是主要 API / 设计评审基线
- 成员方法与自由函数边界已收敛到：
  - `docs/sdk-2d-api-convergence.md`
  - `docs/api-member-free-function-checklist.md`

### 2D 设计文档同步状态

- 当前主干设计文档已统一为中文口径
- 2D 核心设计文档已同步到当前 API 状态：
  - `docs/segment-design.md`
  - `docs/polyline-design.md`
  - `docs/polygon-design.md`
  - `docs/box-design.md`
- 文档同步进度记录在：
  - `docs/design-doc-sync-tracker.md`
- 后续会话在完成有意义任务后，应及时更新 `docs/session-handoff.md`

### 几何 / 算法状态

已实现：

- `BuildMultiPolygonByLines`
  - 开放线网建面
  - 交点切分
  - 重复边清理
  - 近端点自动闭合
  - 简单投影式 auto-extend
  - dangling branch 裁剪
  - outer / hole 嵌套归并
  - synthetic / fake edge 跟踪
  - 候选闭环面积阈值过滤
  - fake-edge 主导的小环抑制
  - 真实边优先的重复边保留
- Boolean
  - arrangement face 提取
  - bounded face 分类
  - 结果边界重建
  - 常规 crossing / containment
  - duplicate-edge 预处理
  - 微小 sliver face 过滤
  - 更强的 face sampling 与多探针 face 分类
  - 针对 `Equal`、`Disjoint`、`FirstContainsSecond`、`SecondContainsFirst`、`Touching` 的 relation-aware 快路径
  - ultra-thin repeated-overlap 家族已提升到 capability 覆盖
  - ring 简化加入近共线碎顶点清理
  - tiny-face 面积阈值改为更保守，避免误删极薄但真实的 overlap 结果
- Offset
  - 从 offset ring 重建结果
  - 基础凹形 / multipolygon 恢复
  - collapsed / near-zero ring 过滤
  - 更好的单 polygon 候选选择
  - `MultiPolygon2d` offset 保留分裂结果
- Topology / Relation
  - 区分 touching / intersecting / contains / equal
  - shared-edge / 共线重叠不再一律误判为 crossing
  - 更严格的 interior 判定，并加入边中点辅助
  - topology 中对 equal duplicate polygon 使用确定性的 parent tie-break

## 当前 2D 稳定性小结

2D 全部完成，不再有已知 gap：
- Boolean：crossing / containment / equal / touching / simple-overlap / ultra-thin repeated-overlap 全部稳定
- Offset：ring rebuild / reverse-edge / hole semantics / `OffsetToMultiPolygon` narrow bridge split 全部稳定
- Topology/Relation：touching / intersecting / contains / equal / shared-edge 判定全部稳定
- `tests/gaps/` 中 2D 相关文件已清空

## 下次开始时优先阅读的文档

- `docs/next-task-prompt.md`
- `docs/delphi-geometry-parity.md`
- `docs/sdk-type-design-review.md`
- `docs/test-capability-coverage.md`
- `docs/design-doc-sync-tracker.md`

如果新问题暴露 2D 缺口，直接查看：

- `src/sdk/GeometryBoolean.cpp` / `src/sdk/GeometryOffset.cpp` / `src/sdk/GeometryTopology.cpp`
- `tests/capabilities/test_relation_boolean.cpp` / `test_offset.cpp` / `test_topology_indexing.cpp`

## 手工验证工作流

当需要反馈时：

- 先明确最小有用的构建 / 测试步骤
- 用户在本地运行
- 用户粘贴相关输出
- 基于精确结果继续补丁

期望用户提供的输出：

- 对于编译失败：
  - 目标名
  - 首个失败文件与行号
  - 主要错误文本
  - 若方便，前后约 20 行上下文
- 对于测试失败：
  - 可执行名或 ctest target 名
  - 失败测试用例名
  - 断言文本
  - 相关 stdout / stderr
- 对于验证成功：
  - 运行了哪个 target / test
  - 是否干净通过

## 当前 3D 设计状态

磁盘上已具备以下 3D 设计层：

- 整体库方向
- 基础值类型
- 参数曲线 / 曲面层
- polyhedron / BRep 拓扑层
- triangle mesh / conversion 层
- 共享 3D service 层
- validation / healing 层
- 第一阶段实现路线图
- section / tessellation / validation 集成
- module / package 布局

关键 3D 结论：

- 2D API 收敛规则仍应约束 3D 命名与分层
- `Plane` 与 `PlaneSurface` 应保持分离
- `Curve3d` / `Surface` 是核心参数协议
- 在做 BRep 算法前，必须先规划 `CurveOnSurface` 与 preimage curve
- `PolyhedronBody` 与 `BrepBody` 不应过早合并
- `BrepCoedge` 是必需项，不是可选项

## 当前 3D 代码状态

当前 3D 代码已不再只停留在值类型与基础 service：

- Phase A 基础值类型已在 `include/types/` 落盘
- 已有 3D 基础服务：
  - `src/sdk/GeometryProjection3d.cpp`
  - `src/sdk/GeometryIntersection3d.cpp`
  - `src/sdk/GeometryRelation3d.cpp`
- Phase B 参数对象核心已补上最小 SDK 面：
  - `include/sdk/Curve3d.h`
  - `include/sdk/Surface.h`
  - `include/sdk/LineCurve3d.h`
  - `include/sdk/PlaneSurface.h`
  - `GeometryProjection` 已加入 `PolyhedronFace3d -> Polygon2d` 的局部平面投影入口
- `trianglemesh-core` 已起步：
  - `include/sdk/TriangleMesh.h`
  - `GeometryValidation` 已加入 `TriangleMesh` 的最小 validation 结果
  - `GeometryTessellation` 已加入 `PlaneSurface -> TriangleMesh` 的最小网格化入口
  - `GeometryMeshConversion` 已加入平面 `PolyhedronFace3d / PolyhedronBody -> TriangleMesh` 的最小转换入口
  - 带孔平面 face 现已通过 projected 2D polygon bridge + hole-bridging triangulation 进入 mesh conversion
  - `GeometryMeshOps` 已加入 triangle normal / vertex normal / triangle adjacency 查询
  - `GeometryMeshOps` 已加入 boundary edge 提取与 closed-mesh 判断
  - `GeometryMeshOps` 已加入 boundary loop 提取
  - `GeometryMeshOps` 已加入 connected components / non-manifold edge 查询
  - `GeometryMeshOps` 已加入 orientation consistency 与 shell 分组查询
  - `GeometryMeshRepair` 已加入流形 triangle mesh 的最小 consistent-orientation repair
  - `GeometryMeshRepair` 已加入 single planar boundary loop 的最小 closing repair
  - `GeometryMeshRepair` 已加入 multi planar boundary loops 的最小批量 closing repair
- `polyhedron-core` 已起步：
  - `include/sdk/PolyhedronLoop3d.h`
  - `include/sdk/PolyhedronFace3d.h`
  - `include/sdk/PolyhedronBody.h`
  - `GeometryValidation` 已加入 `PolyhedronBody` 的最小 validation 结果
  - `GeometrySection` 已加入 `Plane x PolyhedronBody -> projected 2D polygon` 的最小截面入口
- `include/sdk/GeometryTypes.h` 新增：
  - `CurveEval3d`
  - `SurfaceEval3d`
- `include/sdk/GeometryApi.h` 已纳入新的参数对象层头文件
- `tests/capabilities/test_sdk.cpp` 已加入最小 3D capability 覆盖：
  - `LineCurve3d`
  - `PlaneSurface`
  - `CurveEval3d`
  - `SurfaceEval3d`
  - `TriangleMesh`
  - `TriangleNormal` / `VertexNormal` / `TriangleAdjacency`
  - `ExtractBoundaryEdges` / `ExtractBoundaryLoops` / `IsClosedTriangleMesh`
  - `ExtractNonManifoldEdges` / `IsManifoldTriangleMesh` / `ComputeTriangleConnectedComponents`
  - `IsConsistentlyOrientedTriangleMesh` / `ComputeMeshShells`
  - `OrientTriangleMeshConsistently(...)` / `CloseSinglePlanarBoundaryLoop(...)` / `ClosePlanarBoundaryLoops(...)`
  - `MeshValidation3d`
  - `Validate(PolyhedronSection3d, ...)`
  - `Tessellate(PlaneSurface, ...)`
  - `ConvertToTriangleMesh(PolyhedronFace3d / PolyhedronBody, ...)`
  - `Section(PolyhedronBody, Plane, ...)`
  - `BuildSectionTopology(...)`
  - `BuildSectionComponents(...)`
  - `RebuildSectionFaces(...)`
  - `RebuildSectionBody(...)`
  - `RebuildSectionBodies(...)`
  - `ConvertSectionToTriangleMesh(...)`
  - `ClassifySectionContent(...)`
  - `ProjectFaceToPolygon2d(...)`
  - `PolyhedronLoop3d`
  - `PolyhedronFace3d`
  - `PolyhedronBody`
  - `PolyhedronValidation3d`

当前重要约束：

- 这批 3D 参数对象层目前是最小可用协议，不包含高阶 NURBS 或复杂投影 API
- `PlaneSurface` 当前按有限参数域平面曲面实现，便于后续 bounds / tessellation / section 直接消费
- `TriangleMesh` 已接上 `PlaneSurface -> TriangleMesh` 的最小 tessellation 路径
- `TriangleMesh` 已接上 triangle normal / vertex normal / triangle adjacency 的最小查询层
- `TriangleMesh` 已接上 boundary edge 提取与 closed-mesh 判断
- `TriangleMesh` 已接上 boundary loop 提取
- `TriangleMesh` 已接上 connected components / non-manifold edge 的最小查询层
- `TriangleMesh` 已接上 orientation consistency 与 shell 分组的最小查询层
- `TriangleMesh` 已接上流形场景下的最小 consistent-orientation repair
- `TriangleMesh` 已接上 single planar boundary loop 的最小 closing repair
- `TriangleMesh` 已接上 multi planar boundary loops 的最小批量 closing repair
- `PolyhedronBody` 已接上平面 face / body 到 `TriangleMesh` 的最小 conversion 路径
- `PolyhedronFace3d` 已接上局部平面 `Polygon2d` 投影入口，开始进入 projected 2D polygon workflow
- `PolyhedronBody` 已接上 `Plane x PolyhedronBody` 的最小 section 路径，当前支持横切 plane-dominant body、与平面 face 共面的截面回收、以及 edge-only 零面积截面的开链返回
- `GeometrySection` 已接上最小 face rebuild 入口，可将闭合 section polygons 回建为 `PolyhedronFace3d`
- `GeometrySection` 已接上最小 BRep face/body rebuild 入口，可将 plane-dominant section 结果直接回建为 `BrepFace` / `BrepBody`
- `GeometrySection` 已接上多 root `BrepBody` set rebuild，可将分离的 area section 直接拆成多个 `BrepBody`
- `tests/capabilities/test_3d_brep.cpp` 已新增双立方体截面多组件重建用例，验证 `RebuildSectionBrepBodies(...)` 返回 2 个独立 body
- `tests/capabilities/test_3d_brep.cpp` 已新增最小 coedge-loop 编辑链路用例：`InsertCoedge(...) -> FlipCoedgeDirection(...) -> RemoveCoedge(...)`
- `GeometrySection` 已接上最小 face merge 语义，可将嵌套 section polygons 合并成带孔 `PolyhedronFace3d`
- `GeometrySection` 的 face rebuild 结果已保留 polygon-to-face 映射，可追溯 outer / hole 来源
- `GeometrySection` 已接上最小 body rebuild 入口，可将 merged section faces 直接组织为 `PolyhedronBody`
- `GeometrySection` 已接上多 root body-set rebuild 入口，可将分离的 area section 拆成多个 `PolyhedronBody`
- `GeometrySection` 已接上最小 topology 入口，可显式给出 section polygons 的 parent / child / depth 关系
- `GeometrySection` 已接上最小 component 入口，可按 root 将 section polygons / rebuilt faces 分组
- `GeometrySection` 已接上最小 mesh conversion 入口，可直接将闭合 section 结果转成 `TriangleMesh`
- `GeometrySection` 的 body-set / mesh-set 结果已保留 root polygon 映射，便于上层回溯 component 来源
- `GeometrySection` 已接上最小内容分类入口，可区分 `Empty / Curve / Area / Mixed`
- `GeometryValidation` 已接上 `PolyhedronSection3d` 的最小 validation 结果
- 3D 参数对象已扩到 `NurbsCurve3d`、`NurbsSurface`、`RuledSurface`、`OffsetSurface`，并具备最小可消费的 `PointAt / Evaluate / Bounds / Clone` 能力
- `CurveOnSurface` 已具备最小支持曲面映射能力，可返回 UV 点、映射 3D 点与 bounds
- `GeometryProjection` 已接上通用 `ProjectPointToSurface(...)`，当前对 `PlaneSurface` 走解析投影，对一般 `Surface` 走采样 + 局部细化
- `GeometryProjection` 已接上 `ProjectPointToBrepFace(...)`，当前优先走支撑曲面投影 + trim containment，落到 trim 外时保守回退到 trim polyline 边界最近点
- `GeometryProjection` 已接上 `ProjectPointToBrepBody(...)`，当前按 face 聚合最短投影，返回命中的 `faceIndex`
- `GeometryProjection` 已接上 `ProjectPointToBrepEdge(...)`，当前对 `LineCurve3d` 走解析投影，对一般 `Curve3d` 走采样 + 局部细化
- `GeometryProjection` 已接上 `ProjectPointToBrepVertex(...)`
- `GeometryProjection` 已接上通用 `ProjectPointToCurve(...)`，当前对 `LineCurve3d` 走解析投影，对一般 `Curve3d` 走采样 + 局部细化
- `GeometryProjection` 已接上 `ProjectPointToCurveOnSurface(...)`，当前按 3D polyline segment 做最近点投影，并回写 UV
- `GeometryProjection` 已接上 `ProjectPointToTriangleMesh(...)`，当前逐 triangle 做最近点投影
- `GeometryProjection` 已接上 `ProjectPointToPolyhedronFace(...)` / `ProjectPointToPolyhedronBody(...)`，当前优先走支撑平面投影 + projected 2D polygon containment，落到面外时保守回退到边界最近点
- `GeometryRelation3d` 已接上 `LocatePoint(point, PolyhedronFace3d, ...)` 与 `LocatePoint(point, BrepFace, ...)`
- `GeometryRelation3d` 已接上 `LocatePoint(point, PolyhedronBody, ...)`
- `GeometryRelation3d` 已接上 `LocatePoint(point, TriangleMesh, ...)`
- `GeometryRelation3d` 已接上 `LocatePoint(point, BrepBody, ...)`
- `GeometryRelation3d` 已接上 `LocatePoint(point, Curve3d, ...)` 与 `LocatePoint(point, CurveOnSurface, ...)`
- `GeometryMeasure` 已接上 `Distance(point, surface)` / `DistanceSquared(point, surface)`，可直接消费 `PlaneSurface` 与当前最小 `NurbsSurface`
- `GeometryMeasure` 已接上 `Distance(point, Curve3d)` / `DistanceSquared(point, Curve3d)`
- `GeometryMeasure` 已接上 `Distance(point, CurveOnSurface)` / `DistanceSquared(point, CurveOnSurface)`
- `GeometryMeasure` 已接上 `Distance(point, TriangleMesh)` / `DistanceSquared(point, TriangleMesh)`
- `GeometryMeasure` 已接上 `Distance(point, PolyhedronFace)` / `DistanceSquared(point, PolyhedronFace)`
- `GeometryMeasure` 已接上 `Distance(point, PolyhedronBody)` / `DistanceSquared(point, PolyhedronBody)`
- `GeometryIntersection3d` 已接上 `Intersect(Line3d, PolyhedronFace, ...)` / `Intersect(Line3d, PolyhedronBody, ...)`，当前走支撑平面求交 + projected 2D polygon containment
- `GeometryIntersection3d` 已接上 `Intersect(Line3d, Curve3d, ...)`，当前对 `LineCurve3d` 走解析求交，对一般 `Curve3d` 走保守采样
- `GeometryIntersection3d` 已接上 `Intersect(Line3d, CurveOnSurface, ...)`，当前按 3D polyline segment 做逐段求交
- `GeometryIntersection3d` 已接上 `Intersect(Line3d, TriangleMesh, ...)`，当前逐 triangle 做 plane hit + triangle containment
- `GeometryIntersection3d` 已接上 `Intersect(Line3d, BrepVertex, ...)`
- `GeometryIntersection3d` 已接上 `Intersect(Plane, Curve3d, ...)`，当前对 `LineCurve3d` 走解析求交，对一般 `Curve3d` 走采样符号变号定位
- `GeometryIntersection3d` 已接上 `Intersect(Plane, CurveOnSurface, ...)`，当前按 3D polyline segment 做逐段平面穿越检测，并回写 UV
- `GeometryIntersection3d` 已接上 `Intersect(Plane, BrepVertex, ...)`
- `GeometryIntersection3d` 已接上 `Intersect(Plane, BrepEdge, ...)`，当前复用 `plane -> curve` 路径
- `GeometryMeasure` 已接上 `Distance(point, BrepFace)` / `DistanceSquared(point, BrepFace)`，可直接消费 planar / trimmed non-planar `BrepFace`
- `GeometryMeasure` 已接上 `Distance(point, BrepVertex)` / `DistanceSquared(point, BrepVertex)`
- `GeometryMeasure` 已接上 `Distance(point, BrepBody)` / `DistanceSquared(point, BrepBody)`，当前复用 `ProjectPointToBrepBody(...)`
- `GeometryMeasure` 已接上 `Distance(point, BrepEdge)` / `DistanceSquared(point, BrepEdge)`
- `GeometryMeasure` 已接上 `Bounds(PolyhedronFace3d)` / `Bounds(BrepVertex)` / `Bounds(BrepEdge)` / `Bounds(BrepFace)`
- `GeometryIntersection3d` 已接上 `Intersect(Line3d, BrepFace, ...)` 与 `Intersect(Line3d, BrepBody, ...)`，当前复用 `line -> surface` 再走 trim containment / 多 face 聚合
- `GeometryIntersection3d` 已接上 `Intersect(Line3d, BrepEdge, ...)`，当前对 `LineCurve3d` 走解析求交，对一般 `Curve3d` 走保守采样
- `Validate(BrepBody, ...)` 已补最小 edge-use adjacency 校验：当前会拒绝 0 次引用或超过 2 次引用的 edge
- `GeometryIntersection3d` 已接上 `Intersect(Line3d, Surface, ...)`，当前对 `PlaneSurface` 走解析求交，对一般 `Surface` 走保守采样 + 局部细化
- `BrepVertex`、`BrepEdge`、`BrepCoedge`、`BrepLoop`、`BrepFace`、`BrepShell`、`BrepBody` 已不再只是名字 skeleton，已具备最小 topology ownership、bounds、validation 与 conservative healing 入口
- `GeometryBrepConversion` 已新增 `ConvertToBrepBody(PolyhedronBody, ...)` 最小 conversion 入口（plane-surface + line-edge 主导）
- `GeometryBrepEditing` 已新增最小 loop 编辑入口：`InsertCoedge(...)` / `RemoveCoedge(...)` / `FlipCoedgeDirection(...)`
- 3D 服务层公开函数名已补齐并落了最小实现：`GeometryMeasure` / `GeometryHealing` / `Validate(BrepBody, ...)`
- `BrepBody` 已接上受限 `TriangleMesh` conversion，当前支持 plane-surface + line-edge 主导的平面 BRep face
- `tests/capabilities/test_3d_healing.cpp` 已新增 `Heal(BrepBody)` 缺失 trim 回填 capability
- `tests/capabilities/test_3d_section.cpp` 已新增 non-axis-aligned multi-face 截面的稳定 contour count capability
- `tests/capabilities/test_3d_healing.cpp` 已扩展 holed-face 缺失 outer/hole trims 同步回填 capability
- `tests/capabilities/test_3d_conversion.cpp` 已扩展 affine-skew 非轴对齐 `PolyhedronBody` 的 `ConvertToBrepBody(...)` capability
- `tests/capabilities/test_3d_conversion.cpp` 已扩展 support-plane mismatch 输入的 `ConvertToBrepBody(...)` repair capability（support-plane refit）
- `tests/capabilities/test_3d_healing.cpp` 已扩展 `Heal(..., policy=Aggressive)`：open planar single/multi-face sheet 与 holed shell 的确定性闭壳子策略
- `tests/capabilities/test_3d_healing.cpp` 已扩展 aggressive+trim-backfill 组合子场景：open holed shell 且 trims 缺失时可协同稳定修复
- `tests/capabilities/test_3d_conversion.cpp` 已扩展 mild non-planar loop 输入的 `ConvertToBrepBody(...)` repair capability（refit-plane 投影）
- `tests/capabilities/test_3d_conversion.cpp` 已扩展 mild non-planar hole loop 输入的 `ConvertToBrepBody(...)` repair capability（refit-plane 投影）
- `tests/capabilities/test_3d_conversion.cpp` 已新增 planar holed `BrepBody -> TriangleMesh` 的面积保持子场景
- `tests/capabilities/test_3d_conversion.cpp` 已扩展 leading collinear loop 顶点输入的 `ConvertToBrepBody(...)` 稳健法向回退子场景
- `tests/capabilities/test_3d_conversion.cpp` 已扩展 duplicate loop 顶点输入的 `ConvertToBrepBody(...)` 归一化修复子场景
- `tests/capabilities/test_3d_conversion.cpp` 已扩展 duplicate hole loop 顶点输入的 `ConvertToBrepBody(...)` 归一化修复子场景
- `tests/capabilities/test_3d_conversion.cpp` 已扩展 conversion 组合病理输入子场景，验证 refit/fallback/projection/normalization 可叠加修复
- `tests/capabilities/test_3d_conversion.cpp` 已扩展 planar multi-face `BrepBody -> TriangleMesh` 的面积保持子场景
- `tests/capabilities/test_3d_conversion.cpp` 已扩展 planar holed+multi-face `BrepBody -> TriangleMesh` 的面积保持子场景
- `BrepFace` 已接上基于 trim/UV triangulation 的最小 `TriangleMesh` conversion
- `GeometryMeasure::Area(BrepFace)` 已改为消费 `BrepFace -> TriangleMesh`，因此不再局限于平面 trim 面积
- 已新增 `GeometryBrepConversion`，支持 `BrepFace -> PolyhedronFace3d` 与 `BrepBody -> PolyhedronBody` 的平面 trim 回建桥
- `ConvertToPolyhedronBody(BrepBody, ...)` 已带 topology fallback：对 trim 缺失但 plane-surface + line-edge 的 planar body，仍可直接回建 `PolyhedronBody`
- `BrepBody` 已接上最小 `Section(BrepBody, Plane, ...)`，支持 coplanar planar face 回收与一般 mesh-slice 路径
- `GeometryMeasure` 已扩到 `CurveOnSurface` / `BrepFace` / `BrepBody`
- `Heal(BrepBody, ...)` 已可为 plane-surface + line-edge 主导的 planar face 自动回填缺失 trim/preimage，并保守重建 shell closed 标记
- 带孔 `PolyhedronFace3d` 已可经由 projected 2D polygon 工作流转成 `TriangleMesh`
- 当前 `GeometrySection` 仍是保守入口：最小 body rebuild 已补上，但 richer section topology 与更复杂 merge 语义仍未补
- `BrepBody` 已具备最小对象层 / validation / conservative healing / mesh conversion / section / capability 覆盖，但仍只覆盖 plane-surface + line-edge 主导场景，尚未进入实质 BRep 算法

## 推荐的下一个 3D 动作（第二阶段）

上次会话已完成 3D 第一阶段基础 capability（section / brep rebuild / healing / conversion 最小闭环），并新增：
- `tests/support/Fixtures3d.h`：共享 `BuildUnitCubeBody()` fixture
- `tests/capabilities/test_3d_section.cpp`：倾斜截面 Section + Topology + Components
- `tests/capabilities/test_3d_brep.cpp`：RebuildSectionBrepBody 单面 BrepBody
- `tests/capabilities/test_3d_healing.cpp`：保守 Heal(PolyhedronBody) 幂等
- `tests/capabilities/test_3d_conversion.cpp`：ConvertToTriangleMesh 12 triangles / area≈6.0

当前 3D gap（见 `tests/gaps/`）：
- section：non-planar dominant contour stitching、coplanar fragment merge
- brep：coedge-loop editing workflow、non-planar trimmed face repair
- healing：aggressive shell repair、multi-step mesh/body joint healing
- conversion：high-fidelity Brep→mesh、general non-planar polyhedron→Brep

当前最合理的第二阶段推进顺序：
1. 扩展 multi-component section graph（`BuildSectionComponents` / `RebuildSectionBrepBodies`）
2. 扩展 `Heal(BrepBody)` 平面 trim 回填能力并转正为新 capability test
3. 补 `ConvertToBrepBody(PolyhedronBody)` 能力并转正为 capability test
4. 逐步向 non-planar 场景推进（brep coedge-editing / mesh stitching）

详见 `docs/next-task-prompt.md`。
