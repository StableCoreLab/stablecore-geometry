# 设计文档同步跟踪

## 目的

这份文档用于跟踪历史 2D 设计文档与当前代码 / API 状态之间的同步修复工作。

## 状态说明

- `pending`：尚未开始
- `in_progress`：正在处理中
- `done`：当前轮同步已完成

## 文件列表

### `docs/segment-design.md`

- status: `done`
- notes:
  - 已按当前 `Segment2 / LineSegment2 / ArcSegment2` API 与设计结论重写
  - 已同步成员方法与自由函数边界

### `docs/polyline-design.md`

- status: `done`
- notes:
  - 已按当前 `Polyline2 / sdk::Polyline2d / MultiPolyline2d` API 与设计结论重写
  - 已同步 `SegmentCount / VertexCount / VertexAt / StartPoint / EndPoint / Bounds / PointAt / PointAtLength`

### `docs/polygon-design.md`

- status: `done`
- notes:
  - 已按当前 `Polygon2 / sdk::Polygon2d / MultiPolygon2d` API 与设计结论重写
  - 已同步 `OuterRing / HoleCount / HoleAt / Bounds`

### `docs/box-design.md`

- status: `done`
- notes:
  - 已按当前 `Box2` API 与设计结论重写
  - 已同步 `MinPoint / MaxPoint / Bounds` 相关命名

### `docs/sdk-type-design-review.md`

- status: `done`

### `docs/sdk-2d-api-convergence.md`

- status: `done`

### `docs/session-handoff.md`

- status: `done`

### `docs/api-member-free-function-checklist.md`

- status: `done`

### `docs/delphi-geometry-parity.md`

- status: `done`
- notes:
  - 已同步共享预处理入口 `NormalizePolygonByLines` 在 boolean / offset / topology 的接入状态
  - 已同步布尔近退化交点参数聚类压缩、共线族折叠与带孔 `CutPolygon` 裁剪策略
  - 已同步偏移语义翻转恢复（单多边形与多多边形）与孔洞感知 `Contains` 修复

## 进度备注

- 2026-03-27：
  - 已完成 `segment / polyline / polygon / box` 四份 2D 设计文档重写
  - 已将成员方法 vs 自由函数边界规则固定到独立清单与 API 收敛文档
- 2026-04-01：
  - 已完成 `delphi-geometry-parity` 与本轮代码实现对齐，覆盖共享预处理、布尔鲁棒性、偏移恢复与拓扑包含修复
  - 已补充对应能力回归测试覆盖项说明（relation / offset / pathops / topology）
  - 已将 below-tolerance arrangement degeneracy 从 `tests/gaps` 提升到 `tests/capabilities`，并通过定向 gtest 验证
  - 已补充 `NormalizePolygonByLines(...)` 的多级容差重建与边界简化回退，提升近退化 polygon operand 归一化稳定性
  - 已修复本机构建缓存中的过期 MSVC 路径，恢复 capability / gap 测试目标重生成与重建
  - 已将单多边形带孔 offset 语义恢复从 `tests/gaps` 提升到 `tests/capabilities`，并通过定向 gtest 验证
  - 已将一组 branch-scoring / fake-edge 误选场景从 `tests/gaps` 提升到 `tests/capabilities`，验证当前 SearchPoly 候选排序可稳定保留主轮廓
  - 已修正 `Contains(...)` 对 boundary T-junction 的过度保守短路，令 shared-boundary contained 场景可稳定落到 `FirstContainsSecond`
  - 已将一组 reverse-edge / self-intersection offset 恢复场景从 `tests/gaps` 提升到 `tests/capabilities`，并通过定向 gtest 验证
  - 已新增 `OffsetToMultiPolygon(const Polygon2d&, ...)`，补齐单 polygon 内缩发生分裂时的 multipolygon 输出能力
  - 已将 narrow-bridge split 从 `tests/gaps` 提升到 `tests/capabilities`，并通过定向 gtest 验证
  - 已完成 3D gap 骨架落地：新增 section / brep / healing / conversion 四组 3D gap tests，并接入 `stablecore_geometry_gap_gtest`
  - 当前 `gap` 目标已从“空套件”切换为“3D P1 骨架套件”（2D gaps 已清空，3D gaps 已建档）
  - 已新增 3D capability：倾斜切平面下 `Section + BuildSectionTopology + BuildSectionComponents` 单区域闭环验证，section gap 收敛为更高阶歧义 stitching / merge 语义  - 已新增 `tests/support/Fixtures3d.h`，提供共享 `BuildUnitCubeBody()` 单位立方体 fixture
  - 已扩展 section 子能力：coplanar 相邻 face fragment 在 `Section(...)` 中可归并为单 polygon，收窄 face-merge gap 到更高阶歧义 fragment 语义
  - 已新增 `test_3d_brep.cpp`：`RebuildSectionBrepBody` 单面 BrepBody rebuild 转正为 capability
  - 已扩展 brep editing 子能力：最小 loop->face->shell->body ownership-consistent replacement workflow 已转正为 capability
  - 已新增 `test_3d_healing.cpp`：保守 `Heal(PolyhedronBody)` 幂等性转正为 capability
  - 已新增 `test_3d_conversion.cpp`：`ConvertToTriangleMesh(PolyhedronBody)` 12 triangles / area≈6.0 转正为 capability
- 2026-04-02：
  - 已扩展 `Heal(BrepBody)` 能力验证：plane-surface + line-edge 且缺失 trim 的 `BrepFace` 可稳定回填 outer trim
  - 已补齐 `ConvertToBrepBody(PolyhedronBody)` 最小 conversion API 与 capability test，单位立方体可转换为有效 `BrepBody` 且 `FaceCount()==6`
  - 已新增 `RebuildSectionBrepBodies(...)` 多组件能力验证：双立方体截面可稳定重建为 2 个独立 `BrepBody`
  - 已新增 non-axis-aligned multi-face section capability：`x+y+z=1.5` 截单位立方体时 contour/polygon 计数稳定
  - 已补齐最小 coedge-loop editing API 与 capability：`InsertCoedge(...)` / `FlipCoedgeDirection(...)` / `RemoveCoedge(...)` 非平凡编辑链路可稳定 round-trip
  - 已新增 conversion 子能力收敛：affine-skew 非轴对齐 `PolyhedronBody` 可稳定 `ConvertToBrepBody(...)`
  - 已新增 conversion repair 子能力收敛：support-plane mismatch 的 `PolyhedronBody` 可通过 support-plane refit 稳定转换到 `BrepBody`
  - 已新增 conversion repair 子能力收敛：mild non-planar face loop 可通过 refit-plane 投影修复后稳定转换到 `BrepBody`
  - 已扩展 conversion repair 子能力：mild non-planar hole loop 也可通过 refit-plane 投影修复后稳定转换到 `BrepBody`
  - 已新增 conversion 子能力收敛：planar holed `BrepBody` 到 `TriangleMesh` 的面积保持子场景（outer-hole）
  - 已扩展 conversion repair 子能力：leading collinear loop 顶点输入下，`ConvertToBrepBody` 可稳定回退到非共线三点法向拟合
  - 已扩展 conversion repair 子能力：duplicate loop 顶点输入下，`ConvertToBrepBody` 可先做 loop 归一化再执行 refit
  - 已扩展 conversion repair 子能力：duplicate hole loop 顶点输入下，`ConvertToBrepBody` 同样可先归一化再执行 refit
  - 已新增 conversion 组合子能力：support-plane refit / collinear fallback / outer-hole projection / duplicate normalization 在单 face 病理输入下可叠加稳定生效
  - 已扩展 conversion repair 子能力：tiny-scale non-planar outer loop 输入下，`ConvertToBrepBody` 可通过 scale-aware 法向回退稳定完成 refit 修复
  - 已扩展 conversion repair 子能力：tiny-scale non-planar holed face 输入下，`ConvertToBrepBody` 同样可稳定执行 outer/hole 投影修复
  - 已扩展 conversion repair 子能力：tiny-scale non-planar multi-face 输入下，`ConvertToBrepBody` 可稳定逐面 refit 并保持 `FaceCount` 一致
  - 已扩展 conversion repair 子能力：tiny-scale non-planar mixed-content（holed+plain）multi-face 输入下，`ConvertToBrepBody` 可稳定逐面修复并保持拓扑计数稳定
  - 已扩展 conversion repair 子能力：tiny-scale non-planar shared-edge 邻接面输入下，`ConvertToBrepBody` 可稳定逐面修复并保持面计数稳定
  - 已扩展 conversion repair 子能力：tiny-scale non-planar shared-edge 邻接链输入下，`ConvertToBrepBody` 可在多邻接面场景下稳定逐面修复并保持面计数稳定
  - 已扩展 conversion repair 子能力：tiny-scale non-planar shared-edge 邻接链 mixed-content 输入下，`ConvertToBrepBody` 可稳定逐面修复并保持拓扑计数稳定
  - 已扩展 conversion repair 组合子能力：shared-edge 邻接链下，duplicate-loop-normalization 与逐面 refit/projection 可稳定叠加生效
  - 已扩展 conversion repair 组合子能力：shared-edge 邻接链 mixed-content 下，duplicate hole-loop normalization 与逐面 refit/projection 可稳定叠加生效
  - 已扩展 conversion repair 组合子能力：shared-edge 邻接链 mixed-content 下，collinear-leading normal fallback 与 duplicate/hole normalization 可稳定叠加
  - 已扩展 conversion repair 组合子能力：shared-edge 邻接链 mixed-content 下，support-plane refit 可与逐面 projection/normalization 稳定叠加
  - 已扩展 conversion repair 组合子能力：shared-edge 邻接链 mixed-content 下，support-plane mismatch refit 与 collinear-leading fallback 可稳定叠加
  - 已扩展 conversion repair 组合子能力：shared-edge 邻接链 mixed-content 下，support-plane mismatch + collinear-leading + duplicate-hole normalization 可稳定叠加
  - 已扩展 conversion repair 组合子能力：shared-edge 邻接链 mixed-content full-composition 下，outer/hole 双重复顶点 normalization 可与 support-plane mismatch + collinear-leading 稳定叠加
  - 已扩展 conversion repair 子能力：shared-edge 邻接链修复后可全局复用共享顶点/边，避免 face-local 重复拓扑并收敛到共享边一致性子集
  - 已扩展 conversion 子能力：closed-shell 代表性输入（单位立方体）转换后可收敛到共享拓扑 Brep（8 vertices / 12 edges / closed shell）
  - 已扩展 conversion repair 子能力：tiny-scale closed-shell tetrahedron（4 triangular faces, mismatched support planes）经 per-face refit 修复后可收敛为合法 closed BrepBody（IsClosed=true / VertexCount=4 / EdgeCount=6）
  - 已扩展 Brep->mesh 子能力：planar multi-face `BrepBody` 的 representative area-preserving conversion capability
  - 已扩展 Brep->mesh 子能力：planar holed+multi-face 混合 `BrepBody` 的 representative area-preserving conversion capability
  - 已扩展 Brep->mesh 子能力：planar shared-edge 相邻面转换时可全局复用共享 3D 顶点，收敛到 shared-edge feature-preserving 子集
  - 已新增 healing 子能力收敛：带孔平面 `BrepFace` 缺失 outer/hole trims 时可被 `Heal(BrepBody)` 同步回填
  - 已新增 aggressive healing 子能力收敛：`Heal(..., policy=Aggressive)` 支持 open planar single-face shell 的确定性闭壳修复
  - 已扩展 aggressive healing 子能力：覆盖 open planar multi-face sheet 的确定性闭壳修复
  - 已扩展 aggressive healing 子能力：覆盖 shared-edge 邻接的 open planar multi-face sheet 的确定性闭壳修复
  - 已扩展 aggressive healing 子能力：覆盖 open planar holed shell 的确定性闭壳修复
  - 已扩展 aggressive healing 组合子能力：在 open holed shell 且 trims 缺失场景下，闭壳与 trim-backfill 可协同稳定生效
  - 已扩展 aggressive healing 子能力：同一 `BrepBody` 中多个可恢复 open shell 可在单次 healing 中确定性闭壳
  - 已扩展 aggressive healing 子能力：mixed closed/open-shell 输入下，已闭壳保持稳定且可恢复 open shell 可被单次 deterministic 闭壳
  - 已扩展 aggressive healing 子能力：mixed open-shell 输入下支持部分修复（eligible shell 闭壳，ineligible shell 保持 open）
  - 已扩展 aggressive healing 子能力：三壳 mixed 输入下可保持 deterministic（closed shell 保持稳定，eligible open shell 闭壳，ineligible open shell 保持 open）
  - 已扩展 aggressive healing 组合子能力：三壳 mixed 输入下可与 trim-backfill 协同（eligible open shell 先回填 trims 再闭壳）
  - 已扩展 aggressive healing 子能力：三壳 mixed 输入下可对 eligible multi-face open-sheet 执行确定性闭壳，同时保持 closed/ineligible shell 状态稳定
  - 已扩展 aggressive healing 组合子能力：三壳 mixed 输入下可对 eligible holed shell（缺失 outer/hole trims）执行回填后闭壳
  - 已扩展 aggressive healing 组合子能力：三壳 mixed 输入下可对 eligible multi-face holed shell（缺失 trims）执行回填后闭壳
  - 已扩展 aggressive healing 组合子能力：三壳 mixed 输入下可对 eligible multi-face（holed+plain）两面 trims 缺失场景执行回填后闭壳
  - 已扩展 aggressive healing 组合子能力：三壳 mixed 输入下可对 eligible multi-face holed shell 的 support-plane mismatch + missing trims 执行回填后闭壳
  - 已扩展 aggressive healing 组合子能力：support-plane mismatch 的 eligible shell 与 ineligible multi-face shell 共存时，仍可保持 deterministic 闭壳/保留 open 行为
  - 已扩展 aggressive healing 组合子能力：support-plane mismatch + trim-backfill 的 eligible shell 与 ineligible multi-face shell 共存时，仍可保持 deterministic 闭壳/保留 open 行为
  - 已扩展 aggressive healing 组合子能力：support-plane mismatch 的 eligible holed shell（缺失 outer/hole trims）与 ineligible multi-face shell 共存时，仍可保持 deterministic 回填后闭壳/保留 open 行为
  - 已扩展 aggressive healing 组合子能力：support-plane mismatch 的 eligible multi-face（holed+plain，缺失 trims）与 ineligible multi-face shell 共存时，仍可保持 deterministic 回填后闭壳/保留 open 行为
## 对齐完成定义（DoD）

### 2D 对齐完成

- 所有 2D gap 用例不再 `GTEST_SKIP()`，并转入 `tests/capabilities`
- boolean / offset / pathops / topology 共享统一预处理与容差语义，不出现模块间结果漂移
- relation 层级可稳定区分 touching / intersecting / contains / hole-diff，包含同外环不同孔场景

### 3D 对齐完成

- 从第一阶段最小 workflow 升级为 section -> rebuild -> validation -> healing -> conversion 稳定闭环
- `BrepBody` 从 skeleton 发展为可用拓扑一致性与修复链路，覆盖非平面主导场景
- 为 3D 尚未补齐能力建立 `tests/gaps` 体系，避免仅文档跟踪

## 燃尽清单（优先级）

### P0

- 2D boolean 深退化恢复：已关闭 arrangement 低容差退化缺口并转正测试
- 2D offset 语义与窄通道稳定性：已关闭 reverse-edge / self-intersection / hole semantics / narrow bridge split，并完成测试转正

### P1

- 2D SearchPoly 分支评分与 fake-edge candidate ranking：当前已补齐一组代表性误选场景，仍需继续向 Delphi 级评分策略逼近
- 2D relation hierarchy 向 Delphi `GGLPolyRelation` 聚合语义补齐：当前已补齐一组 shared-boundary contained 场景，但公开枚举仍未扩展到更细层级
- 3D gap 测试体系落地（section / brep / healing / conversion）：骨架已完成；基础 brep rebuild / healing / conversion 已转正为 capability tests
- 3D 第一阶段向第二阶段推进：P1/P2/P3 与 P4-B 最小子策略已完成；下一步聚焦真正鲁棒 non-planar repair 与更一般 aggressive policy

## 两周执行顺序建议（更新）

- 第 1 周：已完成 2D P0（boolean + offset）清空，以及 3D P1 骨架建立
- 第 2 周：推进 3D 第二阶段——non-planar section / brep coedge-editing / aggressive healing
