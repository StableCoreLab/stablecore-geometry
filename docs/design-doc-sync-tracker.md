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
  - 已扩展 `test_3d_brep.cpp`：`ConvertToBrepBody -> Section(BrepBody) -> RebuildSectionBrepBodies` 端到端双组件路径转正为 capability
  - 已扩展 `test_3d_brep.cpp`：`RebuildSectionBody` 单面 PolyhedronBody rebuild 与 `RebuildSectionBodies` 双组件 Polyhedron 重建转正为 capability
  - 已扩展 `test_3d_brep.cpp`：`RebuildSectionBrepBody` / `RebuildSectionBrepBodies` 输出补齐 open-shell 语义断言（`ShellCount==1` 且 `IsClosed==false`）
  - 已扩展 brep editing 子能力：最小 loop->face->shell->body ownership-consistent replacement workflow 已转正为 capability
  - 已扩展 brep editing 子能力：multi-face closed-shell（unit-cube Brep）上的 no-op `ReplaceOuterLoop -> ReplaceFace -> ReplaceShell` 也可稳定保持 body/shell 有效性与 closed 语义
  - 已新增 `test_3d_healing.cpp`：保守 `Heal(PolyhedronBody)` 幂等性转正为 capability
  - 已新增 `test_3d_conversion.cpp`：`ConvertToTriangleMesh(PolyhedronBody)` 12 triangles / area≈6.0 转正为 capability
- 2026-04-02：
  - 已扩展 `Heal(BrepBody)` 能力验证：plane-surface + line-edge 且缺失 trim 的 `BrepFace` 可稳定回填 outer trim
  - 已补齐 `ConvertToBrepBody(PolyhedronBody)` 最小 conversion API 与 capability test，单位立方体可转换为有效 `BrepBody` 且 `FaceCount()==6`
  - 已新增 `RebuildSectionBrepBodies(...)` 多组件能力验证：双立方体截面可稳定重建为 2 个独立 `BrepBody`
  - 已新增 non-axis-aligned multi-face section capability：`x+y+z=1.5` 截单位立方体时 contour/polygon 计数稳定
  - 已新增 section 子能力：`Section(BrepBody, Plane)` 在 oblique cut（`x+y+z=1.5`）下稳定得到单区域闭合结果（1 polygon / 1 contour / 6 points）
  - 已新增 section 子能力：`Section(BrepBody, Plane)` 在 coplanar 邻接片段输入下稳定合并为单 polygon（area=2.0），补齐 Brep 路径 face-merge 代表性子样例
  - 已新增 section 子能力：`Section(BrepBody, Plane)` 在双组件输入下稳定得到 2 个 area components（2 polygons / 2 roots / 2 components）
  - 已收敛 section gap 文案：`FaceMergeSemanticsAfterSectionRemainsOpen` 现明确仅覆盖“超出 Polyhedron/Brep 相邻 coplanar union 子集”的更高阶语义
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
    - 已修正 quad-chain 共享顶点 VertexCount/EdgeCount 断言（quad 面第4顶点会被独立投影导致偏差 ~O(z_drift)，已从 capabilities 测试中移除错误断言并补充注释）
    - 已扩展 conversion repair 子能力：tiny-scale triangular-face chain（3 triangular faces, mismatched support planes, 两个共享边）在 per-face refit 后共享顶点精确一致（VertexCount=5 / EdgeCount=7），验证三角面片共享边拓扑精确性
    - 已扩展 conversion repair 子能力：tiny-scale triangular-fan（4 triangular faces sharing apex, mismatched support planes）在 per-face refit 后共享 apex 精确一致（VertexCount=5 / EdgeCount=8），验证多面共享顶点拓扑精确性
    - 已实现 conversion repair 启发式增强：shared-vertex-aware support-plane refit（三点组优先包含跨面共享顶点）以降低 shared-edge 顶点跨面投影偏差
    - 已实现 conversion topology 复用增强：source representative-id 贯穿到 Brep vertex 复用，降低修复后微漂移对 shared-edge 去重的影响
    - 已扩展 conversion capability：support-plane mismatch 的 tiny-scale 3-quad shared-edge chain 可稳定保持共享拓扑（VertexCount=8 / EdgeCount=10）
    - 已扩展 conversion capability：support-plane mismatch + duplicate-loop-normalization 的 tiny-scale 3-quad shared-edge chain 也可稳定保持共享拓扑（VertexCount=8 / EdgeCount=10）
    - 已扩展 conversion capability：shared-chain mixed-content 下 duplicate-hole / support-plane mismatch / 二者组合均可稳定保持共享拓扑计数（VertexCount=12 / EdgeCount=14）
    - 已扩展 conversion capability：shared-chain 其余组合子场景（duplicate-loop、collinear-leading、support-mismatch+collinear、full-composition、dual-duplicate full-composition）也已补齐确定性拓扑计数断言（VertexCount=8/13，EdgeCount=10/15）
    - 已扩展 conversion capability：tiny-scale 基础子场景（non-planar multi-face / mixed-content / shared-edge faces / shared-edge-chain mixed-content）均已补齐确定性拓扑计数断言（8/8、12/12、6/7、12/14）
    - 已扩展 conversion capability：早期 repair 子场景（skewed/mismatched/non-planar cube、single-face holed/collinear/duplicate/composite、tiny-scale face/holed-face）均已补齐确定性拓扑计数断言
    - 已扩展 conversion capability：代表性 repair 场景已补齐壳体语义断言（cube-like 输入稳定 closed shell；shared-chain sheet-like 输入稳定 open shell）
    - 已新增 conversion capability：deformed unit cube（单顶点位移，三面同时非平面）可经 per-face refit + representative-id 复用收敛为有效 closed BrepBody（FaceCount=6/VertexCount=8/EdgeCount=12/IsClosed=true）
    - 已新增 conversion capability：dual-deformed unit cube（双顶点位移，六面均非平面）可经 per-face refit + representative-id 复用收敛为有效 closed BrepBody（FaceCount=6/VertexCount=8/EdgeCount=12/IsClosed=true）
    - 已开始 conversion 跨面联合修复预备：representative-id global snapping pass（repair 后聚合 representative 点并回投影到各自 face support plane）已接入
    - 已扩展 conversion capability：near-equal shared-edge（<eps 顶点扰动）输入下，representative-id 全局目标点平均已参与共享 `BrepVertex` 最终落点，避免首面优先导致的共享顶点位置偏置
    - 已扩展 conversion capability：support-plane mismatch + near-equal shared-edge 输入下，`ConvertToBrepBody(...)` 经 refit 后仍可保持 representative-average 共享顶点落点与确定性拓扑计数（VertexCount=6 / EdgeCount=7）
    - 已扩展 conversion capability：support-plane mismatch + near-equal shared-apex triangular-fan 输入下，`ConvertToBrepBody(...)` 经 refit 后仍可保持 representative-average 共享 apex 顶点落点与确定性拓扑计数（VertexCount=5 / EdgeCount=8）
    - 已扩展 conversion capability：support-plane mismatch + near-equal shared-edge-chain（3 faces）输入下，`ConvertToBrepBody(...)` 经 refit 后仍可保持 representative-average 共享顶点落点与确定性拓扑计数（VertexCount=8 / EdgeCount=10）
    - 已扩展 conversion capability：support-plane mismatch + near-equal shared-corner fan（3 faces，仅共享顶点）输入下，`ConvertToBrepBody(...)` 经 refit 后仍可保持 representative-average 共享顶点落点与确定性拓扑计数（VertexCount=8 / EdgeCount=10）
    - 已扩展 conversion capability：support-plane mismatch + near-equal closed-tetra（4 triangular faces）输入下，`ConvertToBrepBody(...)` 经 refit 后仍可保持 representative-average 共享顶点落点与 closed-shell 确定性拓扑计数（VertexCount=4 / EdgeCount=6）
    - 已扩展 conversion capability：support-plane mismatch + near-equal shared-edge-chain（3 faces）且 middle-face duplicate-loop-normalization 输入下，`ConvertToBrepBody(...)` 经 refit 后仍可保持 representative-average 共享顶点落点与确定性拓扑计数（VertexCount=8 / EdgeCount=10）
    - 已扩展 conversion capability：support-plane mismatch + near-equal shared-chain mixed-content（含 middle-face duplicate-hole-normalization）输入下，`ConvertToBrepBody(...)` 经 refit 后仍可保持 representative-average 共享顶点落点与确定性拓扑计数（VertexCount=12 / EdgeCount=14）
    - 已扩展 conversion capability：support-plane mismatch + near-equal shared-chain full-composition（含 collinear-leading + duplicate-hole-normalization）输入下，`ConvertToBrepBody(...)` 经 refit 后仍可保持 representative-average 共享顶点落点与确定性拓扑计数（VertexCount=13 / EdgeCount=15）
    - 已扩展 conversion capability：support-plane mismatch + near-equal shared-chain dual-duplicate full-composition（含 outer+hole duplicate-normalization）输入下，`ConvertToBrepBody(...)` 经 refit 后仍可保持 representative-average 共享顶点落点与确定性拓扑计数（VertexCount=13 / EdgeCount=15）
    - 已扩展 conversion capability：support-plane mismatch + near-equal closed-tetra dual-shared-vertices（两个共享顶点同时 near-equal 扰动）输入下，`ConvertToBrepBody(...)` 经 refit 后仍可保持 representative-average 共享顶点落点与 closed-shell 确定性拓扑计数（VertexCount=4 / EdgeCount=6）
    - 已扩展 conversion capability：support-plane mismatch + near-equal closed-tetra all-shared-vertices（四个顶点全部 near-equal 扰动）输入下，`ConvertToBrepBody(...)` 经 refit 后可对全部共享顶点同时应用 representative-average 落点，保持 closed-shell 确定性拓扑计数（VertexCount=4 / EdgeCount=6）
    - 已扩展 conversion capability：support-plane mismatch + near-equal closed-prism dual-shared-vertices（三棱柱拓扑，两个不相邻共享顶点各有 near-equal 扰动）输入下，`ConvertToBrepBody(...)` 经 refit 后可跨三角面和四边形面同时稳定应用 representative-average 落点，保持 closed-shell 确定性拓扑计数（VertexCount=6 / EdgeCount=9）
    - 已扩展 conversion capability：support-plane mismatch + near-equal closed-prism all-shared-vertices（三棱柱拓扑，六个共享顶点全部 near-equal 扰动）输入下，`ConvertToBrepBody(...)` 经 refit 后可对全体共享顶点同时稳定应用 representative-average 落点，保持 closed-shell 确定性拓扑计数（VertexCount=6 / EdgeCount=9）
    - 已完成 conversion 稳健性硬化：representative-target 全局聚合失败时自动回退至 representative-id 复用路径，不再直接返回 `InvalidBody`
    - 已增强 conversion 跨面联合修复：repair 后 representative snapping 从单轮提升为最多两轮小步迭代（保持有效性约束）
    - 已扩展 conversion capability：support-plane mismatch + near-equal closed-cuboid all-vertices（2×1×1 矩形盒子，8 顶点全部 near-equal 扰动）输入下，`ConvertToBrepBody(...)` 经 refit 后可对全体共享顶点稳定应用 representative-average 落点，保持 closed-shell 确定性拓扑计数（FaceCount=6 / VertexCount=8 / EdgeCount=12）
  - 已扩展 Brep->mesh 子能力：planar multi-face `BrepBody` 的 representative area-preserving conversion capability
  - 已扩展 Brep->mesh 子能力：planar holed+multi-face 混合 `BrepBody` 的 representative area-preserving conversion capability
  - 已扩展 Brep->mesh 子能力：planar shared-edge 相邻面转换时可全局复用共享 3D 顶点，收敛到 shared-edge feature-preserving 子集
  - 已扩展 Brep->mesh 子能力：disconnected closed-shell Brep 输入可保持组件保真（双立方体 -> 2 个 mesh 连通分量）
  - 已新增 healing 子能力收敛：带孔平面 `BrepFace` 缺失 outer/hole trims 时可被 `Heal(BrepBody)` 同步回填
  - 已新增 section 子能力：unit cube x=0.5 截面（法向 +x）产生四段闭合 1×1 矩形轮廓（perimeter=4.0 / area=1.0），扩展钢筋线周长覆盖到 x 轴方向
  - 已新增 section 子能力：2×2×1 矩形棱柱 z=0.5 截面产生四段闭合 2×2 方形轮廓（perimeter=8.0 / area=4.0），验证非单位截面的确定性钢筋线周长
  - 已新增 healing 子能力：`Heal(BrepBody)` 保守 trim 回填现覆盖 x=0（法向+x）平面，验证第三轴向的 trim 回填稳定性
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
  - 已补强 aggressive healing capability 断言：mixed support-mismatch + ineligible multiface 系列场景新增 shell-level FaceCount 分布断言
- 2026-04-03：
  - 已在 `src/sdk/GeometrySection.cpp` 落地 contour 驱动的 deterministic segment 后处理：输出段由 contour 重建并执行无向去重、短毛刺过滤（长度<=eps），减少 mesh-slice 原始段顺序/重复对统计的影响
  - 已扩展 `tests/capabilities/test_3d_section.cpp`：`ObliquePrismSectionYieldsDeterministicContourLength` 新增 `section.segments.size()==3` 断言，固化钢筋线根数稳定子集
  - 已补齐 `Section(BrepBody, Plane)` 的 coplanar merge 路径：三面共面 horizontal strip 经 Polyhedron->Brep 后截切可稳定合并为单 polygon（area=3.0）
  - 已扩展 conversion capability：support-mismatch near-equal closed-cuboid all-vertices 场景叠加一面 duplicate-loop-normalization 后，`ConvertToBrepBody(...)` 仍可稳定收敛 closed-shell 拓扑计数（FaceCount=6 / VertexCount=8 / EdgeCount=12）
  - 已扩展 conversion capability：support-mismatch near-equal closed-prism all-shared-vertices 场景叠加一面 duplicate-loop-normalization 后，`ConvertToBrepBody(...)` 仍可稳定收敛 closed-shell 拓扑计数（FaceCount=5 / VertexCount=6 / EdgeCount=9）
  - 已扩展 healing capability：`Heal(BrepBody)` 保守 trim 回填新增 oblique 平面 x+y+z=0（法向(1,1,1)）单面子样例，验证非轴对齐平面上的 trim backfill 稳定性
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

### P1-3D 算量必需项（新增）

- `Section3dGapTest::NonPlanarDominantSectionGraphRemainsOpen`：需优先转 capability，确保 non-planar dominant 输入下切面 contour stitching 稳定
  - **已收敛子集**：`UnitCubeMidPlaneSectionYieldsFourSegmentClosedContour`（四段闭合轮廓行列式，unit cube y=0.5）
- `Section3dGapTest::FaceMergeSemanticsAfterSectionRemainsOpen`：需优先转 capability，避免切面输出碎片化影响钢筋线统计
  - **已收敛子集**：`ThreeCoplanarFacesInLStripMergeIntoSinglePolygon`（三面水平排列 strip，area=3）
- `Conversion3dGapTest::GeneralNonPlanarPolyhedronToBrepRepairRemainsOpen`：需优先推进一般 non-planar repair（仍为 open gap）
- `Brep3dGapTest::NonPlanarTrimmedFaceTopologyRepairRemainsOpen`：需补齐 trimmed face topology repair
  - **已收敛子集**：`NonHorizontalPlaneBrepFaceWithoutTrimIsHealedWithBackfilledTrim`（y=0 竖面，法向+y）
- `Healing3dGapTest::AggressiveShellRepairPolicyRemainsOpen`：需补齐更一般 aggressive shell repair policy
  - **已收敛子集**：`AggressiveFourShellTwoEligibleOneIneligibleDeterministicBehavior`（四壳 mixed：1 closed + 2 eligible + 1 ineligible）
- 钢筋线切面结果后处理：需新增 deterministic 能力测试（去重/共线合并/短毛刺抑制/分组统计）
  - **已收敛子集**：`ObliquePrismSectionYieldsDeterministicContourLength`（equilateral 三棱柱水平截面周长断言 ≈3）
  - **已收敛子集**：`ObliquePrismSectionYieldsDeterministicContourLength` 段数断言（segments=3）

## 两周执行顺序建议（更新）

- 第 1 周：已完成 2D P0（boolean + offset）清空，以及 3D P1 骨架建立
- 第 2 周：推进 3D 第二阶段——non-planar section / brep coedge-editing / aggressive healing
