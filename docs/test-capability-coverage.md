# 测试覆盖布局

## 总览

测试已按两类能力拆分：

- `tests/capabilities`
  - 放当前库已经具备、应该稳定成立的能力验证
  - 这些文件已经迁移为 gtest `TEST(...)` 用例
- `tests/gaps`
  - 放当前仍未补齐的能力缺口说明性用例
  - 这些用例使用 `GTEST_SKIP()` 明确标记为已知差距，便于后续逐步转正

## Capability Tests

- `tests/capabilities/test_box.cpp`
  - 盒体基础行为、中心点、边界、有效性
- `tests/capabilities/test_point_vector.cpp`
  - 点、向量、基础段 / 圆弧辅助行为
- `tests/capabilities/test_segment.cpp`
  - 线段、圆弧段的长度、采样、边界盒、方向行为
- `tests/capabilities/test_polyline.cpp`
  - 开闭 polyline 的点访问、长度定位、边界盒
- `tests/capabilities/test_polygon.cpp`
  - polygon 的面积、质心、边界盒、孔洞基础行为
- `tests/capabilities/test_sdk.cpp`
  - SDK 层常用对象和基础 API 行为，以及 `LineCurve3d` / `NurbsCurve3d` / `PlaneSurface` / `NurbsSurface` / `RuledSurface` / `OffsetSurface` / `CurveOnSurface` / `CurveEval3d` / `SurfaceEval3d` / `TriangleMesh` / `TriangleNormal` / `VertexNormal` / `TriangleAdjacency` / `ExtractBoundaryEdges` / `ExtractBoundaryLoops` / `IsClosedTriangleMesh` / `ExtractNonManifoldEdges` / `IsManifoldTriangleMesh` / `ComputeTriangleConnectedComponents` / `IsConsistentlyOrientedTriangleMesh` / `ComputeMeshShells` / `OrientTriangleMeshConsistently(...)` / `CloseSinglePlanarBoundaryLoop(...)` / `ClosePlanarBoundaryLoops(...)` / `MeshValidation3d` / `Validate(PolyhedronSection3d, ...)` / `Validate(BrepBody, ...)` 的 edge-use adjacency 校验 / `Tessellate(PlaneSurface, ...)` / `ConvertToTriangleMesh(PolyhedronFace3d / PolyhedronBody / BrepFace / BrepBody / holed planar face / non-planar trimmed BrepFace, ...)` / `ConvertToPolyhedronFace(...)` / `ConvertToPolyhedronBody(...)` / `ConvertToBrepBody(...)` 的 trim 与 topology fallback / `Section(PolyhedronBody / BrepBody, Plane, ...)` / `BuildSectionTopology(...)` / `BuildSectionComponents(...)` / `RebuildSectionFaces(...)` / `RebuildSectionBrepFaces(...)` / `RebuildSectionBody(...)` / `RebuildSectionBrepBody(...)` / `RebuildSectionBrepBodies(...)` / `RebuildSectionBodies(...)` / `ConvertSectionToTriangleMesh(...)` / `ClassifySectionContent(...)` / `ProjectFaceToPolygon2d(...)` / `ProjectPointToCurve(...)` / `ProjectPointToCurveOnSurface(...)` / `ProjectPointToSurface(...)` / `ProjectPointToTriangleMesh(...)` / `ProjectPointToPolyhedronFace(...)` / `ProjectPointToPolyhedronBody(...)` / `ProjectPointToBrepVertex(...)` / `ProjectPointToBrepEdge(...)` / `ProjectPointToBrepFace(...)` / `ProjectPointToBrepBody(...)` / `LocatePoint(point, Curve3d / CurveOnSurface / PolyhedronFace3d / PolyhedronBody / BrepFace / BrepBody / TriangleMesh, ...)` / `Intersect(Line3d, Curve3d / CurveOnSurface / Surface / TriangleMesh / PolyhedronFace / PolyhedronBody / BrepVertex / BrepEdge / BrepFace / BrepBody, ...)` / `Intersect(Plane, Curve3d / CurveOnSurface / BrepVertex / BrepEdge, ...)` / `GeometryMeasure` 的 point-to-curve / point-to-curve-on-surface / point-to-surface / point-to-trianglemesh / point-to-polyhedron-face / point-to-polyhedron-body / point-to-brep-vertex / point-to-brep-edge / point-to-brep-face / point-to-brep-body distance，以及 `Bounds(PolyhedronFace3d / BrepVertex / BrepEdge / BrepFace)` / `GeometryHealing` / `Heal(BrepBody)` 的 trim 回填 / `BrepVertex` / `BrepEdge` / `BrepCoedge` / `BrepLoop` / `BrepFace` / `BrepShell` / `BrepBody` / `PolyhedronLoop3d` / `PolyhedronFace3d` / `PolyhedronBody` / `PolyhedronValidation3d` 的最小 3D 能力
- `tests/capabilities/test_sdk_algorithms.cpp`
  - 距离、投影、反转、采样、基础算法能力
- `tests/capabilities/test_sdk_multigeometry.cpp`
  - multipolygon、box tree、kd tree、segment search 等索引与聚合能力
- `tests/capabilities/test_transform_sampling.cpp`
  - 平移、旋转、镜像、拉伸、采样变换行为
- `tests/capabilities/test_serialize.cpp`
  - 文本序列化 / 反序列化稳定性
- `tests/capabilities/test_shapes_pathops.cpp`
  - shape/pathops、cut polygon、build multipolygon by lines、near-close、auto-extend、ambiguous fake-edge 当前已具备能力
- `tests/capabilities/test_relation_boolean.cpp`
  - boolean 的 crossing、containment、equal、touching、simple collinear overlap、disjoint union、ultra-thin repeated-overlap 当前已具备能力
- `tests/capabilities/test_offset.cpp`
  - line / arc / polyline / polygon / multipolygon offset、basic rebuilt polygon growth、disjoint multipolygon offset、single-polygon hole semantics recovery、representative reverse-edge/self-intersection recovery、narrow-bridge split via `OffsetToMultiPolygon(...)` 当前已具备能力
- `tests/capabilities/test_topology_indexing.cpp`
  - touching / intersecting / basic contains / equal、duplicate-equal topology parent tie-break 当前已具备能力
- `tests/capabilities/test_3d_section.cpp`
  - 倾斜切平面下 `Section(...) + BuildSectionTopology(...) + BuildSectionComponents(...)` 的单区域闭环能力；并覆盖 non-axis-aligned multi-face 截面的稳定 contour/polygon 计数
  - 覆盖 `Section(BrepBody, Plane)` 的 oblique-cut 子能力：单位立方体先转 Brep 后再截切，稳定得到 1 polygon / 1 contour / 6 points / 1 topology root
  - 覆盖 `Section(BrepBody, Plane)` 的 coplanar 邻接片段合并子能力：2-face 邻接平面片先转 Brep 后截切，稳定得到单 polygon（area=2.0）
  - 覆盖 `Section(BrepBody, Plane)` 的 multi-component 子能力：双立方体先转 Brep 后截切，稳定得到 2 polygons / 2 roots / 2 components
  - 覆盖 coplanar 相邻 face fragment 在 `Section(...)` 中合并为单 polygon 的代表性 face-merge 子集
- `tests/capabilities/test_3d_brep.cpp`
  - 倾斜截面经过 `RebuildSectionBrepBody(...)` 得到只读 topology 完整的单面 `BrepBody`（1 shell / 1 face / 4 coedge loop），双立方体截面经 `RebuildSectionBrepBodies(...)` 稳定拆分为 2 个独立 body；并新增最小 coedge-loop 编辑链路 `InsertCoedge -> FlipCoedgeDirection -> RemoveCoedge`
  - 覆盖端到端 Brep 路径：`ConvertToBrepBody -> Section(BrepBody, Plane) -> RebuildSectionBrepBodies` 在双组件输入下稳定输出 2 个独立重建 body
  - 覆盖 `RebuildSectionBody(...)` / `RebuildSectionBodies(...)` 的 Polyhedron 重建子路径：倾斜截面可重建为单面 `PolyhedronBody`，双组件截面可稳定重建为 2 个独立 `PolyhedronBody`
  - 覆盖 Brep section rebuild 壳体语义断言：`RebuildSectionBrepBody(...)` 与 `RebuildSectionBrepBodies(...)` 输出显式满足 `ShellCount()==1 && IsClosed()==false`
  - 覆盖最小 ownership-consistent 编辑链路：`ReplaceOuterLoop -> ReplaceFace -> ReplaceShell` 可把 loop 级编辑稳定传播回有效 `BrepBody`
  - 覆盖 ownership replacement 在 multi-face closed-shell 的子路径：unit-cube Brep 上 no-op `ReplaceOuterLoop -> ReplaceFace -> ReplaceShell` 后仍保持 `ShellCount=1 / FaceCount=6 / IsClosed=true`
- `tests/capabilities/test_3d_healing.cpp`
  - 保守 `Heal(PolyhedronBody)` 对已合法的单位立方体不改变 face count 且 `HealingIssue3d::None`；`Heal(BrepBody)` 可对 plane-surface + line-edge 且缺失 trim 的 face 进行 trim 回填，并覆盖带孔 face 的 outer/hole trims 同时回填；`Heal(..., policy=Aggressive)` 覆盖可恢复 open planar single/multi-face sheet（含 holed shell）的确定性闭壳子策略，并覆盖 aggressive 闭壳与 trim-backfill 的组合病理子场景
  - `Heal(..., policy=Aggressive)` 可在同一个 body 中同时闭合多个可恢复 open shell，并保持确定性拓扑结果
  - `Heal(..., policy=Aggressive)` 在混合 closed-shell + open-shell 输入下可保持已闭壳稳定，并仅对可恢复 open shell 执行确定性闭壳
  - `Heal(..., policy=Aggressive)` 在 mixed open-shell 输入下支持部分修复：可恢复 planar shell 可闭壳，不可恢复 shell 保持原状
  - `Heal(..., policy=Aggressive)` 在存在 shared-edge 邻接的 planar multi-face open-sheet 上也可执行确定性闭壳，不再局限于边完全不共享的分离面片
  - `Heal(..., policy=Aggressive)` 在三壳 mixed 输入下保持 deterministic：closed shell 保持稳定、eligible open shell 闭壳、ineligible open shell 保持 open
  - `Heal(..., policy=Aggressive)` 在三壳 mixed 输入下可与 conservative trim-backfill 协同：eligible open shell 先回填 trims 后闭壳，ineligible open shell 保持 open
  - `Heal(..., policy=Aggressive)` 在三壳 mixed 输入下支持 eligible multi-face open-sheet 闭壳，同时保持 closed shell 稳定与 ineligible shell open 状态
  - `Heal(..., policy=Aggressive)` 在三壳 mixed 输入下支持 eligible holed shell（含缺失 outer/hole trims）先回填后闭壳，同时保持 ineligible shell open
  - `Heal(..., policy=Aggressive)` 在三壳 mixed 输入下支持 eligible multi-face holed shell（含缺失 trims）回填后闭壳，同时保持 ineligible shell open
  - `Heal(..., policy=Aggressive)` 在三壳 mixed 输入下支持 eligible multi-face（holed+plain）两面 trims 缺失场景回填后闭壳，同时保持 ineligible shell open
  - `Heal(..., policy=Aggressive)` 在三壳 mixed 输入下支持 eligible multi-face holed shell 的 support-plane mismatch + 缺失 trims 组合修复并闭壳，同时保持 ineligible shell open
  - `Heal(..., policy=Aggressive)` 在 support-plane mismatch 的 eligible shell 与 ineligible multi-face shell 共存时，保持 deterministic：eligible 闭壳、ineligible 保持 open
  - `Heal(..., policy=Aggressive)` 在 support-plane mismatch 的 eligible shell 缺失 trims 且与 ineligible multi-face shell 共存时，仍可先回填后闭壳并保持 ineligible open
  - `Heal(..., policy=Aggressive)` 在 support-plane mismatch 的 eligible holed shell（缺失 outer/hole trims）与 ineligible multi-face shell 共存时，仍可先回填后闭壳并保持 ineligible open
  - `Heal(..., policy=Aggressive)` 在 support-plane mismatch 的 eligible multi-face（holed+plain，缺失 trims）与 ineligible multi-face shell 共存时，仍可先回填后闭壳并保持 ineligible open
  - `Heal(..., policy=Aggressive)` 在 mixed support-mismatch + ineligible multiface 系列场景中已补齐 shell-level FaceCount 分布断言，避免仅靠总 FaceCount 掩盖 shell 归属回归
- `tests/capabilities/test_3d_conversion.cpp`
  - 单位立方体（6 quad faces）经 `ConvertToTriangleMesh(PolyhedronBody)` 得到 12 triangles，`SurfaceArea ≈ 6.0`；并可经 `ConvertToBrepBody(PolyhedronBody)` 得到 `FaceCount() == 6` 的有效 `BrepBody`，覆盖 affine-skew 非轴对齐子类输入、support-plane mismatch 可修复子场景（含 shared-chain mixed-content full-composition 下的 support-plane refit）、mild non-planar outer/hole loop 顶点投影修复子场景、leading collinear loop 顶点下的稳健法向回退、duplicate outer/hole loop 顶点归一化修复、tiny-scale non-planar（含 holed/multi-face/mixed-content/shared-edge/shared-chain/shared-chain-mixed-content）输入下的 scale-aware 法向回退与投影修复，以及 duplicate/hole/collinear-leading normalization 与 shared-edge chain 修复的组合稳定性；同时覆盖 planar holed、planar multi-face、以及 planar holed+multi-face `BrepBody` 到 mesh 的面积保持子场景
  - `ConvertToBrepBody(...)` 在 tiny-scale shared-edge 邻接链 mixed-content full-composition 下，支持 outer/hole 双重复顶点归一化与 support-mismatch + collinear-leading 组合修复稳定叠加
  - `ConvertToBrepBody(...)` 在 tiny-scale shared-edge 邻接链修复后可全局复用共享顶点/边，避免按 face 重复建拓扑并保持共享边一致性子集稳定
  - `ConvertToBrepBody(...)` 在 closed-shell 代表性输入（单位立方体）上可收敛到共享拓扑 Brep：1 shell / 8 vertices / 12 edges / closed shell
  - `ConvertToBrepBody(...)` 在 tiny-scale closed-shell 代表性输入（tetrahedron：4 triangular faces, 6 edges, 4 vertices, 所有 support planes mismatched）上，经 per-face refit 修复后可收敛为合法 closed BrepBody（IsClosed=true / VertexCount=4 / EdgeCount=6），验证 closed-shell 下的共享拓扑代表性路径
  - `ConvertToTriangleMesh(BrepBody)` 在 planar shared-edge 相邻面上可全局复用共享 3D 顶点，避免 face-by-face 拼接导致的重复顶点子集
  - `ConvertToTriangleMesh(BrepBody)` 在 disconnected closed-shell 输入上可保持组件保真：双立方体 Brep 转 mesh 后稳定得到 2 个连通分量（每分量 12 triangles）
  - `ConvertToBrepBody(...)` 在 tiny-scale triangular-face chain（3 个三角面，mismatched support planes）上可确定性地保持共享边顶点一致性（每个共享顶点是所有相关三角面的定义顶点，distance=0 → 无投影偏差），结果满足 VertexCount=5 / EdgeCount=7 的精确拓扑共享断言
  - `ConvertToBrepBody(...)` 在 tiny-scale triangular-fan（4 个三角面共享 apex，mismatched support planes）上可确定性地保持共享 apex 顶点一致性（apex 是所有 4 个三角面的定义顶点，distance=0 → 精确保留），结果满足 VertexCount=5 / EdgeCount=8（4 条径向共享边 + 4 条外边）
  - `ConvertToBrepBody(...)` 的 per-face refit 已引入 shared-vertex-aware support-plane 估计启发式：当 outer loop 顶点在 body 内出现多次（跨面共享）时，优先使用包含更多 shared vertices 的三点组估计 refit plane，以减少 shared-edge 顶点跨面投影偏差
  - `ConvertToBrepBody(...)` 已引入 source representative-id 贯穿复用：即使修复后共享顶点出现微小跨面漂移，也能按输入拓扑代表点强制复用同一 BrepVertex，从而恢复 tiny-scale quad shared-edge chain 的确定性拓扑断言（VertexCount=8 / EdgeCount=10）
  - `ConvertToBrepBody(...)` 在 support-plane mismatch 的 tiny-scale quad shared-edge chain（3 quads）上可稳定收敛到共享拓扑（VertexCount=8 / EdgeCount=10），验证 representative-id 驱动复用在非三角面链场景下的代表性有效性
  - `ConvertToBrepBody(...)` 在 support-plane mismatch + duplicate-loop-normalization 的 tiny-scale quad shared-edge chain（中间 face 含重复 leading 顶点）上也可稳定收敛到共享拓扑（VertexCount=8 / EdgeCount=10），验证 representative-id 映射可穿透修复归一化后的 loop cardinality 变化
  - `ConvertToBrepBody(...)` 在 shared-chain mixed-content 子集上已补齐确定性共享拓扑计数断言：duplicate-hole、support-plane mismatch、以及二者组合（support-mismatch + duplicate-hole）场景均稳定满足 VertexCount=12 / EdgeCount=14
  - `ConvertToBrepBody(...)` 在 shared-chain 组合子场景（shared-edge duplicate-loop、mixed-content collinear-leading、support-mismatch+collinear、full-composition、dual-duplicate full-composition）上也已补齐确定性共享拓扑计数断言，分别稳定满足 VertexCount=8/13 与 EdgeCount=10/15 的预期拓扑计数
  - `ConvertToBrepBody(...)` 在 tiny-scale 基础子场景（non-planar multi-face、non-planar mixed-content、non-planar shared-edge faces、non-planar shared-edge-chain mixed-content）上也已补齐确定性共享拓扑计数断言（分别为 8/8、12/12、6/7、12/14）
  - `ConvertToBrepBody(...)` 在早期 repair 子场景（skewed cube、support-plane mismatched cube、mildly non-planar cube face、mildly non-planar holed face、collinear-leading loop、duplicate loop / duplicate-hole loop、composite stress face、tiny-scale face / holed-face）上也已补齐确定性拓扑计数断言（分别覆盖 8/12、8/8、5/5、4/4、10/10 等代表性计数）
  - `ConvertToBrepBody(...)` 已开始引入跨面联合修复步骤：repair 后执行 representative-id global snapping pass（并回投影到各自 face support plane），用于进一步降低 shared-vertex 跨面漂移并为 topology-changing non-planar repair 打基础
  - `ConvertToBrepBody(...)` 已扩展 representative-id 全局目标点聚合：在 near-equal shared-edge 输入中，共享 `BrepVertex` 落点由跨面代表点全局平均驱动，不再依赖首个面点，进一步使共享边一致性约束参与最终顶点位置决策
  - `ConvertToBrepBody(...)` 已为代表性 repair 场景补齐壳体语义断言：cube-like 输入稳定满足 `ShellCount()==1 && IsClosed()==true`，shared-chain sheet-like 输入稳定满足 `ShellCount()==1 && IsClosed()==false`
  - `ConvertToBrepBody(...)` 在 deformed unit cube（单顶点位移，三面同时非平面）场景下可经 per-face refit 逐面修复并通过 representative-id 复用保证共享拓扑，结果满足 FaceCount=6/VertexCount=8/EdgeCount=12/closed shell 确定性拓扑断言
  - `ConvertToBrepBody(...)` 在 dual-deformed unit cube（双顶点位移，六面均非平面）场景下同样可经 per-face refit 逐面修复并保持共享拓扑，结果满足 FaceCount=6/VertexCount=8/EdgeCount=12/closed shell 确定性拓扑断言

## 共享测试支持

- `tests/support/Fixtures3d.h`
  - `geometry::test::BuildUnitCubeBody()` — 单位立方体 `[0,1]³`，6 个四边形 `PolyhedronFace3d`，法向朝外

## Gap Characterization Tests

- `tests/gaps/test_3d_section_gaps.cpp`
  - 记录 non-planar dominant 下的歧义 non-manifold contour stitching 与更高阶 coplanar fragment merge 语义仍未闭合（相邻 coplanar union 子集已在 Polyhedron/Brep 路径覆盖）
- `tests/gaps/test_3d_brep_gaps.cpp`
  - 记录 coedge-loop ownership 编辑链路、non-planar trimmed face topology repair 仍未闭合（ownership gap 已覆盖 single-face + multi-face closed-shell no-op replacement 子集）
- `tests/gaps/test_3d_healing_gaps.cpp`
  - 记录超出 planar open-sheet closure（含 holed shell）子策略的激进修复策略、mesh/body 联合多阶段修复仍未闭合
- `tests/gaps/test_3d_conversion_gaps.cpp`
  - 记录高保真 Brep->mesh 特征保持（超出 planar holed+multi-face area-preserving + shared-edge vertex-reuse + disconnected closed-shell component-preserving 子集）、鲁棒 non-planar polyhedron->Brep repair（超出 affine-planar + support-plane-refit + mild outer/hole loop-projection + collinear-leading-loop + duplicate outer/hole loop-normalization 子集）仍未闭合
- 2D 历史 gap 场景已全部转正到 `tests/capabilities`；当前 `tests/gaps` 专注 3D P1 骨架跟踪

## CMake / gtest 说明

- `tests/CMakeLists.txt` 已切到 gtest
- 优先 `find_package(GTest CONFIG QUIET)` 使用本地 gtest 资源
- 本地不存在时，保留 `FetchContent` 作为后备接入方式
- 当前拆成两个目标：
  - `stablecore_geometry_capabilities_gtest`
  - `stablecore_geometry_gap_gtest`
