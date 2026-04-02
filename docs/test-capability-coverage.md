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
- `tests/capabilities/test_3d_brep.cpp`
  - 倾斜截面经过 `RebuildSectionBrepBody(...)` 得到只读 topology 完整的单面 `BrepBody`（1 shell / 1 face / 4 coedge loop），双立方体截面经 `RebuildSectionBrepBodies(...)` 稳定拆分为 2 个独立 body；并新增最小 coedge-loop 编辑链路 `InsertCoedge -> FlipCoedgeDirection -> RemoveCoedge`
- `tests/capabilities/test_3d_healing.cpp`
  - 保守 `Heal(PolyhedronBody)` 对已合法的单位立方体不改变 face count 且 `HealingIssue3d::None`；`Heal(BrepBody)` 可对 plane-surface + line-edge 且缺失 trim 的 face 进行 trim 回填，并覆盖带孔 face 的 outer/hole trims 同时回填
- `tests/capabilities/test_3d_conversion.cpp`
  - 单位立方体（6 quad faces）经 `ConvertToTriangleMesh(PolyhedronBody)` 得到 12 triangles，`SurfaceArea ≈ 6.0`；并可经 `ConvertToBrepBody(PolyhedronBody)` 得到 `FaceCount() == 6` 的有效 `BrepBody`，覆盖 affine-skew 非轴对齐子类输入，并覆盖 support-plane mismatch 的可修复子场景（support-plane refit）

## 共享测试支持

- `tests/support/Fixtures3d.h`
  - `geometry::test::BuildUnitCubeBody()` — 单位立方体 `[0,1]³`，6 个四边形 `PolyhedronFace3d`，法向朝外

## Gap Characterization Tests

- `tests/gaps/test_3d_section_gaps.cpp`
  - 记录 non-planar dominant 下的歧义 non-manifold contour stitching 与 coplanar fragment merge 语义仍未闭合
- `tests/gaps/test_3d_brep_gaps.cpp`
  - 记录 coedge-loop ownership 编辑链路、non-planar trimmed face topology repair 仍未闭合
- `tests/gaps/test_3d_healing_gaps.cpp`
  - 记录 trim-backfill conservative healing 之外的激进修复策略、mesh/body 联合多阶段修复仍未闭合
- `tests/gaps/test_3d_conversion_gaps.cpp`
  - 记录高保真 Brep->mesh 特征保持、鲁棒 non-planar polyhedron->Brep repair（超出 affine-planar + support-plane-refit 子集）仍未闭合
- 2D 历史 gap 场景已全部转正到 `tests/capabilities`；当前 `tests/gaps` 专注 3D P1 骨架跟踪

## CMake / gtest 说明

- `tests/CMakeLists.txt` 已切到 gtest
- 优先 `find_package(GTest CONFIG QUIET)` 使用本地 gtest 资源
- 本地不存在时，保留 `FetchContent` 作为后备接入方式
- 当前拆成两个目标：
  - `stablecore_geometry_capabilities_gtest`
  - `stablecore_geometry_gap_gtest`
