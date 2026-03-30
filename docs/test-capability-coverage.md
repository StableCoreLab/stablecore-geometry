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
  - SDK 层常用对象和基础 API 行为，以及 `LineCurve3d` / `NurbsCurve3d` / `PlaneSurface` / `NurbsSurface` / `RuledSurface` / `OffsetSurface` / `CurveOnSurface` / `CurveEval3d` / `SurfaceEval3d` / `TriangleMesh` / `TriangleNormal` / `VertexNormal` / `TriangleAdjacency` / `ExtractBoundaryEdges` / `ExtractBoundaryLoops` / `IsClosedTriangleMesh` / `ExtractNonManifoldEdges` / `IsManifoldTriangleMesh` / `ComputeTriangleConnectedComponents` / `IsConsistentlyOrientedTriangleMesh` / `ComputeMeshShells` / `OrientTriangleMeshConsistently(...)` / `CloseSinglePlanarBoundaryLoop(...)` / `ClosePlanarBoundaryLoops(...)` / `MeshValidation3d` / `Validate(PolyhedronSection3d, ...)` / `Validate(BrepBody, ...)` / `Tessellate(PlaneSurface, ...)` / `ConvertToTriangleMesh(PolyhedronFace3d / PolyhedronBody / BrepFace / BrepBody / holed planar face / non-planar trimmed BrepFace, ...)` / `ConvertToPolyhedronFace(...)` / `ConvertToPolyhedronBody(...)` / `Section(PolyhedronBody / BrepBody, Plane, ...)` / `BuildSectionTopology(...)` / `BuildSectionComponents(...)` / `RebuildSectionFaces(...)` / `RebuildSectionBrepFaces(...)` / `RebuildSectionBody(...)` / `RebuildSectionBrepBody(...)` / `RebuildSectionBodies(...)` / `ConvertSectionToTriangleMesh(...)` / `ClassifySectionContent(...)` / `ProjectFaceToPolygon2d(...)` / `GeometryMeasure` / `GeometryHealing` / `Heal(BrepBody)` 的 trim 回填 / `BrepVertex` / `BrepEdge` / `BrepCoedge` / `BrepLoop` / `BrepFace` / `BrepShell` / `BrepBody` / `PolyhedronLoop3d` / `PolyhedronFace3d` / `PolyhedronBody` / `PolyhedronValidation3d` 的最小 3D 能力
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
  - line / arc / polyline / polygon / multipolygon offset、basic rebuilt polygon growth、disjoint multipolygon offset 当前已具备能力
- `tests/capabilities/test_topology_indexing.cpp`
  - touching / intersecting / basic contains / equal、duplicate-equal topology parent tie-break 当前已具备能力

## Gap Characterization Tests

- `tests/gaps/test_boolean_gaps.cpp`
  - 记录低于当前容差尺度的 arrangement 退化、以及更复杂 repeated-edge family / near-degenerate boolean 仍未闭合
- `tests/gaps/test_offset_gaps.cpp`
  - 记录 reverse-edge、self-intersection、multi-failure recovery、hole / outer semantic flip、single-polygon hole semantic recovery、narrow bridge split recovery 仍未闭合
- `tests/gaps/test_searchpoly_gaps.cpp`
  - 记录 SearchPoly 风格 branch scoring / fake-edge ranking 仍明显低于 Delphi
- `tests/gaps/test_relation_gaps.cpp`
  - 记录 relation hierarchy 向 `GGLPolyRelation` 靠拢的剩余差距，以及 hole-aware polygon containment 等尚未稳定闭合的关系语义

## CMake / gtest 说明

- `tests/CMakeLists.txt` 已切到 gtest
- 优先 `find_package(GTest CONFIG QUIET)` 使用本地 gtest 资源
- 本地不存在时，保留 `FetchContent` 作为后备接入方式
- 当前拆成两个目标：
  - `stablecore_geometry_capabilities_gtest`
  - `stablecore_geometry_gap_gtest`
