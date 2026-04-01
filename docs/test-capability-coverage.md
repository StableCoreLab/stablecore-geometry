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
  - SDK 层常用对象和基础 API 行为，以及 `LineCurve3d` / `NurbsCurve3d` / `PlaneSurface` / `NurbsSurface` / `RuledSurface` / `OffsetSurface` / `CurveOnSurface` / `CurveEval3d` / `SurfaceEval3d` / `TriangleMesh` / `TriangleNormal` / `VertexNormal` / `TriangleAdjacency` / `ExtractBoundaryEdges` / `ExtractBoundaryLoops` / `IsClosedTriangleMesh` / `ExtractNonManifoldEdges` / `IsManifoldTriangleMesh` / `ComputeTriangleConnectedComponents` / `IsConsistentlyOrientedTriangleMesh` / `ComputeMeshShells` / `OrientTriangleMeshConsistently(...)` / `CloseSinglePlanarBoundaryLoop(...)` / `ClosePlanarBoundaryLoops(...)` / `MeshValidation3d` / `Validate(PolyhedronSection3d, ...)` / `Validate(BrepBody, ...)` 的 edge-use adjacency 校验 / `Tessellate(PlaneSurface, ...)` / `ConvertToTriangleMesh(PolyhedronFace3d / PolyhedronBody / BrepFace / BrepBody / holed planar face / non-planar trimmed BrepFace, ...)` / `ConvertToPolyhedronFace(...)` / `ConvertToPolyhedronBody(...)` 的 trim 与 topology fallback / `Section(PolyhedronBody / BrepBody, Plane, ...)` / `BuildSectionTopology(...)` / `BuildSectionComponents(...)` / `RebuildSectionFaces(...)` / `RebuildSectionBrepFaces(...)` / `RebuildSectionBody(...)` / `RebuildSectionBrepBody(...)` / `RebuildSectionBrepBodies(...)` / `RebuildSectionBodies(...)` / `ConvertSectionToTriangleMesh(...)` / `ClassifySectionContent(...)` / `ProjectFaceToPolygon2d(...)` / `ProjectPointToCurve(...)` / `ProjectPointToCurveOnSurface(...)` / `ProjectPointToSurface(...)` / `ProjectPointToTriangleMesh(...)` / `ProjectPointToPolyhedronFace(...)` / `ProjectPointToPolyhedronBody(...)` / `ProjectPointToBrepVertex(...)` / `ProjectPointToBrepEdge(...)` / `ProjectPointToBrepFace(...)` / `ProjectPointToBrepBody(...)` / `LocatePoint(point, Curve3d / CurveOnSurface / PolyhedronFace3d / PolyhedronBody / BrepFace / BrepBody / TriangleMesh, ...)` / `Intersect(Line3d, Curve3d / CurveOnSurface / Surface / TriangleMesh / PolyhedronFace / PolyhedronBody / BrepVertex / BrepEdge / BrepFace / BrepBody, ...)` / `Intersect(Plane, Curve3d / CurveOnSurface / BrepVertex / BrepEdge, ...)` / `GeometryMeasure` 的 point-to-curve / point-to-curve-on-surface / point-to-surface / point-to-trianglemesh / point-to-polyhedron-face / point-to-polyhedron-body / point-to-brep-vertex / point-to-brep-edge / point-to-brep-face / point-to-brep-body distance，以及 `Bounds(PolyhedronFace3d / BrepVertex / BrepEdge / BrepFace)` / `GeometryHealing` / `Heal(BrepBody)` 的 trim 回填 / `BrepVertex` / `BrepEdge` / `BrepCoedge` / `BrepLoop` / `BrepFace` / `BrepShell` / `BrepBody` / `PolyhedronLoop3d` / `PolyhedronFace3d` / `PolyhedronBody` / `PolyhedronValidation3d` 的最小 3D 能力
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

## Gap Characterization Tests

- 当前 `tests/gaps` 目标已清空（无有效测试用例）；原 gap 场景已逐步转正到 `tests/capabilities`
- 后续如新增未闭合能力差距，继续在 `tests/gaps` 下补充 `GTEST_SKIP()` 表征用例

## CMake / gtest 说明

- `tests/CMakeLists.txt` 已切到 gtest
- 优先 `find_package(GTest CONFIG QUIET)` 使用本地 gtest 资源
- 本地不存在时，保留 `FetchContent` 作为后备接入方式
- 当前拆成两个目标：
  - `stablecore_geometry_capabilities_gtest`
  - `stablecore_geometry_gap_gtest`
