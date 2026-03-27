# TriangleMesh / Mesh Conversion Design

## 1. Purpose

本文定义 StableCore 三维库中的 `TriangleMesh` 层，以及 mesh 与 polyhedron / BRep 之间的转换服务。

目标是：
- 给 `TriangleMesh` 一个稳定、轻量、工程可用的角色定位
- 明确 mesh 与精确几何对象之间的边界
- 提前规划 tessellation、display、approximation、analysis 的公共入口

## 2. Reference Direction

主要参考：
- GGP `TriangleMesh.cpp`
  - 说明 mesh 内部天然会有 vertex / edge / triangle 的拓扑记录
  - 也说明 mesh 常作为独立几何对象存在，而不是 BRep 的附属缓存
- Delphi `GGLPolyhedron3d.pas`
  - 说明平面面片体与 2D polygon projection / triangulation 路径强相关

StableCore 不需要照抄 GGP 的半边细节，但应保留 mesh 作为独立表达层的地位。

## 3. Position in the 3D Library

`TriangleMesh` 的职责应是：
- 离散几何表示
- display / export / collision / coarse analysis 的通用载体
- BRep / Surface 的 tessellation 结果
- polyhedron 的统一离散输出形式

它不应承担：
- 精确曲面语义
- 参数域语义
- BRep coedge 语义
- healing / boolean 的主表达

## 4. Core Types

建议至少包含：
- `MeshVertex`
- `MeshTriangle`
- `TriangleMesh`
- 可选第二阶段：`MeshHalfedge` / `MeshEdge`

### 4.1 `MeshVertex`

建议字段：
- `Point3d position`
- `Vector3d normal`
- 可选 `Point2d uv`
- `MeshVertexId id`

### 4.2 `MeshTriangle`

建议字段：
- `std::array<MeshVertexId, 3> vertices`
- `MeshTriangleId id`
- 可选 `int materialId`
- 可选 `int groupId`

### 4.3 `TriangleMesh`

建议职责：
- 管理 vertex / triangle 容器
- 提供 bounds、normal recompute、consistency check
- 支持 lightweight tags / groups

建议最小接口：
- `VertexCount()`
- `TriangleCount()`
- `VertexAt(i)`
- `TriangleAt(i)`
- `Bounds()`
- `Transform(...)`
- `Clone()`
- `IsClosed()`
- `Validate()`

## 5. Representation Strategy

第一阶段建议使用 index-based 容器：
- `std::vector<MeshVertex>`
- `std::vector<MeshTriangle>`

优点：
- 序列化简单
- 与 conversion / export 更自然
- 比半边结构更适合起步阶段

第二阶段如果需要更强 mesh editing，再增加 adjacency cache 或 halfedge overlay。

## 6. Optional Cached Topology

不要在第一阶段强制 mesh 拥有复杂拓扑结构。

更合理的策略是：
- 核心 `TriangleMesh` 保持 index-based
- 另设 `MeshTopologyIndex` 或 `MeshAdjacencyCache`

负责：
- vertex -> incident triangles
- edge multiplicity
- boundary edge detection
- connected component split

这与 2D 库里“值类型 + 服务型索引”的路线一致。

## 7. Normals and Attributes

建议把 attribute 支持做成可选层，而不是一开始就做成通用大模板。

第一阶段推荐：
- vertex normal
- triangle normal
- uv
- group/material id

高阶 attribute 可以后置。

## 8. Conversion Services

建议显式拆出转换服务，而不是把它们塞进各几何类成员方法里。

### 8.1 `GeometryMeshConversion`

建议负责：
- `TriangleMesh Tessellate(const Surface&, const TessellationOptions&)`
- `TriangleMesh Tessellate(const BrepBody&, const TessellationOptions&)`
- `TriangleMesh Tessellate(const PolyhedronBody&)`
- `PolyhedronBody BuildPolyhedronFromMesh(...)`
- `BrepBody BuildPlanarBrepFromMesh(...)`

### 8.2 Conversion Rules

必须明确：
- `Surface/BRep -> Mesh` 通常是近似转换
- `Polyhedron -> Mesh` 多数情况下可视为结构保持转换
- `Mesh -> Brep` 只能在受限前提下成立

## 9. Tessellation Options

建议统一一个选项对象：
- `maxChordError`
- `maxAngleError`
- `maxEdgeLength`
- `respectTrimBoundaries`
- `computeNormals`
- `weldVertices`

这样 mesh 生成就不会在不同模块各自定义一套参数。

## 10. Validation and Diagnostics

建议 mesh 校验至少支持：
- 非法索引
- 退化三角形
- 重复顶点引用
- 方向不一致
- 边界边统计
- 非流形边检测

但这些应由服务承担：
- `GeometryMeshValidation`
- `GeometryMeshRepair`

不要把所有诊断都塞进 `TriangleMesh` 本体。

## 11. Spatial Search

mesh 会高频依赖空间索引。

建议单独提供：
- `MeshAABBTree`
- `GeometryMeshSearch`

能力包括：
- triangle overlap candidate query
- ray hit query
- nearest triangle / nearest point
- box / sphere candidate query

## 12. Relation to PolyhedronBody

`PolyhedronBody` 和 `TriangleMesh` 不能直接等同。

区别：
- `PolyhedronBody` 更强调 face / loop / plane 的工程结构
- `TriangleMesh` 更强调统一三角离散表示

推荐关系：
- `PolyhedronBody` 可导出为 `TriangleMesh`
- `TriangleMesh` 可在部分条件下重建 `PolyhedronBody`
- 但二者并不互为默认底层存储

## 13. Recommended Public Surface Area

第一阶段建议公开：
- `TriangleMesh`
- `TessellationOptions`
- `GeometryMeshConversion`
- `GeometryMeshValidation`

不建议第一阶段公开：
- 复杂 mesh editing API
- halfedge 细节
- 通用 attribute schema 系统

## 14. Recommended Implementation Order

建议顺序：
1. `TriangleMesh` index-based core
2. `Bounds()` / `Transform()` / `Clone()` / `Validate()`
3. `MeshTopologyIndex`
4. `TessellationOptions`
5. `PolyhedronBody -> TriangleMesh`
6. `Surface -> TriangleMesh`
7. `BrepBody -> TriangleMesh`
8. `MeshAABBTree`
9. diagnostics / repair

## 15. Key Design Decisions

这层最终要固定的结论是：
- `TriangleMesh` 是独立表达层，不是 BRep 的附属缓存
- 核心表示优先 index-based，而不是一开始就 halfedge-heavy
- mesh conversion 必须是服务层，不应分散在各几何对象成员方法中
- `PolyhedronBody` 与 `TriangleMesh` 相关但不等同
