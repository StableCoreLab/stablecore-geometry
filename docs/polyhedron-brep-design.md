# Polyhedron / BRep Design

## 1. Purpose

本文定义 StableCore 三维库中离散 polyhedron 与精确 BRep 层的分工、对象模型和演进顺序。

目标是：
- 明确 `PolyhedronBody` 和 `BrepBody` 不同职责
- 给 face/loop/edge/coedge/shell/body 建立清晰拓扑语义
- 提前规划 healing / boolean / section / tessellation 要依赖的内部层

## 2. Reference Direction

这层主要参考：
- Delphi `GGLPolyhedron3d.pas`
  - 明确存在 vertex / edge / loop / face / polyhedron 的对象族
  - face 会强依赖 plane、2D 投影 polygon、三角化、point position 等能力
  - 工程数据还会挂额外 domain info，不只是纯数学拓扑
- GGP `BrepBody.cpp`
  - 明确 body 持有 face / edge
  - edge 与 coedge index、surface ownership、clone/reference-fix 有清晰关系
  - transform、bounding box、clone、body fitting 都围绕 topology body 展开

结论：
- StableCore 需要同时保留 polyhedral layer 和 BRep layer
- 但两者不能混成一个类型

## 3. Why Two Body Families Are Necessary

### 3.1 `PolyhedronBody`

定位：
- 工程三角化/平面多边形面片体
- 允许更直接的离散操作
- 更接近 Delphi `TGGLPolyhedron3d` 的工作流

适合：
- plane-face 主导的建模
- 切分、粗布尔、规则构件生成
- 快速导入/导出
- 与现有 2D polygon 算法桥接

### 3.2 `BrepBody`

定位：
- 精确几何与拓扑的实体表达
- face 基于 `Surface`
- edge 基于 `Curve3d` 和可选 preimage

适合：
- 精确 section / projection / offset / boolean
- 参数曲面边界管理
- 拓扑一致性检查和 healing

### 3.3 Do Not Merge Them Too Early

如果把二者合并，接口会立刻变脏：
- polyhedron 需要高效离散面访问
- BRep 需要曲面/曲线句柄和 coedge 方向
- 二者对容差、编辑、转换、校验的要求不同

更合理的路线是：
- 两套对象分开
- 提供显式 conversion 服务

## 4. Topology Core

建议 BRep 层至少包含：
- `BrepVertex`
- `BrepEdge`
- `BrepCoedge`
- `BrepLoop`
- `BrepFace`
- `BrepShell`
- `BrepBody`

### 4.1 `BrepVertex`

职责：
- 存储 3D 点位
- 维护 incident edge / coedge 索引
- 承担 merge / sew 的最小单位

建议最小接口：
- `Position()`
- `SetPosition(...)`
- `EdgeCount()`
- `IncidentEdges()`

### 4.2 `BrepEdge`

职责：
- 持有一条 3D 空间曲线
- 连接两个端点 vertex
- 维护左右两个使用它的 coedge

建议存储：
- `std::shared_ptr<Curve3d> curve`
- `BrepVertexId startVertex`
- `BrepVertexId endVertex`
- `std::array<BrepCoedgeId, 2> uses`

关键点：
- `BrepEdge` 不承担方向，方向由 `BrepCoedge` 决定

### 4.3 `BrepCoedge`

职责：
- 表达 edge 在某个 face/loop 中的定向使用
- 关联 preimage curve

建议存储：
- `BrepEdgeId edge`
- `BrepFaceId face`
- `BrepLoopId loop`
- `bool reversed`
- `std::unique_ptr<Curve2d> preimage`

这是稳定 face boundary 的关键对象，不能省略。

### 4.4 `BrepLoop`

职责：
- 表达 face 上一个闭合边界
- 由有序 coedge 环组成

建议接口：
- `Count()`
- `CoedgeAt(i)`
- `IsOuter()`
- `Bounds2d()`

### 4.5 `BrepFace`

职责：
- 持有支撑曲面
- 管理 outer loop 与 inner loops
- 提供 point-in-face、trim domain、法向一致性基础

建议存储：
- `std::shared_ptr<Surface> surface`
- `std::vector<BrepLoopId> loops`
- `BrepShellId shell`

### 4.6 `BrepShell`

职责：
- 管理一组有方向一致性的 face
- 区分 open shell 与 closed shell

### 4.7 `BrepBody`

职责：
- 管理 vertices / edges / faces / shells
- 维护全局容差和粗 bounds
- 提供 clone、transform、validation 入口

## 5. Polyhedron Core

建议 `PolyhedronBody` 保持更轻的平面面片拓扑：
- `PolyhedronVertex`
- `PolyhedronEdge`
- `PolyhedronLoop`
- `PolyhedronFace`
- `PolyhedronBody`

与 Delphi 对齐的关键经验：
- `PolyhedronFace` 通常要直接支持 plane、box、loop、面积、长度、三角化
- `Face -> projected 2D polygon` 是高频操作

因此 `PolyhedronFace` 建议显式提供：
- `Plane()`
- `Bounds()`
- `Loops()`
- `Area()`
- `Perimeter()`
- `ProjectTo2d(...)`
- `Triangulate(...)`

## 6. Ownership and Identity

### 6.1 Use Stable IDs Internally

不建议一开始公开裸指针式导航 API。

更合理的是：
- 外部提供轻句柄或 index
- 内部维护稳定 `Id`

这样更适合：
- serialization
- diff / patch
- topology repair
- 后续并行算法

### 6.2 Shared Geometry, Owned Topology

建议：
- `Curve3d` / `Surface` 用共享所有权
- topology records 由 body 独占

原因：
- 多个 face 可共享同一底层 surface
- edge/face clone 或 stitch 时常需要保持几何引用关系

这点与 GGP `Clone + reference fix` 的经验一致。

## 7. Validation and Recovery Boundaries

不要把校验和恢复逻辑塞进 core topology 类。

建议拆三层：
- `GeometryBrepValidation`
  - 检查 manifold、loop closure、orientation、一致性
- `GeometryBrepHealing`
  - merge vertices、fix tiny edges、rebuild coedges、repair gaps
- `GeometryBrepBoolean`
  - section graph、classification、result rebuild

Delphi `DangerousToBooleanOperate` 这种经验说明：
- 几何风险判定是有价值的
- 但不应成为 face 本体的一堆杂项成员方法

## 8. Polyhedron vs BRep Conversions

必须提前规划这几条转换：

### 8.1 `PolyhedronBody -> TriangleMesh`

低风险，应该较早支持。

### 8.2 `BrepBody -> TriangleMesh`

需要 tessellation 服务，但也是早期高价值能力。

### 8.3 `PolyhedronBody -> BrepBody`

只在所有 face 都能稳定变成 `PlaneSurface` 且 loops 有效时成立。

### 8.4 `BrepBody -> PolyhedronBody`

需要 loss-aware conversion。

更适合作为：
- planarity-only fast path
- 或 tessellated approximation

## 9. Boolean / Section / Offset Dependencies

如果未来做 3D 布尔，BRep 至少要先具备：
- face-face intersection 输出 curve segments
- edge/face splitting
- loop rebuild on surfaces
- inside/outside classification
- shell stitching

如果未来做 polyhedron 布尔，则至少要先具备：
- plane-face clipping
- face polygon projection
- projected 2D boolean
- face rebuild / merge

这说明：
- `PolyhedronBody` 可以先承接 plane-dominant boolean
- `BrepBody` 适合更晚的精确 boolean

## 10. Recommended First Public Surface Area

第一阶段不建议公开整套 BRep 编辑 API。

更合理的公开面：
- `TriangleMesh`
- `PolyhedronBody`
- `BrepBody`
- 只读查询与转换

例如：
- `FaceCount()`
- `EdgeCount()`
- `ShellCount()`
- `Bounds()`
- `Transform(...)`
- `Clone()`
- `ToTriangleMesh(...)`
- `Validate(...)`

而大量 low-level mutation 先内部使用或 builder 化。

## 11. Builder Strategy

建议不要让用户直接手写完整拓扑连通关系。

优先提供 builder：
- `PolyhedronBuilder`
- `BrepBuilder`
- `ShellBuilder`

它们负责：
- 去重顶点
- 建 edge / coedge
- 检查 loop closure
- 生成 face / shell

这比开放大量 `AddEdge(faceIndex, loopIndex, ...)` 风格接口更稳。

## 12. Domain-Specific Metadata

Delphi `PolyhedronInfo` 说明工程实体经常挂结构化业务信息。

StableCore 不建议把这些直接塞进核心几何类。

建议提供：
- `GeometryUserData`
- `BodyTag`
- `FaceTag`

或键值扩展槽。

核心库只保证：
- 可附加
- 可序列化
- 不影响核心几何语义

## 13. Recommended Implementation Order

建议顺序：
1. `PolyhedronFace` / `PolyhedronBody` 的平面面片版本
2. `BrepVertex` / `BrepEdge` / `BrepCoedge` / `BrepLoop`
3. `BrepFace`
4. `BrepShell` / `BrepBody`
5. `Polyhedron <-> Mesh` conversion
6. `Brep -> Mesh` tessellation
7. validation / healing services
8. plane-dominant polyhedron boolean
9. exact BRep section / boolean

## 14. First-Phase Non-Goals

第一阶段不追求：
- 完整通用 CAD BRep 编辑器
- 所有高级非流形情形
- 全部精确曲面 boolean
- 全部持久化格式兼容

## 15. Key Design Decisions

这一层最终要固定的结论是：
- `PolyhedronBody` 和 `BrepBody` 必须并存，但职责不同
- `BrepCoedge` 不能省略
- face boundary 的 2D preimage 是核心基础设施
- topology、validation、healing、boolean 必须分层
- builder 比裸增删接口更适合作为第一阶段构造方式
