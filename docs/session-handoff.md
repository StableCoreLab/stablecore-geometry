# 会话交接

## 当前上下文

- 工作区：`D:\code\stablecore-geometry`
- 交接更新时间：`2026-03-30`
- 可用环境：`python`
- 后续会话应聚焦于源码与文档编写
- 编译 / 构建 / 运行由用户手动完成
- 不必担心 `gtest` 环境接入，用户会按需要调整 CMake / 构建侧

## 当前关注优先级

1. boolean 在退化 / 重叠场景下的剩余稳健性
2. offset 在更困难场景下的恢复能力
3. SearchPoly 风格的分支评分 / fake-edge 排序
4. 更丰富的 relation 层级
5. 带孔 polygon cut

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

- 2D 已经不是“缺基础类型或普通算法”的问题，而是“恢复深度、歧义消解与复杂场景稳健性仍明显低于 Delphi”
- 3D 还不是“补几个 corner case”就能追平的问题，而是“整体还处在底座 / 最小 workflow 阶段”

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

## 当前 Boolean 状态

Boolean 当前已不再主要卡在普通 crossing / containment / equal / touching / simple overlap 上。

当前 capability 预期包括：

- 更大的多步 overlap 家族
  - `Intersect = 14.0`
  - `Union = 39.0`
  - `Difference = 12.0`
- 近退化 repeated-overlap 家族
  - `Intersect = 10.000002`
  - `Union = 26.000007`
  - `Difference = 6.000003`
- ultra-thin repeated-overlap 家族
  - `Intersect = 10.00000002`
  - `Union = 26.00000007`
  - `Difference = 6.00000003`

当前剩余 boolean gap：

- 低于当前容差尺度的 arrangement 退化
- 更复杂 repeated-edge family
- 更难的 near-degenerate arrangement recovery

## 当前 Offset / Relation 边界

最近一次 capability 清理确认，有些场景仍应归类为 gap，而不是稳定 capability：

- 单 polygon offset 在重建后尚不能稳定保留 hole 语义
- 狭窄连接桥的 inward offset 尚不能稳定分裂为 multipolygon 输出
- hole-aware polygon containment 仍不够稳定，不能放入 capability 覆盖
- 某些 boolean 与 polygon rebuild 测试只在面积 / 语义层面稳定，尚未达到精确结果形状层面的稳定

这些 gap 已体现在：

- `tests/gaps/test_offset_gaps.cpp`
- `docs/test-capability-coverage.md`

## 下次开始时优先阅读的文档

- `docs/next-task-prompt.md`
- `docs/delphi-geometry-parity.md`
- `docs/sdk-type-design-review.md`
- `docs/test-capability-coverage.md`
- `docs/design-doc-sync-tracker.md`

如果专门处理当前 boolean 缺口，直接查看：

- `src/sdk/GeometryBoolean.cpp`
- `tests/capabilities/test_relation_boolean.cpp`
- `tests/gaps/test_boolean_gaps.cpp`

## 推荐的下一个 2D 动作

当前最可靠的下一项任务是：

- 继续补 boolean 稳健性缺口
  - 目标文件：`tests/gaps/test_boolean_gaps.cpp`
  - 聚焦低于当前容差尺度的 arrangement 退化与更复杂 repeated-edge family
  - 优先做机制级增强，而不是针对单个 case 的特判

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
- `GeometryMeasure` 已接上 `Distance(point, surface)` / `DistanceSquared(point, surface)`，可直接消费 `PlaneSurface` 与当前最小 `NurbsSurface`
- `GeometryMeasure` 已接上 `Distance(point, BrepFace)` / `DistanceSquared(point, BrepFace)`，可直接消费 planar / trimmed non-planar `BrepFace`
- `GeometryMeasure` 已接上 `Distance(point, BrepBody)` / `DistanceSquared(point, BrepBody)`，当前复用 `ProjectPointToBrepBody(...)`
- `GeometryMeasure` 已接上 `Distance(point, BrepEdge)` / `DistanceSquared(point, BrepEdge)`
- `GeometryIntersection3d` 已接上 `Intersect(Line3d, BrepFace, ...)` 与 `Intersect(Line3d, BrepBody, ...)`，当前复用 `line -> surface` 再走 trim containment / 多 face 聚合
- `GeometryIntersection3d` 已接上 `Intersect(Line3d, BrepEdge, ...)`，当前对 `LineCurve3d` 走解析求交，对一般 `Curve3d` 走保守采样
- `Validate(BrepBody, ...)` 已补最小 edge-use adjacency 校验：当前会拒绝 0 次引用或超过 2 次引用的 edge
- `GeometryIntersection3d` 已接上 `Intersect(Line3d, Surface, ...)`，当前对 `PlaneSurface` 走解析求交，对一般 `Surface` 走保守采样 + 局部细化
- `BrepVertex`、`BrepEdge`、`BrepCoedge`、`BrepLoop`、`BrepFace`、`BrepShell`、`BrepBody` 已不再只是名字 skeleton，已具备最小 topology ownership、bounds、validation 与 conservative healing 入口
- 3D 服务层公开函数名已补齐并落了最小实现：`GeometryMeasure` / `GeometryHealing` / `Validate(BrepBody, ...)`
- `BrepBody` 已接上受限 `TriangleMesh` conversion，当前支持 plane-surface + line-edge 主导的平面 BRep face
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

## 推荐的下一个 3D 动作

如果继续沿路线图推进，当前最合理的下一步是：

- 继续扩展 `trianglemesh-core`
  - 补更广泛 surface 的 conversion 入口，以及非平面边界 / stitching / boundary merge 场景下的 mesh closing / shell repair
- 或者进入 `polyhedron-core`
  - 在当前 `GeometrySection` 的基础上继续补 richer section topology 与更复杂 merge 语义
  - 再补 projected 2D polygon 驱动的多孔 / 更复杂 planar face 路径
