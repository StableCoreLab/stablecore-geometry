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
- `trianglemesh-core` 已起步：
  - `include/sdk/TriangleMesh.h`
  - `GeometryValidation` 已加入 `TriangleMesh` 的最小 validation 结果
  - `GeometryTessellation` 已加入 `PlaneSurface -> TriangleMesh` 的最小网格化入口
- `polyhedron-core` 已起步：
  - `include/sdk/PolyhedronLoop3d.h`
  - `include/sdk/PolyhedronFace3d.h`
  - `include/sdk/PolyhedronBody.h`
  - `GeometryValidation` 已加入 `PolyhedronBody` 的最小 validation 结果
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
  - `MeshValidation3d`
  - `Tessellate(PlaneSurface, ...)`
  - `PolyhedronLoop3d`
  - `PolyhedronFace3d`
  - `PolyhedronBody`
  - `PolyhedronValidation3d`

当前重要约束：

- 这批 3D 参数对象层目前是最小可用协议，不包含高阶 NURBS 或复杂投影 API
- `PlaneSurface` 当前按有限参数域平面曲面实现，便于后续 bounds / tessellation / section 直接消费
- `TriangleMesh` 已有最小值对象与 validation，尚未接入 tessellation / conversion
- `TriangleMesh` 已接上 `PlaneSurface -> TriangleMesh` 的最小 tessellation 路径
- `PolyhedronBody` 已有平面 face / loop / body 的最小骨架，但尚未接 projected 2D polygon workflow
- `BrepBody` 仍未开始代码落地

## 推荐的下一个 3D 动作

如果继续沿路线图推进，当前最合理的下一步是：

- 继续扩展 `trianglemesh-core`
  - 补 mesh 法向、邻接，以及 polyhedron / surface 更广泛的 conversion 入口
- 或者进入 `polyhedron-core`
  - 将当前平面 face / loop / body 骨架接到 projected 2D polygon workflow
  - 再补最小 planar polyhedron -> mesh 路径
