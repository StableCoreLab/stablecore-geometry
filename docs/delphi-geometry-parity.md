# Delphi 几何能力对齐情况

## 1. 目的

本文记录当前 C++ 仓库与 Delphi 参考实现之间的几何算法能力对齐基线，用于后续能力跟踪与实现对照。

参考基线：

- `D:\code\GFY2.0\Controls\GAEAResource\GCL\Geo2DLib\Source`

## 2. 对比范围

只比较几何算法能力。

纳入范围：

- 2D 几何类型
- 几何判定与容差规则
- 距离、投影、采样与度量
- 段相交与关系逻辑
- polyline 与 polygon 算法
- offset、boolean、topology 与建面逻辑
- 面向几何的空间索引与搜索

明确排除：

- `GGLCoordTrans.pas`
- `GGLDrawFunc2d.pas`

## 3. 当前总体结论

当前 C++ 仓库在 polygon / face 工作流上的几何算法能力，已覆盖比过去更多的 Delphi 能力，但仍未达到完整 Delphi 对齐。

当前状态摘要：

- 基础几何内核：大体覆盖
- 线网建面：已具备切分、重复边清理、近端点自动闭合、简单 auto-extend 与分支裁剪
- polygon boolean：已覆盖普通 crossing、containment、equal、touching、simple overlap，以及更广的一类近退化 repeated-overlap 家族
- polygon relation 与 topology：已具备基础 equal / shared-edge recovery
- polygon offset：已具备 ring 重建、基础 split recovery 与 collapsed-ring 清理
- SearchPoly 级别的高歧义恢复、以及更深层 offset / boolean recovery：仍低于 Delphi

从当前源码、capability tests 与 gap tests 的实际状态看，如果只讨论 2D：

- 基础几何内核与常规 polygon 工作流，大致已达到 Delphi 相关能力的 `70%~80%`
- 高歧义线网、深退化 boolean、困难 offset 恢复等工程恢复层，大致仍只有 Delphi 相关能力的 `40%~60%`

这里的比例是对“当前已落盘代码与测试覆盖范围”的粗略工程判断，不是跑分结果，也不代表生产可靠度百分比。

如果把 3D 也纳入整体对比，则当前 C++ 仓库与 Delphi / GGP 参考系之间的落差更大：

- 当前 3D 代码更接近“第一阶段基础库 + 最小 plane-based workflow”，而不是成熟工程 3D 几何库
- 对照 Delphi `Geo3DLib` 与 GGP 的 3D 模块轮廓，当前 3D 大致仍只处于目标工程能力的 `20%~35%`
- 当前 3D 还明显停留在底座建设阶段，距离完整 polyhedron / BRep / healing / section / lofting / exact body workflow 仍有阶段级差距

换句话说：

- 2D 已经进入“核心能力可用，但工程恢复深度仍明显低于 Delphi”的阶段
- 3D 仍处在“底座与最小 workflow 已起步，但距离 Delphi / GGP 风格工程库还远”的阶段

## 4. 能力分类

### 4.1 已达到或基本达到

- 基础 point / vector / box / segment 抽象
- 线段与圆弧段的度量、投影、切线、法线、按参数取点等操作
- 基础 polyline / polygon 的面积、周长、质心、方向与 bounds 操作
- line、ring、polygon 的点包含判定
- 核心几何类型的平移、旋转、镜像、拉伸变换
- 基础 KD-tree 与 box-tree 查询能力
- 基础按距离与最近点的 segment search

### 4.2 部分达到

- polygon boolean
- polygon offset
- polygon topology 层级
- 直线切 polygon
- 由线输入构建 multipolygon
- polygon relation 聚合
- 面向下游 polygon 工作流的几何搜索基础设施

其中 boolean 当前已具备：

- relation-aware 快路径
- arrangement face 提取与分类
- duplicate-edge 预处理
- tiny-face 抑制
- 更强的 interior-face sampling
- ultra-thin repeated-overlap 家族能力覆盖

但仍未达到 Delphi 的部分主要在：

- 更深层 arrangement 退化恢复
- 更复杂 repeated-edge family 清理
- 与更重型 polygon search 工作流的联动

### 4.3 尚未达到

- 接近 Delphi `SearchPoly` 的分支评分、fake-edge 策略与更强搜索启发式
- 在当前 synthetic-edge 过滤之上，更强的 auto-close / auto-extend 逻辑
- 面向 reverse-edge、无效圆、多轮失败恢复及更复杂自交情形的 offset 深层清理
- 超出当前容差尺度的更难 arrangement degeneracy recovery
- 围绕 `SearchPoly` 的 Delphi 级端到端 polygon search / build 工作流

对于 3D，如果拿 Delphi `Geo3DLib` 与 GGP 的工程目标做参考，当前尚未达到的部分更多，而且不是“少几个接口”，而是还缺完整阶段：

- 更广泛的 surface / section / tessellation / conversion 工作流，而不止平面主导路径
- 更完整的 polyhedron face rebuild、shell stitching、boundary closing 与 repair
- BRep skeleton 的代码落地，以及 coedge / loop / face / body 的基本 ownership 结构
- 更强的 validation / healing 链路，而不只是 mesh 查询与局部 orientation repair
- 更复杂 surface、lofting、body 级 boolean / offset / sewing 等成熟 3D 工程能力

当前 3D 中已经明确补上的一段，是 plane-dominant `section` 最小链：

- `Plane x PolyhedronBody` 已可返回 `segments / contours / polygons`
- 已支持横切、coplanar face、edge-only 零面积截面
- 已支持 `BuildSectionTopology(...)`
- 已支持 `RebuildSectionFaces(...)` / `RebuildSectionBody(...)`
- 已支持 `ConvertSectionToTriangleMesh(...)`
- 已补齐并落了最小实现：`NurbsCurve3d` / `NurbsSurface` / `RuledSurface` / `OffsetSurface` / `CurveOnSurface` / `BrepBody` 等第一阶段公开对象，以及 `GeometryMeasure` / `GeometryHealing` / `Validate(BrepBody, ...)` / `ProjectPointToSurface(...)` / `Distance(point, surface)` / `Intersect(Line3d, Surface, ...)` / `BrepFace -> TriangleMesh` / 受限 `BrepBody -> TriangleMesh` / `ConvertToPolyhedronFace` / `ConvertToPolyhedronBody` / `Section(BrepBody, Plane, ...)` / `RebuildSectionBrepFaces` / `RebuildSectionBrepBody` / `RebuildSectionBrepBodies` / `Heal(BrepBody)` trim 回填入口；其中 `BrepFace` 已可沿 trim/UV 路径消费非平面面片，`BrepBody -> PolyhedronBody` 也已支持 trim 缺失下的 topology fallback

但这仍只是第一阶段的最小 plane-section workflow 与对象层 skeleton，距离 Delphi / GGP 风格的更复杂 section graph、face merge 细语义、非平面 / 非平面主导 section，以及真正可用的 BRep / healing 算法还差明显阶段。

因此，当前仓库与 Delphi 的“尚未达到”状态，2D 与 3D 应分开理解：

- 2D：主要差在恢复深度、歧义消解与更接近生产级的搜索 / 重建策略
- 3D：主要差在整体阶段，当前仍远未形成 Delphi / GGP 风格的完整工程几何工作流

## 5. 当前差距清单

### 5.1 必须补齐

- 面向高度歧义线网的 branch-aware polygon search
- 更广泛的 boolean 稳健性，覆盖低于当前容差尺度的 arrangement 退化与更复杂 repeated-edge family
- 更强的 offset 恢复，覆盖剩余 self-intersection 与 reverse-edge 场景
- boolean 与 offset 之间更一致的预处理能力
- 多条闭合路径可选时，更接近 Delphi 的 fake-segment 与 candidate-ranking 策略

### 5.2 应当补齐

- 更丰富的 polygon relation 层级，接近 Delphi `GGLPolyRelation`
- 对带孔、多切线输入、线网辅助分区的 polygon cut / split 支持更好
- 在线搜索、面提取、boolean、offset 管线之间建立更明确的 topology recovery
- 更强的容差传递，保证相同 `eps` 策略在切分、ring 提取、containment 与重建过程中行为一致

## 6. 当前缺口的关键证据

### 6.1 Boolean 已超出简单 cell 场景，但仍未达到 Delphi 水平

当前 C++ boolean 已不再只是简单扫描 cell。它现在会：

- 对 equal / disjoint / containment / touching 先走 relation-aware 短路路径
- 在 arrangement 前移除重复无向边
- 在 ring 提取阶段清理近共线碎顶点
- 用更保守的 tiny-face 阈值，减少误删极薄但真实的 overlap 结果
- 在 face 分类阶段使用更强的 interior-face sampling

这使它已覆盖：

- crossing
- containment
- equal
- touching
- simple overlap
- 更大多步 collinear-overlap
- 近退化 repeated-overlap
- ultra-thin repeated-overlap

当前 C++ 证据：

- `src/sdk/GeometryBoolean.cpp`
- `tests/capabilities/test_relation_boolean.cpp`

相比 Delphi 仍缺：

- 更难 arrangement degeneracy 下的恢复
- 更复杂 repeated-edge family 清理
- 与更重型 polygon search 工作流的更强联动

### 6.2 BuildMultiPolygonByLines 已在建面前加入修复步骤

当前 C++ 实现已能在开放且带分支的线网中，先做重复边清理、交点切分、近端点自动闭合、简单投影式 auto-extend、dangling branch 裁剪，以及 fake-edge 主导小候选环抑制，然后再进行 polygon 重建。

当前 C++ 证据：

- `src/sdk/GeometryPathOps.cpp`
- `tests/test_shapes_pathops.cpp`

相比 Delphi 仍缺：

- 面向分支的评分与搜索策略
- 更丰富的 fake-edge 插入策略
- 对高度歧义线网更强的恢复

### 6.3 Offset 已能从生成 ring 重建，但仍未达到 Delphi offset 深度

当前 C++ offset 会将生成的 offset ring 再送回 polygon 重建流程，在重建前过滤 collapsed / near-zero ring，并在单 polygon offset 返回多个候选时尽量选择语义更合理的结果。

当前 C++ 证据：

- `src/sdk/GeometryOffset.cpp`
- `tests/test_offset.cpp`

相比 Delphi 仍缺：

- reverse-edge 清理
- 更深的无效圆 / loop 过滤
- 更复杂歧义输出的恢复
- narrow-channel、loop-collapse、hole-inversion 等边界场景处理

## 7. 当前对比基线

在本文下次更新前，后续对齐评审应继续遵循：

- 若某个 C++ 特性只覆盖简单分支与普通 crossing，它仍不能算作完整 Delphi `SearchPoly` 对齐
- 若某个 C++ 特性缺少明确的歧义消解、fake-edge 排序或更重清理阶段，它在生产级 polygon 工作流中仍低于 Delphi
- 若某个 C++ 特性只能处理普通 offset，而无法较好恢复 self-intersection、split output 或 collapse，它仍低于 Delphi 的 offset 能力

## 8. 建议的跟踪重点

下一轮建议继续跟踪：

- polygon boolean 中更深层的 arrangement degeneracy recovery
- 更强的 offset 清理与恢复
- polygon search 中的分支评分与歧义消解
- 更丰富的 polygon relation 层级行为
- 所有 face 操作之前统一的几何预处理

如果同时跟踪 3D，则优先级应单独列出，而不要与 2D parity 混在一起：

- 将当前平面 polyhedron workflow 继续扩展到更一般的 section / face rebuild / face merge
- 继续补 triangle mesh 的 stitching / boundary closing / shell repair
- 逐步落 BRep skeleton，而不是过早转向 exact body boolean
- 在 3D 中优先建立稳定的 conversion / validation / repair 闭环，再谈更高阶 surface 与 body 算法
