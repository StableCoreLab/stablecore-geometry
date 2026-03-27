# 3D Geometry Services Design

## 1. Purpose

本文定义 StableCore 三维库中 predicate / projection / intersection / search 等公共服务层。

目标是：
- 把 3D 基础算法从类型和拓扑中分离出来
- 提前规划容差、空间索引、关系判定、投影和相交的共用内部层
- 为 section / boolean / offset / validation 提供统一底座

## 2. Why a Dedicated Services Layer Is Necessary

如果没有公共服务层，后续很容易出现：
- curve、surface、BRep、mesh 各写一套 epsilon 规则
- projection 和 intersection helper 分散在不同 `.cpp`
- boolean / section / validation 各自维护不同的 search tree 和判定逻辑

2D 已经暴露过类似问题，3D 不应重演。

## 3. Reference Direction

主要参考：
- Delphi `GGLSurface3d.pas`
  - plane/surface 上堆积了大量距离、相交、位置关系、局部几何 helper
- Delphi `GGLBoxTree3d.pas`
  - 说明 3D box tree / candidate pruning 在工程实现里是基础设施
- GGP `Curve3d.cpp` / `Surface.cpp`
  - 说明 nearest parameter、projection、bounds、subdivide 等能力高度共用
- GGP `Topology.cpp`
  - 说明 topology 对缓存状态和 box state 有持续依赖

StableCore 的取向应是：
- 把这些共用能力整理成独立服务层
- 而不是让每个对象自己长出一堆散乱 helper

## 4. Service Layer Scope

建议至少包含：
- predicates / relation
- projection / nearest point
- distance / measurement
- intersection
- spatial search
- validation helpers
- tolerance context

## 5. Tolerance Infrastructure

必须先有统一容差对象，例如：
- `GeometryTolerance3d`
  - `distanceEpsilon`
  - `angleEpsilon`
  - `parameterEpsilon`
  - `boxPadding`

以及算法上下文：
- `GeometryContext3d`
  - `GeometryTolerance3d tolerance`
  - 可选诊断/日志开关
  - 可选 search/index cache

规则：
- 服务默认接收 context 或 tolerance
- 不在每个实现里散落固定 `1e-6` / `1e-9`

## 6. Predicate / Relation Services

建议定义：
- `GeometryPredicate3d`
- `GeometryRelation3d`

第一阶段优先支持：
- point-plane
- point-line
- point-segment
- point-triangle
- line-plane
- plane-plane
- box-box
- box-plane
- triangle-plane

对 body/more complex relation，先作为第二阶段。

## 7. Projection Services

建议定义：
- `GeometryProjection3d`

建议能力：
- `Project(point, line)`
- `Project(point, plane)`
- `Project(point, segment)`
- `Project(point, curve)`
- `Project(point, surface)`
- `Project(curve, plane)`
- `Project(body, plane)`

关键点：
- point-to-curve / point-to-surface 应统一返回结构化结果
- 周期参数面的边界吸附规则要走统一逻辑

## 8. Distance / Measure Services

建议定义：
- `GeometryMeasure3d`

建议能力：
- point-point distance
- point-line distance
- point-plane distance
- point-triangle distance
- curve length
- surface area approximation
- mesh area / volume
- body volume

说明：
- 精确与近似计算要区分接口
- 不要把所有 measure 都塞进对象成员函数

## 9. Intersection Services

建议拆成：
- `GeometryIntersection3d`
- `GeometryIntersectionInternal3d`

公开层负责：
- `Intersect(line, plane)`
- `Intersect(segment, triangle)`
- `Intersect(plane, plane)`
- `Intersect(curve, plane)`
- `Intersect(surface, plane)`
- `Intersect(face, face)`

内部层负责：
- 参数域迭代
- root finding
- candidate pruning
- near-degenerate fallback

## 10. Search / Spatial Index Services

建议把搜索服务独立为：
- `GeometryBoxTree3d`
- `GeometryKDTree3d`
- `GeometryAABBTree3d`
- `GeometrySearch3d`

建议用途划分：
- `BoxTree3d`
  - 一般 box candidate query
- `KDTree3d`
  - point cloud / nearest neighbor
- `AABBTree3d`
  - triangle / face / edge candidate pruning

这层是 section / boolean / validation 的必要前置。

## 11. Validation-Oriented Helpers

建议增加：
- `GeometryValidation3d`
- `GeometryValidationInternal3d`

先支持：
- zero-length edge
- degenerate triangle
- nearly-coplanar-but-inconsistent face sets
- self-intersection candidates
- open shell / non-manifold edge candidates

## 12. Internal Shared Layers To Reserve Early

建议提前规划这些内部文件族：
- `GeometryPredicate3dInternal.*`
- `GeometryProjection3dInternal.*`
- `GeometryIntersection3dInternal.*`
- `GeometrySearch3dInternal.*`
- `GeometryBrepPreprocessInternal.*`
- `GeometryMeshConversionInternal.*`

这样后续：
- section
- body boolean
- sewing
- tessellation
- validation

都能复用同一底层，而不是各写一套局部版本。

## 13. Relationship to Existing 2D SDK

3D 服务层应尽量复用 2D 已经验证过的设计原则：
- 公开接口短名风格
- 结果结构化返回
- 服务与值类型分离
- shared internal layer 早规划

但不要强行把 2D polygon-only helper 机械搬到 3D。

## 14. Recommended First Public Surface Area

第一阶段建议公开：
- `GeometryTolerance3d`
- `GeometryPredicate3d`
- `GeometryProjection3d`
- `GeometryIntersection3d`
- `GeometryMeasure3d`

第一阶段建议先内部化：
- 大型 spatial tree 细节
- face-face / body-body complex intersection plumbing
- recovery / preprocess internals

## 15. Recommended Implementation Order

建议顺序：
1. `GeometryTolerance3d` / `GeometryContext3d`
2. point-line-plane 基础 predicates
3. point / curve / surface projection results
4. line-plane / plane-plane / segment-triangle intersections
5. `GeometryBoxTree3d` 或 `GeometryAABBTree3d`
6. face/mesh candidate pruning helpers
7. validation helpers
8. section-ready intersection plumbing

## 16. Key Design Decisions

这层最终要固定的结论是：
- 3D 基础算法必须单独成层，不应散落在类型或 topology 类中
- tolerance/context 是公共基础设施，不是局部实现细节
- spatial search 是服务层核心组成部分
- projection / relation / intersection 必须共享统一的结果类型和 epsilon 语义
