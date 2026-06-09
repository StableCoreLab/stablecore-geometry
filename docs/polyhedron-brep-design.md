# Polyhedron 与 BRep 设计

## 1. 目的

本文定义 StableCore 三维库中离散 polyhedron 与精确 BRep 层的分工、对象模型和演进顺序。

## 2. 为什么要分层

polyhedron 与 BRep 都表达“体”，但它们服务的目标不同：

- polyhedron 更偏工程平面体工作流
- BRep 更偏精确边界表示与高阶拓扑语义

若两者一开始就合并，会同时污染：

- 数据模型
- 编辑语义
- 校验与恢复路径
- API 稳定性

## 3. `PolyhedronBody` 的定位

`PolyhedronBody` 适合承担：

- 平面主导体的表达
- 基础 face / loop / body 结构
- 与平面 section、projected 2D polygon workflow 的桥接
- 较轻量的工程体路径

## 4. `BrepBody` 的定位

`BrepBody` 适合承担：

- 精确边界表示
- 基于 curve / surface 的边界定义
- 更复杂的拓扑关系
- 后续高阶 boolean、sewing、healing 的目标对象

## 5. `BrepCoedge` 必须存在

BRep 不应只建 vertex / edge / face 三层。

必须保留：

- `BrepVertex`
- `BrepEdge`
- `BrepCoedge`
- `BrepLoop`
- `BrepFace`
- `BrepShell`
- `BrepBody`

原因：

- coedge 才能正确表达边在不同 loop / face 中的方向使用关系
- 没有 coedge，很多后续算法会被迫把方向语义塞进局部补丁

## 6. 当前演进顺序

建议顺序：

1. 先完成 `PolyhedronBody` 的基础工作流
2. 再建立 `BrepBody` skeleton
3. 在 `SCCurveOnSurface` 与基础拓扑表达明确后，再推进更复杂 BRep 算法

## 7. 当前固定结论

当前 3D 设计应固定：

- `PolyhedronBody` 与 `BrepBody` 不应过早合并
- `BrepCoedge` 是必需项
- polyhedron 先服务第一阶段平面主导工作流
- BRep 第一阶段只做 skeleton 与基础校验
