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

## 8. 对象不变量

`PolyhedronBody` 的面、环和顶点索引必须在所属对象内有效；每个面应能确定边界方向和支撑平面，空面、重复顶点和零面积面必须在构造或验证阶段被拒绝。Polyhedron 的布尔和截面流程不得把无效面静默转换为有效面。

`BrepBody` 必须维护从 body 到 shell、face、loop、coedge、edge、vertex 的可追溯关系。`BrepCoedge` 的方向由其所属 loop 决定，同一 `BrepEdge` 可以被多个 coedge 以不同方向引用。拓扑关系不完整时，修复服务应返回可定位的 issue，而不是继续生成不可验证的实体。

## 9. 推荐工作流

平面主导实体推荐采用：

```text
输入几何
  -> 输入有效性检查
  -> Polyhedron 构造
  -> 面/环/方向验证
  -> 布尔、截面或转换
  -> 输出验证或网格化
```

需要精确曲线曲面边界时采用：

```text
曲线/曲面
  -> BRep skeleton
  -> coedge 方向和 loop 闭合检查
  -> face/shell/body 拓扑检查
  -> 高阶运算或 healing
```

转换不得隐式改变调用者选择的实体语义。Polyhedron 转 BRep 或网格化时，应明确记录近似、容差和丢失的信息。

## 10. 兼容性边界

新增拓扑层级、改变 coedge 方向语义或修改 Polyhedron 面索引规则，都属于设计变更，不能仅通过增加成员字段完成。公共结果类型应继续使用结构化 `success/failure/issue` 契约；内部拓扑节点不作为跨 DLL 的稳定句柄公开。

## 11. 验证重点

三维实体设计至少需要验证：

- 空实体、退化面、重复顶点和零体积输入；
- shell/face/loop/coedge 的闭合和方向一致性；
- 布尔或截面后的拓扑完整性；
- 转换和网格化后的几何误差是否在指定容差内；
- 失败时能定位到 body、face、loop、edge 或 vertex。
