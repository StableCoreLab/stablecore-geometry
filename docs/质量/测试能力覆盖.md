# 测试覆盖说明

本文档只保留当前能力覆盖和当前能力差距，不保留历史过程记录。

## 测试分层

- `UnitTests/Capabilities`：已收敛、应稳定支持的能力测试
- `UnitTests/Gaps`：当前仍未闭合或明确未支持的差距测试

## 当前已覆盖的能力

### 二维

- Box、Point、Vector、Segment、Polyline、Polygon 的基础行为
- 距离、投影、反射、采样、偏移、关系判断、布尔运算
- `SearchPoly` 的确定性候选筛选、分支评分、诊断一致性、最小包含查询
- `Shapes` / `PathOps` / `Topology` 相关能力

### 三维

- `Section` 的平面截取、组件重建、BRep/Polyhedron 结果稳定性
- `Healing` 的 trim 回填、open shell 修复、aggressive boundary-cap
- `BodyBoolean` 的闭体布尔、touching 子集、显式不支持契约
- `BrepConversion` 的修复、重建、对齐与拓扑回收

## 当前能力差距

### `Section`

- 更一般的非流形轮廓拼接
- 超出显式 vertex-touch / edge-touch 子集的开放曲线与面积归并
- 非邻接 coplanar 片段的更一般合并
- 多层 boundary-attached 开放曲线裁决

### `Healing`

- 多 shell shared-boundary / shared-edge 裁决的更一般情形
- 复杂开放 shell 与网格/实体联动修复

### `SearchPoly`

- 更丰富的 fake-edge 解释
- 更完整的 ambiguous recovery

### `BodyBoolean`

- 更一般的 touching / overlap 子集
- shell-policy 相关更高阶裁决

## 当前判断

- 已闭合的能力应继续留在 `UnitTests/Capabilities`
- 仍然语义不稳定或覆盖不足的能力，继续留在 `UnitTests/Gaps`
- 如果某个 gap 已经具备明确、稳定、可复现的场景，应尽快转为 capability

## 交付标准

- 至少包含正常路径测试
- 至少包含边界测试
- 只要存在歧义，就必须有 gap 测试
- 结果与诊断必须一致

## 模块重构顺序

如果后续继续做模块级重构，建议按下面顺序推进：

1. `Section`
2. `BodyBoolean`
3. `SearchPoly`
4. `Healing`

这和当前能力收口方向一致：

- 先把平面切体和边线提取稳定下来
- 再把复杂体相交和并差结果稳定下来
- 再把二维闭合搜索和歧义恢复补齐
- 最后补修复链路，让上游输入更稳定
- 
## Current additions

- `UnitTests/Capabilities/Core/TestMissingAlgorithms.cpp`
- `UnitTests/Gaps/Core/TestMissingAlgorithmsGaps.cpp`
- `UnitTests/Gaps/Geometry3d/TestMissingAlgorithmsGaps.cpp`

## Covered APIs

- `TangentPoints`
- `SCLine2d` and `Intersect` / `IntersectExtended`
- `ProjectPointToLineSegment` for 3D
- `SCBoxTree2d::QueryKNearest`
- `SCSegmentSearch2d::QueryKNearest`
- `SCBoxTree3d`
- `SCSegmentSearch3d`
- `SnapPointToSegments3d`

## Remaining gap themes

- epsilon-sensitive tangent boundary classification
- extended intersection overlap / coincidence corner cases
- polyline endpoint de-duplication stability
- 3D zero-length / near-zero-length segment projection boundary
- 3D equal-distance tie breaking for snapping and KNN
