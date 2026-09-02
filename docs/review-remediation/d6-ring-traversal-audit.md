# D6：二维环遍历与排序审计（2026-09-02）

## 定位结果

存在两组相近但尚不能机械合并的 arrangement 实现：

| 位置 | 重复职责 | 额外职责 |
| --- | --- | --- |
| `Source/Geometry2d/SCPathOps.cpp` | 参数聚类排序、半边扇角排序、前一出边选择、候选环遍历与嵌套环组装 | 输入修复、synthetic edge 标记、环评分与输出清理 |
| `Source/Core/Boolean.cpp` | 同一类参数聚类、半边扇角排序、前一出边选择与环遍历 | 面内/面外分类、Boolean 边选择、共线重叠与退化恢复 |
| `Source/Core/SearchPoly.cpp` | 不再复制环遍历 | 调用 `BuildMultiPolygonByLines`，补充网络诊断、候选评分和分支清理回退 |

另有 B-rep 的四份 `AppendLoopVertices*`，属于 3D 拓扑点读取，和 D6 的 2D arrangement 语义不同；不与二维 helper 混合提取。

## 差分回归

`UnitTests/Capabilities/Geometry2d/TestRingTraversalRegression.cpp` 固化：

1. 内外两组独立边在输入顺序置换后，`BuildMultiPolygonByLines` 的多边形数量、洞数量和面积不变。
2. 同一输入通过 `SearchPolygons` 的诊断适配层后，几何结果与直接重建相同。
3. `Difference(outer, inner)` 的洞结构和面积与独立边界重建相同。

比较忽略内部候选/边的存储顺序，仅比较公共几何结果的面积、数量和洞数量；这正是后续内部 helper 替换应保持的契约。

## 提取决定

已提取私有的最小 shared helper：`Source/Detail/DirectedEdgeFans2d.h`。`SCPathOps` 与 `Boolean` 仅迁移扇排序和下一面边选择；两侧仍保留各自的边筛选、epsilon 分层、synthetic edge、长度、访问和分类状态，因此没有耦合修复路径与 Boolean 语义。

交点参数的钳制、近值去重和排序聚类也已迁移至 `Source/Detail/SegmentParameters2d.h`；两侧仍自行根据线段尺度及各自 comparison epsilon 计算参数容差。

最小无状态模型已在 `d6-directed-edge-model.md` 中定义，并补充了交叉、共线重叠和窄缝洞的差分样例。`SearchPoly` 继续作为 `BuildMultiPolygonByLines` 的诊断适配层，而不是重复内核。
