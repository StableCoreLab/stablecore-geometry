# D6：最小无状态有向边模型（设计稿）

## 目标与边界

该内部模型只统一二维 arrangement 的**拓扑遍历**：有向边配对、出边扇排序和“到达顶点后取前一出边”的选择。它不负责输入修复、边的 Boolean 取舍、synthetic edge 诊断、环评分、包含关系或输出多边形组装。

因此 `SCPathOps` 与 `Boolean` 仍各自负责：

- 原始段收集、交点参数聚类和 epsilon 策略；
- 去重、修复、分类和是否把某条边送入模型；
- 收集回路后的有效性检查、方向筛选、洞归属及公开结果。

## 最小内部数据模型

```cpp
namespace Geometry::Detail
{
struct DirectedEdge2d
{
    std::size_t from;
    std::size_t to;
    std::size_t twin;
    double angle;
};

template <typename TDirectedEdge>
void SortOutgoingFans(const std::vector<TDirectedEdge>& edges,
                      std::vector<std::vector<std::size_t>>& outgoing);
[[nodiscard]] std::size_t PreviousOutgoing(const std::vector<std::size_t>& fan,
                                           std::size_t edgeIndex);
template <typename TDirectedEdge>
[[nodiscard]] std::size_t NextFaceEdge(const std::vector<TDirectedEdge>& edges,
                                       const std::vector<std::vector<std::size_t>>& outgoing,
                                       std::size_t edgeIndex);
}
```

`DirectedEdge2d` 不保存 PathOps 的 `synthetic/visited/length`，也不保存 Boolean 的 inside/outside 分类；调用方以派生的本地 edge 类型保存这些状态。两个模板只依赖基类的 `from/to/twin/angle` 字段。模型不暴露在安装头文件或 ABI 中。

## 参数聚类 helper

`Source/Detail/SegmentParameters2d.h` 统一参数钳制、近值去重和排序聚类。调用方仍计算自身的参数容差：PathOps 使用 `kPathOpsComparisonEpsilon`，Boolean 使用 `kBooleanComparisonEpsilon`。helper 不解释交点来源，也不改变“近值去重保留最先插入代表值”的既有语义。

## 排序契约

1. 先按精确的 `angle` 升序排序；helper 不在排序比较器中使用 epsilon。
2. `angle` 精确相等时，按 `to` 顶点索引、再按 edge 索引打破平局，形成确定性的严格弱序。
3. `PreviousOutgoing` 找不到 twin 时返回无效索引；空扇同样返回无效索引。
4. helper 不推断 epsilon，也不删除共线边；这由调用方在建图前完成。

第 2 条在迁移时需作为显式兼容性变化评审：现有实现仅比较角度，等角边的顺序由输入/标准库实现间接决定。epsilon 仅用于调用方的顶点合并、共线处理和建图前筛选；新增回归样例先锁定公共几何输出，再启用确定性平局规则。

## 迁移门槛

`TestRingTraversalRegression.cpp` 已覆盖输入置换、交叉、共线重叠和窄缝洞。提取 helper 前，还须以 Debug 与 RelWithDebInfo 回归 `RelationBooleanTest`、`SearchPolyCapabilityTest` 与 `ShapesPathopsTest`；若任一公共面积、洞结构或失败语义变化，停止迁移并保留局部实现。
