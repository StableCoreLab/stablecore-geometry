# D3 空间索引基线

当前 `SCKDTree2d`、`SCBoxTree2d` 和 `SCBoxTree3d` 保存的是可变 entries 容器；每次查询临时建立 KD/BVH 结构，不存在持久缓存、`Build()` 或 dirty 标志。

因此当前可验证的行为是：`Add`、`Update`、`Remove` 后的下一次查询立即反映新 entries；结果由现有查询排序规则决定。`QueryKNearest` 使用距离、再以 id 打破平局；其 `eps` 参数当前未参与计算，D3 的后续 API 决策必须明确删除该参数或赋予近似搜索语义。

测试 oracle 使用确定性随机输入和直接距离扫描。性能基准应在确定 D3 的缓存模型后再比较“每次重建”和“lazy dirty 缓存”，避免将当前实现误报为持久索引。
