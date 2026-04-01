# 下一步任务提示

你现在在仓库 `D:\code\stablecore-geometry` 中继续工作，目标是推进 3D 几何库第二阶段能力，同时保持 2D 已有能力不退化。

一、工作范围
- 聚焦 3D 算法与测试能力补充
- 2D 方向仅在出现真正缺口时补充，不重复做已完成工作
- 明确排除对 Delphi 参考目录的机械照搬
- 不要做编译、不要跑构建、不要依赖编译结果判断实现是否完成
- 只做源码级实现、静态检查、必要测试代码补充、文档更新
- 完成后给出建议提交信息 `Msg`

二、当前代码状态

### 2D（已稳定）

2D 核心算法全部完成并转正为 capability tests：
- BuildMultiPolygonByLines：branch-aware 评分、fake-edge 抑制、outer/hole 归并
- Boolean：arrangement face 提取、relation-aware fast path、near-degenerate 稳健性
- Offset：ring rebuild、reverse-edge 恢复、hole semantics 恢复、OffsetToMultiPolygon narrow bridge split
- Topology/Relation：touching / intersecting / contains / equal 区分、shared-edge 判定
- 所有 tests/gaps/ 中的 2D gap 已清空，对应能力已转入 tests/capabilities/