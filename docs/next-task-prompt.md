# 下一步任务提示

```text
你现在在仓库 `D:\code\stablecore-geometry` 中继续工作，目标是继续补齐当前 C++ 几何库与 Delphi / GGP 在面相关几何算法上的剩余差距。

一、工作范围
只关注几何算法，明确排除：
- `GGLCoordTrans.pas`
- `GGLDrawFunc2d.pas`

不要做编译，不要跑构建，不要依赖编译结果判断实现是否完成。
只做源码级实现、静态检查、必要测试代码补充、文档更新。
完成后给出建议提交信息 `Msg`。

二、参考实现来源
参考目录：
- Delphi: `D:\code\GFY2.0\Controls\GAEAResource\GCL\Geo2DLib\Source`
- GGP: `D:\code\GGP_ProdN\Source\Geometry`

重点参考：
- Delphi:
  - `GGLSearchPolyFunc2d.pas`
  - `GGLOffsetFunc2d.pas`
  - `GGLPolyRelation.pas`
- GGP 中相关 polygon / topology / offset 实现

不要机械照抄代码，要参考能力和处理思路。

三、当前仓库已经完成的增强
截至当前，已经补过：
1. `BuildMultiPolygonByLines`
- 支持开放线段网络建面
- 支持线段交点切分
- 支持重复边清理
- 支持近端点自动闭合
- 支持简单投影式 auto-extend
- 支持 dangling branch pruning
- 支持 outer / hole 嵌套归并
- 已增加 branch-aware 候选评分：面积、synthetic 周长、branch 顶点数、synthetic-branch 顶点数联合打分
- 已增加候选环 deterministic ranking（score -> area -> perimeter）

2. Boolean
- 已支持 arrangement face 提取
- 已支持 bounded face 分类
- 已支持结果边界重建
- 已支持普通 crossing / containment
- 已新增 duplicate-edge 预处理
- 已新增 tiny sliver face 过滤
- 已新增 relation-aware fast path：
  - `Equal`
  - `Disjoint`
  - `FirstContainsSecond`
  - `SecondContainsFirst`
  - `Touching`
  这些情况优先短路，不再全部进入 arrangement
- 已增加 boolean 输入 operand 的 pathops 风格边界重建预处理（统一 duplicate-edge / near-collinear 清理口径）
- 已增加 split 后 tiny segment 过滤
- 已增加 split 后 degree-2 共线链段合并，收敛 repeated-edge family 的碎段噪声

3. Offset
- 已支持 offset ring 生成后重新建面
- 已支持 concave / multipolygon 基础恢复
- 已新增 collapsed / near-zero ring 过滤
- 已改进单 polygon offset 结果选择，不再只是取最大面积，而是优先保留与原始语义更一致的候选
- `MultiPolygon2d` offset 仍保留分裂结果
- 已增加 reverse-edge / 方向异常恢复：同距离并行尝试原始方向、反向补偿、保守 miter fallback
- 已增加 relation-aware 候选打分（contains/intersecting/touching/disjoint + holeCount 偏差惩罚）
- 已增加 ring 建面多轮失败恢复：原始 rings 失败后尝试反向 rings，再尝试放宽 epsilon

4. Topology / Relation
- 已支持 touching / intersecting / contains / equal 等基本区分
- 已修正 shared-edge / collinear overlap 不再一律误判为 crossing
- 已增强 strict interior 判定，加入边中点辅助
- 已新增 equal duplicate polygon 的 deterministic parent tie-break，避免 topology 在重复等价 polygon 输入下出现不稳定父子归并
- 已增强 relation hierarchy：对 boundary overlap + strict interior 联合判定，收敛 shared-edge touching 与 overlap-intersecting 的区分

5. SearchPoly 风格增强
- 已在 `BuildMultiPolygonByLines` 中增加 synthetic/fake edge 跟踪
- 已增加候选闭环面积阈值过滤
- 已增加 fake-edge-dominated tiny loop 抑制
- 已实现真实边优先的重复边保留策略
- 已补 synthetic-branch dominated tiny loop 抑制

四、上次实际完成的改动
这次已经落地的核心改动：
- `src/sdk/GeometryPathOps.cpp`
  - 候选环评分升级：branch/synthetic-branch 参与，增加 deterministic ranking
- `src/sdk/GeometryBoolean.cpp`
  - boolean operand 预处理统一
  - split 后 tiny segment 过滤
  - split 后 degree-2 共线链段合并
- `src/sdk/GeometryOffset.cpp`
  - reverse-edge / conservative miter fallback
  - relation-aware 候选选择增强
  - ring 建面多轮失败恢复（reverse + relaxed epsilon）
- `src/sdk/GeometryTopology.cpp`
  - boundary overlap 与 strict interior 联合判定
- `tests/capabilities/test_relation_boolean.cpp`
  - 增加 duplicate-edge family 与 repeated-collinear-chain overlap 回归
- `tests/capabilities/test_shapes_pathops.cpp`
  - 增加 branch-heavy ambiguous fake-edge closure 回归
- `tests/capabilities/test_offset.cpp`
  - 增加 clockwise/reverse-edge 与 inward-hole recovery 回归
- `tests/capabilities/test_topology_indexing.cpp`
  - 增加 shared-edge touching 与 overlap-with-interior relation 回归
- `docs/delphi-geometry-parity.md`
  - 已同步上述增强

五、这次修改涉及的主要文件
源码：
- `src/sdk/GeometryPathOps.cpp`
- `src/sdk/GeometryBoolean.cpp`
- `src/sdk/GeometryOffset.cpp`
- `src/sdk/GeometryTopology.cpp`

测试：
- `tests/capabilities/test_shapes_pathops.cpp`
- `tests/capabilities/test_relation_boolean.cpp`
- `tests/capabilities/test_offset.cpp`
- `tests/capabilities/test_topology_indexing.cpp`

文档：
- `docs/delphi-geometry-parity.md`

六、当前已补的测试覆盖
已新增或扩展测试覆盖：
- `test_shapes_pathops.cpp`
  - ambiguous fake-edge closure case
  - branch-heavy ambiguous fake-edge closure case
- `test_relation_boolean.cpp`
  - equal intersection / difference
  - simple collinear overlap boolean
  - disjoint union
  - touching difference
  - duplicate-edge family overlap regression
  - repeated-collinear-chain overlap regression
- `test_offset.cpp`
  - narrow bridge inward offset split
  - clockwise/reverse-edge offset recovery
  - inward-hole semantics recovery
- `test_topology_indexing.cpp`
  - equal relation
  - self-contains basic case
  - duplicate-equal parent tie-break
  - shared-edge touching
  - overlap + strict interior intersecting

不要运行测试，只补源码和测试代码。

七、当前还没补完的重点差距
优先继续补这些，不要重复做已经完成的普通线网修补：

Must Close 剩余重点：
1. boolean 在更复杂 repeated-edge family（不仅 degree-2 链）和更难 near-degenerate 交点下的进一步稳健性
2. offset 在 hole/outer 语义翻转与复杂自交输出上的进一步恢复（在现有 fallback 之上）
3. preprocessing / topology recovery 在 search / boolean / offset 之间进一步统一（抽共享入口）
4. relation hierarchy 向 `GGLPolyRelation` 再靠拢（更细 relation 分层）
5. polygon cut with holes

建议优先级：
1. boolean 退化/重叠稳健性
2. offset 复杂恢复
3. preprocessing 统一
4. relation hierarchy 补强
5. polygon cut with holes

八、修改约束
- 不要回退已有改动
- 不要删除用户已有变化
- 保持现有 API 风格
- 尽量在现有模块内增强
- 测试尽量放在现有测试文件中
- 更新 `docs/delphi-geometry-parity.md`
- 不要编译，不要跑构建

九、这次任务要求
请直接继续改代码，不要只停留在分析。
优先选择能提升整体框架能力的改法，而不是只修单个 case。
完成后输出：
1. 这次实现了哪些能力
2. 还剩哪些没有补
3. 建议提交信息 `Msg`
```
