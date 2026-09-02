# D4 Body Boolean 能力矩阵

本矩阵记录当前已实现且具有确定性契约的子集；它不承诺通用三维 CSG。

| 输入/场景 | Intersect | Union | Difference | 结果契约 |
| --- | --- | --- | --- | --- |
| 空 body、开口 body、非流形 body、无效容差，或平面 face 的边界未落在支撑平面上 | 不支持 | 不支持 | 不支持 | `InvalidInput`，`IsSuccess()==false`，且不产生部分结果 |
| 可确认几何等价或包围盒相离的直线边闭合 body | 支持 | 支持 | 支持 | `issue=None`；空结果使用 `producedEmptyResult`；等价判断同时比较 face 支撑面和边界环，不以拓扑计数或包围盒近似；曲线边不进入快速路径 |
| 轴对齐的内含盒 | 支持 | 支持 | 支持 | 当既有输入恰好表示结果时复用该输入 body |
| 轴对齐单盒重叠 | 支持 | 支持 | 仅当余体仍为一个盒时支持 | 成功的非空单 body 结果为一个有效闭合 shell |
| 轴对齐且面连通的盒并集 | 不适用 | 支持 | 不适用 | 两个已识别盒可形成 L 形或其他直角单 shell；体积遵循容斥关系 |
| 不含重合支撑平面的单 shell、直线边平面凸多面体（包括旋转盒） | 支持正体积相交 | 不支持 | 不支持 | 输出为一个有效闭合 shell；交换输入不改变体积和包围盒 |
| 仅共边或共点接触 | 确定性空相交 | 确定性有序多 body 并集 | 原 body 子集 | `issue=None`；不生成零体积 shell |
| 曲线边界、重合支撑平面、旋转体低维接触、非凸 body、多 shell body、一般并集或一般差集 | 不支持 | 不支持 | 不支持 | `UnsupportedOperation`，`IsSuccess()==false`，且不产生部分结果 |

## 拓扑不变量

每个成功的非空单 body 结果均满足：

1. `body.IsValid()` 为真。
2. `ShellCount()==1`，唯一 shell 为闭合 shell，并通过内部闭合流形验证：每条边恰被两个方向相反的 coedge 使用，所有 face loop 连续闭合。
3. 对平面 face，其 outer loop 与所有 hole loop 顶点均在支撑平面的距离容差内。
4. `IsSuccess()` 与非空 `body`、非空 `bodies` 或 `producedEmptyResult` 一致。
5. 对面连通轴对齐盒并集，结果体积在容差内等于 `volume(A)+volume(B)-volume(intersection)`，交换输入不改变体积或包围盒。
6. 对不含重合支撑平面的凸多面体相交，结果体积和包围盒对输入顺序不变；测试使用独立的半平面裁剪 oracle 回归旋转棱柱盒样本。

通用 CSG、shell 编辑事务和广义拓扑修复仍在当前确定性子集之外。面连通直角盒并集的设计与回归范围见 `d4-orthogonal-box-union-design.md`；通用平面凸体相交的分阶段设计见 `d4-general-polyhedral-boolean-design.md`。
