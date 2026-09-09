# SCGeometry 关系能力收敛修复方案

> 状态：执行基线
>
> 目的：收敛当前 `Quantity` 前置能力实现中的 checked 数值语义；保持既定四个公共 API、模块归属和下游边界不变。

> 本文是当前唯一执行基线。本文后续条款优先于历史实现、原实施文档中的示例代码以及此前的局部修补约定。

## 技术债登记（2026-09-09）

本轮交付已完成公共 API、常规有限坐标范围内的 checked 失败传播、专用 arrangement 隔离及测试验证。以下数值鲁棒性工作明确延期，不能视为已完成：

- `Source/Detail/PolygonPositiveAreaIntersection2d.cpp` 中的 `CheckedIntersectSegments` 目前仍调用 `IntersectKernelSegments`，尚未为每个 Line/Arc 段对建立两阶段局部归一化坐标系；
- 因此，极大/极小有限坐标、近切、近乎平行及参数稳定性带附近的正面积查询，尚不能完全保证将所有不可可靠判定情形映射为 `NumericalIndeterminate`；
- 后续重构必须按本文第 2.1 与第 4.1 节实现局部归一化、解析 checked 求交、参数/角度误差界及回归测试；在此之前，Quantity 不得将该 API 用作极端尺度输入的唯一提交放行依据。

当前发布范围限定为常规有限坐标和已覆盖的 Line/Arc 场景。调用方遇到异常尺度或数值敏感输入时，应将失败结果视为前置条件未满足。

## 0. 实施前固定不变量

本次实施按以下不变量执行，任何实现无法满足时必须返回不确定/失败，不得继续猜测：

1. `B_eps` 只在 `Metrics` 公共入口构造一次；局部数值缩放只用于防止溢出/下溢，不改变点、曲线参数、盒边界或关系结果。
2. 所有内部段对求交统一返回 `Success`、`NoIntersection`、`InvalidInput`、`NumericalIndeterminate` 四态。旧 `Intersect*`、`ClosestPoints*` 的空结果不能直接转换为前三种中的任何一种。
3. `NoIntersection` 必须由当前段型的完整解析证明产生；仅有包围盒分离、严格解析判别式分离或完整参数范围证明才可使用。最近点结果不能单独证明无交。
4. 参数范围使用闭区间 `[0,1]` 的 checked 谓词。参数落入误差带时返回 `NumericalIndeterminate`；禁止用 `clamp` 将段外根变成端点交点。
5. 正长度重叠必须输出两个不同端点；不能因重叠长度小于数值阈值而降级为点。数值阈值只用于判定可靠性和事件去重，不用于改变几何关系。
6. 正面积专用 arrangement 不得复用未 checked 的 `SplitAtParameters`、公共 `PointAt`、参数去重、切线角或空事件回退逻辑；这些步骤必须使用局部解析求值和显式状态。
7. 失败映射固定：非有限结果为 `NonFiniteResult`；面/代表点/填充定位无法完成为 `FaceClassificationFailure`；稳定性带内无法判定为 `NumericalIndeterminate`；公开入口前置不满足为 `InvalidInput`。失败不得变成成功 `false`。

实施顺序固定为：先完成并静态审查段-盒内核，再完成正面积段对和专用 arrangement，最后补测试与公共入口审查。除本基线明确要求外，不扩展公共 API、不修改 Quantity/Render、不改动既有通用 arrangement 契约。

## 1. 结论与边界

当前问题不是单个公式错误，而是两类内核混用了不同契约：

- 既有 `SCSegmentIntersection2d` 用 `None` 同时表达“可靠无交”和“无法计算/输入被默认 epsilon 拒绝”；
- 新增局部求交为绕开短边逐项复制求解，但 Line-Line、Arc-Line、无交证明和容差范围没有同一状态机。

因此不得继续在 `CanProveNo...`、`IntersectLocal...` 或最近点结果上增加特例。本方案只重构下列 Detail 实现：

- `Source/Detail/SegmentBoxRelation2d.cpp`；
- `Source/Detail/PolygonPositiveAreaIntersection2d.cpp`，以及只供该查询调用的专用 arrangement 入口；
- 对应能力测试。

不新增公共 API，不修改 `Metrics.h`、`PolygonTopology.h`、`Relation.h` 的既定接口，不修改 Quantity、Render、Boolean 或现有 `BuildCurveArrangement2d` 的调用契约。

## 2. 统一的内部状态规则

两个内核分别保留私有实现，但都必须使用同一语义：

```cpp
enum class CheckedPairStatus2d
{
    Success,                 // 有有限且经方程/参数验证的交点或重叠区间
    NoIntersection,          // 当前段型的解析方程已可靠证明无公共点
    InvalidInput,
    NumericalIndeterminate
};
```

规则如下：

1. `NoIntersection` 只能由当前段型的完整解析分支产生，不能来自公共 `Intersect` 的 `None`、最近点距离、空事件或默认构造值；
2. 任一除零、溢出、非有限中间值、不能稳定比较的判别式或参数都返回 `NumericalIndeterminate`；
3. 调用方 `eps` 只用于构造 `B_eps` 或多边形规范化；局部求解阈值只能用于验证计算误差，不能放宽参数区间、把正间隙变成交点，或把正长度重叠降为点；
4. 交点必须同时验证：点有限、两个参数位于精确闭区间 `[0, 1]`、两侧重求值与交点一致；
5. 重叠必须保留两个不同端点；只有精确零长度的交集才是单点。

### 2.1 数值归一化与稳定性带

“局部缩放”只用于中间计算，绝不是第二个几何容差：它不得移动输入、不得改变段参数、不得改变 `B_eps`，也不得将段外参数钳制到 `[0, 1]`。

- 段-盒继续使用由 `B_eps` 决定的平移加**统一比例**局部坐标；
- 正面积的每个段对按固定两阶段建立仅用于该段对的平移加统一比例局部坐标，禁止先直接相减：
  1. 收集参与端点、圆心和半径；以所有坐标分量绝对值**及所有半径绝对值**的最大有限值建立 `coordinateScale`。若该最大值为零但输入段仍有效，使用 `1.0` 作为无量纲单位回退；坐标分量和半径必须同时除以同一 `coordinateScale`，再在预缩放空间计算安全中心；
  2. 在预缩放且已平移的空间计算弦长、圆心差和**已预缩放半径**的最大有限正量 `pairScale`，再按 `1 / pairScale` 做统一比例归一化。
  除明确的 `1.0` 单位回退外，`coordinateScale`、`pairScale`、倒数或任一变换结果不可表示、非有限或为零时返回 `NumericalIndeterminate`；不得在原始极端坐标上先计算 `end - start`、圆心差或长度平方；
- 统一比例变换保持 Line/Arc 类型、圆弧角度及参数 `t` 不变。所有交点和参数验证均在该局部坐标完成，输出点再逆变换并验证有限性；
- 对行列式、二次判别式、同圆判定和角区间端点，不使用 `value == 0.0` 或固定绝对阈值。每个谓词必须以归一化局部坐标的 IEEE `double` 单位舍入 `u = std::numeric_limits<double>::epsilon()` 为基础，按实际乘加项的绝对值和计算 `errorBound = C * u * sumAbsTerms`；`C` 必须在对应辅助函数中以具名常量给出并覆盖该函数的有限基本运算次数，不能由调用点随意传入：
  - `value > errorBound`：可靠正；
  - `value < -errorBound`：可靠负；
  - `abs(value) <= errorBound`：只有可由代数恒等式再次确认时才进入零/重合分支，否则返回 `NumericalIndeterminate`。

参数和角区间也必须使用同一三态谓词，而非“浮点精确比较”：可靠位于 `[0, 1]` 内、可靠位于范围外、端点稳定性带内。第三种只能通过将参数钳至端点后重求值并验证原方程残差不超过对应 `errorBound` 后确认端点；无法确认时返回 `NumericalIndeterminate`。这不是几何范围放宽。

局部稳定性带只决定“成功”还是“不确定”；它不能产生交点、删除交点、扩张盒或放宽参数范围。

## 3. 段-盒内核重构

`SegmentBoxRelation2d.cpp` 只保留一个局部坐标框架和一组完整求交器：

| 段型 | 局部盒边 | 解析方法 |
| --- | --- | --- |
| Line | Line | 行列式、共线投影区间 |
| Arc | Line | 圆-线二次方程、圆弧角区间 |

实施要求：

- 删除以公共 `Intersect` 空结果为输入的 `CanProveNoLocalIntersection` 回退；完整解析器直接返回 checked 状态；
- Line-Line 的平行、共线、参数范围和重叠区间均以精确几何区间决定；遇到接近零但无法稳定决定的行列式返回 `NumericalIndeterminate`，不得 clamp；
- Arc-Line 对短盒边也使用同一局部二次方程。判别式、根、角度和参数有任一不稳定时返回 `NumericalIndeterminate`；
- `ClassifyLineOrArc` 只消费 checked 成功/可靠无交结果；任一不确定立即映射为 `IntersectionSolveFailure`；
- 保留一次 `B_eps` 构造、一次局部变换、无额外扩张和既有成功关系映射。

## 4. 正面积查询重构

### 4.1 checked 段对求交

在 `PolygonPositiveAreaIntersection2d.cpp` 实现只供该查询使用的完整 checked 求交，覆盖：

| 段对 | 可靠无交条件 |
| --- | --- |
| Line-Line | 平行非共线，或共线投影严格分离，或非平行参数严格落在范围外 |
| Line-Arc | 二次判别式严格为负，或所有有限根均严格在线段/圆弧参数范围外 |
| Arc-Arc | 同圆时角区间严格分离；异圆时圆心距离与半径关系严格证明分离，或所有有限交点均在至少一条圆弧范围外 |

每个有交结果都需验证点和两个参数；不能证明的边界情形返回 `NumericalIndeterminate`。不得使用 `ClosestKernelSegments` 作为无交证明。

Arc-Arc 必须按下列互斥分支实现，不能只依赖圆心距离的近似比较：

1. 先用第 2.1 节的误差界分类圆心距离和半径差；落在不确定带时失败；
2. 确认同圆后，将每条圆弧规范为带方向的半开角区间；完整圆单独标记，不以起止角相等推断；求出共同角区间：空集为可靠无交，单角为点，正角长度为重叠并输出两个端点；
3. 确认异圆后，先可靠判定外离、内含不接触、外切、内切或两点相交；外离和内含不接触为可靠无交；
4. 对切点/两交点，逐点验证圆方程、两条圆弧角区间和两个参数；任一验证不稳定则返回 `NumericalIndeterminate`；
5. 反向圆弧、端点重合、同圆不同分段和完整圆均必须遵循同一角区间规则。

### 4.2 专用 checked arrangement

现有 `BuildCurveArrangement2d` 的 `SCSegmentIntersection2d` 空结果没有 checked 状态，不能作为本查询的最终依据。新增仅供正面积查询调用的专用入口（可放在 `PolygonPositiveAreaIntersection2d.cpp` 或新增 Detail 文件）：

- 输入和输出保持当前 `BuildPositiveAreaArrangement2d` 所需的 piece/face 信息；
- 事件收集、参数排序去重、分段和面构建使用第 4.1 节 checked 段对结果；
- 不修改既有 `BuildCurveArrangement2d` 的函数签名或其他消费者；
- 任一段对、事件、分段或面构建不确定时，向上返回 `NumericalIndeterminate`；
- 仅在所有面完成面积、代表点和两个填充集严格定位后返回成功。

专用入口不得在 checked 结果之后复用下列未检查步骤：现有 `SplitAtParameters`、段的公共 `PointAt()`、现有参数 `AddParameter` 去重、现有 `TangentAngle`，以及任何以空事件或默认点继续执行的逻辑。它必须提供相应的私有 checked 替代：

- 参数插入、排序和去重以第 2.1 节三态比较执行；无法决定顺序或是否同一事件时返回 `NumericalIndeterminate`；
- 分段使用 Line/Arc 的局部解析求值构造，验证两个端点、参数单调性和新段有效性；不调用带默认 epsilon 的 `PointAt()`；
- 切线角由局部解析导数计算：Line 使用方向向量，Arc 使用半径的正交方向；导数长度或角排序不稳定时返回 `NumericalIndeterminate`；
- 面代表点、面积和严格定位继续沿用 checked 状态，任何空结果或默认值均不得作为 `Outside`、零面积或无事件解释。

这样可同时满足圆弧严格包含、同圆边界、等价分段、孔洞和交换对称性，不再依赖“包围盒分离才允许无交”的过度收缩策略。

### 4.3 状态映射

内部状态不得在跨层时隐式折叠，固定映射如下：

| 来源 | `InvalidInput` | `NumericalIndeterminate` | 正常 `NoIntersection` |
| --- | --- | --- | --- |
| 段-盒局部段对 | `SCBoxSegmentFailure2d::InvalidSegment` | `SCBoxSegmentFailure2d::IntersectionSolveFailure` | 不产生事件，继续分类 |
| 段-盒事件排序/代表点 | 不适用 | `EventOrderingFailure` / `RepresentativePointFailure` | 不适用 |
| 正面积跨输入段对 | `PositiveAreaStatus2d::NumericalIndeterminate`（输入已在规范化入口验证） | `PositiveAreaStatus2d::NumericalIndeterminate` | 不产生交点事件 |
| 正面积专用 arrangement 的事件、去重、分段 | 不适用 | `PositiveAreaStatus2d::NumericalIndeterminate` | 不适用 |
| 正面积面面积、代表点、严格定位 | 不适用 | 见下方固定细分规则 | 不适用 |

公共层继续保持既定不变量：段-盒任一失败使两个 bool 返回 `false`；正面积失败使 `success == false`、`hasPositiveAreaIntersection == false` 且 `failure != None`。任何失败都不得转换为 `Disjoint` 或成功的 `false`。

正面积面处理的失败映射固定为：

- 面面积、代表点坐标、逆变换结果、方程重求值或最终计算结果出现非有限值：`SCPolygonPositiveAreaIntersectionFailure2d::NonFiniteResult`；
- 面环无法构造、代表点无法构造、代表点无法通过 checked 严格环定位、或填充定位因面结构无法完成：`SCPolygonPositiveAreaIntersectionFailure2d::FaceClassificationFailure`；
- 行列式、判别式、同圆关系、角区间、参数顺序或事件归属落入稳定性不确定带：`SCPolygonPositiveAreaIntersectionFailure2d::NumericalIndeterminate`；
- 公开入口参数、段类型或多边形结构在规范化前不满足前置条件：`InvalidInput`；专用 arrangement 已接收规范化输入后，不得再把内部异常伪装成 `InvalidInput`。

## 5. 测试与验收

只增加与上述契约直接对应的用例：

- 段-盒：局部短边上的 Line 穿越、Arc 穿越、完整边界重叠、角外极近掠过但不命中；
- 正面积：同心圆严格包含、圆弧边界相离但包围盒重叠、同圆圆弧重叠/接触、Line-Arc 与 Arc-Arc 的交换对称性；
- 数值稳定性：极大/极小有限坐标的段对局部归一化、稳定性带内返回不确定、稳定性带外按可靠正负分支分类；
- 每个 checked 分支分别断言可靠无交、成功交点和数值不确定的失败传播；
- 维持已有公共头、公共 API、孔洞、正面积和极端尺度回归用例。

验收标准：所有公共成功结果满足既定失败字段不变量；`Contains` / `Intersects` 不会因内部不确定产生假命中；`QueryPolygonPositiveAreaIntersection` 对可可靠判定的 Line/Arc 包含关系成功返回 `true`，不能可靠判定时失败而非成功返回 `false`。

## 6. 实施顺序

1. 先完成段-盒的局部 checked Line-Line、Arc-Line，并删除混合回退；
2. 为正面积实现 checked Line-Line、Line-Arc、Arc-Arc 及局部测试；
3. 将正面积的 arrangement 事件来源替换为专用 checked 入口；
4. 补齐上述回归测试与 umbrella 覆盖；
5. 按仓库离线规则，仅做静态一致性、`git diff --check` 和设计对照；构建/测试由用户环境执行。
