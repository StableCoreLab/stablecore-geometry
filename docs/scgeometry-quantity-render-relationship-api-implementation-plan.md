# SCGeometry Quantity 前置能力与多段线盒关系 API 实施文档

> 状态：实施中（以 `scgeometry-quantity-render-relationship-api-remediation-plan.md` 为当前执行基线）
>
> 范围：为 Quantity 与 Render 提供统一的二维关系能力。本次只交付 SCGeometry 的公共 API、内核与测试；不修改 Quantity 或 Render 的业务实现。

## 1. 决策与交付门槛

Quantity 不得自行实现投影、多边形布尔、面积或容差算法。只有下列两项 API 已在 SCGeometry 完成实现、测试、公共头验证及安装包消费者验证后，Quantity 才能开始相关方案：

```cpp
SCParallelSegmentProjectionRelation2d ClassifyParallelSegmentProjection(
    const Geometry::SCLineSegment2d& first,
    const Geometry::SCLineSegment2d& second,
    const Geometry::SCParallelSegmentProjectionTolerance2d& tolerance);

SCPolygonPositiveAreaIntersectionResult2d QueryPolygonPositiveAreaIntersection(
    const Geometry::SCPolygon2d& first,
    const Geometry::SCPolygon2d& second,
    double eps = Geometry::kDefaultEpsilon);
```

Render 精确框选同时需要下列正式能力：

```cpp
bool Contains(const Geometry::SCBox2d& box,
              const Geometry::SCPolyline2d& polyline,
              double eps = Geometry::kDefaultEpsilon);

bool Intersects(const Geometry::SCBox2d& box,
                const Geometry::SCPolyline2d& polyline,
                double eps = Geometry::kDefaultEpsilon);
```

交付前，`SCSceneSnapshotHitTestService::PreciseBoxHit` 中的 `PolylineContainedInBox` 与 `PolylineIntersectsBox` 只能作为 Render 临时实现；Quantity 不得依赖它们。

四项能力按下列顺序交付和消费：

1. 先交付平行投影分类和多边形正面积交集查询，作为 Quantity 合法性校验方案的前置条件；
2. 再交付盒-多段线 `Contains` 与 `Intersects`，作为 Render 精确框选技术债的替换条件；
3. 本方案只修改 SCGeometry 的公共 API、内核、测试和安装包验证，不在本轮修改 Quantity 或 Render；
4. Quantity 与 Render 的业务接入分别由下游仓库在对应前置能力交付后执行。

## 2. 模块职责与文件落点

四项能力不应全部堆入 `Metrics`。按现有公共 API 的职责划分如下：

| 能力 | 公共头与实现 | 原因 |
| --- | --- | --- |
| 平行线段投影分类 | `Include/Core/Relation.h` / `Source/Core/Relation.cpp` | 与既有 `IsParallel`、`IsSameDirection` 同属几何关系，而非度量 |
| 多边形正面积交集查询 | `Include/Core/PolygonTopology.h` / `Source/Core/PolygonTopology.cpp` | 依赖规范化、曲线 arrangement 与填充集合关系 |
| 盒-多段线包含与相交 | `Include/Core/Metrics.h` / `Source/Core/Metrics.cpp` | 扩展既有盒-点 `Contains`、盒-盒 `Intersects` |
| 段-盒数值内核 | `Source/Detail/SegmentBoxRelation2d.h/.cpp` | 仅供 SCGeometry 使用，统一 Line/Arc 的规范化事件计算，不暴露零容差求交契约 |
| 局部可测试辅助函数 | `Source/Detail/SegmentBoxRelation2d.h` / `Source/Detail/PolygonPositiveAreaIntersection2d.h` | 仅声明与实现纯验证、排序和聚合不变量；不导出、不安装、不建立跨内核操作集 |
| 聚合入口 | `Include/Core/GeometryApi.h` | 必须新增 `#include "Core/Metrics.h"`；`Relation.h` 和 `PolygonTopology.h` 已被聚合 |

`Metrics.h` 不声明多边形正面积交集类型或函数，因此不需要引入 `SCPolygon2d`。这避免无关的头文件依赖扩散，也使 API 的归属与实现内核一致。

`SCPolyline2d` 在这里表示曲线路径，而不是闭合后的填充区域。因此闭合多段线围住选择框、但路径本身不触及选择框时，`Intersects(box, polyline)` 返回 `false`。若 Render 将图元定义为填充面，必须在 Render 层以该图元的正式 `SCPolygon2d` 组合路径关系与填充包含关系；不得改变通用 `SCPolyline2d` 盒关系的语义。

## 3. 公共 API 契约

### 3.1 平行线段投影分类

在 `Relation.h` 增加：

```cpp
enum class SCParallelSegmentProjectionRelation2d
{
    InvalidInput,
    NonParallel,
    NoPositiveLengthIntersection,
    EndpointTouch,
    PositiveLengthIntersection
};

struct GEOMETRY_API SCParallelSegmentProjectionTolerance2d
{
    double angularEpsilon{Geometry::kDefaultEpsilon};
    double projectionEpsilon{Geometry::kDefaultEpsilon};

    [[nodiscard]] bool IsValid() const;
};

[[nodiscard]] GEOMETRY_API SCParallelSegmentProjectionRelation2d
ClassifyParallelSegmentProjection(const SCLineSegment2d& first,
                                  const SCLineSegment2d& second,
                                  const SCParallelSegmentProjectionTolerance2d& tolerance);

```

该函数分类两条线段在 `first` 方向上的一维投影区间；它不要求两线段共线。`InvalidInput` 表示线段为零长度、坐标/中间计算非有限，或容差无效。端点不同但长度不超过 `projectionEpsilon` 的短线段仍是有效输入，其投影关系按下表分类。调用方不得把 `InvalidInput` 解释为不平行或无交集。

`SCParallelSegmentProjectionTolerance2d` 将角度与长度容差分离：`angularEpsilon` 是单位方向叉积绝对值的无量纲阈值，且合法范围严格为 `0 < angularEpsilon < 1`；`projectionEpsilon` 是投影区间的坐标单位阈值，必须有限且为正。由于单位向量叉积绝对值不超过 `1`，`angularEpsilon >= 1` 会使任意方向都被误判为平行，必须视为无效容差，而不是扩大平行判定范围。Quantity 必须使用该类型化容差接口。

对有效输入，先以溢出安全方式将方向向量归一化：以每个向量最大绝对分量缩放后再求长度和单位向量。令 `unitFirst`、`unitSecond` 为结果；当且仅当 `abs(Cross(unitFirst, unitSecond)) <= angularEpsilon` 时视为平行。不得计算 `eps * Length(u) * Length(v)`，以免极大有限坐标溢出。`angularEpsilon` 等于 `1`、略大于 `1` 或更大时不得进入方向分类，必须返回 `InvalidInput`。

投影使用以 `first.StartPoint()` 为原点、`unitFirst` 为轴的溢出安全局部坐标。令区间重叠量为 `min(maximum) - max(minimum)`：

| 条件 | 返回值 |
| --- | --- |
| 单位方向叉积绝对值大于 `angularEpsilon` | `NonParallel` |
| 重叠量小于 `-projectionEpsilon` | `NoPositiveLengthIntersection` |
| 重叠量位于 `[-projectionEpsilon, projectionEpsilon]` | `EndpointTouch` |
| 重叠量严格大于 `projectionEpsilon` | `PositiveLengthIntersection` |

这四种有效关系互斥且完备；反向线段不得改变分类。

`ClassifyParallelSegmentProjection` 返回 `InvalidInput` 时，调用方不得继续解释为 `NonParallel`、`NoPositiveLengthIntersection` 或 `EndpointTouch`；Quantity 必须将该结果转换为 `PreconditionFailed` 并拒绝提交。只有前三种非失败关系才可作为正常筛选结果，`PositiveLengthIntersection` 才进入轮廓正面积查询。

### 3.2 多边形正面积交集查询

在 `PolygonTopology.h` 增加：

```cpp
enum class SCPolygonPositiveAreaIntersectionFailure2d
{
    None,
    InvalidInput,
    NormalizationFailure,
    ArrangementFailure,
    FaceClassificationFailure,
    NumericalIndeterminate,
    NonFiniteResult
};

struct GEOMETRY_API SCPolygonPositiveAreaIntersectionResult2d
{
    bool success{false};
    bool hasPositiveAreaIntersection{false};
    SCPolygonPositiveAreaIntersectionFailure2d failure{
        SCPolygonPositiveAreaIntersectionFailure2d::InvalidInput};
};

[[nodiscard]] GEOMETRY_API SCPolygonPositiveAreaIntersectionResult2d
QueryPolygonPositiveAreaIntersection(const SCPolygon2d& first,
                                     const SCPolygon2d& second,
                                     double eps = Geometry::kDefaultEpsilon);
```

该查询只回答本轮调用方实际需要的面积事实，不公开 `Equal`、包含方向、边界接触或不相交等完整拓扑类别。`eps` 用于输入规范化和普通浮点稳健控制，但不是面积阈值；对于有效输入，`hasPositiveAreaIntersection == true` 当且仅当两个填充集合存在可可靠判定的正面积公共区域。成功且该字段为 `false` 只能表示确认不相交或仅边界接触/重合。规范化、参数合并或面积过滤不得把原始输入中实际存在的正面积公共区域成功判定为无交集；如果这些步骤可能丢失该事实，必须返回 `NumericalIndeterminate`。失败结果必须满足 `success == false`、`failure != None`，且调用方不得将失败解释为无冲突。局部尺度、求交结果、事件顺序、面积符号或严格内部判定无法可靠区分时，必须返回 `NumericalIndeterminate`，不得成功返回 `false`。

`SCPolygonPositiveAreaIntersectionFailure2d` 仅服务于本查询，不修改既有 `SCPolygonTopologyFailure` 及 `NormalizePolygon`、`ClassifyContainment` 等 API 的返回契约。

孔洞是负空间。局部重叠、任一方向包含和填充集合完全相等均返回成功且 `hasPositiveAreaIntersection == true`；角/共边接触但无公共内部区域返回成功且为 `false`。输入无效、`eps` 无效、规范化、arrangement、面代表点或非有限计算失败时返回对应失败值。

### 3.3 多段线与盒关系

在 `Metrics.h` 的二维盒关系后增加：

```cpp
[[nodiscard]] GEOMETRY_API bool Contains(const SCBox2d& box,
                                         const SCPolyline2d& polyline,
                                         double eps = Geometry::kDefaultEpsilon);
[[nodiscard]] GEOMETRY_API bool Intersects(const SCBox2d& box,
                                           const SCPolyline2d& polyline,
                                           double eps = Geometry::kDefaultEpsilon);
```

唯一的几何容差区域为扩张闭盒 `B_eps`：

```text
min = box.MinPoint() - (eps, eps)
max = box.MaxPoint() + (eps, eps)
```

`Intersects` 在且仅在路径与 `B_eps` 有公共点时返回 `true`；`Contains` 在且仅在路径的全部曲线点位于 `B_eps` 时返回 `true`。完全在盒内、穿越盒、与边或角接触均视为相交。空或零段多段线不可选择，返回 `false`。

`box`、`polyline`、`eps` 无效时返回 `false`；构造 `B_eps` 后任一坐标非有限也返回 `false`。调用方 `eps` 只用于构造 `B_eps`，后续内核不得再次把它用作几何扩张。

## 4. 实现设计

### 4.1 平行投影内核

在 `Relation.cpp` 的匿名命名空间实现溢出安全的二维方向归一化和局部投影辅助函数。方向差、局部坐标差、点积或最终区间端点一旦非有限，函数返回 `InvalidInput`。零长度线段返回 `InvalidInput`；端点不同的短线段仍参与正常投影分类，投影交集量小于等于 `projectionEpsilon` 时不得返回 `PositiveLengthIntersection`。不得复用仅以绝对叉积比较的现有 `IsParallel` 实现，也不得在 Quantity 复制此逻辑。

### 4.2 多边形正面积交集内核

在 `PolygonTopology.cpp` 复用 `NormalizePolygon` 的输入检查和 `Detail` 中已有的曲线/面构造基础能力，不得调用面向结果重建的 `Boolean` API 后再用 `Area()` 猜测结果，也不得在 `Metrics.cpp` 重写布尔或面积算法。`SCPolygon2d` 或 `eps` 本身无效时返回 `InvalidInput`；输入通过检查但规范化过程失败时返回 `NormalizationFailure`。正面积查询使用专用的内部 `BuildPositiveAreaArrangement2d` 流程，不改变既有 `BuildCurveArrangement2d(rings, tolerance)` 的调用契约；该流程的求交、事件、参数排序和面面积判定必须返回 checked 状态，不能把调用方 `eps` 作为正面积阈值。构建失败返回 `ArrangementFailure`，数值无法可靠判定返回 `NumericalIndeterminate`。

内部 checked 辅助必须有可执行的状态契约，不能用空结果或默认值表达失败。例如统一使用以下内部状态，并由求交、事件排序、参数去重、面积符号和严格定位辅助返回：

```cpp
enum class SCCheckedGeometryStatus2d
{
    Success,
    NoIntersection,
    InvalidInput,
    NumericalIndeterminate
};

struct SCCheckedIntersection2d
{
    SCCheckedGeometryStatus2d status = SCCheckedGeometryStatus2d::InvalidInput;
    SCSegmentIntersection2d intersection{};
};
```

其中 `NoIntersection` 是已可靠判定的正常结果，不是失败；`InvalidInput` 和 `NumericalIndeterminate` 必须向上分别映射为公开失败原因。事件排序、参数去重、面积判定和严格定位可复用该状态语义，但不得以“没有产生事件”“面积为零”或 `Outside` 默认值代替失败状态。

`BuildPositiveAreaArrangement2d` 的 checked 求交、事件排序、参数去重和面面积判定必须显式返回成功、普通无交集/零面积或不确定状态；任何不确定状态向上返回 `NumericalIndeterminate`。对每个可可靠判定为正面积的面，必须构造候选代表点，并先通过内部 checked 定位确认该点严格位于该面环内部，才可对两个规范化多边形执行填充定位。checked 定位的内部结果必须包含 `success`、`SCPointContainment2d containment` 和失败原因；不得复用会把无效或无法判定情形回退为 `Outside` 的公开 `LocatePoint` 作为失败判定依据。代表点无法构造、任一 checked 定位失败或无法严格位于目标面、面面积或代表点非有限时返回 `FaceClassificationFailure` 或 `NonFiniteResult`，不得跳过该面。必须完成所有面验证：任一面分类失败优先返回失败；仅在所有面均验证成功后，才依据是否存在至少一个同时严格位于两个填充集合内部的代表点返回成功的 `true` 或 `false`。这一定义不需要输出面归属、包含方向或边界接触类别，但必须正确处理孔洞、共边、等价分段和完全相等。交换输入时，`success`、`failure` 与 `hasPositiveAreaIntersection` 均不变。

### 4.3 多段线与盒精确内核

在 `Source/Detail/SegmentBoxRelation2d.h/.cpp` 实现内部分类；`Metrics.cpp` 不得复制该枚举或分类逻辑，两个公开 bool 都只能调用该内核并由其结果派生。内部结果必须显式区分成功关系与失败，失败不得编码为 `Disjoint`：

```cpp
enum class SCBoxSegmentRelation2d
{
    Unknown,
    Disjoint,
    Touching,
    Contained,
    ContainedTouching,
    Crossing
};

enum class SCBoxSegmentFailure2d
{
    None,
    InvalidSegment,
    NonFiniteInput,
    LocalTransformFailure,
    IntersectionSolveFailure,
    EventOrderingFailure,
    RepresentativePointFailure
};

struct SCBoxSegmentClassification2d
{
    bool success{false};
    SCBoxSegmentRelation2d relation{SCBoxSegmentRelation2d::Unknown};
    SCBoxSegmentFailure2d failure{SCBoxSegmentFailure2d::InvalidSegment};
};
```

成功结果必须满足 `success == true`、`failure == None` 且 `relation != Unknown`；失败结果必须满足 `success == false`、`failure != None` 且 `relation == Unknown`。`ClassifySegmentBox` 返回上述结构。任一段分类失败时，本次 `Contains(box, polyline, eps)` 与 `Intersects(box, polyline, eps)` 都必须立即返回 `false`；不得跳过失败段、继续聚合其余段，或将失败段当作 `Disjoint`。公开 API 只允许在所有段均成功后依据 `relation` 聚合结果。

对每次公共调用只构造一次 `B_eps`，并对每个 Line/Arc 段调用 `Detail::ClassifySegmentBox(segment, B_eps)`。映射固定为：

| 内部关系 | `Contains` | `Intersects` |
| --- | --- | --- |
| `Disjoint` | false | false |
| `Touching` | false | true |
| `Contained` | true | true |
| `ContainedTouching` | true | true |
| `Crossing` | false | true |

`Unknown` 仅可存在于失败结果，不参与公开 bool 映射；内核应以成功/失败工厂函数构造结果，避免手工填充不满足上述不变量的结构。

先使用 `SegmentBoxRelation2d` 内核的无额外几何容差盒-盒粗筛判断 `segment.Bounds()` 与 `B_eps`。粗筛及后续事件计算均不得调用公开 `Intersects(..., 0.0)` 或 `Intersect(..., 0.0)`，避免把零容差作为公共 API 契约。

内核先建立一个由 `B_eps` 决定的局部坐标变换（平移到盒中心，并按最大边长缩放）。盒中心必须用溢出安全形式计算（例如先分别除以二再相加），最大边长和缩放因子非有限或为零时返回 `LocalTransformFailure` 并由公开 bool 返回 `false`。该变换必须同时应用于 `B_eps`、待测 `SCLineSegment2d` 的两个端点，以及待测 `SCArcSegment2d` 的圆心和半径；不得只变换盒而在全局坐标上计算曲线。所有 Line/Arc 求交、圆弧参数事件、`PointAt()` 代表点和边界分类都在同一局部几何中完成，段参数 `t` 保持与原段一致。求交解算无法产生有限且满足方程/参数范围的结果时返回 `IntersectionSolveFailure`；事件无法产生有限参数、排序或去重不满足单调性时返回 `EventOrderingFailure`；代表点 `PointAt()` 或其坐标非有限时返回 `RepresentativePointFailure`。这些失败必须沿 `SCBoxSegmentClassification2d` 原样传播。数值稳健阈值仅用于解方程时的浮点误差控制、参数排序和同一事件去重；不得扩大或缩小 `B_eps`，也不得使用调用方 `eps` 进行第二次几何放宽。对边界重叠，必须保留重叠参数区间的两个端点并标记区间内部为边界部分。

将分割参数排序去重后，对相邻参数的开区间在参数中点调用精确 `PointAt()` 并判断相对 `B_eps` 的位置；这是拓扑分区判定，不是几何离散采样。端点、交点和重叠区间单独处理，以区分 `Touching`、`ContainedTouching` 与 `Crossing`。没有边界事件时，连通性保证仅凭段起点即可区分 `Contained` 与 `Disjoint`。

仅支持当前内建的 `SCLineSegment2d` 与 `SCArcSegment2d`。未来增加新的 `ISCSegment2d` 类型时，必须先为该类型扩展精确段-盒内核和测试；不得降级为折线采样。

## 5. 容差、数值与复杂度

- 每个公共入口先验证其适用的有限正容差；其中类型化平行投影容差的 `angularEpsilon` 必须满足 `0.0 < angularEpsilon < 1.0`，`projectionEpsilon` 必须有限且为正。盒与多边形 API 的几何长度容差不适用 `angularEpsilon < 1` 的上界；
- Quantity 调用平行投影分类时必须使用 `SCParallelSegmentProjectionTolerance2d`，不得用单一数值同时表达角度与长度两个量纲；
- 盒关系仅在 `B_eps` 构造时应用调用方 `eps`；`SegmentBoxRelation2d` 的局部数值稳健策略不作第二次几何放宽；
- 段-盒与多边形内核不得因测试引入共享操作集、依赖注入、测试 DLL 导出或额外生产构建。段-盒保留真实输入端到端测试；Polygon 仅对 `BuildPositiveAreaArrangement2d` 所需的 checked 求交、事件、面积符号和定位纯辅助函数做局部测试，不要求为不可构造的内部路径建立模拟运行环境。安装包消费者验证维持既有“安装当前构建后编译并运行消费者”的链路；
- 盒-多段线为 O(S)，S 为段数，每段最多与四条盒边求交；局部坐标变换为每段常数开销。多边形正面积交集查询的复杂度遵循共享 arrangement 内核。

## 6. 单元测试与验证

新增独立的具名用例，避免聚合为一个 `CoversCurrentCapabilities`：

| 文件 | 覆盖 |
| --- | --- |
| `UnitTests/Capabilities/Core/TestRelationClassification.cpp` | 类型化角度/投影容差独立生效、`angularEpsilon == 1` 与大于 `1` 按无效容差处理、略小于 `1` 的合法临界容差仍可分类、同向/反向正长度重叠、分离、端点接触、`-projectionEpsilon`/`projectionEpsilon` 两侧、非平行、零长度输入、非有限容差、极大有限坐标、极短非退化线段按正常关系分类 |
| `UnitTests/Capabilities/Core/TestPolygonPositiveAreaIntersection.cpp` | 不相交、角/共边接触均成功且 `hasPositiveAreaIntersection == false`；部分重叠、双向包含、非严格包含且接触、完全相等均成功且为 `true`；孔洞负空间；小于 `eps` 的狭窄正面积交集必须返回 `true` 或 `NumericalIndeterminate`，不得成功返回 `false`；真实输入覆盖无效输入、无效容差、arrangement 歧义及可构造的规范化/非有限失败；局部 checked 求交、事件、面积符号、代表点/定位失败和失败优先的面聚合；交换对称性；所有成功结果断言 `failure == None`，所有失败结果断言 `hasPositiveAreaIntersection == false && failure != None`；外环起始段轮换、整体反向和等价分段不同仍返回 `true` |
| `UnitTests/Capabilities/Core/TestMetricsPolylineBox.cpp` | 五种成功 `SCBoxSegmentRelation2d` 到公开 bool 的映射；整段沿 `B_eps` 边界的 `ContainedTouching`；边/角接触、穿越、完全在外、距原盒小于 `eps`、跨原盒边界但仍完全在 `B_eps` 内、Line/Arc 混合、圆弧端点外但穿盒、圆弧端点内而中段越界、闭合路径围盒但不接触、无效输入、`B_eps` 溢出、局部坐标极大/极小量级回归；真实输入覆盖无效段、非有限输入及可构造局部变换失败；对无法由稳定真实输入构造的 checked 防御分支做最小局部测试，并断言失败结果均为 `success == false && relation == Unknown && failure != None`，同时断言公开 `Contains` 与 `Intersects` 均为 `false` |

在 `TestUmbrella.cpp` 中仅包含 `Geometry.h` 并调用 `ClassifyParallelSegmentProjection`、`QueryPolygonPositiveAreaIntersection`、`Contains` 与 `Intersects`，验证聚合入口。安装包消费者测试维持既有安装当前构建、配置、编译并运行消费者的流程。执行 Debug 与 RelWithDebInfo 的 `SCGeometryCapabilitiesTests`、`SCGeometryPublicHeadersCompile`、安装包消费者测试及 `git diff --check`。

## 7. Render 接入与完成标准

SCGeometry 交付通过后，Render 仅替换 `SCSceneSnapshotHitTestService::PreciseBoxHit` 中手写的逐段端点、盒边求交和曲线近似逻辑；不得将现有 Crossing 逻辑机械替换为一次 `Geometry::Intersects(selectionBox, polyline, eps)` 调用。

- Window：对每条正式路径调用 `Geometry::Contains(selectionBox, polyline, eps)`；
- Crossing 的路径关系：对每条正式路径调用 `Geometry::Intersects(selectionBox, polyline, eps)`；
- Crossing 的填充面关系：仅当 Render 已将图元正式构造成带洞 `SCPolygon2d` 时，在边界路径关系之外，对选择框角点调用 `Geometry::ContainsPoint(formalPolygon, corner)`。任一角点位于填充面内即命中；洞内角点不命中；
- 不得以任意闭合 `SCPolyline2d` 的隐含内部替代正式 `SCPolygon2d`，也不得为此向 SCGeometry 新增“填充多段线-盒相交”公共 API。

因此，闭合路径包围选择框但路径未接触时，通用 `Intersects` 仍返回 `false`；只有 Render 已确认该图元是正式填充面并通过 `SCPolygon2d` 包含测试时，Crossing 才命中。这既保留当前带洞面选择语义，也不把 UI 选择策略扩散到通用几何关系 API。

Render 只能按既有 `planContours` 的正式面元数据构造该 `SCPolygon2d`：外环必须满足 `closed == true`、`part.kind == Face` 且 `part.localIndex == 0`；后续洞环必须同为闭合 `Face`，且 `part.localIndex` 与其洞环顺序连续对应。外环缺失、洞环不合法、构造失败，或仅有任意闭合但非 `Face` 的路径时，均不得将其隐含内部当作填充面；该图元仅按路径关系参与 Crossing。此规则沿用既有 Render 对正式带洞面的解释，不在 SCGeometry 推导图元或 UI 语义。

Render 下游接入验收至少覆盖：正式闭合面完全包围选择框时 Crossing 命中；选择框完全位于正式洞内时不因填充关系命中；任意闭合但非 `Face` 的路径包围选择框时不因隐含内部命中；边界接触由路径 `Intersects` 命中。

本条目完成必须同时满足：四个公共能力（平行投影分类、多边形正面积交集查询、盒-多段线 `Contains`、盒-多段线 `Intersects`）在各自职责头中导出且可经 `Geometry.h` 调用；Quantity 所需的平行投影分类与轮廓正面积交集查询已正确覆盖要求关系并通过测试；盒-多段线对 Line/Arc 无顶点、包围盒或采样降级；所有规定的构建、单元测试、公共头与安装包验证通过。
