# SCGeometry 通用平面与曲线多边形能力实施文档

> 状态：已完成开发并验证
>
> 范围：落实 `SCGeometry板平面构造与高程求值API需求.md`、`SCGeometry板边界拓扑与曲线环API需求.md` 中属于 SCGeometry 的通用能力。
>
> 本文经多轮开发前审查收敛。审核通过前不修改生产代码。

## 1. 目标与边界

本次交付仅包含：

1. 由三维点或 XY 梯度构造通用平面，以及在 XY 位置安全求 Z。
2. 以既有 `SCPolyline2d` 和 `SCPolygon2d` 为唯一模型的精确曲线环规范化、确定性采样与三角化、区域关系、追加洞环和段定义编解码。

不得新增板专用环、区域或数据类型；不得在公开 API、错误类型和测试中引入板、属性、楼层、UI、Qt 或 Quantity 概念。

## 2. 总体架构与扩展边界

```text
SCPolyline2d / SCPolygon2d / ISCSegment2d
        │
        ▼
Source/Detail/SegmentKernel2d（内部：段匹配、求交、反向、批量分割、采样、编解码）
        │
        ├── Core/PolygonTopology（公开：规范化、关系、追加洞、同源网格化）
        └── Serialize/SegmentCodec（公开：定义载荷编解码）
```

段描述符注册是 SCGeometry 的内部实现细节：**仅允许编译进 `SCGeometry` 同一二进制模块**。不导出注册函数、回调类型或注册句柄，不支持第三方 DLL、动态插件或运行期段注册。未来段类型需要随 SCGeometry 源码添加描述符、测试和重新编译；调用方和 Quantity 无需新增类型分支。

因此保持 `ISCSegment2d`、`SCPolyline2d::SegmentAt()`、`ISCSegment2d::Clone()`、现有 `std::unique_ptr` / `std::shared_ptr` 所有权模型的源和 ABI 兼容；不引入跨 DLL 段句柄，也不扩散所有权重构。

运行时段描述符以内部 `Matches(const ISCSegment2d&)` 识别 Line、Arc 和未来内置类型。内部可使用安全的类型匹配实现，但持久化绝不使用 C++ RTTI 名称、内存地址或显示采样点。

## 3. 段身份、定义版本和内部内核

运行时几何能力与持久化版本严格分离：

- `typeId` 标识一个稳定的几何语义族，例如 `SC.LineSegment2d`、`SC.ArcSegment2d`；它决定反向、精确相交、分割和采样描述符。
- `definitionVersion` 只标识同一 `typeId` 的二进制定义格式。新版本必须可解码并迁移为同一运行时几何语义。
- 几何语义不兼容时必须创建新的 `typeId`，而不是提高 `definitionVersion`。

内部 `SegmentKernel2d` 对每个内置类型描述符提供：

- 精确反向；
- 精确 `SplitAtParameters(const ISCSegment2d&, std::span<const double>)`；
- 确定性采样；
- 已知定义版本的编码和解码；
- 与其他内置类型的精确相交。

批量分割的结果必须按原始参数严格递增，返回每个原子段及其原始参数区间：

```cpp
struct SCSegmentFragment2d
{
    std::unique_ptr<ISCSegment2d> segment{};
    double sourceStartParameter{0.0};
    double sourceEndParameter{1.0};
};
```

排列构建只使用该批量分割能力；禁止通过多次单参数分割推断后续参数。Line/Arc 首版必须完整实现；内部未注册段或缺少能力时，公共调用返回正式失败，不做折线降级。

## 4. 容差模型

所有新 API 拒绝非有限或非正容差。

平面运算新增：

```cpp
struct SCPlaneTolerance
{
    double pointTolerance{kDefaultEpsilon};
    double relativeAngularTolerance{kDefaultEpsilon};
    [[nodiscard]] bool IsValid() const;
};
```

保留需求指定的 `double tolerance = kDefaultEpsilon` 重载，其等价于两个字段均为该值。三点构造先按 `pointTolerance` 判断重合，再以
`|cross(u,v)| <= relativeAngularTolerance * |u| * |v|` 判断共线。近垂直平面以缩放法向的相对阈值判断，避免以线性容差直接比较叉积面积。

二维 `validationTolerance` 与 `deterministicTolerance` 都是坐标单位下的有限正数。对长度为 `L` 的有效段，事件参数合并阈值固定为：

```text
parameterTolerance = min(0.5, validationTolerance / L)
```

只有点坐标在 `validationTolerance` 内，且两个源参数分别在其所属段阈值内一致时，才可合并为同一拓扑事件；坐标接近但参数不同的事件确定失败，不合并。

## 5. 公共平面 API

新增 `Include/Core/PlaneOps.h` / `Source/Core/PlaneOps.cpp`：

```cpp
enum class SCPlaneConstructionFailure
{
    None, InvalidTolerance, NonFiniteInput, CoincidentPoints,
    CollinearPoints, ZeroGradientDirection, NonFiniteResult
};
enum class SCPlaneElevationFailure
{
    None, InvalidTolerance, NonFiniteInput, InvalidPlane,
    VerticalOrNearVerticalPlane, NonFiniteResult
};
struct SCPlaneConstructionResult
{
    bool success{false};
    SCPlane plane{};
    SCPlaneConstructionFailure failure{SCPlaneConstructionFailure::None};
};
struct SCPlaneElevationResult
{
    bool success{false};
    double elevation{0.0};
    SCPlaneElevationFailure failure{SCPlaneElevationFailure::None};
};

[[nodiscard]] GEOMETRY_API SCPlaneConstructionResult CreatePlaneThroughThreePoints(
    const SCPoint3d&, const SCPoint3d&, const SCPoint3d&, const SCPlaneTolerance&);
[[nodiscard]] GEOMETRY_API SCPlaneConstructionResult CreatePlaneFromPointAndXYGradient(
    const SCPoint3d&, const SCVector2d&, double risePerRun, const SCPlaneTolerance&);
[[nodiscard]] GEOMETRY_API SCPlaneElevationResult EvaluatePlaneElevationAtXY(
    const SCPlane&, const SCPoint2d&, const SCPlaneTolerance&);
```

另提供三个 `double tolerance = kDefaultEpsilon` 重载。梯度构造使用归一化 XY 方向 `d` 和 `normal=(-risePerRun*d.x, -risePerRun*d.y, 1)`；高程求值不允许垂直或近垂直平面返回钳制值、无穷值或近似值。

所有结果遵守：成功时 `failure == None` 且完整结果有效；失败时 `failure != None`，不暴露部分平面或高程。

## 6. 公共多边形 API

新增 `Include/Core/PolygonTopology.h` / `Source/Core/PolygonTopology.cpp`：

```cpp
enum class SCPolygonTopologyFailure
{
    None, InvalidValidationTolerance, InvalidDeterministicTolerance,
    InvalidOuterRing, InvalidHoleRing, UnsupportedSegmentType,
    UnsupportedSegmentOperation, DegenerateRing, SelfIntersection,
    BoundaryTouching, BoundaryOverlap, HoleOutsideOuterRing,
    HoleIntersection, HoleContainment, NonFiniteResult,
    TessellationFailure, TriangulationFailure, IndexOverflow,
    InvalidTriangulation, AmbiguousTopology
};
struct SCPolygonNormalizeResult
{
    bool success{false};
    SCPolygon2d polygon{};
    SCPolygonTopologyFailure failure{SCPolygonTopologyFailure::None};
    std::uint32_t loopIndex{0};
    std::uint32_t segmentIndex{0};
};
struct SCTessellatedBoundaryVertex2d
{
    std::uint64_t stableVertexId{0};
    std::uint32_t loopIndex{0};
    std::uint32_t sourceSegmentIndex{0};
    double sourceParameter{0.0};
    SCPoint2d point{};
};
struct SCTessellatedPolygon2d
{
    std::uint32_t algorithmVersion{1};
    SCPolygon2d normalizedPolygon{};
    double deterministicTolerance{0.0};
    std::vector<SCTessellatedBoundaryVertex2d> vertices{};
    std::vector<std::uint32_t> loopStartIndices{};
    std::vector<SCTriangleIndex3> triangles{};
};
struct SCTessellatedPolygonResult
{
    bool success{false};
    SCTessellatedPolygon2d polygon{};
    SCPolygonTopologyFailure failure{SCPolygonTopologyFailure::None};
    std::uint32_t loopIndex{0};
    std::uint32_t segmentIndex{0};
};
enum class SCPolygonContainment
{
    Unknown, StrictInside, InsideHole, Touching, Intersecting, Disjoint
};
struct SCPolygonContainmentResult
{
    bool success{false};
    SCPolygonContainment containment{SCPolygonContainment::Unknown};
    SCPolygonTopologyFailure failure{SCPolygonTopologyFailure::None};
};
struct SCPolygonAppendHoleResult
{
    bool success{false};
    SCPolygon2d polygon{};
    SCPolygonTopologyFailure failure{SCPolygonTopologyFailure::None};
};

[[nodiscard]] GEOMETRY_API SCPolygonNormalizeResult NormalizePolygon(
    const SCPolygon2d& polygon, double validationTolerance);
[[nodiscard]] GEOMETRY_API SCTessellatedPolygonResult TessellateAndTriangulatePolygon(
    const SCPolygon2d& polygon,
    double validationTolerance,
    double deterministicTolerance);
[[nodiscard]] GEOMETRY_API SCPolygonContainmentResult ClassifyContainment(
    const SCPolygon2d& container,
    const SCPolygon2d& candidate,
    double validationTolerance);
[[nodiscard]] GEOMETRY_API SCPolygonAppendHoleResult AppendHole(
    const SCPolygon2d& polygon,
    const SCPolyline2d& hole,
    double validationTolerance);
```

所有成功结果 `failure == None`。失败结果 `failure != None`，且 polygon、顶点和三角形保持默认空值；包含关系失败时必须为 `Unknown`，不得伪装为 `Disjoint`。超过 `uint32_t` 可表示范围的段/环/顶点索引返回 `IndexOverflow`。

## 7. 精确规范化、关系与网格化

`NormalizePolygon` 不调用现有固定容差 `SCPolygon2d::IsValid()`，而按以下顺序通过内部段内核执行：

1. 校验容差、全部环闭合、段连续、段有效和有限性。
2. 对环内非相邻段、相邻段非共享端点、外环与洞、洞与洞执行精确曲线相交；交点按原始参数排序，区间重叠单独记录。
3. 只合并满足第 4 节坐标和参数阈值的同一事件；近邻但不同参数事件返回 `AmbiguousTopology`。
4. 使用精确曲线积分计算有符号面积，拒绝退化环；外环规范为逆时针、洞环规范为顺时针。
5. 反向时倒序段并调用精确反向能力，不采样、不把 Arc 改写为折线。
6. 以批量分割后的原子边建立曲线 arrangement，验证洞在外环填充区内，洞间无接触、相交或互含。

失败记录第一个确定的 `loopIndex/segmentIndex`。输入环与洞顺序不排序；规范化只改变必要的方向，因此相同输入产生同一精确段顺序。

`ClassifyContainment` 对两个规范化 polygon 建立同一份曲线 arrangement，按面分类完整填充集合：

1. 存在穿越或边界重合，返回 `Intersecting`。
2. 仅有边界接触，返回 `Touching`。
3. candidate 填充区严格为 container 填充区真子集，返回 `StrictInside`。
4. candidate 填充区完全位于 container 同一个洞内，返回 `InsideHole`。
5. 其余有公共填充区域为 `Intersecting`；否则为 `Disjoint`。

`AppendHole` 先把候选环规范为无洞 polygon，只有分类为 `StrictInside` 才追加，再整体规范化；洞内、接触、相交、重叠和无效候选均失败。

`TessellateAndTriangulatePolygon` 必须先调用 `NormalizePolygon`，再基于返回的同一份精确 polygon 生成全部输出。每个段返回包含 `t=0` / `t=1` 的严格递增参数点列；每环写入各段 `t=0` 与内部点，跳过各段 `t=1`，避免重复连接点。

Line 的参数序列固定为 `{0,1}`。Arc 使用：

```text
theta = abs(sweepAngle)
q = min(deterministicTolerance, radius) / radius
alpha = 2 * asin(sqrt(q * (2 - q)))
partCount = max(1, ceil(theta / alpha))
t_i = i / partCount, i = 0..partCount
```

若 `alpha` 非有限或不大于 0、`theta / alpha` 非有限、或 `partCount` 超过 `uint32_t` 或实现配置的最大网格分段数，返回 `TessellationFailure`。其他内置段必须声明同等确定的参数规则与最大偏差界。

`stableVertexId` 固定为结果 `vertices` 的零基扁平序号；`loopIndex`、`sourceSegmentIndex`、`sourceParameter` 保留来源。`loopStartIndices.size()` 必须为 `1 + normalizedPolygon.HoleCount()`。`algorithmVersion` 为随结果返回的实例字段；与 `deterministicTolerance` 一起进入 Replay 基线，算法、舍入或 ID 规则不兼容变更时递增。

新建内部 `TriangulateContours2d(pointsByLoop, validationTolerance)`，只接受本次采样的线性闭环和显式容差，返回扁平采样顶点索引；它不调用既有 `Triangulate(const SCPolygon2d&)`。必须校验索引范围、索引互异、正面积、三角形中心在采样填充区内，并验证三角形总面积与采样区域面积一致到规定误差。

## 8. 公共段编解码 API

新增 `Include/Serialize/SegmentCodec.h` / `Source/Serialize/SegmentCodec.cpp`：

```cpp
enum class SCSegmentCodecFailure
{
    None, UnsupportedSegmentType, UnknownTypeId, UnknownDefinitionVersion,
    MalformedPayload, NonFiniteDefinition, InvalidDecodedSegment,
    UnsupportedPlatform
};
struct SCSegmentDefinition
{
    std::string typeId{};
    std::uint32_t definitionVersion{0};
    std::vector<std::byte> definitionPayload{};
};
struct SCSegmentCodecResult
{
    bool success{false};
    SCSegmentDefinition definition{};
    SCSegmentCodecFailure failure{SCSegmentCodecFailure::None};
};
struct SCSegmentDecodeResult
{
    bool success{false};
    std::unique_ptr<ISCSegment2d> segment{};
    SCSegmentCodecFailure failure{SCSegmentCodecFailure::None};
};
[[nodiscard]] GEOMETRY_API SCSegmentCodecResult SerializeSegment(const ISCSegment2d& segment);
[[nodiscard]] GEOMETRY_API SCSegmentDecodeResult DeserializeSegment(
    std::string_view typeId,
    std::uint32_t definitionVersion,
    std::span<const std::byte> definitionPayload);
```

Line 的 `typeId` 固定为 `SC.LineSegment2d`，Arc 固定为 `SC.ArcSegment2d`，当前定义版本均为 1。载荷按固定字段顺序、规范小端、IEEE-754 binary64 位模式编码；不得写对象内存。平台不满足 `is_iec559` 时返回 `UnsupportedPlatform`；解码必须精确消费全部字节、检查有限值和最终 `IsValid()`。新增定义版本的解码器必须迁移为同一 `typeId` 的运行时几何语义。

## 9. 文件、兼容性与验证

计划新增：

- `Include/Core/PlaneOps.h` / `Source/Core/PlaneOps.cpp`；
- `Include/Core/PolygonTopology.h` / `Source/Core/PolygonTopology.cpp`；
- `Include/Serialize/SegmentCodec.h` / `Source/Serialize/SegmentCodec.cpp`；
- 内部 `Source/Detail/SegmentKernel2d.h/.cpp`、曲线 arrangement 和 `TriangulateContours2d`；
- `UnitTests/Capabilities/Core/TestPlaneOps.cpp`、`TestPolygonTopology.cpp`、`TestSegmentCodec.cpp`。

计划重构：

- `Core/ShapeOps`、`Core/Intersection`、`Core/Validation` 的新内部路径使用 `SegmentKernel2d` 与显式容差；
- `Include/Core/GeometryApi.h`、`Include/Geometry.h`、公共头编译测试、CMake 测试清单；
- `docs/session-handoff.md`，记录 API、算法版本和兼容性状态。

保持现有 `ISCSegment2d`、`SCPolyline2d`、`Triangulate`、`SCPolygon2d::IsValid()` 和 `SCSegmentKind2` 的源/ABI 兼容。新 API 不继承旧 API 的固定容差和已知类型限制。

验收除需求列出的用例外，还必须覆盖：

- 平面：极大有限坐标、量纲正确的共线判断、近垂直阈值、double 与结构容差重载等价；
- 段内核：Line/Arc 的反向、批量分割、精确相交、采样、编解码；测试用内置派生段无需调用方类型分支即可进入全部流程；未注册或缺失能力确定失败；
- arrangement：候选包围容器、跨越洞、位于洞内、多个洞、切点接触、重复事件去重、近邻不同参数的 `AmbiguousTopology`；
- 网格化：弦高误差、极小正容差、分段上限、溢出、100 次逐字段确定性和 `algorithmVersion`；
- 编码：固定字节夹具、尾部垃圾、截断、未知类型/版本、非有限载荷和不支持平台；
- 全量构建、单元测试、`PublicHeadersCompile` 和安装包消费者测试。

实施顺序：先实现并测试内部段内核的 Line/Arc 描述符与批量分割；再实现平面能力；随后实现规范化、arrangement、区域关系、网格化和三角化；最后接入段编解码、更新聚合头/文档并运行全量验证。

审核通过标准：内部注册边界、运行时语义与定义版本分离、批量分割、错误契约、确定性网格化与二进制格式均获确认。收到通过后才开始代码实现。
## 10. 本次实施状态

本实施已完成本文档规定的实现范围，并完成以下验证：

- 已实现 `PlaneOps`、`PolygonTopology` 和 `SegmentCodec` 公共 API，并接入 `GeometryApi.h`。
- `SegmentKernel2d` 限定在同一 `SCGeometry` 二进制模块内；Line/Arc 的反向、相交、最近点、采样和批量 `SplitAtParameters` 均通过该内部能力边界执行，分割结果保留原始参数区间。
- 曲线 arrangement 已完成事件参数归并、重叠区间端点分割、重复原子边消除、半边图、确定性面环遍历和面级歧义判定；`ClassifyContainment` 以 arrangement 面样本的填充标签决策，不再回退到顶点包含启发式。
- 已实现独立 `TriangulateContours2d`，支持多孔洞，并校验索引、正面积、三角形中心落区和面积守恒。
- 专项测试覆盖多孔洞、曲线环、候选包围、跨洞、边界相切、重叠、重复原子边和 100 次逐字段确定性重放。
- `SCGeometryCapabilitiesTests` 全量通过：390 个测试、38 个测试套件；`git diff --check` 通过。

后续如新增新的内置段类型、定义格式版本或更高规模的性能目标，应作为新的需求进行设计、实现和回归；这些不属于本文档的未完成项。
