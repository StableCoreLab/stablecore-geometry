# SCGeometry 捕捉算法缺失项实现方案

> 状态：定版待执行
> 版本：V1.0
> 责任范围：把 [`SCGeometry捕捉算法缺失清单.md`](../../../Quantity/Docs/Design/Snap/SCGeometry捕捉算法缺失清单.md) 中列出的 8 项缺失算法落地到 SCGeometry 算法库
> 关联文档：
> - [`SCGeometry捕捉算法缺失清单.md`](../../../Quantity/Docs/Design/Snap/SCGeometry捕捉算法缺失清单.md)（需求源）
> - [`捕捉点服务设计.md`](../../../Quantity/Docs/Design/Snap/捕捉点服务设计.md)（消费方）
> - [`docs/ai-execution-spec.md`](ai-execution-spec.md)、[`docs/coding-style.md`](coding-style.md)、[`docs/library-design.md`](library-design.md)（工程规范）

本方案面向 AI 落地开发，按"一文件一职责、一轮一闭环"的执行规范，逐项给出：API 契约、结果类型、算法要点、文件落位、测试要求、风险与依赖、批次归属。所有命名、目录、风格均遵循 [`docs/coding-style.md`](coding-style.md)，不引入 breaking change。

---

## 一、总体原则

1. **不破坏已发布 API**：仅在 `Include/` 下新增头文件或在现有头尾部追加重载，不修改既有签名。
2. **类型/结果先于算法**：每个新算法配套结构化结果类型，避免裸 `bool` / 裸 `double` 返回。
3. **数值统一**：所有近似比较走 `Support/Epsilon.h`，禁止裸 `1e-9`。Epsilon 选用规则：
   - **几何构造/求交/切点/折线求交**：用 `Geometry::kDefaultEpsilon`（=1e-9，定义于 `Support/Epsilon.h`）作为默认参数 `eps`。
   - **3D 容差场景**：用 `SCGeometryTolerance3d tolerance = {}`（字段 `distanceEpsilon` / `angleEpsilon` / `parameterEpsilon` / `boxPadding`，见 `Core/GeometryTypesPrimitives.h`），不使用 `tolerance.linear`（不存在）。
   - **与 2D `AxisOps` 模块对齐的 API**：现有 2D `SnapPointToSegments` 用 `kAxisOpsDefaultEpsilon`（`Support/Epsilon.h` 中 `extern`，值比 `kDefaultEpsilon` 宽松）。**新增 3D API 不复用 `kAxisOpsDefaultEpsilon`**，统一用 `tolerance.distanceEpsilon`；2D 现有实现保持原样。
4. **三维容差统一**：3D 接口默认参数 `SCGeometryTolerance3d tolerance = {}`，与 `Core/Projection.h` 现有 3D API 一致。
5. **代码/测试/文档同步**：每一项落地必须同步更新 `docs/session-handoff.md`、`docs/todolist.md`、`docs/test-capability-coverage.md`、`docs/design-doc-sync-tracker.md`。
6. **闭环纪律**：每项必须含 capability / edge / gap 三类测试；不稳定行为必须标 gap，不允许静默回退。

---

## 二、缺失项总览与分批

| 编号 | 名称 | 优先级 | 阶段 | 模块落位 |
|---|---|---|---|---|
| #3 | 2D 折线批量求交 | 已完成 | 第二版 | `Core/Intersection.h` |
| #8 | 2D K 近邻查询 | 已完成 | 第二版 | `Geometry2d/SCBoxTree2d.h`、`Geometry2d/SCSegmentSearch2d.h` |
| #1 | 2D 点到圆/弧的切点 | 已完成 | 第三版 | `Core/Tangent.h`（新增） |
| #2 | 2D 无限直线类型与延长线求交 | 已完成 | 第三版 | `Geometry2d/SCLine2d.h`（新增）、`Core/Intersection.h` |
| #7 | 2D 段延长区域交点 | 已完成 | 第三版 | `Core/Intersection.h` |
| #4 | 3D 点到线段最近点 | 已完成 | 3D 后端 | `Core/Projection.h` |
| #5 | 3D 段集空间索引 | 已完成 | 3D 后端 | `Geometry3d/SCSegmentSearch3d.h`、`Geometry3d/SCBoxTree3d.h`（新增） |
| #6 | 3D 段集捕捉通用入口 | 已完成 | 3D 后端 | `Core/Snap3d.h`（新增）或 `Core/AxisOps.h` 追加重载 |

执行顺序建议：批次 1（#3、#8，已完成）→ 批次 2（#1、#2、#7，已完成）→ 批次 3（#4、#5、#6，已完成）。每批内可并行。

---

## 三、缺失项详细方案

### 缺失项 #3：2D 折线批量求交

#### 背景
`Core/Intersection.h` 现有 `Intersect` 只接受单段（`SCLineSegment2d` / `SCArcSegment2d` / `ISCSegment2d`），无 `SCPolyline2d` 重载。`SCPolyline2d` 已在 `Geometry2d/SCPolyline2d.h` 提供 `SegmentCount()`、`SegmentAt(i)`、`Bounds()`。

#### API 契约
追加到 `Include/Core/Intersection.h`（仅函数声明；`SCPolylineIntersectionPoint2d` 结构体定义在 `Results.h`，见下方"结果类型"；Intersection.h 已 `#include "Core/Results.h"`，类型可见）：

```cpp
#include "Geometry2d/SCPolyline2d.h"

namespace Geometry
{
    // 折线 vs 折线：返回所有交点；端点共享只保留一个，
    // 按 (segmentIndexOnFirst, parameterOnFirstSegment) 升序排列。
    [[nodiscard]] GEOMETRY_API std::vector<SCPolylineIntersectionPoint2d>
    Intersect(const SCPolyline2d& first,
              const SCPolyline2d& second,
              double eps = Geometry::kDefaultEpsilon);

    // 折线 vs 单段：用于弧采样折线 vs 单段弧场景。
    // segmentIndexOnSecond 恒为 0；globalParameterOnSecond 与 parameterOnSecondSegment 相同。
    [[nodiscard]] GEOMETRY_API std::vector<SCPolylineIntersectionPoint2d>
    Intersect(const SCPolyline2d& polyline,
              const ISCSegment2d& segment,
              double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API std::vector<SCPolylineIntersectionPoint2d>
    Intersect(const ISCSegment2d& segment,
              const SCPolyline2d& polyline,
              double eps = Geometry::kDefaultEpsilon);
}
```

> 说明：折线版本**统一返回 `SCPolylineIntersectionPoint2d`**（不复用 `SCIntersectionPoint2d`），原因：折线需要携带"命中段索引 + 段内参数 + 折线归一化参数"三组语义，`SCIntersectionPoint2d` 字段不够且语义会混用。`SCPolylineIntersectionPoint2d` 定义见下方"结果类型"。

#### 结果类型
新增结构化类型，避免修改既有 `SCIntersectionPoint2d`：

```cpp
// Include/Core/Results.h 追加
struct GEOMETRY_API SCPolylineIntersectionPoint2d
{
    SCIntersectionKind2d kind{SCIntersectionKind2d::None};  // None / Point / Overlap / Tangent（对齐 SCSegmentIntersection2d）
    SCPoint2d point{};
    std::size_t segmentIndexOnFirst{0};   // 命中的第一折线段索引
    std::size_t segmentIndexOnSecond{0};  // 命中的第二折线段索引（折线 vs 单段时恒为 0）
    double parameterOnFirstSegment{0.0};  // 命中段内参数 ∈ [0,1]
    double parameterOnSecondSegment{0.0};
    double globalParameterOnFirst{0.0};   // 第一折线归一化参数 ∈ [0,1]
    double globalParameterOnSecond{0.0}; // 第二折线归一化参数 ∈ [0,1]（折线 vs 单段时等同段内参数）

    [[nodiscard]] bool IsValid() const { return kind != SCIntersectionKind2d::None && point.IsValid(); }
};
```

> 说明：`kind` 字段对齐 `SCSegmentIntersection2d`（`Results.h:54-64`），用于区分 Point/Overlap/Tangent。共线 Overlap 时填充两个 `SCPolylineIntersectionPoint2d`（首尾端点，`kind=Overlap`）；相切时单个（`kind=Tangent`）。

#### 算法要点
1. **粗筛**：先比较 `first.Bounds()` 与 `second.Bounds()` 是否相交；不相交直接返回空。
2. **段配对粗筛**：构造 `SCBoxTree2d`，把 `first` 的每段以 `entry.id = i, box = segment.Bounds()` 入树；遍历 `second` 每段，用 `Query(box)` 取候选段索引集合，避免 O(N·M) 全配对。
3. **细算**：对每个候选对 `(i, j)`，调用 `Intersect(*first.SegmentAt(i), *second.SegmentAt(j))`。**注意**：`SCPolyline2d::SegmentAt(i)` 返回 `std::unique_ptr<ISCSegment2d>`（见 `Geometry2d/SCPolyline2d.h`），`Intersect` 接受 `const ISCSegment2d&`，必须写 `*SegmentAt(i)` 解引用；空指针场景由 `SCPolyline2d` 内部保证不出现（折线无效时直接返回空）。
4. **去重端点**：相邻段共享端点的交点只保留一次，按 `(point, i, j)` 用 `SCPoint2d::AlmostEquals(point, eps)`（`SCPoint2d` 已有此方法）去重，保留参数较小者。底层 `Intersect` 返回 `SCSegmentIntersection2d`，其内含 `SCIntersectionPoint2d`（有 `AlmostEquals`），去重时可先比较 `SCIntersectionPoint2d::AlmostEquals`，再构造 `SCPolylineIntersectionPoint2d`。
5. **排序**：按 `(segmentIndexOnFirst, parameterOnFirstSegment)` 升序，保证输出确定性。

#### 临时绕行（不阻塞第二版上线）
服务层在 `SCIntersectionSnapResolver` 内展开折线为段列表两两调用 `Intersect(line, line)`。本项落地后服务层切换调用，删除绕行代码。

#### 文件落位
- `Include/Core/Results.h`：追加 `SCPolylineIntersectionPoint2d` 结构体（含 `kind` 字段）。
- `Include/Core/Intersection.h`：追加声明 + `#include "Geometry2d/SCPolyline2d.h"`（现有 Intersection.h 未 include 此头，必须补；Results.h 已 include）。
- `Source/Core/Intersection.cpp`：实现 + 内部 helper `IntersectPolylinePair`。
- `UnitTests/Capabilities/Core/TestIntersectionPolyline.cpp`（新建，capability 用例）；`UnitTests/Gaps/Core/TestIntersectionPolylineGaps.cpp`（新建，gap 用例；`Gaps/Core/` 目录不存在需新建，CMake GLOB_RECURSE 自动覆盖，见 §四第 8 条）。

#### 测试要求
- **Capability**：两条十字相交折线（4 段 × 4 段），命中 1 个交点；相切折线返回 `kind=Tangent`。
- **Edge**：共享端点去重；一条折线完全包含另一条的共线段（产生 Overlap，列出端点）；空折线 / 单段折线；闭折线（首尾相连）。
- **Gap**：自交折线（`first == second`）当前不处理自交，返回空并标注 gap；共线重叠超过 2 段连续时，仅返回首尾两端点（中间段不重复枚举），作为已知限制。

---

### 缺失项 #8：2D K 近邻查询

#### 背景
- `SCBoxTree2d` 现有 `Query(box)` / `QueryContaining(point)`，无 KNN。
- `SCSegmentSearch2d` 有 `Nearest(point)`，无 KNN。
- `SCKDTree2d` 有 `Nearest`，无 KNN；本方案不依赖该类型。

#### API 契约
追加到 `Include/Geometry2d/SCBoxTree2d.h`（新增专用轻量类型，零 ABI 影响——不动既有 `SCBoxTreeHit2d`）。`SCBoxTreeKnnHit2d` 定义在 `SCBoxTree2d.h` 内，而 `GeometryApi.h` 已 `#include "Geometry2d/SCBoxTree2d.h"`（line 35），类型自动对外可见，**无需额外登记到 `GeometryApi.h`**：

```cpp
// 新增结构体（不修改既有 SCBoxTreeHit2d，避免 ABI 破坏）
struct GEOMETRY_API SCBoxTreeKnnHit2d
{
    std::size_t id{0};
    SCBox2d box{};
    double distanceSquared{0.0};
};

// 按 distanceSquared 升序返回；结果自带 box，调用方无需再查 Find()。
[[nodiscard]] std::vector<SCBoxTreeKnnHit2d>
QueryKNearest(const SCPoint2d& point, std::size_t k, double eps = Geometry::kDefaultEpsilon) const;
```

**注意 ABI**：`SCBoxTreeHit2d` 通过 `GeometryApi.h` 已对外发布（外部消费者按当前布局分配/拷贝），直接追加字段会改变结构体大小与字段偏移，属于 **ABI 破坏**。两种修正方案，选其一：

- **方案 A（采用，零 ABI 影响）**：不动 `SCBoxTreeHit2d`；新增专用轻量类型 `SCBoxTreeKnnHit2d`（`id` + `box` + `distanceSquared`），`QueryKNearest` 返回 `std::vector<SCBoxTreeKnnHit2d>`，调用方无需再额外调用 `Find()`。
- **方案 B（备选，ABI 破坏，需主版本号升级）**：在 `SCBoxTreeHit2d` 尾部追加 `distanceSquared`；同步所有现存 `Query` 调用方填充 `0.0`；在 `docs/design-doc-sync-tracker.md` 登记主版本变更。

本方案采用 **方案 A**（用 `SCBoxTreeKnnHit2d`，比 `std::pair` 字段名可读）。`SCBoxTreeKnnHit3d` 是本次新增类型，可直接内含 `distanceSquared` 字段（无 ABI 历史）。

追加到 `Include/Geometry2d/SCSegmentSearch2d.h`（**需补 `#include <limits>`**——现有头文件未含，`QueryKNearest` 默认参数用 `std::numeric_limits<double>::max()`）：

```cpp
[[nodiscard]] std::vector<SCSegmentSearchHit2d>
QueryKNearest(const SCPoint2d& point, std::size_t k, double maxDistance = std::numeric_limits<double>::max()) const;
```

#### 算法要点
1. **box tree KNN**：`SCBoxTree2d` 已是 BVH/AABB 树（`Source/Geometry2d/SCBoxTree2d.cpp` 的 `BoxNode`/`BuildBoxTree`），KNN 应基于优先队列（最小堆，key 为查询点到节点 box 的最近距离平方）遍历：
   - 入堆根节点；弹出最近节点，若叶子则计算每个 entry 的 `DistanceSquared(point, box)`，填入大小为 k 的最大堆；若内部节点则把左右子按最近距离入堆。
   - 堆满后用堆顶距离剪枝：弹出的节点最近距离 > 堆顶时停止。
   - 最终把最大堆内容弹出按升序返回。
2. **segment search KNN**：基于 `Nearest` 同样的段投影（`ProjectPointToSegment`）。当前 `SCSegmentSearch2d` 内部为线性 `entries_`（无 BVH），**首版明确采用线性扫描 + 大小为 k 的最大堆 + `nth_element`**；待后续升级为 BVH 时再切到优先队列路径，公开 API 不变。本项首版**不要求**为 `SCSegmentSearch2d` 先补内部 BVH。
3. **返回**：按 `distanceSquared` 升序；`k=0` 返回空；`k > Size()` 返回全部。
4. **`maxDistance` 过滤**：超过 `maxDistance²` 的命中不入堆。

#### 文件落位
- `Include/Geometry2d/SCBoxTree2d.h`、`Include/Geometry2d/SCSegmentSearch2d.h`：声明。
- `Source/Geometry2d/SCBoxTree2d.cpp`、`Source/Geometry2d/SCSegmentSearch2d.cpp`：实现。
- `UnitTests/Capabilities/Geometry2d/TestBoxTree2d.cpp`（新建，KNN capability 用例）。
- `UnitTests/Capabilities/Geometry2d/TestSegmentSearch2d.cpp`（新建，KNN capability 用例）。
- gap 用例分别放 `UnitTests/Gaps/Geometry2d/TestBoxTree2dGaps.cpp`、`UnitTests/Gaps/Geometry2d/TestSegmentSearch2dGaps.cpp`（新建）。

> **不要复用 `TestSearchPoly.cpp`**：该文件实际测试 `SearchPolygons`（多边形闭环候选搜索），与 `SCSegmentSearch2d` / `SCBoxTree2d` 无关（见现有用例 `SearchPolyCapabilityTest.*`）。

#### 测试要求
- **Capability**：`SCBoxTree2d::QueryKNearest` 构造 10 个 box，查 k=3，返回前 3 且按距离升序；`SCSegmentSearch2d::QueryKNearest` 对 10 条段查 k=3，返回前 3 且按距离升序。
- **Edge**：`SCBoxTree2d::QueryKNearest` 覆盖 `k=0`、`k > size`、点恰好落在 box 上（`distanceSquared=0`）；`SCSegmentSearch2d::QueryKNearest` 额外覆盖 `maxDistance` 过滤。
- **Gap**：`SCSegmentSearch2d` 当前内部为线性 `entries_`，KNN 首版基于线性 + 最大堆，O(N·log k)；待其升级为 BVH 后切到优先队列路径，公开 API 行为保持一致。不影响正确性，仅性能项。

---

### 缺失项 #1：2D 点到圆/弧的切点

#### 背景
`Core/Projection.h` 只有投影系列，无切线几何。第三版 `SCSnapKind::Tangent` 依赖此能力。

#### API 契约
新增 `Include/Core/Tangent.h`：

```cpp
#pragma once

#include <array>
#include <optional>

#include "Export/GeometryExport.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCCircle2d.h"
#include "Support/Epsilon.h"
#include "Types/Geometry2d/SCPoint2.h"

namespace Geometry
{
    struct GEOMETRY_API SCTangentPoints2d
    {
        std::size_t pointCount{0};                       // 0 / 1 / 2
        std::array<SCPoint2d, 2> points{};
        std::array<double, 2> angles{0.0, 0.0};         // 切点在圆/弧上的角度
        bool pointInsideCircle{false};                  // true 表示外部点退化为内部点，无实切线

        [[nodiscard]] bool IsValid() const { return pointCount <= 2; }
    };

    // 外部点 P 到圆 C 的两条切线切点；P 在圆上时返回 1 个（重合），P 在圆内返回 0 个。
    [[nodiscard]] GEOMETRY_API SCTangentPoints2d
    TangentPoints(const SCPoint2d& point, const SCCircle2d& circle, double eps = Geometry::kDefaultEpsilon);

    // 外部点 P 到弧的切点；切点角度必须落在弧的扫描区间内，否则不计入。
    [[nodiscard]] GEOMETRY_API SCTangentPoints2d
    TangentPoints(const SCPoint2d& point, const SCArcSegment2d& arc, double eps = Geometry::kDefaultEpsilon);
}
```

#### 算法要点
1. 设圆心 C、半径 r、外部点 P，d = |PC|。
2. **退化优先判定**：
   - `d <= eps`（P 与 C 重合）：所有方向都"切线"，几何上无意义；返回 `pointCount=0, pointInsideCircle=true`（因 `d=0 < r-eps` 必然成立）。
   - `d < r - eps`（P 在圆内）：`pointCount=0, pointInsideCircle=true`。
   - `|d - r| <= eps`（P 在圆上）：`pointCount=1, points[0]=P, angles[0]=atan2(P.y-C.y, P.x-C.x)`。
3. `d > r + eps`：求两切点。切点 T 同时满足两个几何约束（**直径圆法**，统一推导，不混用反演法）：
   - **约束 1**：T 在原圆上，即 `|CT| = r`（切点必在圆上）。
   - **约束 2**：T 在以 PC 为直径的圆上，即圆心 M=(P+C)/2、半径 d/2（由切线性质 ∠CTP=90° 得出，直径所对圆周角为直角）。
   - 两圆相交：`|CT|=r` 与 `|MT|=d/2` 联立，解出 T1、T2；按角度升序填入 `points`、`angles`。
   - 数值实现：可参数化 T=C+(r·cosθ, r·sinθ) 代入 `|MT|²=(d/2)²` 解 θ，或直接用两圆心距 + 两半径的圆-圆相交公式。
4. **弧版本**：在圆版本基础上，对每个切点角度 `θ`，用新增的 public `SCArcSegment2d::ContainsAngle(θ)`（见 §四公共支撑改动第 2 条；签名无 eps，与 private `IsAngleOnArc` 一致）过滤；保留命中项。
5. **数值稳定**：用 `IsZero` / `IsEqual` 而非裸 `1e-9`；`eps` 透传；切点坐标用 `atan2` 反算角度避免象限错误。

#### 文件落位
- `Include/Core/Tangent.h`（新增）。
- `Source/Core/Tangent.cpp`（新增）。
- `Include/Core/GeometryApi.h`：追加 `#include "Core/Tangent.h"`（注意：不是 `Include/Geometry.h`——后者是面向产品侧的稳定精选入口，仅 5 个 include，注释明确"Keep this header limited to the supported public entry points"；真正聚合 umbrella 是 `Core/GeometryApi.h`）。
- `UnitTests/Capabilities/Core/TestTangent.cpp`（新建，capability 用例）；`UnitTests/Gaps/Core/TestTangentGaps.cpp`（新建，gap 用例；`Gaps/Core/` 目录不存在需新建，见 §四第 8 条）。

#### 测试要求
- **Capability**：P=(10,0)、C=(0,0)、r=5（d=10>2r）→ 两个切点 (2.5, ±2.5·sqrt(3))，角度 ±60°。
  - 推导：T 在圆 `x²+y²=25` 上，且 `|PT|²=75`（勾股 `|PT|²=d²-r²=100-25`）；联立 `(x-10)²+y²=75` 与 `x²+y²=25` 解得 `x=2.5`，`y=±√18.75=±2.5√3`；角度 `atan2(±2.5√3, 2.5)=±60°`。
- **Edge**：P 在圆上（d=r）返回 1 个；P 在圆内返回 0 个且 `pointInsideCircle=true`；**P 与圆心重合（d=0）返回 0 个且 `pointInsideCircle=true`**；弧版本切点角度落在区间外被过滤。
- **Gap**：当 `d` 接近 `r`（`|d-r| < eps`）时，单切点角度数值精度依赖于 eps；标 gap 并在文档说明精度边界。

---

### 缺失项 #2：2D 无限直线类型与延长线求交

#### 背景
现有 `Intersect` 只接受 `SCLineSegment2d` / `SCArcSegment2d`，无无限直线概念；延长线求交无法直接表达。

#### API 契约
新增 `Include/Geometry2d/SCLine2d.h`（header-only inline，对齐 `Types/Geometry3d/SCLine3d.h` 的风格——`SCLine3d` 无 `GEOMETRY_API` 标记、成员 inline 实现）：

```cpp
#pragma once

#include <sstream>
#include <string>

#include "Support/Epsilon.h"
#include "Types/Geometry2d/SCPoint2.h"
#include "Types/Geometry2d/SCVector2.h"

namespace Geometry
{
    // 2D 无限直线：origin + direction·t，t ∈ ℝ
    // header-only inline，与 SCLine3d 一致；不提供 Bounds()（无限几何体无有界包围盒）。
    struct SCLine2d
    {
        SCPoint2d origin{};
        SCVector2d direction{};

        [[nodiscard]] static SCLine2d FromOriginAndDirection(const SCPoint2d& originValue,
                                                              const SCVector2d& directionValue)
        {
            return SCLine2d{originValue, directionValue};
        }

        [[nodiscard]] static SCLine2d FromTwoPoints(const SCPoint2d& p0, const SCPoint2d& p1)
        {
            return SCLine2d{p0, p1 - p0};
        }

        [[nodiscard]] bool IsValid(double eps = kDefaultEpsilon) const
        {
            return origin.IsValid() && direction.IsValid() && direction.Length() > eps;
        }

        [[nodiscard]] SCPoint2d PointAt(double parameter) const
        {
            return origin + direction * parameter;
        }

        [[nodiscard]] std::string DebugString() const
        {
            std::ostringstream stream;
            stream << "SCLine2d{origin=" << origin.DebugString()
                   << ", direction=" << direction.DebugString() << "}";
            return stream.str();
        }
    };
}
```

> 说明：不加 `GEOMETRY_API`、无 `.cpp` 文件，与 `SCLine3d`（`Types/Geometry3d/SCLine3d.h`）严格对齐。`FromPointAndDirection` 改名为 `FromOriginAndDirection`（与 `SCLine3d::FromOriginAndDirection` 一致）；`FromTwoPoints` 保留。

追加到 `Include/Core/Results.h`（**结果结构体定义在 Results.h，对齐现有 `SCSegmentIntersection2d` 约定**，见 `Results.h:54-64`）：

```cpp
// Include/Core/Results.h 追加
struct GEOMETRY_API SCLineIntersection2d
{
    SCIntersectionKind2d kind{SCIntersectionKind2d::None};  // None / Point / Overlap(共线)
    std::array<SCIntersectionPoint2d, 2> points{};  // 复用 SCIntersectionPoint2d；有限重叠时填端点，无限重叠时忽略
    std::size_t pointCount{0};                       // Point=1, finite Overlap=2, infinite Overlap=0, None=0
    bool parallel{false};
    bool collinear{false};
    bool infiniteOverlap{false};  // true only for coincident infinite lines

    [[nodiscard]] bool HasIntersection() const { return kind != SCIntersectionKind2d::None; }
};
```

追加到 `Include/Core/Intersection.h`（**仅函数声明**，Intersection.h 已 `#include "Core/Results.h"`，需补 `#include "Geometry2d/SCLine2d.h"`）：

```cpp
#include "Geometry2d/SCLine2d.h"

namespace Geometry
{
    [[nodiscard]] GEOMETRY_API SCLineIntersection2d
    Intersect(const SCLine2d& first, const SCLine2d& second, double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API SCLineIntersection2d
    Intersect(const SCLine2d& line, const SCLineSegment2d& segment, double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API SCLineIntersection2d
    Intersect(const SCLineSegment2d& segment, const SCLine2d& line, double eps = Geometry::kDefaultEpsilon);
}
```

#### 算法要点
1. 两无限直线参数方程：`P1 = O1 + D1·t`、`P2 = O2 + D2·u`。
2. 行列式 `det = D1.x · D2.y − D1.y · D2.x`：
   - `|det| <= eps`：平行；进一步判断 `O2 - O1` 是否与 `D1` 共线（`cross(O2-O1, D1) ≈ 0`）→ `collinear=true, kind=Overlap, pointCount=0, infiniteOverlap=true`（`不应使用 `points[0]` / `points[1]` 冒充无限重合的端点），否则 `kind=None, parallel=true`。
   - 否则：`t = cross(O2 - O1, D2) / det`、`u = cross(O2 - O1, D1) / det`；交点 = `O1 + D1·t`；`kind=Point, pointCount=1`，`points[0] = {point, t, u}`。
3. **直线 vs 线段**：解出 `t`（直线上参数）与 `u`（线段上参数 ∈ [0,1]）；`u` 越界 → `kind=None`；共线时 `kind=Overlap, pointCount=2, infiniteOverlap=false`（线段两端点）；`points` 用 `SCIntersectionPoint2d` 填充便于去重。
4. **顺序对称**：`Intersect(segment, line)` 应与 `Intersect(line, segment)` 同值（参数位置按声明顺序填入 `points[i].parameterOnFirst/parameterOnSecond`）。

#### 文件落位
- `Include/Geometry2d/SCLine2d.h`（新增，header-only inline，**无对应 .cpp**）。
- `Include/Core/Results.h`：追加 `SCLineIntersection2d` 结构体。
- `Include/Core/Intersection.h`、`Source/Core/Intersection.cpp`：追加重载 + `#include "Geometry2d/SCLine2d.h"`（现有 Intersection.h 未 include 此头，必须补；Results.h 已 include）。
- `Include/Core/GeometryApi.h`：追加 include。
- `UnitTests/Capabilities/Geometry2d/TestLine2d.cpp`（新建，capability 用例）；`UnitTests/Gaps/Geometry2d/TestLine2dGaps.cpp`（新建，gap 用例）。
- `UnitTests/Capabilities/Core/TestCore.cpp`：追加线-线、线-段用例。

#### 测试要求
- **Capability**：两直线相交于 (1,1)；直线与线段相交且 u∈[0,1] 命中。
- **Edge**：平行不共线 → `parallel=true, kind=None`；无限直线完全重合 → `collinear=true, kind=Overlap, pointCount=0, infiniteOverlap=true`；线段参数 u 越界 → `kind=None`。
- **Gap**：直线 vs 线段的有限重叠返回重叠区两端点（`pointCount=2, infiniteOverlap=false`）；若重叠区端点无法精确计算（如浮点误差导致端点判定不稳），降级返回 `pointCount=1` 代表点 + `collinear=true` 标记，标 gap。

---

### 缺失项 #7：2D 段延长区域交点

#### 背景
`Intersect` 只返回段内交点；弧延长相切、线段延长相交无法表达。若 #2 已落地，线段延长场景可由 `SCLine2d` 覆盖，本项聚焦"段 vs 段（含弧）"的延长语义。

#### API 契约
追加到 `Include/Core/Results.h`（enum + 结构体）与 `Include/Core/Intersection.h`（函数声明）：

```cpp
// Include/Core/Results.h 追加（enum 与结构体定义在 Results.h，对齐现有 SCIntersectionKind2d / SCSegmentIntersection2d 约定）
enum class SCExtensionPolicy
{
    None,           // 仅段内（与现有 Intersect 等价）
    ExtendFirst,    // 第一段无限延长
    ExtendSecond,   // 第二段无限延长
    ExtendBoth      // 两段都无限延长
};

struct GEOMETRY_API SCExtendedIntersection2d
{
    SCIntersectionKind2d kind{SCIntersectionKind2d::None};
    std::array<SCIntersectionPoint2d, 2> points{};  // 复用 SCIntersectionPoint2d（Results.h:36）
    std::size_t pointCount{0};
    bool onFirstSegment{false};   // 交点是否落在第一段段内
    bool onSecondSegment{false};  // 交点是否落在第二段段内
    bool infiniteOverlap{false};  // 仅用于延长后两条支撑几何完全重合；此时 onFirstSegment=false 且 onSecondSegment=false
};

// Include/Core/Intersection.h 追加（仅函数声明；Intersection.h 已 include Results.h）
[[nodiscard]] GEOMETRY_API SCExtendedIntersection2d
IntersectExtended(const ISCSegment2d& first,
                  const ISCSegment2d& second,
                  SCExtensionPolicy policy = SCExtensionPolicy::ExtendBoth,
                  double eps = Geometry::kDefaultEpsilon);
```

#### 算法要点
1. **线段 vs 线段**：用 #2 的直线/直线求交；按 `policy` 决定是否校验 `t ∈ [0,1]` / `u ∈ [0,1]`；`onFirstSegment` / `onSecondSegment` 由参数判定。
   - `policy=None` 时，行为必须与现有 `Intersect` 完全一致，不允许返回 `infiniteOverlap=true`。
   - `policy=ExtendBoth` 时，若两段支撑直线完全重合、但原始有限段无法形成可计算的有限重叠区，则返回 `kind=Overlap, pointCount=0, infiniteOverlap=true, onFirstSegment=false, onSecondSegment=false`，不要伪造两个端点。
2. **线段 vs 弧**：线段延长为直线（`SCLine2d::FromTwoPoints`），与弧所在圆求交（解一元二次方程），过滤切点角度是否落在"延长后"区间：
   - `ExtendFirst`：弧角度区间保持 [start, end]；线段可延长。
   - `ExtendSecond`：线段保持 [0,1]；弧按其方向延长至完整圆（即允许任何角度）。
   - `ExtendBoth`：两边都允许延长，相当于直线 vs 圆。
3. **弧 vs 弧**：两弧所在圆求交（最多 2 点），按 policy 过滤角度区间；`ExtendBoth` 时两圆全角度均可命中。
4. **共线 / 重合**：本项区分有限重叠与延长后的无限重合。
   - 若可计算有限重叠区，则返回 `kind=Overlap, pointCount=2, infiniteOverlap=false`，`points` 填重叠端点。
   - 若延长语义下只能得到“支撑几何完全重合”而无可返回的有限端点，则返回 `kind=Overlap, pointCount=0, infiniteOverlap=true, onFirstSegment=false, onSecondSegment=false`。

#### 与 #2 的关系
- 引入 `SCLine2d` 后，`ExtendFirst` / `ExtendSecond` 中"线段延长为直线"复用 #2 实现，避免重复；`infiniteOverlap` 语义也与 #2 对齐。
- 弧延长语义独立，必须在本项实现。

#### 文件落位
- `Include/Core/Results.h`：追加 `SCExtensionPolicy` enum 与 `SCExtendedIntersection2d` 结构体。
- `Include/Core/Intersection.h`：追加 `IntersectExtended` 函数声明（已 include Results.h）。
- `Source/Core/Intersection.cpp`：实现。
- `UnitTests/Capabilities/Core/TestIntersectionExtended.cpp`（新建，capability 用例）；`UnitTests/Gaps/Core/TestIntersectionExtendedGaps.cpp`（新建，gap 用例；`Gaps/Core/` 目录不存在需新建，见 §四第 8 条）。

#### 测试要求
- **Capability**：两线段延长后相交（原本段内不相交）→ 命中且 `onFirstSegment=false, onSecondSegment=false`；线段延长与弧相切 → `kind=Tangent`。
- **Edge**：`policy=None` 与现有 `Intersect` 行为完全一致；`ExtendBoth` 弧-弧全圆命中 2 点；两条共线但彼此分离的线段在 `ExtendBoth` 下返回 `kind=Overlap, pointCount=0, infiniteOverlap=true, onFirstSegment=false, onSecondSegment=false`。
- **Gap**：`ExtendFirst` 弧端点延长角度区间展开方向依赖 `SCArcDirection`，当 `sweepAngle≈0` 时方向不明确；标 gap，建议调用方避免对零弧使用延长。

---

### 缺失项 #4：3D 点到线段最近点

#### 背景
`Core/Projection.h` 中 `ProjectPointToLine(SCPoint3d, SCLine3d, ...)` 投到无限直线；3D 无 `ProjectPointToLineSegment3d`，2D 已有 `ProjectPointToSegment`。

#### API 契约
追加到 `Include/Core/Projection.h`：

```cpp
struct GEOMETRY_API SCSegmentProjection3d
{
    SCPoint3d point{};
    double parameter{0.0};
    double distanceSquared{0.0};
    bool isOnSegment{false};

    [[nodiscard]] bool IsValid() const
    {
        return point.IsValid() && std::isfinite(parameter) && std::isfinite(distanceSquared) &&
               distanceSquared >= 0.0;
    }

    [[nodiscard]] std::string DebugString() const;
};

[[nodiscard]] GEOMETRY_API SCSegmentProjection3d
ProjectPointToLineSegment(const SCPoint3d& point,
                          const SCLineSegment3d& segment,
                          bool clampToSegment = true,
                          const SCGeometryTolerance3d& tolerance = {});

[[nodiscard]] GEOMETRY_API SCSegmentProjection3d
ProjectPointToLineSegment(const SCPoint3d& point,
                          const SCPoint3d& segmentStart,
                          const SCPoint3d& segmentEnd,
                          bool clampToSegment = true,
                          const SCGeometryTolerance3d& tolerance = {});
```

> 注意：现有 2D 重载 `ProjectPointToLineSegment(const SCPoint2d&, const SCLineSegment2d&, bool)` 与 3D 重载参数类型不同，不构成歧义；3D 重载尾部追加默认 `tolerance` 参数，与现有 3D API 风格一致。

#### 算法要点
1. `AB = B - A`，`t = dot(AP, AB) / dot(AB, AB)`。
2. `clampToSegment=true` 时 `t = clamp(t, 0, 1)`；`isOnSegment = (t ∈ [0,1])`。
3. `point = A + AB · t`，`distanceSquared = |P - point|²`。
4. `|AB| <= tolerance.distanceEpsilon` 时返回 `t=0, point=A`（`SCGeometryTolerance3d` 字段为 `distanceEpsilon`，无 `linear` 字段，见 `Core/GeometryTypesPrimitives.h`；`distanceEpsilon` 是长度阈值，直接与 `|AB|` 长度比较，不要与 `|AB|²` 平方项比较）。
5. 与 2D `ProjectPointToSegment` 语义对齐，便于服务层 2D/3D 共用逻辑。

#### 文件落位
- `Include/Core/GeometryTypes.h`：追加 `SCSegmentProjection3d` 结构体定义（与 2D `SCSegmentProjection2d` 同文件，见 `GeometryTypes.h:46`；3D 投影系列如 `SCLineProjection3d` 也在该文件）。
- `Include/Core/Projection.h`：追加 `ProjectPointToLineSegment` 3D 重载声明（放在现有 3D 投影系列末尾，即 `ProjectPointToTriangleMesh` 重载之后，与 3D 系列同组）。
- `Source/Core/Projection.cpp`：实现。
- `UnitTests/Capabilities/Core/TestProjection3d.cpp`（新建，capability 用例）；`UnitTests/Gaps/Core/TestProjection3dGaps.cpp`（新建，gap 用例；`Gaps/Core/` 目录不存在需新建，CMake GLOB_RECURSE 自动覆盖，见 §四第 8 条）。

#### 测试要求
- **Capability**：点 P=(0,0,1) 投到段 A=(0,0,0)→B=(2,0,0)，`t=0, point=(0,0,0), isOnSegment=true`；P=(1,0,1)，`t=0.5, point=(1,0,0)`。
- **Edge**：`clamp=false` 时 `t` 可超出 [0,1]；零长度段（A==B）；P 恰在中点。
- **Gap**：3D 线段无方向归一化约束，`tolerance.distanceEpsilon` 取值影响零段判定，需在测试中显式覆盖；标 gap 仅作精度边界说明。

---

### 缺失项 #5：3D 段集空间索引

#### 背景
`Geometry2d/` 已有 `SCSegmentSearch2d` 与 `SCBoxTree2d`，`Geometry3d/` 无等价空间索引。

#### API 契约
新增 `Include/Geometry3d/SCBoxTree3d.h` 与 `Include/Geometry3d/SCSegmentSearch3d.h`，API 与 2D 版本对齐（2D → 3D 替换）。

```cpp
// Include/Geometry3d/SCBoxTree3d.h
#pragma once
#include <string>
#include <vector>

#include "Core/Metrics.h"
#include "Export/GeometryExport.h"
#include "Support/Epsilon.h"
#include "Types/Geometry3d/SCBox3.h"
#include "Types/Geometry3d/SCPoint3.h"

namespace Geometry
{
    struct GEOMETRY_API SCBoxTreeEntry3d
    {
        std::size_t id{0};
        SCBox3d box{};
        [[nodiscard]] bool IsValid() const { return box.IsValid(); }
    };

    struct GEOMETRY_API SCBoxTreeKnnHit3d
    {
        std::size_t id{0};
        SCBox3d box{};
        double distanceSquared{0.0};
    };

    class GEOMETRY_API SCBoxTree3d
    {
    public:
        SCBoxTree3d() = default;
        explicit SCBoxTree3d(std::vector<SCBoxTreeEntry3d> entries);

        void Clear();
        void Add(std::size_t id, const SCBox3d& box);
        bool Remove(std::size_t id);
        void Update(std::size_t id, const SCBox3d& box);

        [[nodiscard]] std::size_t Size() const;
        [[nodiscard]] bool IsEmpty() const;
        [[nodiscard]] bool IsValid() const;
        [[nodiscard]] bool Contains(std::size_t id) const;
        [[nodiscard]] const SCBoxTreeEntry3d* Find(std::size_t id) const;

        [[nodiscard]] std::vector<std::size_t> Query(const SCBox3d& box, double eps = Geometry::kDefaultEpsilon) const;
        [[nodiscard]] std::vector<std::size_t> QueryContaining(const SCPoint3d& point,
                                                               double eps = Geometry::kDefaultEpsilon) const;
        [[nodiscard]] std::vector<SCBoxTreeKnnHit3d>
        QueryKNearest(const SCPoint3d& point, std::size_t k, double eps = Geometry::kDefaultEpsilon) const;

        [[nodiscard]] std::string DebugString() const;
        [[nodiscard]] const std::vector<SCBoxTreeEntry3d>& Entries() const;
        [[nodiscard]] std::vector<SCBoxTreeEntry3d>& Entries();

    private:
        std::vector<SCBoxTreeEntry3d> entries_{};
    };
}
```

```cpp
// Include/Geometry3d/SCSegmentSearch3d.h
#pragma once
#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "Core/Metrics.h"
#include "Core/Projection.h"
#include "Export/GeometryExport.h"
#include "Geometry3d/ISCCurve3d.h"
#include "Geometry3d/SCLineCurve3d.h"  // 为 Add(SCLineCurve3d) 重载
#include "Support/Epsilon.h"
#include "Types/Geometry3d/SCBox3.h"
#include "Types/Geometry3d/SCPoint3.h"
#include "Types/Geometry3d/SCLineSegment3d.h"

namespace Geometry
{
    struct GEOMETRY_API SCSegmentSearchEntry3d
    {
        std::size_t id{0};
        std::shared_ptr<const ISCCurve3d> curve{};
        SCBox3d box{};
        [[nodiscard]] bool IsValid() const { return curve != nullptr && box.IsValid(); }
    };

    struct GEOMETRY_API SCSegmentSearchHit3d
    {
        std::size_t id{0};
        SCPoint3d point{};
        double distanceSquared{0.0};
        double parameter{0.0};
        bool isOnSegment{false};
        [[nodiscard]] bool IsValid() const { return point.IsValid() && distanceSquared >= 0.0; }
    };

    class GEOMETRY_API SCSegmentSearch3d
    {
    public:
        SCSegmentSearch3d() = default;
        explicit SCSegmentSearch3d(std::vector<SCSegmentSearchEntry3d> entries);

        void Clear();
        std::size_t Add(std::shared_ptr<const ISCCurve3d> curve);
        std::size_t Add(const ISCCurve3d& curve);
        std::size_t Add(const SCLineSegment3d& segment);  // 内部包一层 SCLineCurve3d
        std::size_t Add(const SCLineCurve3d& curve);       // 便利重载，对齐 2D 的 Add(SCArcSegment2d)
        bool Remove(std::size_t id);

        [[nodiscard]] std::size_t Size() const;
        [[nodiscard]] bool IsEmpty() const;
        [[nodiscard]] bool IsValid() const;
        [[nodiscard]] bool Contains(std::size_t id) const;
        [[nodiscard]] const SCSegmentSearchEntry3d* Find(std::size_t id) const;

        [[nodiscard]] std::vector<std::size_t> QueryIntersecting(const SCBox3d& box,
                                                                 double eps = Geometry::kDefaultEpsilon) const;
        [[nodiscard]] std::vector<std::size_t> QueryIntersecting(const ISCCurve3d& curve,
                                                                 double eps = Geometry::kDefaultEpsilon) const;
        [[nodiscard]] std::vector<SCSegmentSearchHit3d>
        QueryWithinDistance(const SCPoint3d& point, double maxDistance) const;
        [[nodiscard]] std::optional<SCSegmentSearchHit3d> Nearest(const SCPoint3d& point) const;
        [[nodiscard]] std::vector<SCSegmentSearchHit3d>
        QueryKNearest(const SCPoint3d& point, std::size_t k,
                      double maxDistance = std::numeric_limits<double>::max()) const;

        [[nodiscard]] std::string DebugString() const;
        [[nodiscard]] const std::vector<SCSegmentSearchEntry3d>& Entries() const;
        [[nodiscard]] std::vector<SCSegmentSearchEntry3d>& Entries();

    private:
        std::size_t nextId_{0};
        std::vector<SCSegmentSearchEntry3d> entries_{};
    };
}
```

#### 算法要点
1. **`SCBoxTree3d` 必须首版即实现 BVH**（镜像 2D 的 `BoxNode`/`BuildBoxTree`，源码见 `Source/Geometry2d/SCBoxTree2d.cpp`）：3D AABB 节点按最长轴中位数分裂，叶子节点持有 entry 索引列表。不允许先做线性 `entries_` 再"升级"——2D 已有 BVH 参考，3D 必须对齐。
2. **`SCSegmentSearch3d` 首版可线性 `entries_`**（与当前 `SCSegmentSearch2d` 一致），公开 API 与 2D 对齐；后续 2D 升级为 BVH 时 3D 同步升级。
3. **box 与 box 相交**：3D AABB 三轴区间重叠判定；可直接复用 §四新增的 `Intersects(SCBox3d, SCBox3d)`（见公共支撑改动）。
4. **box 包含点**：三轴区间同时包含；复用 `Contains(SCBox3d, SCPoint3d)`。
5. **KNN**：`SCBoxTree3d` 用 BVH 优先队列（与 #8 `SCBoxTree2d` 同策略）；`SCSegmentSearch3d` 用线性 + 大小为 k 的最大堆（与 #8 `SCSegmentSearch2d` 同策略）。
6. **段 vs box 粗筛**：用 `curve.Bounds()` 与查询 box 比较；命中后再调用 `ProjectPointToCurve` 精算。
7. **`SCCurveProjection3d` → `SCSegmentSearchHit3d` 转换**（`Nearest`/`QueryWithinDistance`/`QueryKNearest` 内部）：调用 `ProjectPointToCurve(point, curve, tolerance)` 得 `SCCurveProjection3d`（`GeometryTypes.h:128-140`，字段 `success/point/parameter/distanceSquared`）；若 `!success` 跳过该 curve；`isOnSegment` 由 `curve.ParameterRange().Contains(projection.parameter)` 判定（`SCIntervald::Contains` 已存在）；构造 `SCSegmentSearchHit3d{id, point, distanceSquared, parameter, isOnSegment}`。
8. **`SCLineCurve3d` 依赖**：`Add(const SCLineSegment3d&)` 内部用 `SCLineCurve3d::FromLine(SCLine3d::FromOriginAndDirection(seg.startPoint, seg.endPoint - seg.startPoint), SCIntervald{0.0, 1.0})` 包一层（`SCLineCurve3d` 已存在，见 `Include/Geometry3d/SCLineCurve3d.h`，是 `ISCCurve3d` 的 final 实现；`SCLineSegment3d` 字段为 `startPoint`/`endPoint`，见 `Types/Geometry3d/SCLineSegment3d.h:14-15`，无 `Start()`/`End()` 访问器）。

#### 文件落位
- `Include/Geometry3d/SCBoxTree3d.h`、`Include/Geometry3d/SCSegmentSearch3d.h`（新增）。
- `Source/Geometry3d/SCBoxTree3d.cpp`、`Source/Geometry3d/SCSegmentSearch3d.cpp`（新增）。
- `Include/Core/GeometryApi.h`：追加 include。
- `UnitTests/Capabilities/Geometry3d/TestBoxTree3d.cpp`、`TestSegmentSearch3d.cpp`（新建，capability 用例）；`UnitTests/Gaps/Geometry3d/TestBoxTree3dGaps.cpp`、`TestSegmentSearch3dGaps.cpp`（新建，gap 用例）。

#### 测试要求
- **Capability**：插入若干 3D box / curve，`Query(box)`、`QueryContaining(point)`、`Nearest`、`QueryKNearest(k=2)` 命中正确。
- **Edge**：空容器；`k > Size()`；`maxDistance` 过滤；点落在 box 边界（eps 容差）。
- **Gap**：`SCSegmentSearch3d` 首版线性 `entries_`（与 2D 镜像），O(N) 查询；待 2D 升级 BVH 后同步升级，公开 API 不变。`SCBoxTree3d` 首版即 BVH，无此 gap。

---

### 缺失项 #6：3D 段集捕捉通用入口

#### 背景
`Core/AxisOps.h` 现有 `SnapPointToSegments` 只支持 2D `ISCSegment2d`；3D 无等价入口。

#### API 契约
新增 `Include/Core/Snap3d.h`（与 2D 入口 `AxisOps.h` 解耦，避免 2D 头被 3D 依赖污染）：

```cpp
#pragma once

#include <span>

#include "Export/GeometryExport.h"
#include "Geometry3d/ISCCurve3d.h"
#include "Types/Geometry3d/SCPoint3.h"

namespace Geometry
{
    struct GEOMETRY_API SCSnapResult3d
    {
        bool snapped{false};
        SCPoint3d point{};
        double distanceSquared{0.0};
        std::size_t segmentIndex{0};   // 命中段在输入 span 中的下标
        double parameter{0.0};         // 段内参数（曲线为曲线参数，线段为 [0,1]）

        [[nodiscard]] bool IsValid() const { return !snapped || point.IsValid(); }
    };

    // 点到 3D 段集的最近点捕捉；maxDistance 为最大捕捉半径。
    [[nodiscard]] GEOMETRY_API SCSnapResult3d
    SnapPointToSegments3d(const SCPoint3d& point,
                          std::span<const ISCCurve3d* const> curves,
                          double maxDistance,
                          const SCGeometryTolerance3d& tolerance = {});

    // 便利重载：线段集合（内部封装为 SCLineCurve3d）
    [[nodiscard]] GEOMETRY_API SCSnapResult3d
    SnapPointToSegments3d(const SCPoint3d& point,
                          std::span<const SCLineSegment3d* const> segments,
                          double maxDistance,
                          const SCGeometryTolerance3d& tolerance = {});
}
```

#### 算法要点
1. 复用 #5 的 `SCSegmentSearch3d::Nearest`：把 `curves` 入树，`Nearest(point)` 直接得到最近段与最近点。
2. `maxDistance` 过滤：预计算 `bestDistanceSquared = maxDistance * maxDistance`（与 2D `AxisOps.cpp:282` 一致），`distanceSquared > bestDistanceSquared` 时 `snapped=false`。
3. **结果填充 `segmentIndex`**：`SCSegmentSearch3d` 当前用自增 `id`（见 #5 API 契约 line 616-619，**无 `userData` 字段、无 `Add(curve, userData)` 重载**，2D `SCSegmentSearchEntry2d` 也无 `userData`，见 `SCSegmentSearch2d.h:17-27`）。**采用调用方维护映射方案**：`Snap3d` 内部维护 `std::unordered_map<std::size_t, std::size_t> idToSpanIndex`，每次 `tree.Add(curve)` 返回 id 后记录 `idToSpanIndex[id] = spanIndex`；`Nearest` 返回 `SCSegmentSearchHit3d.id` 后查表得 spanIndex，填入 `SCSnapResult3d.segmentIndex`。**不修改 #5 的 `SCSegmentSearch3d` API 与 `SCSegmentSearchEntry3d` 结构体**（保持与 2D 镜像，不引入 userData）。
4. **退化**：`curves.empty()` → `snapped=false`；`maxDistance < 0` → `snapped=false`（与 2D `SnapPointToSegments` 的 `!(maxDistance >= 0.0)` 守卫对齐，见 `AxisOps.cpp:277`）；`maxDistance == 0` 时进入主循环，`bestDistanceSquared = 0` 仅命中距离平方为 0 的段。

#### 与 2D `SnapPointToSegments` 的实现差异（有意为之）
- 2D `SnapPointToSegments`（`Core/AxisOps.cpp` 现有实现）走**全量遍历 + `ProjectPointToAxis`**，未使用空间索引，ε 用 `kAxisOpsDefaultEpsilon`。
- 3D `SnapPointToSegments3d` 走**空间索引路径**（`SCSegmentSearch3d::Nearest`），ε 用 `tolerance.distanceEpsilon`。
- 差异原因：3D 后端为新模块，首版即按"索引化"路径设计，避免后续二次重构；2D 现有实现保持不变，后续可作为独立优化项切换到 `SCSegmentSearch2d`（不在本方案范围）。
- **不强制要求 2D/3D 实现风格一致**，但**返回结构体字段必须严格对齐**：`SCSnapResult3d` 与 `SCSnapResult2d`（`Core/Results.h`）字段一致——`snapped` / `point` / `distanceSquared` / `segmentIndex` / `parameter`，**不增加 `isOnSegment` 字段**（2D 现状无此字段，3D 也不加，避免字段不对齐；调用方需判断是否段内时用 `parameter ∈ [0,1]` 自行计算）。

#### 与 #4、#5 的依赖
- 依赖 #4：3D 线段投影（用于 `SCLineSegment3d` 重载）。
- 依赖 #5：3D 段集索引（用于 `Nearest`）。
- 若 #5 尚未落地，可临时用全量遍历 + `ProjectPointToCurve`；但发布前必须切到 #5。

#### 文件落位
- `Include/Core/Snap3d.h`（新增）。
- `Source/Core/Snap3d.cpp`（新增）。
- `Include/Core/GeometryApi.h`：追加 include。
- `UnitTests/Capabilities/Core/TestSnap3d.cpp`（新建，capability 用例）；`UnitTests/Gaps/Core/TestSnap3dGaps.cpp`（新建，gap 用例；`Gaps/Core/` 目录不存在需新建，见 §四第 8 条）。

#### 测试要求
- **Capability**：3 条 3D 线段，P 在中间段附近，命中最近点；`maxDistance` 适当放大时命中。
- **Edge**：`curves.empty()`；`maxDistance=0`（进入主循环，仅命中距离为 0 的段）；P 恰在线段端点；P 在线段延长线上（`snapped=true`，调用方用 `parameter` 范围自行判断是否段内）。
- **Gap**：当 `maxDistance` 跨段相近时，仅返回单个最近；多候选 Tab 切换由服务层基于 `QueryKNearest` 实现，本入口不负责；标 gap。

---

## 四、公共支撑改动

为支持上述算法，需要以下统一改动（不视为独立缺失项，归为公共工程）：

1. **`SCBoxTreeHit2d` 不动字段**：已对外发布，追加字段属 ABI 破坏。**新增专用类型 `SCBoxTreeKnnHit2d`**（`id` + `box` + `distanceSquared`，定义在 `Geometry2d/SCBoxTree2d.h`），`SCBoxTree2d::QueryKNearest` 返回 `std::vector<SCBoxTreeKnnHit2d>`。`SCBoxTreeKnnHit3d` 为本次新增类型，可直接内含 `distanceSquared` 字段。
2. **`SCArcSegment2d` 新增 public `ContainsAngle`**：现有 `IsAngleOnArc` 是 private，签名 `bool IsAngleOnArc(double candidateAngle) const`（**无 eps 参数**，见 `Geometry2d/SCArcSegment2d.h:48`）。**采用"新增 public 方法"方案，签名与 `IsAngleOnArc` 一致（无 eps）**：
   ```cpp
   // Include/Geometry2d/SCArcSegment2d.h 追加 public 方法
   [[nodiscard]] bool ContainsAngle(double angle) const;
   ```
   实现内部直接转发 `return IsAngleOnArc(angle);`。不采用 `friend` 方案（污染依赖）。供 #1、#7 复用。
3. **`Include/Core/GeometryApi.h` umbrella 更新**：追加新增头文件 include（`Tangent.h`、`Snap3d.h`、`SCLine2d.h`、`SCBoxTree3d.h`、`SCSegmentSearch3d.h`）。注意：`Include/Geometry.h` 是产品侧稳定精选入口，不要动。
4. **`docs/test-capability-coverage.md`**：新增模块行（`Tangent`、`SCLine2d`、`IntersectExtended`、`ProjectPointToLineSegment3d`、`SCBoxTree3d`、`SCSegmentSearch3d`、`Snap3d`、折线求交、KNN）。
5. **`docs/design-doc-sync-tracker.md`**：登记本次新增的发布面与 gap 同步项。
6. **CMakeLists.txt 无需手动登记**：根 `CMakeLists.txt` 用 `file(GLOB_RECURSE)` 自动扫描 `Include/*.h` 与 `Source/*.cpp`（带 `CONFIGURE_DEPENDS`）；新增 cpp 放到对应 `Source/<Module>/` 目录即可。仅当新增 `Source/` 下顶层目录时才需检查递归扫描覆盖。
7. **`Metrics.h` 追加 3D box 工具**（#5 依赖）：现有 `Core/Metrics.h` 仅提供 `Intersects(SCBox2d, SCBox2d)` 与 `Contains(SCBox2d, SCPoint2d)`，无 3D 版本。本次追加：
   ```cpp
   [[nodiscard]] GEOMETRY_API bool Intersects(const SCBox3d& a, const SCBox3d& b, double eps = Geometry::kDefaultEpsilon);
   [[nodiscard]] GEOMETRY_API bool Contains(const SCBox3d& box, const SCPoint3d& point, double eps = Geometry::kDefaultEpsilon);
   [[nodiscard]] GEOMETRY_API double DistanceSquared(const SCPoint3d& point, const SCBox3d& box);
   [[nodiscard]] GEOMETRY_API double DistanceSquared(const SCPoint3d& point, const SCLineSegment3d& segment);
   ```
   供 `SCBoxTree3d`、`SCSegmentSearch3d` 复用；2D 版本保持不变。
8. **新增 `UnitTests/Gaps/Core/` 目录**（#1、#2、#3、#4、#6、#7、#8 的 gap 用例落位）：现有 `UnitTests/Gaps/` 下只有 `Geometry2d/`、`Geometry3d/`，无 `Core/`。本次新增 `Gaps/Core/` 目录存放 Core 模块的 gap 测试（如 `TestTangentGaps.cpp`、`TestIntersectionPolylineGaps.cpp`、`TestProjection3dGaps.cpp`、`TestSnap3dGaps.cpp`、`TestIntersectionExtendedGaps.cpp`）。`UnitTests/CMakeLists.txt` 的 `file(GLOB_RECURSE Gaps/*.cpp)` 会自动扫到新目录，**无需改 CMake**；命名遵循现有 `Gaps/<Module>/Test<Area>Gaps.cpp` 约定。

---

## 五、批次与执行路线

### 批次 1：已完成（原 P0，第二版前必补）
- **目标**：#3 折线批量求交、#8 K 近邻查询。
- **任务分派**：B 类（沿既定方向扩展，contract 清晰）。
- **验收**：
  - 折线求交返回确定性顺序、端点去重正确；
  - KNN 跨 `SCBoxTree2d` / `SCSegmentSearch2d` 行为一致，按距离升序；
  - 服务层切换调用并删除绕行代码（#3）/启用候选 Tab 切换（#8）。
- **测试**：每项 capability + edge + gap 至少 3 个用例。
- **同步文档**：`session-handoff.md`、`todolist.md`、`test-capability-coverage.md`、`design-doc-sync-tracker.md`。

### 批次 2：已完成（原 P1，第三版前必补）
- **目标**：#1 切点、#2 无限直线/延长线求交、#7 段延长区域交点。
- **任务分派**：A 类（涉及策略：弧延长角度区间、共线 Overlap 语义、切点精度边界），建议 `gpt-5.2` + `high`。
- **依赖顺序**：#2 先行（`SCLine2d` 落地）→ #1、#7 可并行。
- **验收**：
  - 切点数量与角度满足几何关系；
  - 直线 vs 直线 / 直线 vs 线段 行为对称；
  - `IntersectExtended(policy=None)` 与现有 `Intersect` 行为完全一致（黄金对比测试）；
  - 弧延长场景覆盖 `ExtendBoth` / `ExtendFirst` / `ExtendSecond`。
- **测试**：每项 capability + edge + gap 至少 3 用例，#7 必须含与现有 `Intersect` 的等价性回归。

### 批次 3：已完成（原 P2，3D 后端前必补）
- **目标**：#4 3D 点到线段最近点、#5 3D 段集空间索引、#6 3D 段集捕捉入口。
- **任务分派**：A 类（3D 新建模块，需保证与 2D 镜像一致且不引入 3D 退化），建议 `gpt-5.2` + `high`。
- **依赖顺序**：#4 → #5 → #6（#6 依赖 #4、#5）。
- **验收**：
  - `ProjectPointToLineSegment3d` 与 2D `ProjectPointToSegment` 语义对齐；
  - 3D 索引 API 与 2D 镜像，行为一致；
  - `SnapPointToSegments3d` 命中正确，`maxDistance` 过滤生效；
  - 3D 后端启用前全量测试通过。
- **测试**：每项 capability + edge + gap 至少 3 用例；#6 必须含空输入、零距离、跨段相近场景。

---

## 五.1 后续待办

当前已完成 P0 / P1 / P2 的主功能交付，但以下 gap 仍保留为后续待办：

- `TangentPoints` 的 epsilon 边界与近切稳定性
- `IntersectExtended` 的退化重合与无限重合语义边界
- 折线交点的端点去重与排序稳定性
- 3D `ProjectPointToLineSegment` 的零长 / 近零长段判定边界
- `SnapPointToSegments3d` 的等距并列候选 tie-break 约定
- `SCSegmentSearch3d` 的 KNN 边界组合覆盖

这些项已通过 `UnitTests/Gaps/...` 中的 gap 用例显式保留，后续若语义进一步稳定，再按能力测试标准提升。

---

## 六、命名与风格合规检查清单

每一项落地前必须自检：

- [ ] 命名空间 `Geometry::`，类型 PascalCase，函数 PascalCase，变量 lowerCamelCase，常量 `k` 前缀。
- [ ] 维度后缀统一 `2d` / `3d`，不出现 `2D` / `3D`。
- [ ] 文件名 PascalCase，一文件一职责。
- [ ] 公开 API 使用 `[[nodiscard]]` + `GEOMETRY_API`，短名风格（`Query`、`Nearest`、`Bounds`），不使用 `GetXxx`。
- [ ] 复杂结果使用结构体（至少含 `IsValid`；`DebugString` 按现有模块习惯按需提供），不返回裸 `bool`。
- [ ] 数值比较走 `Support/Epsilon.h`，禁止裸 `1e-9`。
- [ ] 3D API 默认参数 `SCGeometryTolerance3d tolerance = {}`。
- [ ] 不修改已发布 API contract，仅在尾部追加重载或新增头文件。
- [ ] 新增头文件在 `Include/Core/GeometryApi.h` 中追加 include（不要动 `Include/Geometry.h`）。
- [ ] 新增 cpp 放在对应 `Source/<Module>/` 目录下即可，`CMakeLists.txt` 用 `file(GLOB_RECURSE Source/*.cpp)` 自动扫描（带 `CONFIGURE_DEPENDS`），**不要手动改 CMakeLists.txt**；仅当新增 `Source/` 下顶层目录时才需确认递归扫描覆盖。

---

## 七、测试与验收总要求

1. **三类测试齐全**：每个新 API 至少 1 个 capability 用例 + 1 个 edge 用例 + 1 个 gap 用例（gap 用例必须显式标注 gap 并在文档说明）。
2. **确定性**：所有用例跨运行结果一致；浮点比较用 `AlmostEquals(eps)`。
3. **回归**：#7 `policy=None` 必须与现有 `Intersect` 行为完全一致，作为回归基线。
4. **覆盖矩阵**：`docs/test-capability-coverage.md` 同步追加新 API 行，标注 capability / edge / gap 用例数。
5. **CI**：`ci-windows-cmake` 必须通过；不允许新增失败用例。
6. **文档同步**：每个批次收尾时同步更新四份文档（`session-handoff.md`、`todolist.md`、`test-capability-coverage.md`、`design-doc-sync-tracker.md`）。

---

## 八、风险与对策

| 风险 | 影响 | 对策 |
|---|---|---|
| `SCBoxTreeHit2d` 追加字段破坏 ABI | 高 | 已对外发布类型，追加字段会改变布局。**采用方案 A**：新增 `SCBoxTreeKnnHit2d` 专用类型（`id` + `box` + `distanceSquared`），`QueryKNearest` 返回 `std::vector<SCBoxTreeKnnHit2d>`，不动既有结构体；`SCBoxTreeKnnHit3d` 为新增类型可直接含 `distanceSquared`。 |
| 弧延长角度区间方向歧义 | 中 | #7 中显式要求 `SCArcDirection`，零弧场景标 gap 不调用。 |
| 切点精度在 `d≈r` 时退化 | 低 | #1 测试覆盖 `d-r ∈ (0, eps)` 边界，标 gap。 |
| 3D `SCSegmentSearch3d` 首版线性 O(N) 性能 | 低 | 首版线性 `entries_`（与 2D `SCSegmentSearch2d` 现状一致），公开 API 与 2D 镜像；待 2D 升级 BVH 后同步升级。`SCBoxTree3d` 首版即 BVH，无此风险。 |
| 折线共线 Overlap 端点枚举不完整 | 中 | #3 标 gap：连续共线段只返回首尾端点；调用方按需扩展。 |
| 服务层绕行代码切换滞后 | 中 | 批次 1 收尾时强制删除 `SCIntersectionSnapResolver` 内绕行代码并切换调用。 |

---

## 九、与捕捉点服务设计的对应关系

| 缺失项 | 服务设计章节 | 服务层调用点 |
|---|---|---|
| #1 切点 | §五 第三版 `Tangent=13` | `TangentSnapResolver` |
| #2 无限直线/延长线求交 | §五 第三版 `Extension=14` | `ExtensionSnapResolver` |
| #3 折线批量求交 | §六 主流程 IntersectionResolver | `SCIntersectionSnapResolver` |
| #4 3D 点到线段最近点 | §六 主流程 NearestResolver（3D） | `SCNearestSnapResolver3d` |
| #5 3D 段集空间索引 | §八 空间索引选型（3D） | `SCSpatialQueryService`（3D） |
| #6 3D 段集捕捉入口 | §六 主流程 NearestResolver（3D） | `SCNearestSnapResolver3d` |
| #7 段延长区域交点 | §六 主流程 IntersectionResolver（弧场景） | `SCIntersectionSnapResolver` |
| #8 K 近邻查询 | §六 主流程 候选 Tab 切换 | `SCCandidateTabSwitcher` |

> 服务层切换顺序：批次 1 落地后切换 #3、#8；批次 2 落地后切换 #1、#2、#7；批次 3 落地后启用 3D 后端（A-07）。

---

## 十、交付物清单

每个批次交付：

- 头文件：`Include/` 下新增/追加声明。
- 源文件：`Source/` 下新增/追加实现。
- 测试：`UnitTests/Capabilities/` 下新增测试文件，含 capability / edge / gap。
- 文档同步：四份文档同步更新。
- 提交信息：`[Core] ...`、`[Geometry2d] ...`、`[Geometry3d] ...` 按模块前缀。

最终交付：

- 8 个缺失 API 全部落地并通过测试。
- 服务层切换调用，删除绕行代码。
- `docs/test-capability-coverage.md` 中 8 个新 API 行齐全且用例数 ≥ 3。
- `SCGeometry捕捉算法缺失清单.md` 中"临时绕行方案"章节全部标记为"已收敛到几何库"。

---

## 十一、附录：现有可复用能力速查

| 能力 | 头文件 | 备注 |
|---|---|---|
| 2D 段投影 | `Core/Projection.h` `ProjectPointToSegment` | #3、#8 复用 |
| 2D 段求交 | `Core/Intersection.h` `Intersect(segment, segment)` | #3 段配对细算 |
| 2D box 树 | `Geometry2d/SCBoxTree2d.h` | #3 粗筛、#8 KNN |
| 2D 段索引 | `Geometry2d/SCSegmentSearch2d.h` | #8 KNN |
| 2D KD-Tree | `Geometry2d/SCKDTree2d.h` | 当前方案不依赖 |
| 圆/弧类型 | `Geometry2d/SCCircle2d.h`、`SCArcSegment2d.h` | #1、#7 复用 |
| 3D 曲线投影 | `Core/Projection.h` `ProjectPointToCurve` | #5、#6 复用 |
| 3D 直线 | `Types/Geometry3d/SCLine3d.h` | #4 参考 |
| 3D 线段 | `Types/Geometry3d/SCLineSegment3d.h` | #4、#6 输入 |
| 3D AABB | `Types/Geometry3d/SCBox3.h` | #5 复用 |
| 3D 曲线接口 | `Geometry3d/ISCCurve3d.h` | #5、#6 输入 |
| 2D 折线 | `Geometry2d/SCPolyline2d.h` | #3 输入 |

---

本方案为 AI 落地开发的直接执行依据，按批次推进，每轮闭环，禁止只写代码不写测试，禁止把不稳定行为伪装成能力。
