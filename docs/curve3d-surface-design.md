# Curve3d / Surface Design

## 1. Purpose

本文定义 StableCore 三维库中参数化曲线与参数化曲面层的公共设计。

目标不是一次做完整 CAD 曲线曲面内核，而是先把后续所有 3D 算法都依赖的公共协议固定下来：
- 曲线和曲面的最小公共接口
- 派生类型分层
- 参数域、导数、投影、裁剪、包围盒的统一语义
- 与 2D preimage、BRep face/edge、mesh tessellation 的连接点

## 2. Reference Direction

这层设计主要吸收两类参考：
- Delphi `GGLSurface3d.pas`
  - 说明了工程 3D 中 plane/surface 会很早承载距离、相交、投影、局部坐标、2D/3D 桥接能力
  - 也说明 2D break polygon / break lines 与 3D 面切分是强耦合的
- GGP `Curve3d.cpp` / `Surface.cpp`
  - 给出了较完整的参数化基类协议
  - 包括参数域、导数、投影、subdivide、bounding box、preimage 参数桥接等能力

StableCore 不直接照抄两边接口，而是取其共同核心，收紧成更稳定的 SDK 风格。

## 3. Scope

本层覆盖：
- `Curve3d`
- `TrimmedCurve3d`
- `LineCurve3d`
- `ArcCurve3d`
- `EllipseCurve3d`
- `NurbsCurve3d`
- `Surface`
- `PlaneSurface`
- `RuledSurface`
- `RevolvedSurface`
- `OffsetSurface`
- `NurbsSurface`
- 与之配套的 projection / derivative / evaluation result 类型

本层暂不覆盖：
- `BrepFace` / `BrepEdge`
- `TriangleMesh`
- 实体布尔
- 曲面缝合
- 复杂 healing

## 4. Design Goals

### 4.1 Keep Curve and Surface as Parametric Protocols

`Curve3d` 是一维参数对象。

`Surface` 是二维参数对象。

它们的职责是：
- 提供稳定求值协议
- 提供参数域信息
- 提供投影、导数、基础边界能力
- 为更高层 topology / boolean / section 提供精确几何支撑

它们不直接承担：
- 拓扑 ownership
- 布尔结果组织
- healing 策略

### 4.2 Value-Friendly Inputs, Polymorphic Outputs

点、向量、范围、box 等仍用透明值类型。

曲线和曲面允许多态，因为：
- 派生族天然存在
- BRep face/edge 需要抽象底层几何
- trimming / offset / conversion 需要返回基类指针或句柄

但 StableCore 应限制多态接口深度，避免形成 GGP 那种过宽的大基类。

### 4.3 Parameter Domain Is First-Class

参数域不是附属信息，而是这层的核心基础设施。

必须统一定义：
- 曲线参数区间 `Intervald`
- 曲面 `U/V` 双参数区间
- 是否闭合
- 是否周期
- 修剪后对象与底层母体对象的关系

后续：
- point at length
- nearest parameter
- subdivision
- trim / split
- preimage edge

都依赖这套规则。

### 4.4 2D / 3D Bridge Must Be Planned Up Front

Delphi 和 GGP 都表明：
- 曲面上的边界往往需要 2D preimage 曲线
- 面布尔、裁剪、投影、切分经常在曲面参数域中转成 2D 问题

因此这层必须预留：
- `CurveOnSurface`
- `SurfaceParameterBox`
- `ProjectToSurface`
- `LiftFromSurfaceParam`

而不是把 2D 桥接留到 BRep 再临时拼接。

## 5. Common Result Types

建议先定义一批轻结果类型，避免以后每个派生类都自定义返回结构。

### 5.1 `CurveEval3d`

建议字段：
- `Point3d point`
- `Vector3d d1`
- `Vector3d d2`
- `Vector3d d3`
- `int maxOrder`

用途：
- 单次求值时统一返回点和导数
- 减少重复 virtual dispatch

### 5.2 `SurfaceEval3d`

建议字段：
- `Point3d point`
- `Vector3d du`
- `Vector3d dv`
- `Vector3d duu`
- `Vector3d duv`
- `Vector3d dvv`
- `int maxOrder`

用途：
- 法向、曲率、局部 frame 都可建立在此结果上

### 5.3 `CurveProjectionResult`

建议字段：
- `double t`
- `Point3d point`
- `double distance`
- `bool onBoundary`
- `bool converged`

### 5.4 `SurfaceProjectionResult`

建议字段：
- `double u`
- `double v`
- `Point3d point`
- `double distance`
- `bool onBoundary`
- `bool converged`

### 5.5 `CurveSplitResult` / `SurfaceSplitResult`

建议用小聚合返回：
- `std::unique_ptr<Curve3d> first`
- `std::unique_ptr<Curve3d> second`

以及：
- `std::unique_ptr<Surface> first`
- `std::unique_ptr<Surface> second`

避免模糊的 out-parameter 风格。

## 6. `Curve3d` Base Protocol

### 6.1 Responsibilities

`Curve3d` 负责：
- 参数区间和端点语义
- 点与导数求值
- 弧长与参数桥接
- 最近点/投影
- 局部边界信息

### 6.2 Recommended Minimal Public Interface

建议最小协议：
- `Intervald Range() const`
- `double StartT() const`
- `double EndT() const`
- `bool IsClosed() const`
- `bool IsPeriodic() const`
- `Point3d StartPoint() const`
- `Point3d EndPoint() const`
- `Point3d PointAt(double t) const`
- `CurveEval3d Evaluate(double t, int maxOrder = 1) const`
- `Vector3d FirstDerivativeAt(double t) const`
- `Box3d Bounds() const`
- `Box3d Bounds(const Intervald& range) const`
- `double Length() const`
- `double Length(const Intervald& range) const`
- `CurveProjectionResult Project(const Point3d& point) const`
- `double ParameterAtLength(double baseT, double arcLength, double tolerance) const`
- `std::unique_ptr<Curve3d> Clone() const`
- `std::unique_ptr<Curve3d> Trim(const Intervald& range) const`
- `CurveSplitResult Split(double t) const`
- `std::unique_ptr<Curve2d> ProjectToPlane(const Plane& plane) const`

### 6.3 Optional Second-Stage Interface

第一阶段可先不公开，但内部应允许扩展：
- `Cone DirectionCone(...)`
- `TiltBox3d TiltBounds(...)`
- `Frame3d FrenetFrameAt(...)`
- `bool CanSubdivide(double t) const`
- `std::unique_ptr<NurbsCurve3d> ToNurbs() const`

这部分更接近 GGP 的 rich geometry protocol，可后置。

## 7. Curve Type Family

### 7.1 `LineCurve3d`

定位：
- 最简单的解析曲线
- 同时服务直线边、section 结果、拟合退化结果

建议存储：
- `Point3d origin`
- `Direction3d direction`
- `Intervald range`

### 7.2 `LineSegmentCurve3d`

如果后续发现 `LineCurve3d + finite range` 语义足够，可以不单独公开这个类型。

默认建议：
- SDK 公开 `Line3d` 和 `LineSegment3d` 值类型
- 参数化层只保留 `LineCurve3d`

### 7.3 `ArcCurve3d` / `EllipseCurve3d`

建议作为较早支持的解析曲线，因为：
- Delphi break-arc 工程路径显示圆弧非常常见
- offset / section / revolve / loft 也都强依赖圆弧/椭圆弧能力

建议存储：
- `Plane plane`
- `Point3d center`
- `Vector3d majorAxis`
- `Vector3d minorAxis`
- `Intervald angleRange`

### 7.4 `NurbsCurve3d`

作为统一 fallback 曲线表达。

建议职责：
- 作为复杂解析曲线转换目标
- 提供 tessellation、projection、split 的统一实现支撑
- 不急着先做满所有编辑操作

## 8. `Surface` Base Protocol

### 8.1 Responsibilities

`Surface` 负责：
- 二维参数域
- 点、导数、法向、局部 frame 求值
- 点到曲面的投影
- 裁切 / split / trim
- 参数域与 3D 的桥接

### 8.2 Recommended Minimal Public Interface

建议最小协议：
- `Intervald RangeU() const`
- `Intervald RangeV() const`
- `double StartU() const`
- `double EndU() const`
- `double StartV() const`
- `double EndV() const`
- `bool IsClosedU() const`
- `bool IsClosedV() const`
- `Point3d PointAt(double u, double v) const`
- `SurfaceEval3d Evaluate(double u, double v, int maxOrder = 1) const`
- `Vector3d NormalAt(double u, double v) const`
- `Box3d Bounds() const`
- `SurfaceProjectionResult Project(const Point3d& point) const`
- `Box2d ParameterBox() const`
- `std::unique_ptr<Surface> Clone() const`
- `std::unique_ptr<Surface> Trim(const Box2d& uvBox) const`
- `SurfaceSplitResult SplitU(double u) const`
- `SurfaceSplitResult SplitV(double v) const`

### 8.3 Internal-Only Second Stage

后续可在内部先实现，再观察是否公开：
- `bool CanSplitU(double u) const`
- `bool CanSplitV(double v) const`
- `std::unique_ptr<NurbsSurface> ToNurbs() const`
- `Frame3d FrameAt(double u, double v) const`
- `bool TryGetIsoCurveU(double u, std::unique_ptr<Curve3d>& curve) const`
- `bool TryGetIsoCurveV(double v, std::unique_ptr<Curve3d>& curve) const`

## 9. Surface Type Family

### 9.1 `PlaneSurface`

优先级应最高。

原因：
- Delphi `TGGLPlane3d` 已证明 plane 既是 surface 又是大量 3D 运算的基础设施
- BRep face 里绝大多数稳定起步能力都可以先从 plane face 做起

建议 `Plane` 仍是值类型，`PlaneSurface` 是参数化对象：
- `Plane` 用于几何关系和半空间运算
- `PlaneSurface` 用于 face support surface、UV 坐标、preimage 映射

### 9.2 `RuledSurface`

建议较早支持。

原因：
- lofting、sweep side faces、section bridge 常会生成 ruled surface
- 实现复杂度低于通用 NURBS

### 9.3 `RevolvedSurface`

建议早期纳入，但可以先内部使用。

原因：
- Delphi `IsArcType` 与旋转体侧面提示了这类面在工程数据里常见
- 柱面、锥面、回转面都可统一落在这条线

### 9.4 `OffsetSurface`

建议作为第二阶段曲面类型。

原因：
- 它与 normal continuity 和 singularity 强耦合
- 太早公开会把 offset/healing 复杂度引进来

### 9.5 `NurbsSurface`

作为复杂 surface 的统一 fallback。

建议策略：
- 第一阶段先支持只读评估、split、trim、project
- 编辑能力后置

## 10. Preimage and Curve-On-Surface

这是 BRep 能否落稳的关键中间层。

建议增加内部对象：
- `CurveOnSurface`
  - `std::shared_ptr<Surface> surface`
  - `std::unique_ptr<Curve2d> preimage`
  - 可选 `std::shared_ptr<Curve3d> spaceCurve`

职责：
- 表达 edge 在 face 参数域中的 2D 曲线
- 支撑 loop orientation、face area、surface trim、point-in-face

公开 API 不必立刻暴露整个对象，但内部必须先定义。

## 11. Bounds and Approximation Strategy

不要要求每个曲线/曲面都直接提供完美解析 bounds。

建议分三层：
- `Bounds()`
  - 返回可靠、可用于粗判的 AABB
- `TightBounds()`
  - 后续按需增加
- `Tessellate(...)`
  - 为 mesh / display / approximate section 服务

这样能避免一开始就把所有派生类卡死在 tight box 精度上。

## 12. Tolerance Strategy

这一层至少需要统一三类容差：
- `distanceEpsilon`
- `angleEpsilon`
- `parameterEpsilon`

关键规则：
- 参数域比较优先用 `parameterEpsilon`
- 点落曲线/曲面上优先看 3D distance
- 周期边界吸附时，需要同时考虑 parameter 和 distance

Delphi 与 GGP 都显示：闭合/周期对象如果没有边界吸附规则，投影和 trim 很容易不稳定。

## 13. Recommended Implementation Order

建议顺序：
1. `CurveEval3d` / `SurfaceEval3d` / projection result 类型
2. `Curve3d` / `Surface` 抽象基类
3. `PlaneSurface`
4. `LineCurve3d`
5. `ArcCurve3d`
6. `NurbsCurve3d`
7. `RuledSurface`
8. `NurbsSurface`
9. `CurveOnSurface` 内部层

## 14. Non-Goals For First Phase

第一阶段不追求：
- 所有解析曲面的完整枚举
- 高阶曲率分析 API 全量公开
- 完整编辑型 NURBS 内核
- curve/surface boolean
- 所有类型的 exact tight bounds

## 15. Key Design Decisions

这层最终要固定的结论是：
- `Curve3d` / `Surface` 是 3D 算法库的参数化协议核心
- `Plane` 与 `PlaneSurface` 必须分离
- 2D preimage 桥接必须提前设计
- projection / split / trim / bounds 是公共协议，不应散落到派生类私有 helper
- NURBS 是 fallback，不是整个 3D API 的唯一中心
