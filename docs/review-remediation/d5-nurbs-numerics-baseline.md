# D5：NURBS、投影与求交数值基线

## 当前契约（2026-09-01）

- `SCNurbsCurve3d` 与 `SCNurbsSurface` 是非有理 B-spline：控制点没有权重，名称中的 NURBS 不代表已经支持 rational NURBS。
- 参数在 `PointAt` 中钳制到有效参数域；开区间右端点由末控制点的基函数承接。
- `SCNurbsCurve3d::Evaluate` 支持 0--2 阶请求，`SCNurbsSurface::Evaluate` 支持 0--1 阶请求；当前一、二阶导数均为有限差分近似。
- `ProjectPointToCurve`、`ProjectPointToSurface` 和 `Intersect(line, surface)` 对无效几何返回默认失败结果，不返回部分命中。
- 平面 NURBS 的投影和线求交在 `1e-4` 参数误差内一致；求交走平面识别/仿射投影加速，点投影仍包含数值细化，因此不承诺位级一致。

## 已固化的基准与退化样例

`UnitTests/Capabilities/Geometry3d/TestNurbsNumerics.cpp` 覆盖：

1. 一次曲线的端点、参数钳制、一阶常量导数、零二阶导数，以及点投影和垂线求交；这间接校验开区间 B-spline 基函数和右端点规则。
2. 双线性曲面的张量积插值、`u/v` 偏导及法向方向。
3. 平面 NURBS 上点投影与垂线求交的点、参数和距离平方数值基线。
4. 缺少节点或节点非单调的无效曲线/曲面：投影和求交必须失败，不能输出部分结果。

## 后续实现约束

解析 basis/导数、共享曲线与曲面 basis helper、以及 Newton/信赖域投影求交可在这些测试之上演进，但须保留上述边界与失败语义。rational 权重、周期曲线及高阶曲面导数仍是单独设计决定，不能在此基线下隐式宣称已支持。
