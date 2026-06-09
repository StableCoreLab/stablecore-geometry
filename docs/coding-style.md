# StableCore Geometry 代码规范

## 1. 命名空间与目录

### 1.1 目录结构

对外头文件目录保持清晰。

```text
Include/
```

这里描述的是源码树中的目录风格；安装后的公开头文件根目录使用 `Include/SCGeometry/`。

### 1.2 命名空间

内部命名空间保持简洁：

```cpp
namespace Geometry
{
}
```

原则：

- 对外源码路径使用 `Include/`
- 对外安装路径使用 `Include/SCGeometry`
- 对内命名空间使用 `Geometry::`

## 2. 类型命名

### 2.1 使用 PascalCase

```cpp
struct Point2d;
struct Vector2d;
struct ISCSegment2d;
struct Box2d;
struct ProjectionResult;
```

### 2.2 维度后缀统一

统一使用：

```text
2d / 3d
```

例如：

```cpp
Point2d   // 正确
Point2D   // 不建议
Point2    // 不建议
```

## 3. 函数命名

### 3.1 使用 PascalCase

```cpp
Distance
DistanceSquared
Dot
Cross
ProjectPointToSegment
IntersectSegments
```

### 3.2 布尔函数必须表达语义

```cpp
IsZero
IsEqual
IsParallel
IsPointOnSegment
```

### 3.3 禁止无意义前缀

不建议：

```cpp
CalcDistance
DoIntersect
HandleSegment
```

## 4. 变量命名

### 4.1 普通变量使用 lowerCamelCase

```cpp
startPoint
endPoint
distanceSquared
intersectionPoint
```

### 4.2 成员变量统一使用 `m_` 前缀

```cpp
class ISCSegment2d
{
public:
    Point2d m_start;
    Point2d m_end;
};
```

### 4.3 布尔变量必须可读

```cpp
bool isValid;
bool isParallel;
bool hasIntersection;
```

## 5. 常量规范

### 5.1 使用 `k` 前缀

```cpp
constexpr double kDefaultEpsilon = 1e-9;
constexpr double kPi = 3.14159265358979323846;
```

### 5.2 禁止裸常量

不建议：

```cpp
fabs(a - b) < 1e-9
```

应改为：

```cpp
IsZero(a - b)
```

## 6. 核心数值工具统一

所有几何判定都应收敛到统一工具接口，例如：

```cpp
namespace Geometry
{
constexpr double kDefaultEpsilon = 1e-9;

bool IsZero(double value, double eps = kDefaultEpsilon);
bool IsEqual(double lhs, double rhs, double eps = kDefaultEpsilon);
bool IsLess(double lhs, double rhs, double eps = kDefaultEpsilon);
bool IsGreater(double lhs, double rhs, double eps = kDefaultEpsilon);
}
```

这组接口是几何库稳定性的核心，不应在各模块中各写一套近似比较。

## 7. 结果类型规范

### 7.1 复杂算法不要只返回 `bool`

不建议：

```cpp
bool IntersectSegments(...);
```

### 7.2 应返回结构化结果

```cpp
struct SegmentIntersectionResult
{
    bool m_intersect{false};
    bool m_parallel{false};
    bool m_collinear{false};
    Point2d m_point{};
};
```

调用方通常还关心：

- 是否平行
- 是否共线
- 是否唯一交点
- 交点坐标
- 参数位置

## 8. 文件命名规范

### 8.1 使用 PascalCase

```text
Point2d.h
Vector2d.h
ISCSegment2d.h
Distance2d.h
Distance2d.cpp
Intersect2d.h
```

### 8.2 一文件一职责

每个文件只承载一个清晰主题，不要把无关工具堆在一起。

## 9. 接口设计规则

### 9.1 公开 API 保持短名风格

优先：

- `Count()`
- `Bounds()`
- `PointAt()`
- `Length()`

避免重新扩散：

- `GetCount()`
- `GetBounds()`
- `GetPointAt()`

### 9.2 成员函数与自由函数边界要清楚

成员函数适合：

- 直接读取对象自身状态
- 不依赖外部上下文的天然属性

自由函数适合：

- 涉及多个对象
- 需要容差或上下文
- 算法复杂度较高

## 10. 测试与文档一致性

新增或修改 API 时，应同步更新：

- 对应测试
- 相关设计文档
- `docs/session-handoff.md` 中的当前状态说明

代码、测试、文档三者应保持同一口径。
