# 库设计说明
## 1. 定位

StableCore Geometry 是一套面向工程计算的 C++ 几何库，目标是提供稳定、可测试、可发布的几何能力，而不是演示型样例库。

## 2. 源码结构

仓库源码当前采用以下主结构：

```text
Include/
Source/
UnitTests/
docs/
```

这里的 `Include/` 指的是仓库源码中的公开头文件目录，不是安装后的目录。

公开头按语义分层：

- `Core`
- `Geometry2d`
- `Geometry3d`
- `Brep`
- `Support`
- `Types`
- `Serialize`
- `Export`

## 3. 安装结构

安装后，公开头文件会落在：

```text
Include/SCGeometry/
```

CMake 包文件会落在：

```text
Lib/cmake/SCGeometry/
```

## 4. 公共入口

统一入口为：

```cpp
#include "Geometry.h"
```

如果只需要某个模块能力，也可以直接包含对应模块头文件。

## 5. 分层原则

### 5.1 类型层

类型层负责表达几何数据和少量天然属性，例如：

- `SCPoint2d`
- `SCVector2d`
- `SCBox2d`
- `SCPoint3d`
- `SCVector3d`
- `SCBox3d`
- `SCLine3d`
- `SCPlane`
- `SCTriangle3d`

### 5.2 服务层

服务层负责复杂算法、重建、修复和诊断，例如：

- `Section`
- `Healing`
- `BodyBoolean`
- `SearchPoly`
- `BrepConversion`

### 5.3 容差层

容差和数值辅助集中放置在：

- `Support/Epsilon.h`
- `Support/Geometry2d/*`
- `Core/GeometryTypes.h` 中的相关导出类型

## 6. 模块职责

### 6.1 Core

负责 umbrella 类型、跨模块操作和最常用的公共入口。

### 6.2 Geometry2d

负责二维基本图元、路径工具和二维几何服务。

### 6.3 Geometry3d

负责曲线、曲面、三维求值和相关几何服务。

### 6.4 Brep

负责 BRep、polyhedron、mesh、healing、拓扑和布尔服务。

### 6.5 Support / Types

负责基础辅助类型、数值工具和实现细节类型。

### 6.6 SCBox2

`SCBox2<T>` 作为 2D 包围盒，职责是表达最小轴对齐边界范围，适合保留为成员能力的内容包括：

- `MinPoint()`
- `MaxPoint()`
- `Bounds()`
- `IsValid()`
- `Expand(...)`

`SCBox2<T>` 的边界约束是保持轻量和值类型语义，不承担以下职责：

- 复杂空间索引逻辑
- 多对象布尔流程
- 高层业务语义

当前固定结论是：

- box 保持轻量值类型
- API 命名使用短名风格
- 只提供边界对象天然成立的能力

## 7. 发布约束

- 公共 API 只保留当前发布面需要的命名
- 默认安装面只保留当前发布面，不再把实现细节暴露到默认头集合中
- 需要跨 DLL 的公共数据结构继续通过 `GEOMETRY_API` 导出

## 8. 当前结论

- `Geometry.h` 是唯一推荐的 umbrella 入口
- `Core` 是最常用的发布入口
- `Geometry2d` 和 `Geometry3d` 负责几何能力本身
- `Brep` 负责更高阶实体服务
- `Types` 和 `Support` 负责底层基础设施
