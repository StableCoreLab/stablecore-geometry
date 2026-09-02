# Geometry 命名计划（UTF-8）

本文记录当前公开 API 的命名事实和后续命名决策，不把未执行的重命名描述成现状。

## 当前已执行规则

- 公开命名空间为 `Geometry`。
- 公开类型保留 `SC` 前缀，例如 `SCPoint2d`、`SCVector3d`、`SCPlane`、`SCBrepBody`。
- 接口类型保留 `ISC` 前缀，例如 `ISCCurve3d`、`ISCSurface`、`ISCSegment2d`。
- 公共函数使用无前缀 PascalCase，例如 `Intersect`、`Validate`、`ConvertToBrepBody`。
- `Geometry.h` 是统一 umbrella；模块头可按需直接包含。
- `Geometry::Sdk` 不作为新的公开命名入口，不新增兼容别名。

## 目录与命名对应

| 目录 | 当前职责 | 命名状态 |
|---|---|---|
| `Include/Types/Geometry2d` | 2D 值类型 | 已执行，保留 `SC*` |
| `Include/Types/Geometry3d` | 3D 值类型 | 已执行，保留 `SC*` |
| `Include/Geometry2d` | 2D 曲线、路径和索引 API | 已执行 |
| `Include/Geometry3d` | 3D 曲线、曲面和索引 API | 已执行 |
| `Include/Brep` | Polyhedron、B-rep、网格和拓扑 API | 已执行 |
| `Include/Core` | 结果类型与跨模块算法 API | D1 中继续收敛边界 |

## 兼容与迁移策略

- 不在未评审的 D1/D2 设计完成前批量移除 `SC`/`ISC` 前缀。
- 不新增 `Point2d`、`Vector3d` 等无前缀别名；文档和示例统一使用真实 `SC*` 名称。
- 任何未来重命名必须先提供新旧 API 对照表、弃用周期和编译迁移示例。
- 仅为兼容已有消费者保留必要的转发头；转发头不得扩大默认安装 API。

## 待决事项

- D1：确定 `Types`、`Core`、2D、3D、Brep 的最终依赖方向和结果类型归属。
- D2：确定是否引入统一的失败/容差契约，以及相应的新旧 API 过渡方式。
- D4/D5：分别确定 B-rep 主表示和 NURBS/BSpline 命名，当前 `SCNurbs*` 名称暂不改变。

## ABI 规则

- 跨 DLL 的结果结构和可变数据按现有导出策略使用导出宏。
- header-only 值类型不因命名或可见性额外添加导出宏。
- 新增公开类型必须同时更新 `Geometry.h`、对应模块文档、能力矩阵和安装验证。
