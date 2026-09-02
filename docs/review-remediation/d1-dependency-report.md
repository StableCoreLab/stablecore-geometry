# D1 依赖与公开头报告

审查日期：2026-09-01。此报告记录实施前的实际依赖，作为后续分层迁移的基线。

## 当前模块边

| 从 | 到 | 结论 |
|---|---|---|
| `Types/Geometry2d`、`Types/Geometry3d` | `Support` 与同层类型 | 符合底层方向 |
| `Geometry2d`、`Geometry3d` | `Types`、`Support`、部分 `Core` 结果 | 基本符合 |
| `Brep` | 3D、Core 结果和 2D 辅助 | 符合上层依赖 |
| `Core/Measure.h` | `Brep/*`、`Geometry3d/*` | 反向依赖，D1 迁移目标 |
| `Core/GeometryApi.h` | Brep、2D、3D、Core 全量模块 | 仅作为 umbrella 聚合器保留，不得成为基础模块依赖 |

## 公开头编译目标

`SCGeometryPublicHeadersCompile` 编译 `UnitTests/Support/PublicHeadersCompile.cpp`，该源文件逐一包含当前 95 个非 detail 公开头。目标链接 `SCGeometry` 以复用消费者的 include 路径、C++20 特性和导出定义。

该目标用于发现跨模块头的缺失直接 include 和包含顺序问题；后续 D1 实施阶段应继续升级为每个头独立翻译单元的检查。

## 兼容头迁移结论

扫描未发现 `Geometry::Sdk`、Legacy、Compat 或 deprecated 兼容头的实际使用。因此当前没有可安全执行的“旧头转发”机械迁移；新增转发头必须在具体公开路径迁移时一并提出，并记录弃用版本。

## 下一步

1. 将 `Core/Measure.h` 中依赖 Brep 的能力迁往上层模块或改为仅依赖前置声明/结果类型。
2. 将 `Core/GeometryApi.h` 明确为 umbrella，不作为模块内部 include。
3. 为每个候选路径变更提交旧路径、新路径、兼容期和消费者编译验证表。
