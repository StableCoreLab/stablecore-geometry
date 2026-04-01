# 设计文档同步跟踪

## 目的

这份文档用于跟踪历史 2D 设计文档与当前代码 / API 状态之间的同步修复工作。

## 状态说明

- `pending`：尚未开始
- `in_progress`：正在处理中
- `done`：当前轮同步已完成

## 文件列表

### `docs/segment-design.md`

- status: `done`
- notes:
  - 已按当前 `Segment2 / LineSegment2 / ArcSegment2` API 与设计结论重写
  - 已同步成员方法与自由函数边界

### `docs/polyline-design.md`

- status: `done`
- notes:
  - 已按当前 `Polyline2 / sdk::Polyline2d / MultiPolyline2d` API 与设计结论重写
  - 已同步 `SegmentCount / VertexCount / VertexAt / StartPoint / EndPoint / Bounds / PointAt / PointAtLength`

### `docs/polygon-design.md`

- status: `done`
- notes:
  - 已按当前 `Polygon2 / sdk::Polygon2d / MultiPolygon2d` API 与设计结论重写
  - 已同步 `OuterRing / HoleCount / HoleAt / Bounds`

### `docs/box-design.md`

- status: `done`
- notes:
  - 已按当前 `Box2` API 与设计结论重写
  - 已同步 `MinPoint / MaxPoint / Bounds` 相关命名

### `docs/sdk-type-design-review.md`

- status: `done`

### `docs/sdk-2d-api-convergence.md`

- status: `done`

### `docs/session-handoff.md`

- status: `done`

### `docs/api-member-free-function-checklist.md`

- status: `done`

### `docs/delphi-geometry-parity.md`

- status: `done`
- notes:
  - 已同步共享预处理入口 `NormalizePolygonByLines` 在 boolean / offset / topology 的接入状态
  - 已同步布尔近退化交点参数聚类压缩、共线族折叠与带孔 `CutPolygon` 裁剪策略
  - 已同步偏移语义翻转恢复（单多边形与多多边形）与孔洞感知 `Contains` 修复

## 进度备注

- 2026-03-27：
  - 已完成 `segment / polyline / polygon / box` 四份 2D 设计文档重写
  - 已将成员方法 vs 自由函数边界规则固定到独立清单与 API 收敛文档
- 2026-04-01：
  - 已完成 `delphi-geometry-parity` 与本轮代码实现对齐，覆盖共享预处理、布尔鲁棒性、偏移恢复与拓扑包含修复
  - 已补充对应能力回归测试覆盖项说明（relation / offset / pathops / topology）
  - 已将 below-tolerance arrangement degeneracy 从 `tests/gaps` 提升到 `tests/capabilities`，并通过定向 gtest 验证
  - 已补充 `NormalizePolygonByLines(...)` 的多级容差重建与边界简化回退，提升近退化 polygon operand 归一化稳定性
  - 已修复本机构建缓存中的过期 MSVC 路径，恢复 capability / gap 测试目标重生成与重建
  - 已将单多边形带孔 offset 语义恢复从 `tests/gaps` 提升到 `tests/capabilities`，并通过定向 gtest 验证
  - 已将一组 branch-scoring / fake-edge 误选场景从 `tests/gaps` 提升到 `tests/capabilities`，验证当前 SearchPoly 候选排序可稳定保留主轮廓
  - 已修正 `Contains(...)` 对 boundary T-junction 的过度保守短路，令 shared-boundary contained 场景可稳定落到 `FirstContainsSecond`
  - 已将一组 reverse-edge / self-intersection offset 恢复场景从 `tests/gaps` 提升到 `tests/capabilities`，并通过定向 gtest 验证
  - 已新增 `OffsetToMultiPolygon(const Polygon2d&, ...)`，补齐单 polygon 内缩发生分裂时的 multipolygon 输出能力
  - 已将 narrow-bridge split 从 `tests/gaps` 提升到 `tests/capabilities`，并通过定向 gtest 验证
  - 已完成 3D gap 骨架落地：新增 section / brep / healing / conversion 四组 3D gap tests，并接入 `stablecore_geometry_gap_gtest`
  - 当前 `gap` 目标已从“空套件”切换为“3D P1 骨架套件”（2D gaps 已清空，3D gaps 已建档）

## 对齐完成定义（DoD）

### 2D 对齐完成

- 所有 2D gap 用例不再 `GTEST_SKIP()`，并转入 `tests/capabilities`
- boolean / offset / pathops / topology 共享统一预处理与容差语义，不出现模块间结果漂移
- relation 层级可稳定区分 touching / intersecting / contains / hole-diff，包含同外环不同孔场景

### 3D 对齐完成

- 从第一阶段最小 workflow 升级为 section -> rebuild -> validation -> healing -> conversion 稳定闭环
- `BrepBody` 从 skeleton 发展为可用拓扑一致性与修复链路，覆盖非平面主导场景
- 为 3D 尚未补齐能力建立 `tests/gaps` 体系，避免仅文档跟踪

## 燃尽清单（优先级）

### P0

- 2D boolean 深退化恢复：已关闭 arrangement 低容差退化缺口并转正测试
- 2D offset 语义与窄通道稳定性：已关闭 reverse-edge / self-intersection / hole semantics / narrow bridge split，并完成测试转正

### P1

- 2D SearchPoly 分支评分与 fake-edge candidate ranking：当前已补齐一组代表性误选场景，仍需继续向 Delphi 级评分策略逼近
- 2D relation hierarchy 向 Delphi `GGLPolyRelation` 聚合语义补齐：当前已补齐一组 shared-boundary contained 场景，但公开枚举仍未扩展到更细层级
- 3D gap 测试体系落地（section / brep / healing / conversion）：已完成骨架搭建并接入测试目标
- 3D 第一阶段向第二阶段推进：先 validation/healing 闭环，再扩 section graph 语义

## 两周执行顺序建议

- 第 1 周：聚焦 2D P0（boolean + offset），清空对应 skip
- 第 2 周：补 SearchPoly + relation，并建立 3D gap 测试骨架
