# 文档同步跟踪

本文只保留当前同步状态。

## 已对齐内容

- 源码公开入口：`Include/Geometry.h`
- 安装后的公开头文件根目录：`Include/SCGeometry`
- 公共命名空间目标：`Geometry`
- 工程身份：`SCGeometry`
- 目录分层：`Core`、`Geometry2d`、`Geometry3d`、`Brep`、`Support`、`Types`、`Serialize`、`Export`
- 命名规则：见 `docs/final-naming-plan.md`
- 测试分层：见 `docs/test-capability-coverage.md`

## 当前同步要求

- 文档必须只描述当前发布 API
- 不再保留历史专题叙述
- 不再保留历史过程记录
- 能力差距可以保留，但必须是当前仍有效的差距

## 当前待同步项

- `Section` 的当前 gap 说明
- `Healing` 的当前 gap 说明
- `SearchPoly` 的当前 gap 说明
- `BodyBoolean` 的当前 gap 说明

## CI 同步范围

这部分内容和仓库里的 GitHub Actions、自动修复脚本保持一致，作为当前交接事实保留。

- `ci-windows-cmake` 负责 Windows + CMake preset 的 configure/build/test
- 触发方式包括 push 到 `main`、pull request 和手动触发
- `ci-autofix-on-failure` 只在 `ci-windows-cmake` 失败时触发
- 自动修复只执行确定性、可验证、低风险的修复规则
- 自动修复入口是 `scripts/ci-autofix.ps1`

## 结论

- 代码、测试和文档现在都应围绕当前发布版结构书写
- 如果新能力已经稳定，就应尽快从 gap 侧迁入 capability 侧
- 
## Current sync additions

- Added public surface for `TangentPoints`, `SCLine2d`, `IntersectExtended`, `ProjectPointToLineSegment` for 3D, `SCBoxTree3d`, `SCSegmentSearch3d`, and `SnapPointToSegments3d`.
- Added tests:
  - `UnitTests/Capabilities/Core/TestMissingAlgorithms.cpp`
  - `UnitTests/Gaps/Core/TestMissingAlgorithmsGaps.cpp`
  - `UnitTests/Gaps/Geometry3d/TestMissingAlgorithmsGaps.cpp`
- Remaining sync themes:
  - epsilon-sensitive tangent and intersection boundary wording
  - polyline de-duplication and ordering stability
  - 3D projection / snapping tie-break contracts
