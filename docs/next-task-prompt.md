# 下一次任务提示词

你正在继续 `stablecore-geometry` 的 Delphi 老链路替代工作。不要重新做范围分析，直接按下面的状态和优先级继续写代码与文档。

## 工作约束

- 工作区：`D:\code\stablecore\stablecore-geometry`
- 只写代码、测试代码、文档
- 不要编译
- 不要跑构建
- 每完成一个阶段：
  - 更新 `docs/session-handoff.md`
  - 更新 `docs/next-task-prompt.md`
  - 同步相关覆盖/跟踪文档
  - 直接提交

## 当前状态（2026-04-04）

已完成 fast-track 第一批接口与测试骨架，提交：`00fb60d` `Add Delphi fast-track SDK interfaces and test matrix`

本批新增：

- `docs/delphi-interface-fasttrack.md`
  - Delphi 实际能力到 C++ SDK 目标接口总表
- `docs/delphi-test-fasttrack-matrix.md`
  - 接口到 contract / capability / gap 测试矩阵
- `include/sdk/GeometrySearchPoly.h`
- `src/sdk/GeometrySearchPoly.cpp`
- `include/sdk/GeometryBodyBoolean.h`
- `src/sdk/GeometryBodyBoolean.cpp`
- `tests/capabilities/test_searchpoly_sdk.cpp`
- `tests/capabilities/test_3d_body_boolean_sdk.cpp`
- `tests/gaps/test_3d_body_boolean_gaps.cpp`

当前策略已经固定为：

1. 接口先行
2. 测试先行
3. 产品只依赖稳定 SDK 名称
4. 算法深度工作放到接口后面继续补

## 下一轮优先级

### P1：继续深化 `GeometryBodyBoolean`

优先做：

- 在现有 identical/disjoint closed-body 子集基础上继续补更有业务价值的 overlap 子集
- 保持 `InvalidInput` / `UnsupportedOperation` contract 稳定
- 未覆盖语义继续留在 `tests/gaps/test_3d_body_boolean_gaps.cpp`

目标：

- 产品侧可以先按 `IntersectBodies / UnionBodies / DifferenceBodies` 开发
- 算法侧逐批把 capability tests 转绿

### P2：继续深化 `GeometrySearchPoly`

优先做：

- 在现有 diagnostics + ranking 的基础上继续补 branch scoring
- 增加更明确的 fake-edge explanation
- 继续把 Delphi 风格 smart-search 差距显式留在 `tests/gaps/test_searchpoly_gaps.cpp`

目标：

- `GeometrySearchPoly.h` 稳定对外，内部实现继续加深但不打断产品侧接入

### P3：持续完成“稳定 API”所需重构

优先推进：

- `GeometrySearchPoly`
  - 把当前对 `BuildMultiPolygonByLines(...)` 的薄封装，重构为独立实现层
  - 外部只保留 `GeometrySearchPoly.h`
- `GeometryBodyBoolean`
  - 未来实现时区分 `PolyhedronBody` 路径与 `BrepBody` 路径，但保持统一 public contract
- `GeometryBrepConversion`
  - non-planar repair 内部流程已拆成更清晰的 pass：
    - support-plane scoring
    - representative target aggregation
    - cross-face snapping
    - topology reconciliation
  - 下一步继续把这四个 pass 的数据流用于更一般的多面 closed-shell non-planar repair，而不只停留在 closed-cuboid representative 子集
- `GeometryHealing`
  - 将 conservative trim-backfill 与 aggressive shell policy 进一步拆层
- `GeometrySection`
  - 将 contour extraction、deterministic segment normalization、coplanar fragment merge 拆成更稳定的内部阶段
- 统一 SDK 风格：
  - `Options / Result / Issue` 命名统一
  - 稳定 API 放在 `include/sdk`
  - 内部 helper 尽量不要再以产品可直接依赖的方式扩散

详细清单见：`docs/rename-followup-todo.md`

## 本轮完成后的文档同步要求

至少同步这些文件：

- `docs/session-handoff.md`
- `docs/next-task-prompt.md`
- `docs/test-capability-coverage.md`
- `docs/design-doc-sync-tracker.md`
- `docs/rename-followup-todo.md`

如果接口矩阵或测试矩阵状态发生变化，也同步：

- `docs/delphi-interface-fasttrack.md`
- `docs/delphi-test-fasttrack-matrix.md`

## 交付口径

完成后请输出：

1. 这轮新增了哪些稳定接口/能力
2. 哪些 gap 仍保留
3. 哪些“稳定 API”重构事项已推进
4. 提交号
