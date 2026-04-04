# 重命名/稳定 API 后续待办

此文件保留给后续必须持续收敛、但不要求在单轮内全部做完的重构与接口迁移事项。

目标不是追求“大重写”，而是逐步把对外 API 稳定下来，让产品侧只依赖 `include/sdk`，内部算法可继续迭代。

## P1：公共 API 面收口

- `GeometrySearchPoly`
  - 外部正式入口固定为 `include/sdk/GeometrySearchPoly.h`
  - 产品侧后续不应再直接依赖 `BuildMultiPolygonByLines(...)`
  - 需要把 candidate ranking、synthetic-edge diagnostics、ambiguous-result 语义逐步并入正式 result 结构
- `GeometryBodyBoolean`
  - 外部正式入口固定为 `include/sdk/GeometryBodyBoolean.h`
  - 后续即使内部实现拆为 `PolyhedronBody` 路径和 `BrepBody` 路径，public contract 也保持统一
- `Geometry.h`
  - 继续作为稳定 umbrella header，并只聚合稳定 `include/sdk` 入口
  - 新增 Delphi-facing SDK 时，优先从这里统一暴露，而不是让产品侧直接拼接内部头文件

## P2：需要继续拆层的内部算法

- `GeometryBrepConversion`
  - 需要把当前 non-planar repair 拆成清晰 pass：
    - support-plane scoring
    - representative target aggregation
    - cross-face snapping
    - topology reconciliation
  - 目标是让后续 capability 扩张时，不必继续把复杂启发式堆在单个大函数里
- `GeometryHealing`
  - 需要进一步拆分 conservative trim-backfill 与 aggressive shell policy
  - 目标是让“保守修复”和“激进修复”各自拥有清晰入口与测试边界
- `GeometrySection`
  - 需要把 contour extraction、deterministic segment normalization、coplanar fragment merge 分阶段组织
  - 目标是让 section 能力继续扩张时，contract 更稳定、内部更可替换

## P3：接口风格统一

- 统一 Delphi-facing SDK 的 `Options / Result / Issue` 设计风格
- 统一 Delphi-facing SDK 的 `Options / Result / Issue` 设计风格，并尽量让 `Geometry.h` 作为唯一产品侧 umbrella 入口
- 尽量避免把临时 helper 直接暴露给产品侧
- 新增 capability 时，优先补公共 SDK 入口，再补内部深实现
- 对外承诺的接口一旦进入 `include/sdk`，后续尽量只增不破

## P4：文档与跟踪

- 每推进一批接口或重构，都要同步：
  - `docs/session-handoff.md`
  - `docs/next-task-prompt.md`
  - `docs/test-capability-coverage.md`
  - `docs/design-doc-sync-tracker.md`
- 如果接口矩阵或测试矩阵状态变化，还要同步：
  - `docs/delphi-interface-fasttrack.md`
  - `docs/delphi-test-fasttrack-matrix.md`
- AI 分派时参考：
  - `docs/ai-task-routing.md`

## 当前建议的下一批动作

- 深化 `GeometrySearchPoly`
- 推进 `GeometryBodyBoolean` 第一批 deterministic capability
- 只做有助于稳定 API 的重构，避免在产品即将接入时做大范围接口扰动
