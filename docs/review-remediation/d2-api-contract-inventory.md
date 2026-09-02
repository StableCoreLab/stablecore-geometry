# D2 API 契约盘点与弃用清单

审查日期：2026-09-01。本文记录当前行为，不将迁移目标视为已实施 API。

## 失败模型现状

| API | 当前失败表示 | 现状测试 | 迁移目标 |
|---|---|---|---|
| `SCMatrix2d::Inverse` | 零矩阵 | `FailureSentinelContractTest.MatrixInverseUsesZeroMatrixForSingularInputs` | 新增 `TryInverse` 或 optional 接口后弃用哨兵返回 |
| `SCMatrix3d::Inverse` | 零矩阵 | 同上 | 同 2D，接口和失败语义保持对称 |
| `SCTransform2d::Inverse` | `std::nullopt` | `FailureSentinelContractTest.TransformInverseDiffersBetweenTwoAndThreeDimensions` | 作为 transform 新接口的基准模型 |
| `SCTransform3d::Inverse` | 默认 identity transform | 同上 | 新增 optional/issue 接口，旧接口弃用 |
| `TryNormalize` | `false`，且不修改 caller output | `FailureSentinelContractTest.TryNormalizeFailureLeavesCallerOutputUnchanged` | 保持 bool + out 参数，补齐 3D 对应契约或引入返回结果类型 |
| 投影/求交结果 | `success`/`issue` 字段或空 optional，因 API 而异 | 既有 Core/3D capability 测试 | D2/D5 分别收敛到带 issue 的结果结构 |

## 弃用清单（尚未生效）

| 旧 API | 替代 API | 兼容期 | 删除条件 |
|---|---|---|---|
| `SCMatrix2d::Inverse`、`SCMatrix3d::Inverse` 的哨兵失败语义 | 待定义 `TryInverse`/`InverseResult` | 至少一个发布周期 | 所有库内和包消费者迁移完成 |
| `SCTransform3d::Inverse` 的 identity 失败哨兵 | 待定义 optional/issue 接口 | 至少一个发布周期 | 3D 调用方完成差分迁移 |
| 分散的投影失败约定 | 按 D5 的统一 projection result | 依功能包确定 | 基准与退化测试通过 |

## 实施约束

1. 先新增接口与行为测试，不修改现有函数的返回类型。
2. 新旧接口在兼容期内进行差分测试。
3. 只有在所有公共消费者完成迁移后，才添加 deprecation 属性并计划删除旧接口。
