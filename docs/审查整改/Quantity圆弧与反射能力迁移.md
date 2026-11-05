# Quantity 圆弧与反射能力迁移

## 范围与实现决策

依据 `Quantity/PendingAlgorithms/SCGeometry/Docs/SCGeometry缺失能力需求单.md` 补齐三点定弧、两点半径方向及大/小弧定弧、二维反射工厂。本次只修改 Geometry，Quantity 的调用点切换和临时目录删除由其后续集成处理。

当前库已有 `SCArcSegment2d` 的方向、有效性、取点、长度与包围盒实现，以及 `Mirror(...)` 几何镜像重载。迁移直接复用这些能力，不引入 `PendingArc2d`、`PendingArcPointAt`、重复方向字段、私有圆周常量或独立临时测试工程。

新增圆弧构造服务位于 `Core/ArcConstruction2d.h`，实现放在对应 `.cpp` 并使用现有 DLL 导出机制。`SCArcConstructionResult2d` 只包含正式 `arc` 和额外状态 `radiusClamped`；两个 `Create*2d` 入口调用同一解算服务，不另写算法。新头已接入 `Geometry.h`、CMake 安装清单和逐头独立编译检查。

三点解算以起点为局部原点并共同缩放，避免绝对坐标平方导致的溢出与消减。`eps` 是距离容差：点间距不大于 `eps` 视为重合，相对于最长边的高不大于 `eps` 视为共线；不沿用临时实现中直接把面积与距离容差比较的方式。

两点解算根据方向和大/小弧直接选择圆心，无需枚举候选并评分。圆心高度用分解后的平方差计算，不形成半径平方；半圆归入小弧，大弧半圆请求失败。两个解算器均检查正式圆弧有效性，不把零扫角或数值上已不可区分的大弧转换成整圆。

反射并入 `SCTransform2d::Reflection`，返回 `std::optional<SCTransform2d>`，与现有 `Inverse()` 保持一致。已有 `Mirror(...)` 复用该工厂的线性变换，并继续在镜面局部坐标计算，避免远原点平移项抵消。无效方向的旧接口仍保持输入不变。

## Quantity 调用迁移对照

| 临时入口 | 正式入口 | 调用变化 |
|---|---|---|
| `Pending::CreateArcThroughThreePoints2d` | `Geometry::CreateArcThroughThreePoints2d` | 签名保持一致，包含 `Core/ArcConstruction2d.h` |
| `Pending::CreateArcFromStartEndRadius2d` | `Geometry::CreateArcFromStartEndRadius2d` | 签名保持一致，保留 `largeArc` |
| `Pending::SolveArcThroughThreePoints` | `Geometry::SolveArcThroughThreePoints` | 几何字段通过 `result->arc` 访问，方向调用 `arc.Direction()` |
| `Pending::SolveArcFromStartEndRadius` | `Geometry::SolveArcFromStartEndRadius` | 同上，`result->radiusClamped` 保留 |
| `Pending::PendingArcPointAt` | `result->arc.PointAt(parameter)` | 预览若需要夹紧参数，由调用方显式夹到 `[0,1]` |
| `Pending::CreateReflectionTransform2d` | `Geometry::SCTransform2d::Reflection` | 保留可判别失败语义，包含 `Types/Geometry2d/SCTransform2d.h` |
| `Pending::CreateReflectionTransform2dOrIdentity` | `SCTransform2d::Reflection(...).value_or(SCTransform2d::Identity())` | 显式指定回退策略 |

老版四种两点弧命令映射仍为：`atACS=(CounterClockwise,false)`、`atCS=(Clockwise,false)`、`atACL=(CounterClockwise,true)`、`atCL=(Clockwise,true)`。

三点、两点和反射接口均拒绝非有限输入；解算器的距离容差必须为有限正数。圆弧结果还须满足当前 `SCArcSegment2d::IsValid()` 契约，包括最小扫角与可表示长度。相较临时实现，无法形成有效正式圆弧的输入会返回失败，而非返回后续算法无法使用的结果。

提交前审查补充了成弧后的输入约束检查：两端点到输入端点的距离均不得超过 `eps`；三点弧还复用已有有限弧投影检查 `through` 到所选弧段的距离。距离以 `hypot` 度量，不以距离平方比较，不因坐标量级隐式扩大容差。圆心舍入使约束失真时返回空值，便捷入口保持原输出不变。例如 `(1e16,0)→(1e16+2,0)`、半径 `2` 的圆弧在默认容差下必须失败。大尺度输入若需要更宽容差，调用者须显式传入；对应大半径测试也显式指定其距离容差。

## 验证覆盖

`UnitTests/Capabilities/Core/TestArcConstruction2d.cpp` 覆盖顺逆三点弧、经过中间点的大弧、平移与缩放稳定性、两点弧四种选择、半径夹紧及半圆边界、非法输入及失败输出保持、反射的坐标轴/斜轴/平移轴、行列式、自逆性和已有 `Mirror` 端点一致性。原有镜像测试继续用于兼容回归。

用户已针对本次任务授权生成解决方案、编译并运行单元测试，覆盖仓库默认离线模式限制。2026-11-05 验证结果：

- 使用 `GenerateGeometryVS2022.bat` 在 `D:/code/StableCore/Build/GeometryVs2022` 生成 VS2022 x64 解决方案，Debug 完整构建通过，包含公开头独立编译检查。
- 使用 `ctest --test-dir D:/code/StableCore/Build/GeometryVs2022 -C Debug --output-on-failure --parallel 4` 执行全量检查，登记 1105 项，其中原有禁用用例 16 项、启用检查 1089 项。新增 7 项圆弧与反射测试及原有镜像回归均通过。
- 首轮 1087 项通过，两项失败：安装包消费检查缺少 `SCTransform2d` 的直接头文件引用；原有 `NurbsNumerics3dTest.FixedLargeCoordinateWorkloadPerformance` 在并行负载下中位耗时约 254 ms，超出 250 ms 门槛。
- 补齐消费者直接引用后，用 `ctest --test-dir D:/code/StableCore/Build/GeometryVs2022 -C Debug --rerun-failed --output-on-failure --parallel 1` 复测两项，均通过。安装消费检查实际调用并链接两个圆弧解算入口、两个便捷入口及反射工厂。未修改 NURBS 实现或其性能门槛。
- 全量执行与必要复测合计覆盖全部 1089 项启用检查；`git diff --check` 通过。

提交前审查问题修复后的验证：Debug 完整构建再次通过；使用 `ctest --test-dir D:/code/StableCore/Build/GeometryVs2022 -C Debug --output-on-failure --parallel 4 -R "ArcConstruction2dTest|Reflection2dTest|TransformSamplingTest|SCGeometry.PackageConsumer"` 运行相关回归，22 项全部通过。新增两项测试覆盖圆心舍入失真时拒绝并保持输出、大坐标下可准确构造时成功，以及默认/显式距离容差对最终端点残差的约束。原有缩放测试同时增加三点残差断言。本次修复后未重复执行无关模块的全量测试。
