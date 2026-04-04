# 云端执行与关机接力

## 目标

你本地关机后，构建和测试由云端持续执行。

## 已新增

- GitHub Actions 工作流：Windows + CMake preset 自动执行 configure/build/test
- 触发条件：push 到 main、pull request、手动触发
- GitHub Actions 自动修复工作流：当 `ci-windows-cmake` 失败时触发 `ci-autofix-on-failure`

## 自动修复能力边界

- 会自动重跑 configure/build/test 并保存复现场景日志
- 会执行确定性修复钩子：`scripts/ci-autofix.ps1`
- 若产生代码改动：会先二次执行 configure/build/test 验证；验证通过才自动创建修复 PR（不自动合并）
- 若未产生改动：自动创建 issue，提示人工修复并附日志定位入口
- 若产生改动但二次验证失败：自动创建 issue，提示人工修复并附日志定位入口
- 对于同类失败会做 issue 去重：若已有同标题 open issue，则只追加评论记录新 run，不重复开新 issue

注意：当前自动修复是“可控规则型”而非通用 AI 改码器，默认只做安全、可预测的修复步骤。

当前已内置的确定性规则包括：
- workflow/doc 的行尾归一化（LF）
- 针对 3D capability 测试常见编译故障的 using 别名补齐（`test_3d_brep.cpp` / `test_3d_conversion.cpp`）

## 使用方式

1. 推送当前提交到远端。
2. 打开仓库 Actions 页面查看流水线 `ci-windows-cmake`。
3. 等待 `build-and-test` 完成，查看失败用例与日志。

## 对应命令（云端执行内容）

- `cmake --preset vs2022-x64`
- `cmake --build --preset vs2022-x64-build --config RelWithDebInfo`
- `ctest --preset vs2022-x64-test -C RelWithDebInfo --output-on-failure`

## 失败时定位建议

1. 先看 Configure 步骤是否成功（CMake 生成阶段错误）。
2. 再看 Build 步骤（编译错误，含文件与行号）。
3. 最后看 Test 步骤（断言失败与具体测试名）。

## 下轮接力优先项

1. 必需-3：`GeneralNonPlanarPolyhedronToBrepRepairRemainsOpen` 继续收敛 capability 子集。
2. 钢筋线后处理：补齐去重、共线合并、根数统计稳定断言。
3. 视 CI 失败特征扩展 `scripts/ci-autofix.ps1`（每次只加可验证、低风险规则）。
