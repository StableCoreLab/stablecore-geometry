# T4 Healing 四 shell 场景参数化验收

核对日期：2026-11-05。仅整理两项结果检查相同的 Healing 测试，不修改生产源码或修复策略。

## 输入和共同检查

新文件 UnitTests/Capabilities/Geometry3d/TestHealingFourShellArbitration3d.cpp 用两个具名参数选择各自的局部输入构造函数：

| 参数 | 输入构造 | 顶点数 | 边数 | 竞争对关系 |
|---|---|---:|---:|---|
| PartialOverlap | BuildPartialOverlapBody | 23 | 28 | 两个 shell 的平面区域部分重叠 |
| SharedEdgeCompetition | BuildSharedEdgeCompetitionBody | 21 | 28 | 两个 shell 共享边界顶点对并竞争封口 |

两套原始坐标、索引、边、面和 shell 构造分别保留，没有把部分重叠改造成共享边，也没有借用同一个可写 body。函数指针只选择构造入口，输入在各测试体执行时独立创建。构造函数位于新文件的匿名命名空间；不向公共夹具头或库接口增加依赖。

共同测试保留原有 17 条 ASSERT 及执行语句：输入有效且为四 shell；以默认 SCGeometryTolerance3d 和 Aggressive 策略调用 Heal；输出成功且有效；shell 闭合状态依次为 true／false／false／true，面数依次为 3／2／2／3，总面数为 10；两个新封口面的外部 trim 有效。没有添加循环替换逐项断言，失败仍定位到具体 shell 和面。

另三项原测试保持独立：部分重叠对旁的独立壳、已有闭壳，以及共享边竞争对旁的已有闭壳／顶点接触壳。它们的 shell 数、原闭合状态、面数或总面数与本参数组不同，不能仅因构造语句相似直接套用同一检查。

## 过滤名称迁移

| 旧名称 | 当前名称 |
|---|---|
| Healing3dCapabilityTest.AggressiveHealingClosesIndependentAndVertexTouchShellsWhileSkippingPartialOverlapPair | Healing3d/FourShellArbitration3dTest.ClosesIndependentAndVertexTouchShellsWhileKeepingPairOpen/PartialOverlap |
| Healing3dCapabilityTest.AggressiveHealingClosesIndependentAndVertexTouchShellsWhileSkippingCompetingPair | Healing3d/FourShellArbitration3dTest.ClosesIndependentAndVertexTouchShellsWhileKeepingPairOpen/SharedEdgeCompetition |

CTest 与 GTest 使用相同的具名后缀；PrintTo 输出参数名称，不依赖函数指针地址。原文件仍保留其他三项测试，两个不再需要的 Fixtures3d.h 包含已移除。CMake 只登记新文件一次。发现顺序调整，总登记数量保持 1092；旧名称过滤器需按表迁移。

## 正文与发现集合核对

对工厂构造去掉新增的命名空间缩进和 return body 包装，再将共同检查接回原测试，可逐字还原原正文。UTF-8／LF 归一化 SHA256：

- 部分重叠原正文：b27bc6feb8d487da117cc59aedfdeec8d9ef07c43fba2404bd5c53c03912eff8。
- 共享边竞争原正文：d738dbe2917959a56b333d0473c3eb602c3ba8ed18783a52a1208e09330421b3。
- 共同 17 条检查及修复执行后缀：af1e32d6a6cdb3f73da2b02b87fac75c5bf54167c4c9aaaf2863fa1d348a429d。

原两文件中其他三项测试正文分别比对，均未改变。参数名与工厂映射、各自顶点／边数、匿名链接属性、默认容差及策略也已逐项核对。

将迁移前两项名称按上表替换后，与迁移后的完整发现集合排序比较，1092 项名称的 SHA256 一致：4813680ad0e04eeddec6188a6ad14f89bd07ebbb5756fb7ec263e223dc1f1852。该指纹只证明本次名称集合保持，不表示发现顺序保持或一般 Healing 能力完成。

两项 RelWithDebInfo 原生定向测试已通过，最终两配置门禁见下节。原 L/T 审查范围与此前测试归位见 [L/T 低风险项验收](LT低风险项验收.md)。

## 最终门禁与复审

本阶段最终 RelWithDebInfo／Debug 编译及完整 CTest 均退出 0，各 1092 项登记、1076 项执行通过、16 项既有禁用，含 120 个公开头独立编译、106 个低层文件依赖检查及安装消费者。RelWithDebInfo 全量约 44.08 秒，Debug 全量约 301.07 秒；该时间仅记录本次运行，不作性能改善声明。Debug 日志逐项确认两个新参数场景实际执行通过，完整运行名称集合指纹亦为 4813680ad0e04eeddec6188a6ad14f89bd07ebbb5756fb7ec263e223dc1f1852。最新 Debug 安装副本与运行库 SHA256 一致：eee6fcaa8789eef07d108a6feeb5e4fa6782d98543e1594c7d0934b6b2e30b6f。

两轮从输入独立性、构造逆向还原、参数到工厂映射、共同 17 条检查、其余三项正文及发现／执行路由复审，未发现本批新问题。全部新改动未提交，整体结构收敛目标仍继续；本参数化不证明一般 Healing 能力或其余数值边界完整。
