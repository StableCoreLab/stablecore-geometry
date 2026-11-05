# L/T 低风险项源码与测试组织验收

核对日期：2026-11-05。编号来自最初审查记录 9767303 的 L1–L14、T1–T6；D1–D8 的逐项表继续见结构收敛验收对照。本文记录实际结构证据与兼容边界，不把一般几何算法、任意输入数值完整性或高阶拓扑修复改称完成。路径相对于仓库根目录。

## L 条目

| 编号 | 当前源码证据与边界 |
|---|---|
| L1 | Core/GeometryTypes.h 已为兼容聚合头，保留既有类型的传递包含；其 SCCurveEval3d 已由 GeometryTypesPrimitives.h 引入，不再重复前置声明。该头及 Results.h、Section.h 的多余尾部空行本轮清理。不能把兼容聚合中的类型包含仅因没有头内直接引用就全部删除。 |
| L2 | Support/Geometry2d/Predicate2.h 无注释掉的 requires；代码规范明确 IsZero／IsEqual 已有，IsLess／IsGreater 尚未提供统一接口。文档不再承诺不存在的函数。 |
| L3 | Geometry2d/SCSegmentSearch2d.h、SCBoxTree2d.h 未直接包含 Core 算法头；公开声明所需基元／类型直接包含，算法实现归于 Source/Core。完整公开头及低层传递依赖由现有门禁核对。 |
| L4 | 当前 Source/Core/SCPathOps.cpp 不再包含 Brep/Topology.h；Source/Geometry2d/MultiPolygon2d.cpp、MultiPolyline2d.cpp 不包含 Core/ShapeOps.h；BrepConversion.cpp 不包含 iostream。容器共同操作使用 Source/Detail/GeometryContainerOps.h。 |
| L5 | 导出规范区分新头文件值类型、具有库实现的接口、既有导出结果及单一定义的外部数据。Epsilon.h 的外部标量和临界角数组使用 GEOMETRY_DATA_API，安装消费者实际读取；不删除兼容导出或把模板别名加上类导出标记。 |
| L6 | SCMatrix2d 提供 FromRows、AlmostEquals、operator==，SCTransform2d 提供 IsIdentity；原缺口所指主接口均已存在。 |
| L7 | SCEllipse2d::Bounds 用旋转椭圆解析极值；Circle／Ellipse 通过共同采样助手生成多边形，三种形状提供参数比较。旧默认采样数 Circle=64、Ellipse=128 及最小分段门槛继续保留兼容；共享算法不等于默认参数相同，不能据此宣称默认数值已经统一。 |
| L8 | SCNurbsCurve3d::IsClosed 计算实际参数域两端点，IsPeriodic 单独返回周期声明。周期声明不保证当前表示闭合或接缝光滑；现行契约及回归见 H8/H9 说明。 |
| L9 | SCOffsetSurface 求偏移点的导数并重算法线，TestOffsetSurfaceEval3d.cpp 覆盖弯曲基面、边界及零偏移对照；不以代表性回归声明任意偏移曲面数值精度。 |
| L10 | Edge／Coedge／Loop／Shell 有所属容器数量的有效性入口，Face 的对应入口具名为 IsValidWithEdgeCount，Body 消费这些检查。无容器上下文的局部 IsValid 不承诺拓扑引用一定存在。 |
| L11 | 曲线吸附主入口为 SnapPointToCurves3d，旧曲线重载弃用转发；SCSegmentKind2d、SCArcDirection2d 为同类型主名称，原枚举符号保留。结果字段 segmentIndex 按 ABI 保留，不据字段旧名认定主入口未迁移。 |
| L12 | Heal(TriangleMesh) 先定向，再以 oriented.mesh 封闭平面边界，成功结果不丢失先前定向；无法封口时保留原有定向成功回退，不扩大一般修复支持范围。 |
| L13 | Core/MeshOps 为只读查询，Core/MeshRepair 返回修复副本；三角边枚举由低层共同助手复用，重复返回已清理。Brep 旧头为兼容转发，职责边界见专门说明。 |
| L14 | fast preset 不再重复指定默认的 unity=OFF；README 提供实际 cmake --install 命令，安装消费者验证布局。当前 preset 使用版本 3，安装采用 CLI 命令。 |

## T 条目

| 编号 | 当前源码证据与边界 |
|---|---|
| T1 | 测试支持头的路径大小写按 Support/GeometryTestSupport.h 使用；测试文件清单为显式路径，大小写检查应对照文件树。当前逐路径核对 460 处测试本地 include，均存在且大小写一致；Windows 门禁不能单独证明 Linux 编译通过。 |
| T2 | 自定义断言通过 ADD_FAILURE_AT 报告非致命失败；单次失败和连续两次宏失败回归核对调用位置及继续执行，不再通过 runtime_error 丢失逐条报告。 |
| T3 | 原重复的 kPi 定义已统一引用共享常量；测试树中不再定义本地 kPi 或用 acos(-1) 计算圆周率。TestDllExport.cpp 的圆周率字面量是独立的导入数据校验值，继续保留，不能改成常量与自身比较。 |
| T4 | 原两个转换系列的 53 项对应关系、构造与断言证据见转换场景验收；夹具按主题分头，复杂场景是否合并取决于输入和断言，不只看行数。两项稳定回归已从 Gaps 归入 Capabilities；随后四 shell 的两套输入共用具名参数及共同 17 条检查，原构造和另外三项测试正文保持；本轮三壳双面场景另共用内部输入工厂，分别保留原 11／12 条断言。 |
| T5 | 三维测试使用维度后缀；历史批次名称和新名称见迁移表。此前两个迁移用例分别归入 Projection3dTest、SegmentSearch3dTest；后续新增 FourShellArbitration3dTest 的两个具名参数。当前三维能力与差距共 74 个源文件，文件名及显式 suite 均通过维度后缀核对；原批次的 70 个文件及 264 个过滤名称仍是阶段快照。 |
| T6 | 测试覆盖说明已补二维曲线适配和任意 UV 分片覆盖，删除已闭合的三维零长度投影缺口及失效旧路径。当前 Gaps 恰为 16 项 DISABLED_ 占位，不执行，也不计为执行通过；稳定能力已归位。 |

## T4 具体场景整理

整理前按语法边界扫描 Capabilities 的测试体，至少 100 行的共 15 项，均为 Healing 场景，最大 132 行；此计数不作为拆分依据。已直接核对 TestHealingPartialOverlapShells3d.cpp 的 AggressiveHealingClosesIndependentAndVertexTouchShellsWhileSkippingPartialOverlapPair，以及 TestHealingVertexTouchCombinations3d.cpp 的 AggressiveHealingClosesIndependentAndVertexTouchShellsWhileSkippingCompetingPair：二者分别构造部分重叠对／共享边竞争对，但四 shell 的修复执行及结果断言相同，已在后续阶段整理为两个具名参数，详见 [Healing 四 shell 场景验收](T4Healing四壳场景验收.md)。其余长测试继续按拓扑输入依赖和独立判据判断；本组整理后，至少 100 行的测试体由 15 项降至 13 项，不以行数作为缺陷或完成标准。上述四 shell 场景自 ASSERT_TRUE(body.IsValid()) 至测试末尾的 17 条断言及执行语句逐字一致，UTF-8／LF 正文后缀 SHA256 均为 af1e32d6a6cdb3f73da2b02b87fac75c5bf54167c4c9aaaf2863fa1d348a429d；该指纹证明可共享的结果检查，不证明两套输入拓扑相同。

## 两项稳定回归归位

| 旧过滤名称 | 当前过滤名称 | 当前源文件 |
|---|---|---|
| MissingAlgorithms3dGapTest.ProjectPointToLineSegmentRespectsTinyToleranceBoundary | Projection3dTest.ProjectPointToLineSegmentRespectsTinyToleranceBoundary | UnitTests/Capabilities/Geometry3d/TestProjection3d.cpp |
| MissingAlgorithms3dGapTest.SegmentSearch3dKNearestBeyondRepresentativeCases | SegmentSearch3dTest.KNearestRespectsRadiusAndEqualDistanceIdOrder | UnitTests/Capabilities/Geometry3d/TestSegmentSearch3d.cpp |

原 UnitTests/Gaps/Geometry3d/TestMissingAlgorithms3dGaps.cpp 只保留禁用的三维等距吸附缺口。两个新能力文件仅直接包含所需公开头，不依赖 Geometry.h 聚合；CMake 各登记一次。测试发现顺序因归位而调整，总登记数不变。

按 UTF-8／LF 归一化，两个测试从左花括号到右花括号的正文 SHA256 分别为：

- 投影：24828146017ace6c751ec7281bbb6533528a30061aea1d77a0f8d4ed3779a0f0。
- 段搜索：a46751c02b8a8b20a0419eb91692f0740c44e0edf3deeb3aa4b4cb4cdd8eb27f。

迁移前将两项旧名称替换为上述新名称，再与迁移后的完整发现集合排序比较，1092 项名称的 SHA256 一致，均为 1096175ac7a0f9dc2983cf66562450de30762aebccd37bee3d619f7a97a4f4ed。该指纹只证明本次两名称替换以外的名称集合保持，不能推出发现顺序保持或所有数值能力完整。实际 CTest 命令路由两项用例到能力测试二进制；原断言及语句顺序保持。

完整门禁、后续复审及当前工作区状态以本阶段最终执行记录为准。

相关说明：[公开接口与导出规范](../规范/公开接口与导出规范.md)、[网格查询与修复职责](L13网格查询与修复职责.md)、[转换场景验收](T4转换场景验收.md)、[三维测试名称迁移](T5测试命名迁移.md)。

## 稳定回归归位阶段门禁（历史记录）

指定 GenerateGeometryVS2022.bat 生成解决方案，L1 公开头清理后的 Debug／RelWithDebInfo 重新编译及完整 CTest 均退出 0，各 1092 项登记、1076 项执行通过、16 项既有禁用，含 120 个公开头独立编译、106 个低层文件依赖检查及安装消费者。最终 RelWithDebInfo 约 41.96 秒，Debug 约 193.68 秒。两项迁移测试均由能力测试二进制执行，当前 Debug 日志逐项确认通过；正文指纹再次比对保持。

最新 Debug 安装副本与运行库 SHA256 一致：eee6fcaa8789eef07d108a6feeb5e4fa6782d98543e1594c7d0934b6b2e30b6f。两轮核对迁移正文、CMake 登记、执行路由、兼容包含及当前文档状态后，未发现本批新的问题；当时尚待核查的 T4 四 shell 场景已在后续参数化，见前节；其余指定范围继续验收，整体目标未完成，所有新改动保持未提交。

## 四 shell 参数化阶段门禁

本阶段最终 RelWithDebInfo／Debug 编译及完整 CTest 均退出 0，各 1092 项登记、1076 项执行通过、16 项既有禁用，含 120 个公开头独立编译、106 个低层文件依赖检查及安装消费者。RelWithDebInfo 全量约 44.08 秒，Debug 全量约 301.07 秒；该时间仅记录本次运行，不作性能改善声明。Debug 日志逐项确认两个新参数场景实际执行通过，完整运行名称集合指纹亦为 4813680ad0e04eeddec6188a6ad14f89bd07ebbb5756fb7ec263e223dc1f1852。最新 Debug 安装副本与运行库 SHA256 一致：eee6fcaa8789eef07d108a6feeb5e4fa6782d98543e1594c7d0934b6b2e30b6f。

四 shell 参数化阶段的 461 处测试本地 include 路径大小写一致，当时 74 个三维测试源文件及 suite 命名符合维度后缀，README 与 docs 的 98 处相对文件链接均有效。两轮复审后未发现本批新问题，原四 shell 具体待办已完成，其余复杂场景和指定数值边界按具体契约继续验收。

## T4 三壳双面场景输入工厂（2026-11-05）

TestHealingMultiFaceShellCombinations3d.cpp 中，以下两个既有测试重复构造闭壳、带孔／普通双面开壳、非共面开壳。20 个顶点和 20 条边的构造语句除一个顶点的 Z 高度及注释外完全相同，现由文件匿名命名空间的 BuildThreeShellWithHoledAndPlainFaces(nonPlanarHeight) 每次独立创建 body；没有共享可写测试实例或跨文件公共夹具。

| 保留的 Healing3dCapabilityTest 测试名 | 非共面高度 | 保留的独立检查 |
|---|---|---|
| AggressiveHealingThreeShellEligibleMultiFaceHoledMissingTrims | 0.13 | 原 11 条断言，包括三壳闭合状态、总面数及带孔面的 trim |
| AggressiveHealingThreeShellEligibleMultiFaceBothTrimsMissing | 0.11 | 原 12 条断言，另检查普通面的 OuterTrim |

两者原先均没有给两个合格面提供 trim；不能把测试名当作输入差异，也不能为共用构造删除普通面补回的独立断言。两个 TEST 及其名称、默认 SCGeometryTolerance3d／Aggressive 调用保持，不把独立断言合并成条件化的参数检查。另两项多面开片／单带孔面三壳测试因输入不同继续独立，正文不变。移除本文件未使用的 Support/Fixtures3d.h 包含。

将工厂去除一层缩进、return 还原为原 body 声明、代入对应高度并补回原注释和包含后，可逐字还原本阶段输入文件；原文件 SHA256 为 1006eb7994a26826bc5ff08195bde6d14e21295b5d0afa59ba9623fa4d996598，整理后为 01a46aa609f34d11a8e7235dc205324bcc5f496df2f5d7649175f40fba50da29。两项从 ASSERT_TRUE(body.IsValid()) 起的原后缀 SHA256 分别为 43a10a41bfe0727b9eea2969182e98096a806f014305f6eb8a54a37226806802、05126c7e13d241ed9918874332fc72cbd50fa532f4a4131fefa79979a4f1bb66，保持原有差别。

7 项 Healing3dCapabilityTest.AggressiveHealingThreeShell* 定向在整理前后均通过。分别核对输入展开／全部断言，以及 body 值所有权／每次独立 Clone／测试登记后，未发现本批新增问题。此阶段不修改生产源码、CMake 或公开接口；完整两配置门禁按最终执行记录验收。

同时对照 Projection.cpp、M31 迁移说明及其回归，修正 M28 容差迁移说明遗留的“端点形式夹取后强制 isOnSegment=true”表述。当前端点、具体段及基类投影均报告未夹取参数域归属；AxisOps 的既有夹取标志策略属于单独入口，历史阶段记录继续保留。

本阶段最终指定脚本生成、隔离 RelWithDebInfo／Debug 编译及完整 CTest 均退出 0，各 1094 项登记、1078 项执行通过、16 项既有禁用；完整测试分别用时 46.03／289.79 秒。含 120 个公开头独立项目、106 个低层文件依赖检查及安装后消费者。两项保留名称的用例在最终 Debug 二进制实际执行通过，完整名称集合 SHA256 继续为 fc3a0efc007498d8e9eeaf7690afd7c75e439c04fa3a0958fa9463bd19f596aa；Debug 运行库与最新安装副本 SHA256 同为 8c2930a366f48600a0406a8e043fe22338f5a791ef295387984173486738d614。当前 460 处测试本地包含路径大小写正确，README／docs 的 99 处相对文件链接有效；本批及累计改动未暂存、未提交，整体验收继续。
