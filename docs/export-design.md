# DLL 导出层设计文档

## 1. 文档目的

本文档用于定义几何库未来生成 `dll + import lib + headers` 时的导出规则。

本文档解决的是“怎么导出”，不是“几何语义怎么定义”。

重点覆盖以下内容：

- `GEOMETRY_API` 宏如何组织
- 哪些类型允许作为导出层的一部分
- 哪些模板类型只允许停留在内部实现层
- 是否需要显式实例化
- 当前内部容器、多态和 STL 结构是否允许直接跨 DLL 边界暴露

本文档是 `sdk-design.md` 的工程落地版约束。

## 1.1 文档关系

- `library-design.md` 负责总设计和模块边界
- `sdk-design.md` 负责发布形态和 ABI 总原则
- 本文档负责把 ABI 总原则具体化为可落地的导出规则

执行要求：

- 若未来开始做 DLL 构建，本文件中的规则必须先于正式导出代码落地
- 若某个类型的对外暴露方式与本文档冲突，应优先修改设计，不得先硬导出再补文档

## 2. 当前执行结论

第一版 DLL 导出层统一遵守以下原则：

- 导出层主路径只支持 `double`
- 导出层不直接承诺任意模板实例
- 导出层不直接暴露当前 `Polyline2<T>` 和 `Polygon2<T>` 的 STL/多态内部组织
- 导出层优先导出稳定值类型和受控工厂/算法入口
- 复杂对象若需要跨 DLL 边界长期稳定存在，应后续单独设计 ABI 包装层

## 3. `GEOMETRY_API` 宏规则

未来应统一引入导出宏头文件，例如：

```cpp
// include/export/GeometryExport.h
#if defined(_WIN32)
#  if defined(GEOMETRY_BUILD_DLL)
#    define GEOMETRY_API __declspec(dllexport)
#  elif defined(GEOMETRY_USE_DLL)
#    define GEOMETRY_API __declspec(dllimport)
#  else
#    define GEOMETRY_API
#  endif
#else
#  define GEOMETRY_API
#endif
```

执行要求：

- 不允许在各个头文件里散落重复的导出宏定义
- 所有未来需要正式导出的类型或函数，必须统一依赖同一个导出宏头文件
- `GEOMETRY_BUILD_DLL` 用于库自身编译
- `GEOMETRY_USE_DLL` 用于外部程序使用 DLL

## 4. 当前建议可直接导出的对象

第一版建议优先导出简单、稳定、值语义明确的对象。

推荐对象：

- `Point2d`
- `Vector2d`
- `Box2d`

条件是：

- 它们的布局保持简单
- 不依赖 STL 容器
- 不依赖虚继承层
- 不把复杂所有权语义暴露给外部

这类对象更适合作为 SDK 头文件中的基础数据类型。

## 5. 当前不建议直接导出的对象

以下对象当前不建议直接作为稳定 ABI 类型导出：

- `Segment2<T>`
- `LineSegment2<T>`
- `ArcSegment2<T>`
- `Polyline2<T>`
- `Polygon2<T>`

原因各不相同，但都足够重要。

### 5.1 `Segment2<T>` 不建议直接导出

原因：

- 它是抽象基类
- 依赖虚函数分发
- 当前并未围绕 DLL 边界设计生命周期和分配释放策略

结论：

- `Segment2<T>` 当前只作为内部抽象层存在
- 不把它直接作为 DLL 稳定 ABI 主体

### 5.2 `LineSegment2<T>` 与 `ArcSegment2<T>` 暂不直接承诺为 ABI 类型

虽然它们比 `Segment2<T>` 更稳定，但当前仍不建议直接把模板形态拿去做 DLL 承诺。

执行结论：

- 若未来需要直接暴露线段和圆弧对象，应只考虑 `LineSegment2d` 与 `ArcSegment2d`
- 即便如此，也应先完成导出宏、显式实例化和头文件组织收敛
- 在正式 SDK 阶段前，不把它们现在的模板头文件直接视为最终发布接口

### 5.3 `Polyline2<T>` 与 `Polygon2<T>` 当前禁止直接作为 ABI 类型导出

这是当前最需要明确写死的规则。

原因：

- `Polyline2<T>` 当前内部使用 `std::vector<std::shared_ptr<Segment2<T>>>`
- `Polygon2<T>` 直接依赖 `Polyline2<T>`
- 这意味着它们的当前实现同时暴露了：
  - STL 容器
  - 智能指针
  - 抽象基类层次
  - 动态分发和所有权细节

这些都不适合作为第一版 DLL 稳定 ABI 承诺。

执行结论：

- 当前 `Polyline2d` 不能直接以现有内部结构对外导出
- 当前 `Polygon2d` 不能直接以现有内部结构对外导出
- 若未来外部程序需要创建或消费连续线/多边形，应单独设计 ABI 包装方式

## 6. 模板导出规则

当前模板导出必须遵循以下硬规则：

- 模板可以存在于内部语义层
- 模板不能默认等于“自动属于 DLL 对外 API”
- 只有被明确选中的具体实例，才允许进入导出候选范围

第一版执行结论：

- 只考虑 `double` 实例进入 DLL 收敛层
- 不考虑把 `int` 版本做成正式导出承诺
- 不考虑让调用方自行依赖内部模板实现来完成 DLL SDK 接入

## 7. 显式实例化规则

若未来决定让某些 `double` 版本类型真正进入编译产物导出层，则必须使用显式实例化策略收敛实现边界。

建议形式：

```cpp
extern template class Point2<double>;
extern template class Vector2<double>;
extern template class Box2<double>;
```

并在对应 `.cpp` 中做：

```cpp
template class Point2<double>;
template class Vector2<double>;
template class Box2<double>;
```

执行要求：

- 只有准备正式导出的具体实例，才做显式实例化规划
- 若某个模板类型仍需要完全头文件化，则不得同时把它描述成“已收进 DLL 的稳定 ABI 类型”

## 8. 头文件组织规则

未来对外 SDK 头文件应与内部实现头文件分层。

建议结构：

```text
include/
  export/
    GeometryExport.h
  sdk/
    GeometryTypes.h
    GeometryApi.h
  types/
  algorithm/
```

执行结论：

- `types/` 下当前头文件优先视为内部设计与实现头文件
- 正式 SDK 阶段应新增 `sdk/` 入口头文件，而不是让外部调用方直接无边界包含整个内部目录
- `sdk/` 头文件只能暴露被正式承诺的 API 面

## 9. 算法导出优先级

相较于直接导出复杂类型，第一版更适合优先导出受控算法入口。

例如未来可优先考虑：

- 点到线段距离
- 点到边段投影
- 线段求交
- 包围盒关系判断

这些 API 的优势是：

- 输入输出更容易收敛到简单值类型
- 比直接暴露复杂对象图更稳定
- 更符合 DLL 场景

执行建议：

- 第一版 SDK 优先考虑“值类型 + 算法函数”
- `Polyline2d` 和 `Polygon2d` 这类复杂对象延后进入 ABI 层

## 10. 对当前代码的落地影响

从现在开始，代码和文档应按以下方式理解：

- `Point2<T>`、`Vector2<T>`、`Box2<T>` 仍可继续模板化实现
- `Segment2<T>` 体系继续作为内部抽象和算法支撑层
- `Polyline2<T>`、`Polygon2<T>` 当前只服务内部设计与实现，不等于未来 DLL 稳定对象
- 算法主路径继续优先围绕 `double` 收敛

这意味着当前开发方向不用推翻，但以后不能再默认认为：

- “头文件里有什么，SDK 就直接给什么”

## 11. 当前执行清单

后续若进入 DLL 导出实现阶段，必须至少完成以下事项：

1. 新增统一导出宏头文件
2. 明确 `sdk/` 对外头文件入口
3. 选择第一批正式导出的 `double` 类型
4. 明确哪些模板做显式实例化
5. 明确哪些复杂对象继续留在内部，不直接跨 DLL 边界暴露

## 12. 最终执行结论

后续开发必须按以下原则执行：

- `GEOMETRY_API` 统一通过单一宏头文件定义
- 第一版 DLL 导出层只围绕 `double` 收敛
- 当前模板类型体系不直接等于 DLL 稳定接口
- `Polyline2d` 与 `Polygon2d` 当前禁止以现有内部结构直接对外导出
- 复杂几何对象的 ABI 包装层必须后续单独设计，不得直接把当前 STL/多态实现暴露出去

本文档即为后续 DLL 导出层的工程设计依据。
