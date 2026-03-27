# Rename Follow-Up Todo

## Current Status

当前仓库有两条主线：
- 2D API 命名统一已经做了一轮源码级修改，但还没有编译和测试验证。
- 3D 设计文档已经起草，并且已确认 Delphi 3D 参考目录应使用 `D:\code\GFY2.0\Controls\GAEAResource\GCL\Geo3DLib\Source`。

这份待办文档优先服务于 2D 命名统一的收尾和恢复上下文。

## Todo List

### 1. 先做最小验证

不要一开始全量重编，优先最小闭环。

建议顺序：
1. 编译当前几何库和受影响测试目标。
2. 优先跑这些 capability tests：
   - `test_segment`
   - `test_polyline`
   - `test_polygon`
   - `test_sdk_multigeometry`
   - `test_point_vector`
   - `test_box`
3. 如果上述通过，再看是否需要补跑：
   - `test_shapes_pathops`
   - 其他 capability tests

### 2. 如果编译或测试失败，优先排查这些点

- 旧命名漏改
- 头文件声明和 `.cpp` 定义不一致
- `types/*` 改名后，SDK 包装层或序列化层还有旧调用
- `MultiPolygon2d::Count()` / `MultiPolyline2d::Count()` / `PolygonTopology2d::Count()` 调用是否全部同步
- `ArcSegment2::SignedSweep()` 这类改名后的实现是否仍有误替换问题
- 测试辅助 `tests/support/GeometryTestSupport.h` 与基础类型新名字是否一致

### 3. 验证通过后再做文档同步

命名修改验证通过后，再统一更新仍写旧 `GetXxx` 风格的设计文档。

优先文档：
- `docs/sdk-2d-api-convergence.md`
- `docs/sdk-type-design-review.md`
- 其他 2D 类型设计文档

### 4. 2D 稳定后再继续 3D 细化

建议顺序：
1. `geometry-3d-types-design.md`
2. `curve3d-surface-design.md`
3. `polyhedron-brep-design.md`

## Files To Check First

如果下次开始时还没有编译结果，优先静态检查这些文件：
- `include/types/Box2.h`
- `include/types/Segment2.h`
- `include/types/LineSegment2.h`
- `include/types/ArcSegment2.h`
- `include/types/Polyline2.h`
- `include/types/Polygon2.h`
- `include/sdk/MultiPolyline2d.h`
- `include/sdk/MultiPolygon2d.h`
- `include/sdk/GeometryTopology.h`
- `src/sdk/Polyline2d.cpp`
- `src/sdk/Polygon2d.cpp`
- `src/sdk/MultiPolyline2d.cpp`
- `src/sdk/MultiPolygon2d.cpp`
- `src/sdk/GeometryTopology.cpp`
- `src/serialize/GeometryText.cpp`
- `tests/capabilities/test_segment.cpp`
- `tests/capabilities/test_polyline.cpp`
- `tests/capabilities/test_polygon.cpp`
- `tests/capabilities/test_sdk_multigeometry.cpp`
- `tests/support/GeometryTestSupport.h`

## How To Send Me Results Later

如果你晚点手动编译或跑测试，建议按这个格式把结果发给我：

### 编译失败时
- 目标名
- 首个报错文件和行号
- 核心报错文本
- 前后若干行上下文

### 测试失败时
- 测试可执行文件或 ctest target 名
- 失败用例名
- 断言信息
- 相关 stdout / stderr

### 验证通过时
- 跑了哪些目标/测试
- 是否全部通过

## Prompt For Restoring Context

下次如果要让我快速恢复当前记忆并继续工作，你可以直接把下面这段发给我：

```text
你现在在仓库 `D:\code\stablecore\stablecore-geometry` 中继续工作。

先读这些文件恢复上下文：
1. `docs/session-handoff.md`
2. `docs/sdk-2d-api-convergence.md`
3. `docs/rename-followup-todo.md`
4. `docs/geometry-3d-library-design.md`

当前状态要点：
- 2D API 命名统一已经做了一轮源码级修改，主要把旧 `GetXxx` 风格改成短名风格，例如：
  - `GetMinPoint` -> `MinPoint`
  - `GetMaxPoint` -> `MaxPoint`
  - `GetKind` -> `Kind`
  - `GetStartPoint` -> `StartPoint`
  - `GetEndPoint` -> `EndPoint`
  - `GetBoundingBox` -> `Bounds`
  - `GetPointAt` -> `PointAt`
  - `GetPointAtLength` -> `PointAtLength`
  - `GetSegmentCount` -> `SegmentCount`
  - `GetVertexCount` -> `VertexCount`
  - `GetVertex` -> `VertexAt`
  - `GetOuterRing` -> `OuterRing`
  - `GetHoleCount` -> `HoleCount`
  - `GetHole` -> `HoleAt`
- `MultiPolyline2d::PolylineCount()` 和 `MultiPolygon2d::PolygonCount()` 已收口为 `Count()`。
- `PolygonTopology2d::PolygonCount()` 已收口为 `Count()`。
- 这些改动还没有编译和测试验证。
- 用户会手动编译和跑测试，并把结果贴回来。
- 不要使用沙箱；本地 sandbox 在这个环境里经常失败。
- 过程里少提问，优先直接做代码和文档工作。
- 3D 设计文档已经写好，并且 Delphi 3D 参考目录应以 `D:\code\GFY2.0\Controls\GAEAResource\GCL\Geo3DLib\Source` 为准。

这次任务目标：
- 先根据用户贴回来的编译/测试结果修复 2D 命名统一带来的遗留问题。
- 如果还没有结果，就先静态检查与命名统一相关的文件，找可能的漏改和声明/定义不一致。
- 在 2D 收口稳定后，再继续细化 3D 设计文档。
```

## Minimal Next Step

如果下次用户还没给编译结果，最先做：
- 检查 2D 命名统一这一批文件是否还有声明/定义不一致
- 检查是否还有旧 `GetXxx` 风格残留在代码而不是纯文档中
- 然后再根据用户贴回来的编译/测试结果继续修
