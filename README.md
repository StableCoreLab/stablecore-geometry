# stablecore-geometry

一套面向工程计算的 C++ 几何库。

## 目标

- 提供可复用的二维、三维几何能力
- 保持公开 API 稳定、可测试、可发布
- 以 section、healing、boolean、conversion、validation、search 为核心工作流

## 当前范围

- 二维值类型与二维几何服务
- 三维值类型、曲线、曲面、BRep、polyhedron 支持
- section、healing、boolean、projection、relation、measurement、validation、mesh conversion 服务
- 文本序列化辅助与搜索索引辅助

## 公开入口

外部使用优先包含统一入口：

```cpp
#include "Geometry.h"
```

`Geometry.h` 是稳定的 umbrella 入口，会聚合当前主要公开头文件。

需要更细的包含粒度时，可直接使用下列模块：

- `Core`：算法结果类型与跨模块操作，包括布尔、修复、转换和拓扑服务
- `Geometry2d`：二维基本图元与路径工具
- `Geometry3d`：曲线、曲面与三维求值辅助
- `Brep`：BRep、polyhedron、mesh 实体与拓扑数据结构；旧高层服务头保留转发兼容，主入口归入 `Core`，详见 [模块依赖与头文件迁移](docs/设计/模块依赖与头文件迁移.md)

更底层的公开面分得更细：

- `Support/Epsilon.h`：共享容差常量
- `Support/Geometry2d`：二维标量、谓词、归一化辅助
- `Types/Geometry2d`：二维值类型
- `Types/Geometry3d`：三维值类型与线性代数辅助
- `Serialize/GeometryText.h`：文本序列化辅助
- `Export/GeometryExport.h`：导入导出标注

需要跨 DLL 的公开结果结构统一从对应公共头导出并使用导出宏；header-only 值类型不额外添加 `GEOMETRY_API`：

- `Core/GeometryTypes.h`
- 其他公开 API 头中直接导出的值类型

umbrella 中可直接获得的常用类型（均保留 `SC` 前缀）包括：

- `SCPoint2d`, `SCVector2d`, `SCBox2d`
- `SCPoint3d`, `SCVector3d`, `SCDirection3d`, `SCBox3d`, `SCIntervald`
- `SCLine3d`, `SCRay3d`, `SCLineSegment3d`, `SCTriangle3d`
- `SCTransform3d`, `SCMatrix3d`, `SCPlane`

如果只需要某一小块能力，建议直接包含对应头文件，而不是总是走 umbrella。

发布安装会刻意收口：

- `Types/Compat` 与 `Types/Detail` 不进入默认安装面
- 被公开头引用的 `Geometry3d/Detail` 辅助头随公开头一起安装

## 设计原则

- 面向工程，而不是面向演示
- 数据结构清晰
- 容差语义显式
- 核心算法测试先行

更多说明见：

- [`docs/设计/库设计说明.md`](docs/设计/库设计说明.md)
- [`docs/设计/多面体与BRep设计.md`](docs/设计/多面体与BRep设计.md)
- [`docs/设计/平面与多边形拓扑设计.md`](docs/设计/平面与多边形拓扑设计.md)
- [`docs/设计/段内核与序列化设计.md`](docs/设计/段内核与序列化设计.md)
- [`docs/设计/关系内核设计.md`](docs/设计/关系内核设计.md)
- [`docs/使用/使用说明.md`](docs/使用/使用说明.md)

## 构建

默认本地构建环境为 Visual Studio 2022 x64；以下脚本和 preset 需要 CMake 3.21 或更新版本。

常用方式：

1. 通过脚本生成解决方案：

```bat
GenerateGeometryVS2022.bat
```

该脚本默认生成到 `../Build/GeometryVs2022`（本仓库位置对应 `D:\code\StableCore\Build\GeometryVs2022`），并输出生成的解决方案路径；不会自动打开 IDE。第一个参数可覆盖生成目录。

2. 或直接使用 CMake preset：

```bat
cmake --preset vs2022-x64
cmake --build --preset vs2022-x64-build --config Release
cmake --build --preset vs2022-x64-build --config RelWithDebInfo
```

仓库还包含 `.vscode/settings.json`，可供 VS Code CMake Tools 直接复用同一套 preset。

顶层工程默认构建测试；作为子工程使用时默认关闭。启用测试时需预装 GTest，或提供 `../ThirdParty/GoogleTest` 本地源码；配置过程不会联网下载 GTest。`BUILD_SHARED_LIBS=ON` 为默认动态库模式，设置为 `OFF` 可构建静态库。

生成解决方案后，可执行：

```bat
cmake --build ..\Build\GeometryVs2022 --config Debug
ctest --test-dir ..\Build\GeometryVs2022 -C Debug --output-on-failure
cmake --build ..\Build\GeometryVs2022 --config RelWithDebInfo
ctest --test-dir ..\Build\GeometryVs2022 -C RelWithDebInfo --output-on-failure
```

## 安装

安装布局按配置与平台分开：

### 发布布局

- `Bin/x64/Release`：DLL 和可执行文件
- `Bin/x64/RelWithDebInfo`：带调试信息的 DLL 和可执行文件
- `Lib/x64/Release`：导入库或静态库
- `Lib/x64/RelWithDebInfo`：带调试信息的导入库或静态库
- `Include/SCGeometry`：公开头文件
- `Lib/cmake/SCGeometry`：CMake 包文件

上述为新配置的 Windows x64 默认值；安装目录可通过 `GNUInstallDirs` 变量配置。已有构建目录会保留 `CMakeCache.txt` 中的安装目录设置，例如旧配置可能仍使用小写 `bin/lib/include`。非 Windows 平台的共享库安装在对应的 `${CMAKE_INSTALL_LIBDIR}` 平台和配置目录下。

安装后，下游项目可这样使用：

```cmake
find_package(SCGeometry CONFIG REQUIRED)
target_link_libraries(your_target PRIVATE SCGeometry::Geometry)
```

本地安装前缀示例：

```bat
cmake --install ..\Build\GeometryVs2022 --config Release --prefix D:\code\stablecore\install\Geometry
cmake --install ..\Build\GeometryVs2022 --config RelWithDebInfo --prefix D:\code\stablecore\install\Geometry
```
