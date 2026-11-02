#pragma once

namespace Geometry
{
    enum class SCSegmentKind2
    {
        Line,
        Arc
    };

    enum class SCArcDirection
    {
        Clockwise,
        CounterClockwise
    };

    // 新代码使用带维度的名称；保留原枚举定义以维持公开符号和已有派生类的 ABI。
    using SCSegmentKind2d = SCSegmentKind2;
    using SCArcDirection2d = SCArcDirection;
}  // namespace Geometry
