#pragma once

namespace Geometry::Detail
{
    // 既有二维算法使用的固定下限；调用者传入的容差仍由各入口单独处理。
    inline constexpr double kComparisonEpsilon2d = 1e-12;
    inline constexpr double kParallelDirectionSineEpsilon2d = 1e-9;
    inline constexpr double kAreaEpsilon2d = 1e-10;
    inline constexpr double kRebuildFallbackEpsilon2d = 1e-8;
}
