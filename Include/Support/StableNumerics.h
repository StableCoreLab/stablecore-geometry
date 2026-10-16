#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <optional>

namespace Geometry::Detail
{
    // 有限性是结果的一部分；调用方不得把失败值钳制成有效参数。
    [[nodiscard]] inline std::optional<double> StableSpan(double first, double last)
    {
        const double value = last - first;
        if (!std::isfinite(first) || !std::isfinite(last) || !std::isfinite(value)) return std::nullopt;
        return value;
    }

    [[nodiscard]] inline std::optional<double> StableProduct(double first, double second, double third = 1)
    {
        if (!std::isfinite(first) || !std::isfinite(second) || !std::isfinite(third)) return std::nullopt;
        if (first==0 || second==0 || third==0) return 0.0;
        int a=0,b=0,c=0;
        const double mantissa=std::frexp(first,&a)*std::frexp(second,&b)*std::frexp(third,&c);
        const double value=std::scalbn(mantissa,a+b+c);
        return std::isfinite(value) ? std::optional<double>(value) : std::nullopt;
    }

    [[nodiscard]] inline std::optional<double> StableLerp(double first, double last, double weight)
    {
        if (!std::isfinite(first) || !std::isfinite(last) || !std::isfinite(weight)) return std::nullopt;
        const double value = std::lerp(first, last, weight);
        return std::isfinite(value) ? std::optional<double>(value) : std::nullopt;
    }

    [[nodiscard]] inline std::optional<double> StableMidpoint(double first, double last)
    {
        if (!std::isfinite(first) || !std::isfinite(last)) return std::nullopt;
        return std::midpoint(first, last);
    }

    [[nodiscard]] inline std::optional<double> StableAdvance(double parameter, double step)
    {
        if (!std::isfinite(parameter) || !std::isfinite(step)) return std::nullopt;
        const double value = parameter + step;
        return std::isfinite(value) ? std::optional<double>(value) : std::nullopt;
    }

    [[nodiscard]] inline bool StableContains(double first, double last, double value, double epsilon = 0)
    {
        if (!std::isfinite(first) || !std::isfinite(last) || !std::isfinite(value) ||
            !std::isfinite(epsilon) || epsilon < 0 || first > last) return false;
        return (value >= first || first - value <= epsilon) &&
               (value <= last || value - last <= epsilon);
    }

    [[nodiscard]] inline std::optional<double> StableNormalizedParameter(double first, double last, double value)
    {
        if (!StableContains(first, last, value)) return std::nullopt;
        if (first == last) return 0.0;
        if (value == first) return 0.0;
        if (value == last) return 1.0;
        const double span = last - first;
        if (std::isfinite(span)) return (value - first) / span;
        // 异号有限端点的跨度可溢出；共同缩放不改变比值。
        const double scale = std::max(std::abs(first), std::abs(last));
        const double ratio = (value / scale - first / scale) / (last / scale - first / scale);
        return std::isfinite(ratio) ? std::optional<double>(ratio) : std::nullopt;
    }

    // 比较已经可表示的面积/体积与 factor*epsilon^power，不形成容差幂。
    // 空值表示输入或中间几何量不可表示，不能当作确定退化或确定非退化。
    [[nodiscard]] inline std::optional<bool> MagnitudeWithinPower(
        double magnitude, double epsilon, unsigned power, double factor = 1)
    {
        if (!std::isfinite(magnitude) || magnitude < 0 || !std::isfinite(epsilon) || epsilon < 0 ||
            !std::isfinite(factor) || factor <= 0 || power > 3) return std::nullopt;
        if (magnitude == 0) return true;
        if (epsilon == 0 && power != 0) return false;
        int leftExponent = 0, epsilonExponent = 0, factorExponent = 0;
        const double left = std::frexp(magnitude, &leftExponent);
        const double unit = std::frexp(epsilon, &epsilonExponent);
        double right = std::frexp(factor, &factorExponent);
        for (unsigned i = 0; i < power; ++i) right *= unit;
        int normalization = 0;
        right = std::frexp(right, &normalization);
        const int rightExponent = factorExponent + static_cast<int>(power) * epsilonExponent + normalization;
        return leftExponent < rightExponent || (leftExponent == rightExponent && left <= right);
    }
}
