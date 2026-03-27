#pragma once

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

#include "common/Epsilon.h"

namespace geometry
{
struct Intervald
{
    double min{0.0};
    double max{0.0};

    [[nodiscard]] static constexpr Intervald FromMinMax(double minValue, double maxValue)
    {
        return Intervald{minValue, maxValue};
    }

    [[nodiscard]] constexpr bool IsValid() const
    {
        return std::isfinite(min) && std::isfinite(max) && min <= max;
    }

    [[nodiscard]] constexpr double Length() const
    {
        return max - min;
    }

    [[nodiscard]] constexpr bool Contains(double value, double eps = kDefaultEpsilon) const
    {
        return value >= min - eps && value <= max + eps;
    }

    [[nodiscard]] constexpr bool AlmostEquals(const Intervald& other, double eps = kDefaultEpsilon) const
    {
        return std::abs(min - other.min) <= eps && std::abs(max - other.max) <= eps;
    }

    [[nodiscard]] std::string DebugString() const
    {
        std::ostringstream stream;
        stream << "Intervald{min=" << min << ", max=" << max << "}";
        return stream.str();
    }
};
} // namespace geometry
