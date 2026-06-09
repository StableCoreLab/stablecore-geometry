#pragma once

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

#include "Support/Epsilon.h"

namespace Geometry
{
    struct SCIntervald
    {
        double min{0.0};
        double max{0.0};

        [[nodiscard]] static constexpr SCIntervald FromMinMax(double minValue, double maxValue)
        {
            return SCIntervald{minValue, maxValue};
        }

        [[nodiscard]] bool IsValid() const
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

        [[nodiscard]] bool AlmostEquals(const SCIntervald& other, double eps = kDefaultEpsilon) const
        {
            return std::abs(min - other.min) <= eps && std::abs(max - other.max) <= eps;
        }

        [[nodiscard]] std::string DebugString() const
        {
            std::ostringstream stream;
            stream << "SCIntervald{min=" << min << ", max=" << max << "}";
            return stream.str();
        }
    };

}  // namespace Geometry
