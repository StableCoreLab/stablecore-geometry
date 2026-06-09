#pragma once

#include <type_traits>

#include "Support/Geometry2d/Predicate2.h"

namespace Geometry
{
    template <typename T>
    [[nodiscard]] bool TryNormalize(const SCVector2<T>& vector, SCVector2<double>& normalized, double eps = kDefaultEpsilon)
    {
        if (IsZero(vector, eps))
        {
            return false;
        }

        const double length = static_cast<double>(vector.Length());
        normalized = SCVector2<double>(static_cast<double>(vector.x) / length, static_cast<double>(vector.y) / length);
        return true;
    }
}  // namespace Geometry
