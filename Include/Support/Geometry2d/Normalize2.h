#pragma once

#include <type_traits>

#include "Support/Geometry2d/Predicate2.h"

namespace Geometry
{
    template <typename T>
    [[deprecated("Use SCVector2::TryNormalized() instead")]]
    [[nodiscard]] bool TryNormalize(const SCVector2<T>& vector, SCVector2<double>& normalized, double eps = kDefaultEpsilon)
    {
        return vector.TryNormalized(normalized, eps);
    }
}  // namespace Geometry
