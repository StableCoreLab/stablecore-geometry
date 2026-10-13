#pragma once

#include <type_traits>

namespace Geometry
{
    template <typename T>
    using SCGeometryScalarType = std::conditional_t<std::is_floating_point_v<T>, T, double>;
}  // namespace Geometry
