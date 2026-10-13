#pragma once

#include <array>
#include <cmath>
#include <sstream>
#include <string>
#include <type_traits>

namespace Geometry::Detail
{
    template <typename T, std::size_t Dimension>
    [[nodiscard]] constexpr bool CoordinateStorageIsValid(const std::array<T, Dimension>& coordinates)
    {
        if constexpr (std::is_floating_point_v<T>)
        {
            for (const T coordinate : coordinates)
            {
                if (!std::isfinite(coordinate))
                {
                    return false;
                }
            }
        }
        return true;
    }

    template <typename T, std::size_t Dimension>
    [[nodiscard]] constexpr bool CoordinateStorageAlmostEquals(const std::array<T, Dimension>& lhs,
                                                               const std::array<T, Dimension>& rhs,
                                                               double epsilon)
    {
        if constexpr (std::is_floating_point_v<T>)
        {
            for (std::size_t i = 0; i < Dimension; ++i)
            {
                if (!(std::abs(static_cast<double>(lhs[i] - rhs[i])) <= epsilon))
                {
                    return false;
                }
            }
            return true;
        }
        else
        {
            (void)epsilon;
            return lhs == rhs;
        }
    }

    template <typename T, std::size_t Dimension>
    [[nodiscard]] std::string CoordinateStorageDebugString(const char* typeName,
                                                           const std::array<T, Dimension>& coordinates)
    {
        static_assert(Dimension <= 3);
        constexpr std::array<const char*, 3> names{"x", "y", "z"};
        std::ostringstream stream;
        stream << typeName << "{";
        for (std::size_t i = 0; i < Dimension; ++i)
        {
            if (i > 0)
            {
                stream << ", ";
            }
            stream << names[i] << "=" << coordinates[i];
        }
        stream << "}";
        return stream.str();
    }
}  // namespace Geometry::Detail
