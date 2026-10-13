#pragma once

#include <algorithm>
#include <cstddef>

namespace Geometry::Detail
{
    template <typename Range>
    [[nodiscard]] std::size_t ContainerCount(const Range& values)
    {
        return values.size();
    }

    template <typename Range>
    [[nodiscard]] bool ContainerIsEmpty(const Range& values)
    {
        return values.empty();
    }

    template <typename Range>
    [[nodiscard]] bool ContainerIsValid(const Range& values)
    {
        return std::all_of(values.begin(), values.end(), [](const auto& value) { return value.IsValid(); });
    }

    template <typename Range>
    [[nodiscard]] const typename Range::value_type* FindEntryById(const Range& values, std::size_t id)
    {
        const auto it = std::find_if(values.begin(), values.end(), [id](const auto& value) { return value.id == id; });
        return it == values.end() ? nullptr : &*it;
    }
}  // namespace Geometry::Detail
