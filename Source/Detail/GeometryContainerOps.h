#pragma once

#include <algorithm>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

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

    template <typename Range, typename IsValid>
    [[nodiscard]] bool ContainerIsValid(const Range& values, IsValid isValid)
    {
        return std::all_of(values.begin(), values.end(), isValid);
    }

    template <typename Range>
    [[nodiscard]] bool ContainerIsValid(const Range& values)
    {
        return ContainerIsValid(values, [](const auto& value) { return value.IsValid(); });
    }

    template <typename Range, typename CountOf>
    [[nodiscard]] std::size_t ContainerTotalCount(const Range& values, CountOf countOf)
    {
        std::size_t count = 0;
        for (const auto& value : values)
        {
            count += countOf(value);
        }
        return count;
    }

    // 返回删除前下标到删除后下标的映射；无匹配时不修改序列。
    template <typename Range>
    [[nodiscard]] std::optional<std::vector<std::size_t>> EraseEntriesByIdWithMapping(Range& values, std::size_t id)
    {
        std::vector<std::size_t> mapping(values.size());
        std::size_t retained = 0;
        for (std::size_t index = 0; index < values.size(); ++index)
        {
            mapping[index] = values[index].id == id ? values.size() : retained++;
        }
        if (retained == values.size())
        {
            return std::nullopt;
        }
        values.erase(std::remove_if(values.begin(), values.end(),
                                    [id](const auto& value) { return value.id == id; }),
                     values.end());
        return std::move(mapping);
    }

    template <typename Range>
    [[nodiscard]] auto FindEntryIteratorById(Range& values, std::size_t id)
    {
        return std::find_if(values.begin(), values.end(), [id](const auto& value) { return value.id == id; });
    }

    template <typename Range>
    [[nodiscard]] const typename Range::value_type* FindEntryById(const Range& values, std::size_t id)
    {
        const auto it = FindEntryIteratorById(values, id);
        return it == values.end() ? nullptr : &*it;
    }
}  // namespace Geometry::Detail
