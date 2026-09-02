#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace Geometry::Detail
{
    inline void AddClampedParameter(std::vector<double>& parameters, double value, const double mergeTolerance)
    {
        value = std::clamp(value, 0.0, 1.0);
        for (const double existing : parameters)
        {
            if (std::abs(existing - value) <= mergeTolerance)
            {
                return;
            }
        }
        parameters.push_back(value);
    }

    [[nodiscard]] inline std::vector<double> CompactSortedParameters(std::vector<double> parameters,
                                                                     const double parameterTolerance)
    {
        if (parameters.empty())
        {
            return parameters;
        }

        std::sort(parameters.begin(), parameters.end());
        std::vector<double> compacted;
        compacted.reserve(parameters.size());

        std::size_t clusterStart = 0;
        while (clusterStart < parameters.size())
        {
            std::size_t clusterEnd = clusterStart + 1;
            double weighted = parameters[clusterStart];
            while (clusterEnd < parameters.size() &&
                   parameters[clusterEnd] <= parameters[clusterEnd - 1] + parameterTolerance)
            {
                weighted += parameters[clusterEnd];
                ++clusterEnd;
            }

            const double representative = weighted / static_cast<double>(clusterEnd - clusterStart);
            compacted.push_back(std::clamp(representative, 0.0, 1.0));
            clusterStart = clusterEnd;
        }

        if (compacted.front() > parameterTolerance)
        {
            compacted.insert(compacted.begin(), 0.0);
        } else
        {
            compacted.front() = 0.0;
        }

        if (compacted.back() < 1.0 - parameterTolerance)
        {
            compacted.push_back(1.0);
        } else
        {
            compacted.back() = 1.0;
        }

        return compacted;
    }
}  // namespace Geometry::Detail
