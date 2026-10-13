#pragma once

#include <algorithm>
#include <sstream>
#include <string>
#include <type_traits>

#include "Support/Epsilon.h"
#include "Types/Geometry3d/SCPoint3.h"

namespace Geometry
{
    template <typename T>
    using SCBoxMeasureType3 = std::conditional_t<std::is_floating_point_v<T>, T, double>;

    template <typename T>
    class SCBox3
    {
    public:
        using ValueType = T;
        using MeasureType = SCBoxMeasureType3<T>;

        constexpr SCBox3() = default;
        constexpr SCBox3(SCPoint3<T> minValue, SCPoint3<T> maxValue, bool isValid = true)
            : minPoint_(minValue), maxPoint_(maxValue), valid_(isValid && IsOrdered(minValue, maxValue))
        {
        }

        [[nodiscard]] static constexpr SCBox3 FromMinMax(const SCPoint3<T>& minValue, const SCPoint3<T>& maxValue)
        {
            return SCBox3{minValue, maxValue, true};
        }

        [[nodiscard]] constexpr bool IsValid() const
        {
            return valid_ && minPoint_.IsValid() && maxPoint_.IsValid() && IsOrdered(minPoint_, maxPoint_);
        }

        [[nodiscard]] constexpr const SCPoint3<T>& MinPoint() const { return minPoint_; }
        [[nodiscard]] constexpr const SCPoint3<T>& MaxPoint() const { return maxPoint_; }
        [[nodiscard]] constexpr bool operator==(const SCBox3&) const = default;
        [[nodiscard]] constexpr bool operator!=(const SCBox3&) const = default;

        [[nodiscard]] constexpr bool AlmostEquals(const SCBox3& other, double eps = kDefaultEpsilon) const
        {
            if constexpr (std::is_floating_point_v<T>)
            {
                if (valid_ != other.valid_)
                {
                    return false;
                }
                return !valid_ ||
                       (minPoint_.AlmostEquals(other.minPoint_, eps) && maxPoint_.AlmostEquals(other.maxPoint_, eps));
            }

            (void)eps;
            return *this == other;
        }

        [[nodiscard]] constexpr MeasureType Width() const
        {
            return static_cast<MeasureType>(maxPoint_.x) - static_cast<MeasureType>(minPoint_.x);
        }
        [[nodiscard]] constexpr MeasureType Height() const
        {
            return static_cast<MeasureType>(maxPoint_.y) - static_cast<MeasureType>(minPoint_.y);
        }
        [[nodiscard]] constexpr MeasureType Depth() const
        {
            return static_cast<MeasureType>(maxPoint_.z) - static_cast<MeasureType>(minPoint_.z);
        }
        [[nodiscard]] constexpr SCPoint3<MeasureType> Center() const
        {
            return SCPoint3<MeasureType>{
                (static_cast<MeasureType>(minPoint_.x) + static_cast<MeasureType>(maxPoint_.x)) * 0.5,
                (static_cast<MeasureType>(minPoint_.y) + static_cast<MeasureType>(maxPoint_.y)) * 0.5,
                (static_cast<MeasureType>(minPoint_.z) + static_cast<MeasureType>(maxPoint_.z)) * 0.5};
        }

        constexpr void ExpandToInclude(const SCPoint3<T>& point)
        {
            if (!IsValid())
            {
                minPoint_ = point;
                maxPoint_ = point;
                valid_ = point.IsValid();
                return;
            }
            minPoint_.x = std::min(minPoint_.x, point.x);
            minPoint_.y = std::min(minPoint_.y, point.y);
            minPoint_.z = std::min(minPoint_.z, point.z);
            maxPoint_.x = std::max(maxPoint_.x, point.x);
            maxPoint_.y = std::max(maxPoint_.y, point.y);
            maxPoint_.z = std::max(maxPoint_.z, point.z);
        }

        [[nodiscard]] std::string DebugString() const
        {
            std::ostringstream stream;
            if constexpr (std::is_same_v<T, double>)
            {
                stream << "SCBox3d{";
            }
            else if constexpr (std::is_same_v<T, int>)
            {
                stream << "SCBox3i{";
            }
            else
            {
                stream << "SCBox3{";
            }
            stream << "minPoint=" << minPoint_.DebugString() << ", maxPoint=" << maxPoint_.DebugString()
                   << ", valid=" << (valid_ ? "true" : "false") << "}";
            return stream.str();
        }

    private:
        [[nodiscard]] static constexpr bool IsOrdered(const SCPoint3<T>& minPoint, const SCPoint3<T>& maxPoint)
        {
            return minPoint.x <= maxPoint.x && minPoint.y <= maxPoint.y && minPoint.z <= maxPoint.z;
        }

        SCPoint3<T> minPoint_{};
        SCPoint3<T> maxPoint_{};
        bool valid_{false};
    };

    using SCBox3d = SCBox3<double>;
    using SCBox3i = SCBox3<int>;
}  // namespace Geometry
