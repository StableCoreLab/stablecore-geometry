#pragma once

#include <cassert>
#include <sstream>
#include <string>
#include <type_traits>

#include "Support/Epsilon.h"
#include "Types/Geometry2d/SCPoint2.h"

namespace Geometry
{
    template <typename T>
    using SCBoxMeasureType = std::conditional_t<std::is_floating_point_v<T>, T, double>;

    template <typename T>
    class SCBox2
    {
    public:
        using ValueType = T;
        using MeasureType = SCBoxMeasureType<T>;

        constexpr SCBox2() = default;

        constexpr SCBox2(const SCPoint2<T>& minPoint, const SCPoint2<T>& maxPoint)
            : min_(minPoint), max_(maxPoint), valid_(IsOrdered(minPoint, maxPoint))
        {
        }

        [[nodiscard]] static constexpr SCBox2 FromMinMax(const SCPoint2<T>& minPoint, const SCPoint2<T>& maxPoint)
        {
            return SCBox2(minPoint, maxPoint);
        }

        [[nodiscard]] constexpr bool IsValid() const
        {
            return valid_ && min_.IsValid() && max_.IsValid() && IsOrdered(min_, max_);
        }

        [[nodiscard]] constexpr const SCPoint2<T>& MinPoint() const
        {
            return min_;
        }

        [[nodiscard]] constexpr const SCPoint2<T>& MaxPoint() const
        {
            return max_;
        }

        [[nodiscard]] constexpr MeasureType Width() const
        {
            assert(IsValid());
            return static_cast<MeasureType>(max_.x) - static_cast<MeasureType>(min_.x);
        }

        [[nodiscard]] constexpr MeasureType Height() const
        {
            assert(IsValid());
            return static_cast<MeasureType>(max_.y) - static_cast<MeasureType>(min_.y);
        }

        [[nodiscard]] constexpr MeasureType Area() const
        {
            assert(IsValid());
            return Width() * Height();
        }

        [[nodiscard]] constexpr SCPoint2<MeasureType> Center() const
        {
            assert(IsValid());
            return SCPoint2<MeasureType>(
                (static_cast<MeasureType>(min_.x) + static_cast<MeasureType>(max_.x)) / static_cast<MeasureType>(2),
                (static_cast<MeasureType>(min_.y) + static_cast<MeasureType>(max_.y)) / static_cast<MeasureType>(2));
        }

        constexpr void ExpandToInclude(const SCPoint2<T>& point)
        {
            if (!IsValid())
            {
                min_ = point;
                max_ = point;
                valid_ = true;
                return;
            }

            if (point.x < min_.x)
            {
                min_.x = point.x;
            }
            if (point.y < min_.y)
            {
                min_.y = point.y;
            }
            if (point.x > max_.x)
            {
                max_.x = point.x;
            }
            if (point.y > max_.y)
            {
                max_.y = point.y;
            }
        }

        constexpr void ExpandToInclude(const SCBox2& box)
        {
            if (!box.IsValid())
            {
                return;
            }

            if (!IsValid())
            {
                *this = box;
                return;
            }

            ExpandToInclude(box.min_);
            ExpandToInclude(box.max_);
        }

        [[nodiscard]] constexpr bool operator==(const SCBox2& other) const = default;
        [[nodiscard]] constexpr bool operator!=(const SCBox2& other) const = default;

        [[nodiscard]] constexpr bool AlmostEquals(const SCBox2& other, double eps = kDefaultEpsilon) const
        {
            if constexpr (std::is_floating_point_v<T>)
            {
                if (valid_ != other.valid_)
                {
                    return false;
                }

                if (!valid_)
                {
                    return true;
                }

                return min_.AlmostEquals(other.min_, eps) && max_.AlmostEquals(other.max_, eps);
            }

            (void)eps;
            return *this == other;
        }

        [[nodiscard]] std::string DebugString() const
        {
            std::ostringstream stream;
            stream << "SCBox2{min=" << min_.DebugString() << ", max=" << max_.DebugString()
                   << ", valid=" << (valid_ ? "true" : "false") << "}";
            return stream.str();
        }

    private:
        static constexpr bool IsOrdered(const SCPoint2<T>& minPoint, const SCPoint2<T>& maxPoint)
        {
            return minPoint.x <= maxPoint.x && minPoint.y <= maxPoint.y;
        }

        SCPoint2<T> min_{};
        SCPoint2<T> max_{};
        bool valid_{false};
    };

    using SCBox2d = SCBox2<double>;
    using SCBox2i = SCBox2<int>;
}  // namespace Geometry
