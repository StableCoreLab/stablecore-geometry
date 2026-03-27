#pragma once

#include <cassert>
#include <cmath>
#include <sstream>
#include <string>
#include <type_traits>

#include "common/Epsilon.h"
#include "types/Point2.h"

namespace geometry
{
template <typename T>
using BoxMeasureType = std::conditional_t<std::is_floating_point_v<T>, T, double>;

template <typename T>
class Box2
{
public:
    using ValueType = T;
    using MeasureType = BoxMeasureType<T>;

    constexpr Box2() = default;

    constexpr Box2(const Point2<T>& minPoint, const Point2<T>& maxPoint)
        : min_(minPoint), max_(maxPoint), valid_(IsOrdered(minPoint, maxPoint))
    {
    }

    [[nodiscard]] static constexpr Box2 FromMinMax(
        const Point2<T>& minPoint,
        const Point2<T>& maxPoint)
    {
        return Box2(minPoint, maxPoint);
    }

    [[nodiscard]] constexpr bool IsValid() const
    {
        return valid_ && min_.IsValid() && max_.IsValid() && IsOrdered(min_, max_);
    }

    [[nodiscard]] constexpr const Point2<T>& MinPoint() const
    {
        return min_;
    }

    [[nodiscard]] constexpr const Point2<T>& MaxPoint() const
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

    [[nodiscard]] constexpr Point2<MeasureType> Center() const
    {
        assert(IsValid());
        return Point2<MeasureType>(
            (static_cast<MeasureType>(min_.x) + static_cast<MeasureType>(max_.x)) / static_cast<MeasureType>(2),
            (static_cast<MeasureType>(min_.y) + static_cast<MeasureType>(max_.y)) / static_cast<MeasureType>(2));
    }

    constexpr void ExpandToInclude(const Point2<T>& point)
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

    constexpr void ExpandToInclude(const Box2& box)
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

    [[nodiscard]] constexpr bool operator==(const Box2& other) const = default;
    [[nodiscard]] constexpr bool operator!=(const Box2& other) const = default;

    [[nodiscard]] constexpr bool AlmostEquals(
        const Box2& other,
        double eps = kDefaultEpsilon) const
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
        stream << "Box2{min=" << min_.DebugString() << ", max=" << max_.DebugString()
               << ", valid=" << (valid_ ? "true" : "false") << "}";
        return stream.str();
    }

private:
    static constexpr bool IsOrdered(const Point2<T>& minPoint, const Point2<T>& maxPoint)
    {
        return minPoint.x <= maxPoint.x && minPoint.y <= maxPoint.y;
    }

    Point2<T> min_{};
    Point2<T> max_{};
    bool valid_{false};
};

using Box2d = Box2<double>;
using Box2i = Box2<int>;
}

