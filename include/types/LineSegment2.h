#pragma once

#include <algorithm>

#include "types/Segment2.h"

namespace geometry
{
template <typename T>
class LineSegment2 : public Segment2<T>
{
public:
    using BaseType = Segment2<T>;
    using PointType = typename BaseType::PointType;
    using VectorType = typename BaseType::VectorType;
    using LengthType = typename BaseType::LengthType;

    constexpr LineSegment2() = default;
    constexpr LineSegment2(const PointType& startPoint, const PointType& endPoint)
        : startPoint_(startPoint), endPoint_(endPoint)
    {
    }

    [[nodiscard]] SegmentKind2 GetKind() const override
    {
        return SegmentKind2::Line;
    }

    [[nodiscard]] PointType GetStartPoint() const override
    {
        return startPoint_;
    }

    [[nodiscard]] PointType GetEndPoint() const override
    {
        return endPoint_;
    }

    [[nodiscard]] LengthType Length() const override
    {
        return (endPoint_ - startPoint_).Length();
    }

    [[nodiscard]] Box2<T> GetBoundingBox() const override
    {
        if (!IsValid())
        {
            return Box2<T>();
        }

        Box2<T> box;
        box.ExpandToInclude(startPoint_);
        box.ExpandToInclude(endPoint_);
        return box;
    }

    [[nodiscard]] PointType GetPointAt(double parameter) const override
    {
        return GetPointAtLength(static_cast<LengthType>(parameter) * Length(), false);
    }

    [[nodiscard]] PointType GetPointAtLength(LengthType distanceFromStart, bool clampToSegment = false) const override
    {
        if (!IsValid())
        {
            return startPoint_;
        }

        const LengthType length = Length();
        if (length <= LengthType{})
        {
            return startPoint_;
        }

        const LengthType clampedDistance = clampToSegment
            ? static_cast<LengthType>(detail::ClampDouble(
                static_cast<double>(distanceFromStart),
                0.0,
                static_cast<double>(length)))
            : distanceFromStart;

        const double ratio = static_cast<double>(clampedDistance) / static_cast<double>(length);
        return Interpolate(ratio);
    }

    [[nodiscard]] bool IsValid() const override
    {
        return !IsZero(endPoint_ - startPoint_);
    }

private:
    [[nodiscard]] PointType Interpolate(double ratio) const
    {
        const double x = static_cast<double>(startPoint_.x) +
                         (static_cast<double>(endPoint_.x) - static_cast<double>(startPoint_.x)) * ratio;
        const double y = static_cast<double>(startPoint_.y) +
                         (static_cast<double>(endPoint_.y) - static_cast<double>(startPoint_.y)) * ratio;
        return PointType(static_cast<T>(x), static_cast<T>(y));
    }

    PointType startPoint_{};
    PointType endPoint_{};
};

using LineSegment2d = LineSegment2<double>;
using LineSegment2i = LineSegment2<int>;
}
