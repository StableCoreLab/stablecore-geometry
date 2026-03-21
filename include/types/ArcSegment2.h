#pragma once

#include <cmath>

#include "types/Segment2.h"

namespace geometry
{
template <typename T>
class ArcSegment2 : public Segment2<T>
{
public:
    using BaseType = Segment2<T>;
    using PointType = typename BaseType::PointType;
    using VectorType = typename BaseType::VectorType;
    using LengthType = typename BaseType::LengthType;

    constexpr ArcSegment2() = default;
    constexpr ArcSegment2(
        const PointType& center,
        T radius,
        T startAngle,
        T endAngle,
        ArcDirection direction)
        : center_(center),
          radius_(radius),
          startAngle_(startAngle),
          endAngle_(endAngle),
          direction_(direction)
    {
    }

    [[nodiscard]] SegmentKind2 GetKind() const override
    {
        return SegmentKind2::Arc;
    }

    [[nodiscard]] PointType GetStartPoint() const override
    {
        return PointAtAngle(AngleAsDouble(startAngle_));
    }

    [[nodiscard]] PointType GetEndPoint() const override
    {
        return PointAtAngle(AngleAsDouble(endAngle_));
    }

    [[nodiscard]] LengthType Length() const override
    {
        if (!IsValid())
        {
            return LengthType{};
        }

        const double sweep = SignedSweep();
        return static_cast<LengthType>(std::abs(sweep) * static_cast<double>(radius_));
    }

    [[nodiscard]] Box2<T> GetBoundingBox() const override
    {
        if (!IsValid())
        {
            return Box2<T>();
        }

        Box2<T> box;
        box.ExpandToInclude(GetStartPoint());
        box.ExpandToInclude(GetEndPoint());

        const double startAngle = AngleAsDouble(startAngle_);
        const double signedSweep = SignedSweep();

        static constexpr double kCriticalAngles[] = {
            0.0,
            detail::kPi * 0.5,
            detail::kPi,
            detail::kPi * 1.5
        };

        for (const double candidateAngle : kCriticalAngles)
        {
            if (detail::IsAngleOnArc(candidateAngle, startAngle, signedSweep))
            {
                box.ExpandToInclude(PointAtAngle(candidateAngle));
            }
        }

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
            return GetStartPoint();
        }

        const LengthType length = Length();
        if (length <= LengthType{})
        {
            return GetStartPoint();
        }

        const LengthType clampedDistance = clampToSegment
            ? static_cast<LengthType>(detail::ClampDouble(
                static_cast<double>(distanceFromStart),
                0.0,
                static_cast<double>(length)))
            : distanceFromStart;

        const double ratio = static_cast<double>(clampedDistance) / static_cast<double>(length);
        const double angle = AngleAsDouble(startAngle_) + SignedSweep() * ratio;
        return PointAtAngle(angle);
    }

    [[nodiscard]] bool IsValid() const override
    {
        if (!detail::IsFiniteValue(center_.x) || !detail::IsFiniteValue(center_.y))
        {
            return false;
        }

        if (!detail::IsFiniteValue(radius_) || !(radius_ > T{}))
        {
            return false;
        }

        if (!detail::IsFiniteValue(startAngle_) || !detail::IsFiniteValue(endAngle_))
        {
            return false;
        }

        const double sweep = SignedSweep();
        return !IsZero(sweep);
    }

private:
    [[nodiscard]] double AngleAsDouble(T angle) const
    {
        return static_cast<double>(angle);
    }

    [[nodiscard]] double SignedSweep() const
    {
        return detail::SignedSweep(AngleAsDouble(startAngle_), AngleAsDouble(endAngle_), direction_);
    }

    [[nodiscard]] PointType PointAtAngle(double angle) const
    {
        const double radius = static_cast<double>(radius_);
        const double x = static_cast<double>(center_.x) + radius * std::cos(angle);
        const double y = static_cast<double>(center_.y) + radius * std::sin(angle);
        return PointType(static_cast<T>(x), static_cast<T>(y));
    }

    PointType center_{};
    T radius_{};
    T startAngle_{};
    T endAngle_{};
    ArcDirection direction_{ArcDirection::CounterClockwise};
};

using ArcSegment2d = ArcSegment2<double>;
using ArcSegment2i = ArcSegment2<int>;
}
