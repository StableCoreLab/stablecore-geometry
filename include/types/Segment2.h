#pragma once

#include <algorithm>
#include <cmath>
#include <type_traits>

#include "algorithm/Predicate2.h"
#include "types/Box2.h"
#include "types/Point2.h"
#include "types/Vector2.h"

namespace geometry
{
enum class SegmentKind2
{
    Line,
    Arc
};

enum class ArcDirection
{
    Clockwise,
    CounterClockwise
};

namespace detail
{
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kTwoPi = 2.0 * kPi;

[[nodiscard]] inline double ClampDouble(double value, double low, double high)
{
    return std::max(low, std::min(value, high));
}

[[nodiscard]] inline double NormalizeAngle(double angle)
{
    angle = std::fmod(angle, kTwoPi);
    if (angle < 0.0)
    {
        angle += kTwoPi;
    }
    return angle;
}

[[nodiscard]] inline double SignedSweep(double startAngle, double endAngle, ArcDirection direction)
{
    if (direction == ArcDirection::CounterClockwise)
    {
        return NormalizeAngle(endAngle - startAngle);
    }

    return -NormalizeAngle(startAngle - endAngle);
}

[[nodiscard]] inline bool IsAngleOnArc(double candidateAngle, double startAngle, double signedSweep, double eps = kDefaultEpsilon)
{
    if (signedSweep >= 0.0)
    {
        const double delta = NormalizeAngle(candidateAngle - startAngle);
        return delta <= signedSweep + eps;
    }

    const double delta = NormalizeAngle(startAngle - candidateAngle);
    return delta <= (-signedSweep) + eps;
}

template <typename T>
[[nodiscard]] constexpr bool IsFiniteValue(T value)
{
    if constexpr (std::is_floating_point_v<T>)
    {
        return std::isfinite(static_cast<double>(value));
    }
    else
    {
        return true;
    }
}

} // namespace detail

template <typename T>
class Segment2
{
public:
    using PointType = Point2<T>;
    using VectorType = Vector2<T>;
    using LengthType = typename VectorType::LengthType;

    virtual ~Segment2() = default;

    [[nodiscard]] virtual SegmentKind2 GetKind() const = 0;
    [[nodiscard]] virtual PointType GetStartPoint() const = 0;
    [[nodiscard]] virtual PointType GetEndPoint() const = 0;
    [[nodiscard]] virtual LengthType Length() const = 0;
    [[nodiscard]] virtual Box2<T> GetBoundingBox() const = 0;
    [[nodiscard]] virtual PointType GetPointAt(double parameter) const = 0;
    [[nodiscard]] virtual PointType GetPointAtLength(LengthType distanceFromStart, bool clampToSegment = false) const = 0;
    [[nodiscard]] virtual bool IsValid() const = 0;
};

using Segment2d = Segment2<double>;
using Segment2i = Segment2<int>;
}
