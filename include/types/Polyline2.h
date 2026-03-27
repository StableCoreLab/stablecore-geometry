#pragma once

#include <cassert>
#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

#include "algorithm/Predicate2.h"
#include "types/Segment2.h"

namespace geometry
{
enum class PolylineClosure
{
    Open,
    Closed
};

template <typename T>
class Polyline2
{
public:
    using ValueType = T;
    using SegmentType = Segment2<T>;
    using SegmentPtr = std::shared_ptr<SegmentType>;
    using PointType = Point2<T>;
    using LengthType = typename SegmentType::LengthType;

    Polyline2() = default;

    explicit Polyline2(PolylineClosure closure)
        : closure_(closure)
    {
    }

    Polyline2(std::vector<SegmentPtr> segments, PolylineClosure closure = PolylineClosure::Open)
        : segments_(std::move(segments)), closure_(closure)
    {
    }

    [[nodiscard]] std::size_t SegmentCount() const
    {
        return segments_.size();
    }

    [[nodiscard]] const SegmentType& SegmentAt(std::size_t index) const
    {
        assert(index < segments_.size());
        assert(segments_[index] != nullptr);
        return *segments_[index];
    }

    [[nodiscard]] PolylineClosure Closure() const
    {
        return closure_;
    }

    [[nodiscard]] bool IsClosed() const
    {
        return closure_ == PolylineClosure::Closed;
    }

    [[nodiscard]] std::size_t VertexCount() const
    {
        if (segments_.empty())
        {
            return 0;
        }

        return IsClosed() ? segments_.size() : segments_.size() + 1;
    }

    [[nodiscard]] PointType VertexAt(std::size_t index) const
    {
        assert(index < VertexCount());
        if (segments_.empty())
        {
            return PointType{};
        }

        if (!IsClosed() && index == segments_.size())
        {
            return SegmentAt(segments_.size() - 1).EndPoint();
        }

        return SegmentAt(index).StartPoint();
    }

    [[nodiscard]] PointType StartPoint() const
    {
        if (segments_.empty() || segments_.front() == nullptr)
        {
            return PointType{};
        }

        return segments_.front()->StartPoint();
    }

    [[nodiscard]] PointType EndPoint() const
    {
        if (segments_.empty() || segments_.back() == nullptr)
        {
            return PointType{};
        }

        return segments_.back()->EndPoint();
    }

    [[nodiscard]] LengthType Length() const
    {
        LengthType total{};
        for (const auto& segment : segments_)
        {
            if (segment != nullptr)
            {
                total += segment->Length();
            }
        }

        return total;
    }

    [[nodiscard]] Box2<T> Bounds() const
    {
        if (!IsValid())
        {
            return Box2<T>();
        }

        Box2<T> box;
        for (const auto& segment : segments_)
        {
            box.ExpandToInclude(segment->Bounds());
        }

        return box;
    }

    [[nodiscard]] PointType PointAt(double parameter) const
    {
        return PointAtLength(LengthAt(parameter), false);
    }

    [[nodiscard]] PointType PointAtLength(LengthType distanceFromStart, bool clampToPath = false) const
    {
        if (segments_.empty())
        {
            return PointType{};
        }

        if (!IsValid())
        {
            return StartPoint();
        }

        const LengthType totalLength = Length();
        if (totalLength <= LengthType{})
        {
            return StartPoint();
        }

        if (clampToPath)
        {
            distanceFromStart = ClampLength(distanceFromStart, LengthType{}, totalLength);
        }

        if (distanceFromStart < LengthType{})
        {
            return segments_.front()->PointAtLength(distanceFromStart, false);
        }

        if (distanceFromStart > totalLength)
        {
            return segments_.back()->PointAtLength(
                distanceFromStart - PrefixLength(segments_.size() - 1),
                false);
        }

        for (std::size_t i = 0; i < segments_.size(); ++i)
        {
            const LengthType segmentStart = PrefixLength(i);
            const LengthType segmentLength = segments_[i]->Length();
            const LengthType segmentEnd = segmentStart + segmentLength;
            if (distanceFromStart <= segmentEnd || i + 1 == segments_.size())
            {
                return segments_[i]->PointAtLength(distanceFromStart - segmentStart, false);
            }
        }

        return EndPoint();
    }

    [[nodiscard]] LengthType LengthAt(double parameter) const
    {
        return static_cast<LengthType>(parameter) * Length();
    }

    [[nodiscard]] double ParameterAtLength(LengthType distanceFromStart, bool clampToPath = false) const
    {
        const LengthType totalLength = Length();
        if (totalLength <= LengthType{})
        {
            return 0.0;
        }

        if (clampToPath)
        {
            distanceFromStart = ClampLength(distanceFromStart, LengthType{}, totalLength);
        }

        return static_cast<double>(distanceFromStart) / static_cast<double>(totalLength);
    }

    [[nodiscard]] bool IsValid() const
    {
        if (segments_.empty())
        {
            return false;
        }

        LengthType totalLength{};
        for (std::size_t i = 0; i < segments_.size(); ++i)
        {
            const auto& segment = segments_[i];
            if (segment == nullptr || !segment->IsValid())
            {
                return false;
            }

            totalLength += segment->Length();
            if (i > 0 && !IsEqual(segments_[i - 1]->EndPoint(), segment->StartPoint()))
            {
                return false;
            }
        }

        if (totalLength <= LengthType{})
        {
            return false;
        }

        if (IsClosed())
        {
            return IsEqual(segments_.back()->EndPoint(), segments_.front()->StartPoint());
        }

        return true;
    }

private:
    [[nodiscard]] static LengthType ClampLength(LengthType value, LengthType low, LengthType high)
    {
        if (value < low)
        {
            return low;
        }
        if (value > high)
        {
            return high;
        }
        return value;
    }

    [[nodiscard]] LengthType PrefixLength(std::size_t count) const
    {
        LengthType total{};
        for (std::size_t i = 0; i < count; ++i)
        {
            total += segments_[i]->Length();
        }
        return total;
    }

    std::vector<SegmentPtr> segments_{};
    PolylineClosure closure_{PolylineClosure::Open};
};

using Polyline2d = Polyline2<double>;
using Polyline2i = Polyline2<int>;
}

