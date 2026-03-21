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

    [[nodiscard]] std::size_t GetSegmentCount() const
    {
        return segments_.size();
    }

    [[nodiscard]] const SegmentType& GetSegment(std::size_t index) const
    {
        assert(index < segments_.size());
        assert(segments_[index] != nullptr);
        return *segments_[index];
    }

    [[nodiscard]] PolylineClosure GetClosure() const
    {
        return closure_;
    }

    [[nodiscard]] bool IsClosed() const
    {
        return closure_ == PolylineClosure::Closed;
    }

    [[nodiscard]] std::size_t GetVertexCount() const
    {
        if (segments_.empty())
        {
            return 0;
        }

        return IsClosed() ? segments_.size() : segments_.size() + 1;
    }

    [[nodiscard]] PointType GetVertex(std::size_t index) const
    {
        assert(index < GetVertexCount());
        if (segments_.empty())
        {
            return PointType{};
        }

        if (!IsClosed() && index == segments_.size())
        {
            return GetSegment(segments_.size() - 1).GetEndPoint();
        }

        return GetSegment(index).GetStartPoint();
    }

    [[nodiscard]] PointType GetStartPoint() const
    {
        if (segments_.empty() || segments_.front() == nullptr)
        {
            return PointType{};
        }

        return segments_.front()->GetStartPoint();
    }

    [[nodiscard]] PointType GetEndPoint() const
    {
        if (segments_.empty() || segments_.back() == nullptr)
        {
            return PointType{};
        }

        return segments_.back()->GetEndPoint();
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

    [[nodiscard]] Box2<T> GetBoundingBox() const
    {
        if (!IsValid())
        {
            return Box2<T>();
        }

        Box2<T> box;
        for (const auto& segment : segments_)
        {
            box.ExpandToInclude(segment->GetBoundingBox());
        }

        return box;
    }

    [[nodiscard]] PointType GetPointAt(double parameter) const
    {
        return GetPointAtLength(GetLengthAt(parameter), false);
    }

    [[nodiscard]] PointType GetPointAtLength(LengthType distanceFromStart, bool clampToPath = false) const
    {
        if (segments_.empty())
        {
            return PointType{};
        }

        if (!IsValid())
        {
            return GetStartPoint();
        }

        const LengthType totalLength = Length();
        if (totalLength <= LengthType{})
        {
            return GetStartPoint();
        }

        if (clampToPath)
        {
            distanceFromStart = ClampLength(distanceFromStart, LengthType{}, totalLength);
        }

        if (distanceFromStart < LengthType{})
        {
            return segments_.front()->GetPointAtLength(distanceFromStart, false);
        }

        if (distanceFromStart > totalLength)
        {
            return segments_.back()->GetPointAtLength(
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
                return segments_[i]->GetPointAtLength(distanceFromStart - segmentStart, false);
            }
        }

        return GetEndPoint();
    }

    [[nodiscard]] LengthType GetLengthAt(double parameter) const
    {
        return static_cast<LengthType>(parameter) * Length();
    }

    [[nodiscard]] double GetParameterAtLength(LengthType distanceFromStart, bool clampToPath = false) const
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
            if (i > 0 && !IsEqual(segments_[i - 1]->GetEndPoint(), segment->GetStartPoint()))
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
            return IsEqual(segments_.back()->GetEndPoint(), segments_.front()->GetStartPoint());
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
