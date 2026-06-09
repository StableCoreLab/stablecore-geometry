#include "Geometry2d/SCPolyline2d.h"

#include <cassert>
#include <memory>
#include <sstream>
#include <utility>

#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Support/Geometry2d/Predicate2.h"

namespace Geometry
{
    namespace
    {
        [[nodiscard]] std::vector<std::shared_ptr<ISCSegment2d>> CloneSegments(
            const std::vector<std::shared_ptr<ISCSegment2d>>& segments)
        {
            std::vector<std::shared_ptr<ISCSegment2d>> clones;
            clones.reserve(segments.size());
            for (const auto& segment : segments)
            {
                if (segment == nullptr)
                {
                    clones.push_back(nullptr);
                    continue;
                }

                std::unique_ptr<ISCSegment2d> clone = segment->Clone();
                clones.push_back(std::shared_ptr<ISCSegment2d>(std::move(clone)));
            }
            return clones;
        }

        [[nodiscard]] std::vector<std::shared_ptr<ISCSegment2d>> MakeLineSegments(const std::vector<SCPoint2d>& points,
                                                                               SCPolylineClosure closure)
        {
            std::vector<std::shared_ptr<ISCSegment2d>> segments;
            if (points.size() < 2)
            {
                return segments;
            }

            std::size_t effectivePointCount = points.size();
            if (closure == SCPolylineClosure::Closed && points.size() >= 3 && points.front().AlmostEquals(points.back()))
            {
                effectivePointCount -= 1;
            }

            if (effectivePointCount < 2)
            {
                return segments;
            }

            const std::size_t segmentCount =
                closure == SCPolylineClosure::Closed ? effectivePointCount : effectivePointCount - 1;
            segments.reserve(segmentCount);
            for (std::size_t i = 0; i < segmentCount; ++i)
            {
                const std::size_t nextIndex =
                    closure == SCPolylineClosure::Closed ? (i + 1) % effectivePointCount : i + 1;
                segments.push_back(std::make_shared<SCLineSegment2d>(points[i], points[nextIndex]));
            }
            return segments;
        }
    }  // namespace

    struct SCPolyline2d::Impl
    {
        std::vector<std::shared_ptr<ISCSegment2d>> segments{};
        SCPolylineClosure closure{SCPolylineClosure::Open};

        Impl() = default;

        explicit Impl(SCPolylineClosure closure) : closure(closure)
        {
        }

        Impl(std::vector<std::shared_ptr<ISCSegment2d>> segments, SCPolylineClosure closure)
            : segments(std::move(segments)), closure(closure)
        {
        }
    };

    SCPolyline2d::SCPolyline2d() : impl_(std::make_unique<Impl>())
    {
    }

    SCPolyline2d::SCPolyline2d(SCPolylineClosure closure) : impl_(std::make_unique<Impl>(closure))
    {
    }

    SCPolyline2d::SCPolyline2d(std::vector<SCPoint2d> points, SCPolylineClosure closure)
        : impl_(std::make_unique<Impl>(MakeLineSegments(points, closure), closure))
    {
    }

    SCPolyline2d::SCPolyline2d(std::vector<std::shared_ptr<ISCSegment2d>> segments, SCPolylineClosure closure)
        : impl_(std::make_unique<Impl>(std::move(segments), closure))
    {
    }

    SCPolyline2d::SCPolyline2d(std::unique_ptr<Impl> impl) : impl_(std::move(impl))
    {
    }

    SCPolyline2d::SCPolyline2d(const SCPolyline2d& other)
        : impl_(std::make_unique<Impl>(CloneSegments(other.impl_->segments), other.impl_->closure))
    {
    }

    SCPolyline2d::SCPolyline2d(SCPolyline2d&& other) noexcept = default;

    SCPolyline2d& SCPolyline2d::operator=(const SCPolyline2d& other)
    {
        if (this != &other)
        {
            impl_ = std::make_unique<Impl>(CloneSegments(other.impl_->segments), other.impl_->closure);
        }
        return *this;
    }

    SCPolyline2d& SCPolyline2d::operator=(SCPolyline2d&& other) noexcept = default;

    SCPolyline2d::~SCPolyline2d() = default;

    bool SCPolyline2d::IsValid() const
    {
        if (impl_->segments.empty())
        {
            return false;
        }

        double totalLength = 0.0;
        for (std::size_t i = 0; i < impl_->segments.size(); ++i)
        {
            const auto& segment = impl_->segments[i];
            if (segment == nullptr || !segment->IsValid())
            {
                return false;
            }

            totalLength += segment->Length();
            if (i > 0 && !IsEqual(impl_->segments[i - 1]->EndPoint(), segment->StartPoint()))
            {
                return false;
            }
        }

        if (!(totalLength > 0.0))
        {
            return false;
        }

        if (IsClosed())
        {
            return IsEqual(impl_->segments.back()->EndPoint(), impl_->segments.front()->StartPoint());
        }

        return true;
    }

    bool SCPolyline2d::IsClosed() const
    {
        return impl_->closure == SCPolylineClosure::Closed;
    }

    std::size_t SCPolyline2d::PointCount() const
    {
        return VertexCount();
    }

    std::size_t SCPolyline2d::VertexCount() const
    {
        if (impl_->segments.empty())
        {
            return 0;
        }

        return IsClosed() ? impl_->segments.size() : impl_->segments.size() + 1;
    }

    std::size_t SCPolyline2d::SegmentCount() const
    {
        return impl_->segments.size();
    }

    double SCPolyline2d::Length() const
    {
        double total = 0.0;
        for (const auto& segment : impl_->segments)
        {
            if (segment != nullptr)
            {
                total += segment->Length();
            }
        }
        return total;
    }

    SCPoint2d SCPolyline2d::PointAt(std::size_t index) const
    {
        return VertexAt(index);
    }

    SCPoint2d SCPolyline2d::PointAt(double parameter) const
    {
        return PointAtLength(LengthAt(parameter), false);
    }

    SCPoint2d SCPolyline2d::VertexAt(std::size_t index) const
    {
        assert(index < VertexCount());
        if (impl_->segments.empty())
        {
            return SCPoint2d{};
        }

        if (!IsClosed() && index == impl_->segments.size())
        {
            return impl_->segments.back()->EndPoint();
        }

        return impl_->segments.at(index)->StartPoint();
    }

    SCPoint2d SCPolyline2d::StartPoint() const
    {
        if (impl_->segments.empty() || impl_->segments.front() == nullptr)
        {
            return SCPoint2d{};
        }
        return impl_->segments.front()->StartPoint();
    }

    SCPoint2d SCPolyline2d::EndPoint() const
    {
        if (impl_->segments.empty() || impl_->segments.back() == nullptr)
        {
            return SCPoint2d{};
        }
        return impl_->segments.back()->EndPoint();
    }

    double SCPolyline2d::LengthAt(double parameter) const
    {
        return parameter * Length();
    }

    double SCPolyline2d::ParameterAtLength(double distanceFromStart, bool clampToPath) const
    {
        const double totalLength = Length();
        if (!(totalLength > 0.0))
        {
            return 0.0;
        }

        if (clampToPath)
        {
            if (distanceFromStart < 0.0)
            {
                distanceFromStart = 0.0;
            } else if (distanceFromStart > totalLength)
            {
                distanceFromStart = totalLength;
            }
        }

        return distanceFromStart / totalLength;
    }

    SCPoint2d SCPolyline2d::PointAtLength(double distanceFromStart, bool clampToPath) const
    {
        if (impl_->segments.empty())
        {
            return SCPoint2d{};
        }

        if (!IsValid())
        {
            return StartPoint();
        }

        const double totalLength = Length();
        if (!(totalLength > 0.0))
        {
            return StartPoint();
        }

        if (clampToPath)
        {
            if (distanceFromStart < 0.0)
            {
                distanceFromStart = 0.0;
            } else if (distanceFromStart > totalLength)
            {
                distanceFromStart = totalLength;
            }
        }

        if (distanceFromStart < 0.0)
        {
            return impl_->segments.front()->PointAtLength(distanceFromStart, false);
        }

        double prefixLength = 0.0;
        for (std::size_t i = 0; i < impl_->segments.size(); ++i)
        {
            const double segmentLength = impl_->segments[i]->Length();
            const double segmentEnd = prefixLength + segmentLength;
            if (distanceFromStart <= segmentEnd || i + 1 == impl_->segments.size())
            {
                return impl_->segments[i]->PointAtLength(distanceFromStart - prefixLength, false);
            }
            prefixLength = segmentEnd;
        }

        return EndPoint();
    }

    std::unique_ptr<ISCSegment2d> SCPolyline2d::SegmentAt(std::size_t index) const
    {
        assert(index < impl_->segments.size());
        if (index >= impl_->segments.size() || impl_->segments[index] == nullptr)
        {
            return nullptr;
        }

        return impl_->segments[index]->Clone();
    }

    SCBox2d SCPolyline2d::Bounds() const
    {
        if (!IsValid())
        {
            return SCBox2d{};
        }

        SCBox2d box;
        for (const auto& segment : impl_->segments)
        {
            box.ExpandToInclude(segment->Bounds());
        }
        return box;
    }

    std::string SCPolyline2d::DebugString() const
    {
        std::ostringstream stream;
        stream << "SCPolyline2d{closure=" << (IsClosed() ? "Closed" : "Open") << ", pointCount=" << PointCount()
               << ", segmentCount=" << SegmentCount() << ", valid=" << (IsValid() ? "true" : "false")
               << ", bounds=" << Bounds().DebugString() << "}";
        return stream.str();
    }
}  // namespace Geometry
