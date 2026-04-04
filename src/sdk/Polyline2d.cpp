#include "sdk/Polyline2d.h"

#include <memory>
#include <sstream>
#include <utility>

#include "types/LineSegment2.h"
#include "types/Polyline2.h"

namespace geometry::sdk
{
namespace
{
[[nodiscard]] geometry::PolylineClosure ToInternalClosure(PolylineClosure closure)
{
    return closure == PolylineClosure::Closed ? geometry::PolylineClosure::Closed
                                              : geometry::PolylineClosure::Open;
}

[[nodiscard]] geometry::Polyline2d MakeInternalPolyline(
    const std::vector<Point2d>& points,
    PolylineClosure closure)
{
    std::vector<std::shared_ptr<geometry::Segment2d>> segments;
    if (points.size() < 2)
    {
        return geometry::Polyline2d(std::move(segments), ToInternalClosure(closure));
    }

    const std::size_t segmentCount =
        closure == PolylineClosure::Closed ? points.size() : points.size() - 1;
    segments.reserve(segmentCount);
    for (std::size_t i = 0; i < segmentCount; ++i)
    {
        const Point2d& start = points[i];
        const Point2d& end = points[(i + 1) % points.size()];
        segments.push_back(std::make_shared<geometry::LineSegment2d>(
            start,
            end));
    }

    return geometry::Polyline2d(std::move(segments), ToInternalClosure(closure));
}
} // namespace

struct Polyline2d::Impl
{
    geometry::Polyline2d polyline;

    Impl() = default;

    explicit Impl(PolylineClosure closure)
        : polyline(ToInternalClosure(closure))
    {
    }

    explicit Impl(geometry::Polyline2d value)
        : polyline(std::move(value))
    {
    }
};

Polyline2d::Polyline2d() : impl_(std::make_unique<Impl>()) {}

Polyline2d::Polyline2d(PolylineClosure closure) : impl_(std::make_unique<Impl>(closure)) {}

Polyline2d::Polyline2d(std::vector<Point2d> points, PolylineClosure closure)
    : impl_(std::make_unique<Impl>(MakeInternalPolyline(points, closure)))
{
}

Polyline2d::Polyline2d(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

Polyline2d::Polyline2d(const Polyline2d& other)
    : impl_(std::make_unique<Impl>(other.impl_->polyline))
{
}

Polyline2d::Polyline2d(Polyline2d&& other) noexcept = default;

Polyline2d& Polyline2d::operator=(const Polyline2d& other)
{
    if (this != &other)
    {
        impl_ = std::make_unique<Impl>(other.impl_->polyline);
    }
    return *this;
}

Polyline2d& Polyline2d::operator=(Polyline2d&& other) noexcept = default;

Polyline2d::~Polyline2d() = default;

bool Polyline2d::IsValid() const
{
    return impl_->polyline.IsValid();
}

bool Polyline2d::IsClosed() const
{
    return impl_->polyline.IsClosed();
}

double Polyline2d::Length() const
{
    return impl_->polyline.Length();
}

std::size_t Polyline2d::PointCount() const
{
    return impl_->polyline.VertexCount();
}

std::size_t Polyline2d::SegmentCount() const
{
    return impl_->polyline.SegmentCount();
}

Point2d Polyline2d::PointAt(std::size_t index) const
{
    return impl_->polyline.VertexAt(index);
}

Box2d Polyline2d::Bounds() const
{
    return impl_->polyline.Bounds();
}

std::string Polyline2d::DebugString() const
{
    std::ostringstream stream;
    stream << "Polyline2d{closure=" << (IsClosed() ? "Closed" : "Open")
           << ", pointCount=" << PointCount()
           << ", segmentCount=" << SegmentCount()
           << ", valid=" << (IsValid() ? "true" : "false")
           << ", bounds=" << Bounds().DebugString() << "}";
    return stream.str();
}
} // namespace geometry::sdk

