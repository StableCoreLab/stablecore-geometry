#include "sdk/MultiPolyline2d.h"

#include <sstream>
#include <utility>

#include "sdk/GeometryShapeOps.h"

namespace geometry::sdk
{
MultiPolyline2d::MultiPolyline2d(std::vector<Polyline2d> polylines)
    : polylines_(std::move(polylines))
{
}

MultiPolyline2d::MultiPolyline2d(std::initializer_list<Polyline2d> polylines)
    : polylines_(polylines)
{
}

std::size_t MultiPolyline2d::Count() const
{
    return polylines_.size();
}

std::size_t MultiPolyline2d::PolylineCount() const
{
    return polylines_.size();
}

bool MultiPolyline2d::IsEmpty() const
{
    return polylines_.empty();
}

bool MultiPolyline2d::IsValid() const
{
    for (const auto& polyline : polylines_)
    {
        if (!polyline.IsValid())
        {
            return false;
        }
    }

    return true;
}

void MultiPolyline2d::Clear()
{
    polylines_.clear();
}

void MultiPolyline2d::Add(Polyline2d polyline)
{
    polylines_.push_back(std::move(polyline));
}

const Polyline2d& MultiPolyline2d::PolylineAt(std::size_t index) const
{
    return polylines_.at(index);
}

Polyline2d& MultiPolyline2d::PolylineAt(std::size_t index)
{
    return polylines_.at(index);
}

const Polyline2d& MultiPolyline2d::operator[](std::size_t index) const
{
    return polylines_[index];
}

Polyline2d& MultiPolyline2d::operator[](std::size_t index)
{
    return polylines_[index];
}

std::size_t MultiPolyline2d::PointCount() const
{
    std::size_t count = 0;
    for (const auto& polyline : polylines_)
    {
        count += polyline.PointCount();
    }
    return count;
}

std::size_t MultiPolyline2d::SegmentCount() const
{
    std::size_t count = 0;
    for (const auto& polyline : polylines_)
    {
        count += polyline.SegmentCount();
    }
    return count;
}

Box2d MultiPolyline2d::Bounds() const
{
    Box2d box;
    for (const auto& polyline : polylines_)
    {
        box.ExpandToInclude(polyline.Bounds());
    }
    return box;
}

std::string MultiPolyline2d::DebugString() const
{
    std::ostringstream stream;
    stream << "MultiPolyline2d{count=" << Count()
           << ", pointCount=" << PointCount()
           << ", segmentCount=" << SegmentCount()
           << ", valid=" << (IsValid() ? "true" : "false")
           << ", bounds=" << Bounds().DebugString() << "}";
    return stream.str();
}

const std::vector<Polyline2d>& MultiPolyline2d::Data() const
{
    return polylines_;
}

std::vector<Polyline2d>& MultiPolyline2d::Data()
{
    return polylines_;
}
} // namespace geometry::sdk
