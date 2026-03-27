#include "sdk/MultiPolygon2d.h"

#include <sstream>
#include <utility>

#include "sdk/GeometryShapeOps.h"

namespace geometry::sdk
{
MultiPolygon2d::MultiPolygon2d(std::vector<Polygon2d> polygons)
    : polygons_(std::move(polygons))
{
}

MultiPolygon2d::MultiPolygon2d(std::initializer_list<Polygon2d> polygons)
    : polygons_(polygons)
{
}

std::size_t MultiPolygon2d::Count() const
{
    return polygons_.size();
}

bool MultiPolygon2d::IsEmpty() const
{
    return polygons_.empty();
}

bool MultiPolygon2d::IsValid() const
{
    for (const auto& polygon : polygons_)
    {
        if (!polygon.IsValid())
        {
            return false;
        }
    }

    return true;
}

void MultiPolygon2d::Clear()
{
    polygons_.clear();
}

void MultiPolygon2d::Add(Polygon2d polygon)
{
    polygons_.push_back(std::move(polygon));
}

const Polygon2d& MultiPolygon2d::PolygonAt(std::size_t index) const
{
    return polygons_.at(index);
}

Polygon2d& MultiPolygon2d::PolygonAt(std::size_t index)
{
    return polygons_.at(index);
}

const Polygon2d& MultiPolygon2d::operator[](std::size_t index) const
{
    return polygons_[index];
}

Polygon2d& MultiPolygon2d::operator[](std::size_t index)
{
    return polygons_[index];
}

std::size_t MultiPolygon2d::PointCount() const
{
    std::size_t count = 0;
    for (const auto& polygon : polygons_)
    {
        count += polygon.PointCount();
    }
    return count;
}

std::size_t MultiPolygon2d::SegmentCount() const
{
    std::size_t count = 0;
    for (const auto& polygon : polygons_)
    {
        count += polygon.SegmentCount();
    }
    return count;
}

std::size_t MultiPolygon2d::HoleCount() const
{
    std::size_t count = 0;
    for (const auto& polygon : polygons_)
    {
        count += polygon.HoleCount();
    }
    return count;
}

Box2d MultiPolygon2d::Bounds() const
{
    Box2d box;
    for (const auto& polygon : polygons_)
    {
        box.ExpandToInclude(polygon.Bounds());
    }
    return box;
}

std::string MultiPolygon2d::DebugString() const
{
    std::ostringstream stream;
    stream << "MultiPolygon2d{count=" << Count()
           << ", pointCount=" << PointCount()
           << ", segmentCount=" << SegmentCount()
           << ", holeCount=" << HoleCount()
           << ", valid=" << (IsValid() ? "true" : "false")
           << ", bounds=" << Bounds().DebugString() << "}";
    return stream.str();
}

const std::vector<Polygon2d>& MultiPolygon2d::Data() const
{
    return polygons_;
}

std::vector<Polygon2d>& MultiPolygon2d::Data()
{
    return polygons_;
}
} // namespace geometry::sdk


