#include "Geometry2d/SCMultiPolygon2d.h"

#include <sstream>
#include <utility>

#include "Core/ShapeOps.h"

namespace Geometry
{
    SCMultiPolygon2d::SCMultiPolygon2d(std::vector<SCPolygon2d> polygons) : polygons_(std::move(polygons))
    {
    }

    SCMultiPolygon2d::SCMultiPolygon2d(std::initializer_list<SCPolygon2d> polygons) : polygons_(polygons)
    {
    }

    std::size_t SCMultiPolygon2d::Count() const
    {
        return polygons_.size();
    }

    bool SCMultiPolygon2d::IsEmpty() const
    {
        return polygons_.empty();
    }

    bool SCMultiPolygon2d::IsValid() const
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

    void SCMultiPolygon2d::Clear()
    {
        polygons_.clear();
    }

    void SCMultiPolygon2d::Add(SCPolygon2d polygon)
    {
        polygons_.push_back(std::move(polygon));
    }

    const SCPolygon2d& SCMultiPolygon2d::PolygonAt(std::size_t index) const
    {
        return polygons_.at(index);
    }

    SCPolygon2d& SCMultiPolygon2d::PolygonAt(std::size_t index)
    {
        return polygons_.at(index);
    }

    const SCPolygon2d& SCMultiPolygon2d::operator[](std::size_t index) const
    {
        return polygons_[index];
    }

    SCPolygon2d& SCMultiPolygon2d::operator[](std::size_t index)
    {
        return polygons_[index];
    }

    std::size_t SCMultiPolygon2d::PointCount() const
    {
        std::size_t count = 0;
        for (const auto& polygon : polygons_)
        {
            count += polygon.PointCount();
        }
        return count;
    }

    std::size_t SCMultiPolygon2d::SegmentCount() const
    {
        std::size_t count = 0;
        for (const auto& polygon : polygons_)
        {
            count += polygon.SegmentCount();
        }
        return count;
    }

    std::size_t SCMultiPolygon2d::HoleCount() const
    {
        std::size_t count = 0;
        for (const auto& polygon : polygons_)
        {
            count += polygon.HoleCount();
        }
        return count;
    }

    SCBox2d SCMultiPolygon2d::Bounds() const
    {
        SCBox2d box;
        for (const auto& polygon : polygons_)
        {
            box.ExpandToInclude(polygon.Bounds());
        }
        return box;
    }

    std::string SCMultiPolygon2d::DebugString() const
    {
        std::ostringstream stream;
        stream << "SCMultiPolygon2d{count=" << Count() << ", pointCount=" << PointCount()
               << ", segmentCount=" << SegmentCount() << ", holeCount=" << HoleCount()
               << ", valid=" << (IsValid() ? "true" : "false") << ", bounds=" << Bounds().DebugString() << "}";
        return stream.str();
    }

    const std::vector<SCPolygon2d>& SCMultiPolygon2d::Polygons() const
    {
        return polygons_;
    }

    std::vector<SCPolygon2d>& SCMultiPolygon2d::Polygons()
    {
        return polygons_;
    }

    const std::vector<SCPolygon2d>& SCMultiPolygon2d::Data() const
    {
        return Polygons();
    }

    std::vector<SCPolygon2d>& SCMultiPolygon2d::Data()
    {
        return Polygons();
    }
}  // namespace Geometry
