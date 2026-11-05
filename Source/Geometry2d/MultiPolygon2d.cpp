#include "Geometry2d/SCMultiPolygon2d.h"

#include <cmath>
#include <sstream>
#include <utility>

#include "../Detail/GeometryContainerOps.h"

namespace Geometry
{
    SCMultiPolygon2d::SCMultiPolygon2d(std::vector<SCPolygon2d> polygons) : polygons_(std::move(polygons))
    {
    }

    SCMultiPolygon2d::SCMultiPolygon2d(std::initializer_list<SCPolygon2d> polygons) : polygons_(polygons)
    {
    }

    std::size_t SCMultiPolygon2d::Size() const
    {
        return Detail::ContainerCount(polygons_);
    }

    std::size_t SCMultiPolygon2d::Count() const
    {
        return Size();
    }

    bool SCMultiPolygon2d::IsEmpty() const
    {
        return Detail::ContainerIsEmpty(polygons_);
    }

    bool SCMultiPolygon2d::IsValid() const
    {
        return Detail::ContainerIsValid(polygons_);
    }

    bool SCMultiPolygon2d::IsValid(double eps) const
    {
        if (!(eps > 0.0) || !std::isfinite(eps))
        {
            return false;
        }
        return Detail::ContainerIsValid(polygons_, [eps](const SCPolygon2d& value) {
            return value.IsValid(eps);
        });
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
        return Detail::ContainerTotalCount(polygons_, [](const SCPolygon2d& value) {
            return value.PointCount();
        });
    }

    std::size_t SCMultiPolygon2d::SegmentCount() const
    {
        return Detail::ContainerTotalCount(polygons_, [](const SCPolygon2d& value) {
            return value.SegmentCount();
        });
    }

    std::size_t SCMultiPolygon2d::HoleCount() const
    {
        return Detail::ContainerTotalCount(polygons_, [](const SCPolygon2d& value) {
            return value.HoleCount();
        });
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

    SCBox2d SCMultiPolygon2d::Bounds(double eps) const
    {
        SCBox2d box;
        for (const SCPolygon2d& polygon : polygons_)
        {
            const SCBox2d polygonBounds = polygon.Bounds(eps);
            if (!polygonBounds.IsValid())
            {
                return {};
            }
            box.ExpandToInclude(polygonBounds);
        }
        return box;
    }

    std::string SCMultiPolygon2d::DebugString() const
    {
        std::ostringstream stream;
        stream << "SCMultiPolygon2d{count=" << Size() << ", pointCount=" << PointCount()
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
