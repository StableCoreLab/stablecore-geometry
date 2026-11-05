#include "Geometry2d/SCMultiPolyline2d.h"

#include <cmath>
#include <sstream>
#include <utility>

#include "../Detail/GeometryContainerOps.h"

namespace Geometry
{
    SCMultiPolyline2d::SCMultiPolyline2d(std::vector<SCPolyline2d> polylines) : polylines_(std::move(polylines))
    {
    }

    SCMultiPolyline2d::SCMultiPolyline2d(std::initializer_list<SCPolyline2d> polylines) : polylines_(polylines)
    {
    }

    std::size_t SCMultiPolyline2d::Size() const
    {
        return Detail::ContainerCount(polylines_);
    }

    std::size_t SCMultiPolyline2d::Count() const
    {
        return Size();
    }

    bool SCMultiPolyline2d::IsEmpty() const
    {
        return Detail::ContainerIsEmpty(polylines_);
    }

    bool SCMultiPolyline2d::IsValid() const
    {
        return Detail::ContainerIsValid(polylines_);
    }

    bool SCMultiPolyline2d::IsValid(double eps) const
    {
        if (!(eps > 0.0) || !std::isfinite(eps))
        {
            return false;
        }
        return Detail::ContainerIsValid(polylines_, [eps](const SCPolyline2d& value) {
            return value.IsValid(eps);
        });
    }

    void SCMultiPolyline2d::Clear()
    {
        polylines_.clear();
    }

    void SCMultiPolyline2d::Add(SCPolyline2d polyline)
    {
        polylines_.push_back(std::move(polyline));
    }

    const SCPolyline2d& SCMultiPolyline2d::PolylineAt(std::size_t index) const
    {
        return polylines_.at(index);
    }

    SCPolyline2d& SCMultiPolyline2d::PolylineAt(std::size_t index)
    {
        return polylines_.at(index);
    }

    const SCPolyline2d& SCMultiPolyline2d::operator[](std::size_t index) const
    {
        return polylines_[index];
    }

    SCPolyline2d& SCMultiPolyline2d::operator[](std::size_t index)
    {
        return polylines_[index];
    }

    std::size_t SCMultiPolyline2d::PointCount() const
    {
        return Detail::ContainerTotalCount(polylines_, [](const SCPolyline2d& value) {
            return value.PointCount();
        });
    }

    std::size_t SCMultiPolyline2d::SegmentCount() const
    {
        return Detail::ContainerTotalCount(polylines_, [](const SCPolyline2d& value) {
            return value.SegmentCount();
        });
    }

    SCBox2d SCMultiPolyline2d::Bounds() const
    {
        SCBox2d box;
        for (const auto& polyline : polylines_)
        {
            box.ExpandToInclude(polyline.Bounds());
        }
        return box;
    }

    SCBox2d SCMultiPolyline2d::Bounds(double eps) const
    {
        SCBox2d box;
        for (const SCPolyline2d& polyline : polylines_)
        {
            const SCBox2d bounds = polyline.Bounds(eps);
            if (!bounds.IsValid())
            {
                return {};
            }
            box.ExpandToInclude(bounds);
        }
        return box;
    }

    std::string SCMultiPolyline2d::DebugString() const
    {
        std::ostringstream stream;
        stream << "SCMultiPolyline2d{count=" << Size() << ", pointCount=" << PointCount()
               << ", segmentCount=" << SegmentCount() << ", valid=" << (IsValid() ? "true" : "false")
               << ", bounds=" << Bounds().DebugString() << "}";
        return stream.str();
    }

    const std::vector<SCPolyline2d>& SCMultiPolyline2d::Polylines() const
    {
        return polylines_;
    }

    std::vector<SCPolyline2d>& SCMultiPolyline2d::Polylines()
    {
        return polylines_;
    }

    const std::vector<SCPolyline2d>& SCMultiPolyline2d::Data() const
    {
        return Polylines();
    }

    std::vector<SCPolyline2d>& SCMultiPolyline2d::Data()
    {
        return Polylines();
    }
}  // namespace Geometry
