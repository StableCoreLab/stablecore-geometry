#include "Geometry2d/SCMultiPolyline2d.h"

#include <sstream>
#include <utility>

#include "Core/ShapeOps.h"

namespace Geometry
{
    SCMultiPolyline2d::SCMultiPolyline2d(std::vector<SCPolyline2d> polylines) : polylines_(std::move(polylines))
    {
    }

    SCMultiPolyline2d::SCMultiPolyline2d(std::initializer_list<SCPolyline2d> polylines) : polylines_(polylines)
    {
    }

    std::size_t SCMultiPolyline2d::Count() const
    {
        return polylines_.size();
    }

    bool SCMultiPolyline2d::IsEmpty() const
    {
        return polylines_.empty();
    }

    bool SCMultiPolyline2d::IsValid() const
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
        std::size_t count = 0;
        for (const auto& polyline : polylines_)
        {
            count += polyline.PointCount();
        }
        return count;
    }

    std::size_t SCMultiPolyline2d::SegmentCount() const
    {
        std::size_t count = 0;
        for (const auto& polyline : polylines_)
        {
            count += polyline.SegmentCount();
        }
        return count;
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

    std::string SCMultiPolyline2d::DebugString() const
    {
        std::ostringstream stream;
        stream << "SCMultiPolyline2d{count=" << Count() << ", pointCount=" << PointCount()
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
