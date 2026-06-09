#pragma once

#include <initializer_list>
#include <string>
#include <vector>

#include "Export/GeometryExport.h"
#include "Geometry2d/SCPolyline2d.h"

namespace Geometry
{
    class GEOMETRY_API SCMultiPolyline2d
    {
    public:
        SCMultiPolyline2d() = default;
        explicit SCMultiPolyline2d(std::vector<SCPolyline2d> polylines);
        SCMultiPolyline2d(std::initializer_list<SCPolyline2d> polylines);

        [[nodiscard]] std::size_t Count() const;
        [[nodiscard]] bool IsEmpty() const;
        [[nodiscard]] bool IsValid() const;

        void Clear();
        void Add(SCPolyline2d polyline);

        [[nodiscard]] const SCPolyline2d& PolylineAt(std::size_t index) const;
        [[nodiscard]] SCPolyline2d& PolylineAt(std::size_t index);

        [[nodiscard]] const SCPolyline2d& operator[](std::size_t index) const;
        [[nodiscard]] SCPolyline2d& operator[](std::size_t index);

        [[nodiscard]] std::size_t PointCount() const;
        [[nodiscard]] std::size_t SegmentCount() const;
        [[nodiscard]] SCBox2d Bounds() const;
        [[nodiscard]] std::string DebugString() const;

        [[nodiscard]] const std::vector<SCPolyline2d>& Polylines() const;
        [[nodiscard]] std::vector<SCPolyline2d>& Polylines();
        [[nodiscard]] const std::vector<SCPolyline2d>& Data() const;
        [[nodiscard]] std::vector<SCPolyline2d>& Data();

    private:
        std::vector<SCPolyline2d> polylines_{};
    };
}  // namespace Geometry
