#pragma once

#include <cstddef>
#include <initializer_list>
#include <string>
#include <vector>

#include "export/GeometryExport.h"
#include "sdk/Polyline2d.h"

namespace geometry::sdk
{
class GEOMETRY_API MultiPolyline2d
{
public:
    MultiPolyline2d() = default;
    explicit MultiPolyline2d(std::vector<Polyline2d> polylines);
    MultiPolyline2d(std::initializer_list<Polyline2d> polylines);

    [[nodiscard]] std::size_t Count() const;
    [[nodiscard]] std::size_t PolylineCount() const;
    [[nodiscard]] bool IsEmpty() const;
    [[nodiscard]] bool IsValid() const;

    void Clear();
    void Add(Polyline2d polyline);

    [[nodiscard]] const Polyline2d& PolylineAt(std::size_t index) const;
    [[nodiscard]] Polyline2d& PolylineAt(std::size_t index);

    [[nodiscard]] const Polyline2d& operator[](std::size_t index) const;
    [[nodiscard]] Polyline2d& operator[](std::size_t index);

    [[nodiscard]] std::size_t PointCount() const;
    [[nodiscard]] std::size_t SegmentCount() const;
    [[nodiscard]] Box2d Bounds() const;
    [[nodiscard]] std::string DebugString() const;

    [[nodiscard]] const std::vector<Polyline2d>& Data() const;
    [[nodiscard]] std::vector<Polyline2d>& Data();

private:
    std::vector<Polyline2d> polylines_{};
};
} // namespace geometry::sdk
