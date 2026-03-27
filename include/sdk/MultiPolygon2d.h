#pragma once

#include <cstddef>
#include <initializer_list>
#include <string>
#include <vector>

#include "export/GeometryExport.h"
#include "sdk/Polygon2d.h"

namespace geometry::sdk
{
class GEOMETRY_API MultiPolygon2d
{
public:
    MultiPolygon2d() = default;
    explicit MultiPolygon2d(std::vector<Polygon2d> polygons);
    MultiPolygon2d(std::initializer_list<Polygon2d> polygons);

    [[nodiscard]] std::size_t Count() const;
    [[nodiscard]] bool IsEmpty() const;
    [[nodiscard]] bool IsValid() const;

    void Clear();
    void Add(Polygon2d polygon);

    [[nodiscard]] const Polygon2d& PolygonAt(std::size_t index) const;
    [[nodiscard]] Polygon2d& PolygonAt(std::size_t index);

    [[nodiscard]] const Polygon2d& operator[](std::size_t index) const;
    [[nodiscard]] Polygon2d& operator[](std::size_t index);

    [[nodiscard]] std::size_t PointCount() const;
    [[nodiscard]] std::size_t SegmentCount() const;
    [[nodiscard]] std::size_t HoleCount() const;
    [[nodiscard]] Box2d Bounds() const;
    [[nodiscard]] std::string DebugString() const;

    [[nodiscard]] const std::vector<Polygon2d>& Data() const;
    [[nodiscard]] std::vector<Polygon2d>& Data();

private:
    std::vector<Polygon2d> polygons_{};
};
} // namespace geometry::sdk

