#pragma once

#include <cstddef>
#include <initializer_list>
#include <string>
#include <vector>

#include "Export/GeometryExport.h"
#include "Geometry2d/SCPolygon2d.h"

namespace Geometry
{
    class GEOMETRY_API SCMultiPolygon2d
    {
    public:
        SCMultiPolygon2d() = default;
        explicit SCMultiPolygon2d(std::vector<SCPolygon2d> polygons);
        SCMultiPolygon2d(std::initializer_list<SCPolygon2d> polygons);

        [[nodiscard]] std::size_t Count() const;
        [[nodiscard]] bool IsEmpty() const;
        [[nodiscard]] bool IsValid() const;

        void Clear();
        void Add(SCPolygon2d polygon);

        [[nodiscard]] const SCPolygon2d& PolygonAt(std::size_t index) const;
        [[nodiscard]] SCPolygon2d& PolygonAt(std::size_t index);

        [[nodiscard]] const SCPolygon2d& operator[](std::size_t index) const;
        [[nodiscard]] SCPolygon2d& operator[](std::size_t index);

        [[nodiscard]] std::size_t PointCount() const;
        [[nodiscard]] std::size_t SegmentCount() const;
        [[nodiscard]] std::size_t HoleCount() const;
        [[nodiscard]] SCBox2d Bounds() const;
        [[nodiscard]] std::string DebugString() const;

        [[nodiscard]] const std::vector<SCPolygon2d>& Polygons() const;
        [[nodiscard]] std::vector<SCPolygon2d>& Polygons();
        [[nodiscard]] const std::vector<SCPolygon2d>& Data() const;
        [[nodiscard]] std::vector<SCPolygon2d>& Data();

    private:
        std::vector<SCPolygon2d> polygons_{};
    };
}  // namespace Geometry
