#pragma once
#include <memory>
#include <string>
#include <vector>
#include "Export/GeometryExport.h"
#include "Geometry2d/SCPolyline2d.h"

namespace Geometry
{
    class GEOMETRY_API SCPolygon2d
    {
    public:
        SCPolygon2d();
        explicit SCPolygon2d(SCPolyline2d outerRing);
        SCPolygon2d(SCPolyline2d outerRing, std::vector<SCPolyline2d> holes);
        SCPolygon2d(const SCPolygon2d& other);
        SCPolygon2d(SCPolygon2d&& other) noexcept;
        SCPolygon2d& operator=(const SCPolygon2d& other);
        SCPolygon2d& operator=(SCPolygon2d&& other) noexcept;
        ~SCPolygon2d();

        [[nodiscard]] bool IsValid() const;
        [[nodiscard]] std::size_t PointCount() const;
        [[nodiscard]] std::size_t SegmentCount() const;
        [[nodiscard]] std::size_t HoleCount() const;
        [[nodiscard]] SCPolyline2d OuterRing() const;
        [[nodiscard]] SCPolyline2d HoleAt(std::size_t index) const;
        [[nodiscard]] double Area() const;
        [[nodiscard]] double Perimeter() const;
        [[nodiscard]] SCPoint2d Centroid() const;
        [[nodiscard]] SCBox2d Bounds() const;
        [[nodiscard]] std::string DebugString() const;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}  // namespace Geometry
