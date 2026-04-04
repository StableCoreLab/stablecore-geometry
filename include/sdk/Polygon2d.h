#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "export/GeometryExport.h"
#include "sdk/Polyline2d.h"
#include "sdk/GeometryTypes.h"

namespace geometry::sdk
{
class GEOMETRY_API Polygon2d
{
public:
    Polygon2d();
    explicit Polygon2d(Polyline2d outerRing);
    Polygon2d(Polyline2d outerRing, std::vector<Polyline2d> holes);
    Polygon2d(const Polygon2d& other);
    Polygon2d(Polygon2d&& other) noexcept;
    Polygon2d& operator=(const Polygon2d& other);
    Polygon2d& operator=(Polygon2d&& other) noexcept;
    ~Polygon2d();

    [[nodiscard]] bool IsValid() const;
    [[nodiscard]] std::size_t PointCount() const;
    [[nodiscard]] std::size_t SegmentCount() const;
    [[nodiscard]] std::size_t HoleCount() const;
    [[nodiscard]] Polyline2d OuterRing() const;
    [[nodiscard]] Polyline2d HoleAt(std::size_t index) const;
    [[nodiscard]] double Area() const;
    [[nodiscard]] Box2d Bounds() const;
    [[nodiscard]] std::string DebugString() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace geometry::sdk
