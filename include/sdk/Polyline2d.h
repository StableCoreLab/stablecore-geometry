#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "export/GeometryExport.h"
#include "sdk/GeometryTypes.h"

namespace geometry::sdk
{
enum class PolylineClosure
{
    Open,
    Closed
};

class GEOMETRY_API Polyline2d
{
public:
    Polyline2d();
    explicit Polyline2d(PolylineClosure closure);
    Polyline2d(std::vector<Point2d> points, PolylineClosure closure = PolylineClosure::Open);
    Polyline2d(const Polyline2d& other);
    Polyline2d(Polyline2d&& other) noexcept;
    Polyline2d& operator=(const Polyline2d& other);
    Polyline2d& operator=(Polyline2d&& other) noexcept;
    ~Polyline2d();

    [[nodiscard]] bool IsValid() const;
    [[nodiscard]] bool IsClosed() const;
    [[nodiscard]] std::size_t PointCount() const;
    [[nodiscard]] std::size_t SegmentCount() const;
    [[nodiscard]] double Length() const;
    [[nodiscard]] Point2d PointAt(std::size_t index) const;
    [[nodiscard]] Box2d Bounds() const;
    [[nodiscard]] std::string DebugString() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    explicit Polyline2d(std::unique_ptr<Impl> impl);

};
} // namespace geometry::sdk
