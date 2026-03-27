#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "export/GeometryExport.h"
#include "sdk/GeometryShapeOps.h"
#include "sdk/MultiPolygon2d.h"

namespace geometry::sdk
{
enum class PolygonContainment2d
{
    Disjoint,
    Touching,
    Intersecting,
    FirstContainsSecond,
    SecondContainsFirst,
    Equal
};

struct GEOMETRY_API PolygonRelation2d
{
    std::size_t firstIndex{0};
    std::size_t secondIndex{0};
    PolygonContainment2d relation{PolygonContainment2d::Disjoint};
};

struct GEOMETRY_API PolygonTopologyNode2d
{
    std::size_t polygonIndex{0};
    std::size_t parentIndex{static_cast<std::size_t>(-1)};
    std::vector<std::size_t> children{};
};

class GEOMETRY_API PolygonTopology2d
{
public:
    PolygonTopology2d() = default;
    explicit PolygonTopology2d(const MultiPolygon2d& polygons, double eps = 1e-9);

    bool Build(const MultiPolygon2d& polygons, double eps = 1e-9);

    [[nodiscard]] std::size_t Count() const;
    [[nodiscard]] bool IsEmpty() const;
    [[nodiscard]] bool IsValid() const;

    [[nodiscard]] const MultiPolygon2d& Polygons() const;
    [[nodiscard]] const PolygonTopologyNode2d& Node(std::size_t index) const;
    [[nodiscard]] const std::vector<std::size_t>& Roots() const;
    [[nodiscard]] const std::vector<std::size_t>& ChildrenOf(std::size_t index) const;
    [[nodiscard]] std::size_t ParentOf(std::size_t index) const;
    [[nodiscard]] std::string DebugString() const;

private:
    MultiPolygon2d polygons_{};
    std::vector<PolygonTopologyNode2d> nodes_{};
    std::vector<std::size_t> roots_{};
};

[[nodiscard]] GEOMETRY_API bool ContainsPoint(const Polyline2d& ring, const Point2d& point, double eps = 1e-9);
[[nodiscard]] GEOMETRY_API bool ContainsPoint(const Polygon2d& polygon, const Point2d& point, double eps = 1e-9);
[[nodiscard]] GEOMETRY_API bool Contains(const Polygon2d& outer, const Polygon2d& inner, double eps = 1e-9);
[[nodiscard]] GEOMETRY_API PolygonContainment2d Relate(
    const Polygon2d& first,
    const Polygon2d& second,
    double eps = 1e-9);
[[nodiscard]] GEOMETRY_API PolygonTopology2d BuildPolygonTopology(
    const MultiPolygon2d& polygons,
    double eps = 1e-9);
} // namespace geometry::sdk

