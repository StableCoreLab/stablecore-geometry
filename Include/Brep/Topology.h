#pragma once
#include <string>
#include <vector>
#include "Core/ShapeOps.h"
#include "Export/GeometryExport.h"
#include "Geometry2d/SCMultiPolygon2d.h"
#include "Support/Epsilon.h"

namespace Geometry
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
        explicit PolygonTopology2d(const SCMultiPolygon2d& polygons, double eps = Geometry::kDefaultEpsilon);

        bool Build(const SCMultiPolygon2d& polygons, double eps = Geometry::kDefaultEpsilon);

        [[nodiscard]] std::size_t Count() const;
        [[nodiscard]] bool IsEmpty() const;
        [[nodiscard]] bool IsValid() const;

        [[nodiscard]] const SCMultiPolygon2d& Polygons() const;
        [[nodiscard]] const PolygonTopologyNode2d& Node(std::size_t index) const;
        [[nodiscard]] const std::vector<std::size_t>& Roots() const;
        [[nodiscard]] const std::vector<std::size_t>& ChildrenOf(std::size_t index) const;
        [[nodiscard]] std::size_t ParentOf(std::size_t index) const;
        [[nodiscard]] std::string DebugString() const;

    private:
        SCMultiPolygon2d polygons_{};
        std::vector<PolygonTopologyNode2d> nodes_{};
        std::vector<std::size_t> roots_{};
    };

    [[nodiscard]] GEOMETRY_API bool ContainsPoint(const SCPolyline2d& ring,
                                                  const SCPoint2d& point,
                                                  double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API bool ContainsPoint(const SCPolygon2d& polygon,
                                                  const SCPoint2d& point,
                                                  double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API bool Contains(const SCPolygon2d& outer,
                                             const SCPolygon2d& inner,
                                             double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API PolygonContainment2d Relate(const SCPolygon2d& first,
                                                           const SCPolygon2d& second,
                                                           double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API PolygonTopology2d BuildPolygonTopology(const SCMultiPolygon2d& polygons,
                                                                      double eps = Geometry::kDefaultEpsilon);
}  // namespace Geometry
