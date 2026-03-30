#pragma once

#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

#include "export/GeometryExport.h"
#include "sdk/GeometryTypes.h"

namespace geometry::sdk
{
class GEOMETRY_API PolyhedronLoop3d
{
public:
    PolyhedronLoop3d() = default;
    explicit PolyhedronLoop3d(std::vector<Point3d> vertices) : vertices_(std::move(vertices)) {}

    [[nodiscard]] bool IsValid(double eps = geometry::kDefaultEpsilon) const
    {
        if (vertices_.size() < 3)
        {
            return false;
        }

        for (const Point3d& vertex : vertices_)
        {
            if (!vertex.IsValid())
            {
                return false;
            }
        }

        for (std::size_t i = 0; i < vertices_.size(); ++i)
        {
            const Point3d& current = vertices_[i];
            const Point3d& next = vertices_[(i + 1) % vertices_.size()];
            if (current.AlmostEquals(next, eps))
            {
                return false;
            }
        }

        return true;
    }

    [[nodiscard]] std::size_t VertexCount() const
    {
        return vertices_.size();
    }

    [[nodiscard]] Point3d VertexAt(std::size_t index) const
    {
        return vertices_.at(index);
    }

    [[nodiscard]] const std::vector<Point3d>& Vertices() const
    {
        return vertices_;
    }

    [[nodiscard]] Box3d Bounds() const
    {
        Box3d bounds{};
        for (const Point3d& vertex : vertices_)
        {
            bounds.ExpandToInclude(vertex);
        }
        return bounds;
    }

    [[nodiscard]] std::string DebugString() const
    {
        std::ostringstream stream;
        stream << "PolyhedronLoop3d{vertexCount=" << VertexCount() << "}";
        return stream.str();
    }

private:
    std::vector<Point3d> vertices_{};
};
} // namespace geometry::sdk
