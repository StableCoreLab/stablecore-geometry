#pragma once

#include <array>
#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

#include "Core/GeometryTypes.h"
#include "Export/GeometryExport.h"

namespace Geometry
{
    class GEOMETRY_API TriangleMesh
    {
    public:
        using TriangleIndices = std::array<std::size_t, 3>;

        TriangleMesh() = default;
        TriangleMesh(std::vector<SCPoint3d> vertices, std::vector<TriangleIndices> triangles)
            : vertices_(std::move(vertices)), triangles_(std::move(triangles))
        {
        }

        [[nodiscard]] bool IsEmpty() const
        {
            return vertices_.empty() && triangles_.empty();
        }

        [[nodiscard]] std::size_t VertexCount() const
        {
            return vertices_.size();
        }

        [[nodiscard]] std::size_t TriangleCount() const
        {
            return triangles_.size();
        }

        [[nodiscard]] const std::vector<SCPoint3d>& Vertices() const
        {
            return vertices_;
        }

        [[nodiscard]] std::vector<SCPoint3d>& Vertices()
        {
            return vertices_;
        }

        [[nodiscard]] const std::vector<TriangleIndices>& Triangles() const
        {
            return triangles_;
        }

        [[nodiscard]] std::vector<TriangleIndices>& Triangles()
        {
            return triangles_;
        }

        [[nodiscard]] SCPoint3d VertexAt(std::size_t index) const
        {
            return vertices_.at(index);
        }

        [[nodiscard]] TriangleIndices TriangleIndicesAt(std::size_t index) const
        {
            return triangles_.at(index);
        }

        [[nodiscard]] SCTriangle3d TriangleAt(std::size_t index) const
        {
            const TriangleIndices tri = TriangleIndicesAt(index);
            return SCTriangle3d{vertices_.at(tri[0]), vertices_.at(tri[1]), vertices_.at(tri[2])};
        }

        [[nodiscard]] bool IsValid(double eps = Geometry::kDefaultEpsilon) const
        {
            for (const SCPoint3d& vertex : vertices_)
            {
                if (!vertex.IsValid())
                {
                    return false;
                }
            }

            for (const TriangleIndices& tri : triangles_)
            {
                if (tri[0] >= vertices_.size() || tri[1] >= vertices_.size() || tri[2] >= vertices_.size())
                {
                    return false;
                }

                if (tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2])
                {
                    return false;
                }

                if (SCTriangle3d{vertices_[tri[0]], vertices_[tri[1]], vertices_[tri[2]]}.IsDegenerate(eps))
                {
                    return false;
                }
            }

            return true;
        }

        [[nodiscard]] SCBox3d Bounds() const
        {
            SCBox3d bounds{};
            for (const SCPoint3d& vertex : vertices_)
            {
                bounds.ExpandToInclude(vertex);
            }
            return bounds;
        }

        [[nodiscard]] double SurfaceArea() const
        {
            double total = 0.0;
            for (std::size_t i = 0; i < TriangleCount(); ++i)
            {
                total += TriangleAt(i).Area();
            }
            return total;
        }

        [[nodiscard]] TriangleMesh Transformed(const SCTransform3d& transform) const
        {
            std::vector<SCPoint3d> transformedVertices;
            transformedVertices.reserve(vertices_.size());
            for (const SCPoint3d& vertex : vertices_)
            {
                transformedVertices.push_back(transform.Apply(vertex));
            }
            return TriangleMesh(std::move(transformedVertices), triangles_);
        }

        [[nodiscard]] std::string DebugString() const
        {
            std::ostringstream stream;
            stream << "TriangleMesh{vertexCount=" << VertexCount() << ", triangleCount=" << TriangleCount() << "}";
            return stream.str();
        }

    private:
        std::vector<SCPoint3d> vertices_{};
        std::vector<TriangleIndices> triangles_{};
    };
}  // namespace Geometry
