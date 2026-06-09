#pragma once
#include <sstream>
#include <string>
#include <vector>

#include "Support/Epsilon.h"
#include "Export/GeometryExport.h"
#include "Types/Geometry3d/SCBox3.h"
#include "Types/Geometry3d/SCPoint3.h"

namespace Geometry
{
    class GEOMETRY_API PolyhedronLoop3d
    {
    public:
        PolyhedronLoop3d() = default;
        explicit PolyhedronLoop3d(std::vector<SCPoint3d> vertices) : vertices_(std::move(vertices))
        {
        }

        [[nodiscard]] bool IsValid(double eps = Geometry::kDefaultEpsilon) const
        {
            if (vertices_.size() < 3)
            {
                return false;
            }

            for (const SCPoint3d& vertex : vertices_)
            {
                if (!vertex.IsValid())
                {
                    return false;
                }
            }

            for (std::size_t i = 0; i < vertices_.size(); ++i)
            {
                const SCPoint3d& current = vertices_[i];
                const SCPoint3d& next = vertices_[(i + 1) % vertices_.size()];
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

        [[nodiscard]] SCPoint3d VertexAt(std::size_t index) const
        {
            return vertices_.at(index);
        }

        [[nodiscard]] const std::vector<SCPoint3d>& Vertices() const
        {
            return vertices_;
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

        [[nodiscard]] std::string DebugString() const
        {
            std::ostringstream stream;
            stream << "PolyhedronLoop3d{vertexCount=" << VertexCount() << "}";
            return stream.str();
        }

    private:
        std::vector<SCPoint3d> vertices_{};
    };
}  // namespace Geometry
