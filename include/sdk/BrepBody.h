#pragma once

#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

#include "export/GeometryExport.h"
#include "sdk/BrepEdge.h"
#include "sdk/BrepShell.h"
#include "sdk/BrepVertex.h"

namespace geometry::sdk
{
class GEOMETRY_API BrepBody
{
public:
    BrepBody() = default;
    BrepBody(std::vector<BrepVertex> vertices, std::vector<BrepEdge> edges, std::vector<BrepShell> shells)
        : vertices_(std::move(vertices)), edges_(std::move(edges)), shells_(std::move(shells))
    {
    }

    [[nodiscard]] bool IsEmpty() const
    {
        return vertices_.empty() && edges_.empty() && shells_.empty();
    }

    [[nodiscard]] bool IsValid(const GeometryTolerance3d& tolerance = {}) const
    {
        if (shells_.empty())
        {
            return false;
        }

        for (const BrepVertex& vertex : vertices_)
        {
            if (!vertex.IsValid())
            {
                return false;
            }
        }

        for (const BrepEdge& edge : edges_)
        {
            if (!edge.IsValid(tolerance) || edge.StartVertexIndex() >= vertices_.size() ||
                edge.EndVertexIndex() >= vertices_.size())
            {
                return false;
            }
        }

        for (const BrepShell& shell : shells_)
        {
            if (!shell.IsValid(tolerance))
            {
                return false;
            }

            for (const BrepFace& face : shell.Faces())
            {
                if (!LoopReferencesAreValid(face.OuterLoop()))
                {
                    return false;
                }

                for (const BrepLoop& hole : face.HoleLoops())
                {
                    if (!LoopReferencesAreValid(hole))
                    {
                        return false;
                    }
                }
            }
        }

        return true;
    }

    [[nodiscard]] std::size_t VertexCount() const
    {
        return vertices_.size();
    }

    [[nodiscard]] std::size_t EdgeCount() const
    {
        return edges_.size();
    }

    [[nodiscard]] std::size_t ShellCount() const
    {
        return shells_.size();
    }

    [[nodiscard]] std::size_t FaceCount() const
    {
        std::size_t total = 0;
        for (const BrepShell& shell : shells_)
        {
            total += shell.FaceCount();
        }
        return total;
    }

    [[nodiscard]] BrepVertex VertexAt(std::size_t index) const
    {
        return vertices_.at(index);
    }

    [[nodiscard]] BrepEdge EdgeAt(std::size_t index) const
    {
        return edges_.at(index);
    }

    [[nodiscard]] BrepShell ShellAt(std::size_t index) const
    {
        return shells_.at(index);
    }

    [[nodiscard]] const std::vector<BrepVertex>& Vertices() const
    {
        return vertices_;
    }

    [[nodiscard]] const std::vector<BrepEdge>& Edges() const
    {
        return edges_;
    }

    [[nodiscard]] const std::vector<BrepShell>& Shells() const
    {
        return shells_;
    }

    [[nodiscard]] Box3d Bounds() const
    {
        Box3d bounds{};
        for (const BrepVertex& vertex : vertices_)
        {
            bounds.ExpandToInclude(vertex.Point());
        }
        for (const BrepShell& shell : shells_)
        {
            const Box3d shellBounds = shell.Bounds();
            if (!shellBounds.IsValid())
            {
                continue;
            }

            bounds.ExpandToInclude(shellBounds.MinPoint());
            bounds.ExpandToInclude(shellBounds.MaxPoint());
        }
        return bounds;
    }

    [[nodiscard]] std::string DebugString() const
    {
        std::ostringstream stream;
        stream << "BrepBody{vertexCount=" << VertexCount()
               << ", edgeCount=" << EdgeCount()
               << ", shellCount=" << ShellCount() << "}";
        return stream.str();
    }

private:
    [[nodiscard]] bool LoopReferencesAreValid(const BrepLoop& loop) const
    {
        for (const BrepCoedge& coedge : loop.Coedges())
        {
            if (coedge.EdgeIndex() >= edges_.size())
            {
                return false;
            }
        }

        return true;
    }

    std::vector<BrepVertex> vertices_{};
    std::vector<BrepEdge> edges_{};
    std::vector<BrepShell> shells_{};
};
} // namespace geometry::sdk
