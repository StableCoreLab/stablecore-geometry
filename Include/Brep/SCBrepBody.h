#pragma once

#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

#include "Brep/SCBrepEdge.h"
#include "Brep/SCBrepFace.h"
#include "Brep/SCBrepShell.h"
#include "Brep/SCBrepVertex.h"
#include "Types/Geometry3d/SCGeometryTypes3d.h"
#include "Export/GeometryExport.h"

namespace Geometry
{
    class GEOMETRY_API SCBrepBody
    {
    public:
        SCBrepBody() = default;
        SCBrepBody(std::vector<SCBrepVertex> vertices, std::vector<SCBrepEdge> edges, std::vector<SCBrepShell> shells)
            : vertices_(std::move(vertices)), edges_(std::move(edges)), shells_(std::move(shells))
        {
        }

        [[nodiscard]] bool IsEmpty() const
        {
            return vertices_.empty() && edges_.empty() && shells_.empty();
        }

        [[nodiscard]] bool IsValid(const SCGeometryTolerance3d& tolerance = {}) const
        {
            if (shells_.empty())
            {
                return false;
            }

            for (const SCBrepVertex& vertex : vertices_)
            {
                if (!vertex.IsValid())
                {
                    return false;
                }
            }

            for (const SCBrepEdge& edge : edges_)
            {
                if (!edge.IsValid(tolerance) || edge.StartVertexIndex() >= vertices_.size() ||
                    edge.EndVertexIndex() >= vertices_.size())
                {
                    return false;
                }
            }

            for (const SCBrepShell& shell : shells_)
            {
                if (!shell.IsValid(tolerance))
                {
                    return false;
                }

                for (const SCBrepFace& face : shell.Faces())
                {
                    if (!LoopReferencesAreValid(face.OuterLoopRef()))
                    {
                        return false;
                    }

                    for (const SCBrepLoop& hole : face.HoleLoops())
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

        [[deprecated("Use IsValid(const SCGeometryTolerance3d&)")]]
        [[nodiscard]] bool IsValid(double eps) const
        {
            SCGeometryTolerance3d tolerance;
            tolerance.distanceEpsilon = eps;
            return IsValid(tolerance);
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
            for (const SCBrepShell& shell : shells_)
            {
                total += shell.FaceCount();
            }
            return total;
        }

        [[nodiscard]] SCBrepVertex VertexAt(std::size_t index) const
        {
            return vertices_.at(index);
        }

        [[nodiscard]] const SCBrepVertex& VertexAtRef(std::size_t index) const
        {
            return vertices_.at(index);
        }

        [[nodiscard]] SCBrepEdge EdgeAt(std::size_t index) const
        {
            return edges_.at(index);
        }

        [[nodiscard]] const SCBrepEdge& EdgeAtRef(std::size_t index) const
        {
            return edges_.at(index);
        }

        [[nodiscard]] SCBrepShell ShellAt(std::size_t index) const
        {
            return shells_.at(index);
        }

        [[nodiscard]] const SCBrepShell& ShellAtRef(std::size_t index) const
        {
            return shells_.at(index);
        }

        [[nodiscard]] const std::vector<SCBrepVertex>& Vertices() const
        {
            return vertices_;
        }

        [[nodiscard]] const std::vector<SCBrepEdge>& Edges() const
        {
            return edges_;
        }

        [[nodiscard]] const std::vector<SCBrepShell>& Shells() const
        {
            return shells_;
        }

        [[nodiscard]] SCBox3d Bounds() const
        {
            SCBox3d bounds{};
            for (const SCBrepVertex& vertex : vertices_)
            {
                bounds.ExpandToInclude(vertex.Point());
            }
            for (const SCBrepShell& shell : shells_)
            {
                const SCBox3d shellBounds = shell.Bounds();
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
            stream << "SCBrepBody{vertexCount=" << VertexCount() << ", edgeCount=" << EdgeCount()
                   << ", shellCount=" << ShellCount() << "}";
            return stream.str();
        }

    private:
        [[nodiscard]] bool LoopReferencesAreValid(const SCBrepLoop& loop) const
        {
            for (const SCBrepCoedge& coedge : loop.Coedges())
            {
                if (coedge.EdgeIndex() >= edges_.size())
                {
                    return false;
                }
            }

            return true;
        }

        std::vector<SCBrepVertex> vertices_{};
        std::vector<SCBrepEdge> edges_{};
        std::vector<SCBrepShell> shells_{};
    };
}  // namespace Geometry


