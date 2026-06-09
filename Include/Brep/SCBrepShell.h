#pragma once

#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

#include "Core/GeometryTypesPrimitives.h"
#include "Brep/SCBrepFace.h"
#include "Export/GeometryExport.h"
#include "Types/Geometry3d/SCBox3.h"

namespace Geometry
{
    class GEOMETRY_API SCBrepShell
    {
    public:
        SCBrepShell() = default;
        explicit SCBrepShell(std::vector<SCBrepFace> faces, bool closed = false) : faces_(std::move(faces)), closed_(closed)
        {
        }

        [[nodiscard]] bool IsValid(const SCGeometryTolerance3d& tolerance = {}) const
        {
            if (faces_.empty())
            {
                return false;
            }

            for (const SCBrepFace& face : faces_)
            {
                if (!face.IsValid(tolerance))
                {
                    return false;
                }
            }

            return true;
        }

        [[nodiscard]] bool IsClosed() const
        {
            return closed_;
        }

        [[nodiscard]] std::size_t FaceCount() const
        {
            return faces_.size();
        }

        [[nodiscard]] SCBrepFace FaceAt(std::size_t index) const
        {
            return faces_.at(index);
        }

        [[nodiscard]] const std::vector<SCBrepFace>& Faces() const
        {
            return faces_;
        }

        [[nodiscard]] SCBox3d Bounds() const
        {
            SCBox3d bounds{};
            for (const SCBrepFace& face : faces_)
            {
                const SCBox3d faceBounds = face.Bounds();
                if (!faceBounds.IsValid())
                {
                    continue;
                }

                bounds.ExpandToInclude(faceBounds.MinPoint());
                bounds.ExpandToInclude(faceBounds.MaxPoint());
            }
            return bounds;
        }

        [[nodiscard]] std::string DebugString() const
        {
            std::ostringstream stream;
            stream << "SCBrepShell{faceCount=" << FaceCount() << ", closed=" << (closed_ ? "true" : "false") << "}";
            return stream.str();
        }

    private:
        std::vector<SCBrepFace> faces_{};
        bool closed_{false};
    };
}  // namespace Geometry


