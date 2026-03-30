#pragma once

#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

#include "export/GeometryExport.h"
#include "sdk/BrepFace.h"

namespace geometry::sdk
{
class GEOMETRY_API BrepShell
{
public:
    BrepShell() = default;
    explicit BrepShell(std::vector<BrepFace> faces, bool closed = false)
        : faces_(std::move(faces)), closed_(closed)
    {
    }

    [[nodiscard]] bool IsValid(const GeometryTolerance3d& tolerance = {}) const
    {
        if (faces_.empty())
        {
            return false;
        }

        for (const BrepFace& face : faces_)
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

    [[nodiscard]] BrepFace FaceAt(std::size_t index) const
    {
        return faces_.at(index);
    }

    [[nodiscard]] const std::vector<BrepFace>& Faces() const
    {
        return faces_;
    }

    [[nodiscard]] Box3d Bounds() const
    {
        Box3d bounds{};
        for (const BrepFace& face : faces_)
        {
            const Box3d faceBounds = face.Bounds();
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
        stream << "BrepShell{faceCount=" << FaceCount()
               << ", closed=" << (closed_ ? "true" : "false") << "}";
        return stream.str();
    }

private:
    std::vector<BrepFace> faces_{};
    bool closed_{false};
};
} // namespace geometry::sdk
