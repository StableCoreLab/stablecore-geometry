#pragma once

#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

#include "export/GeometryExport.h"
#include "sdk/PolyhedronFace3d.h"

namespace geometry::sdk
{
class GEOMETRY_API PolyhedronBody
{
public:
    PolyhedronBody() = default;
    explicit PolyhedronBody(std::vector<PolyhedronFace3d> faces) : faces_(std::move(faces)) {}

    [[nodiscard]] bool IsEmpty() const
    {
        return faces_.empty();
    }

    [[nodiscard]] bool IsValid(double eps = geometry::kDefaultEpsilon) const
    {
        if (faces_.empty())
        {
            return false;
        }

        for (const PolyhedronFace3d& face : faces_)
        {
            if (!face.IsValid(eps))
            {
                return false;
            }
        }

        return true;
    }

    [[nodiscard]] std::size_t FaceCount() const
    {
        return faces_.size();
    }

    [[nodiscard]] PolyhedronFace3d FaceAt(std::size_t index) const
    {
        return faces_.at(index);
    }

    [[nodiscard]] const std::vector<PolyhedronFace3d>& Faces() const
    {
        return faces_;
    }

    [[nodiscard]] Box3d Bounds() const
    {
        Box3d bounds{};
        for (const PolyhedronFace3d& face : faces_)
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
        stream << "PolyhedronBody{faceCount=" << FaceCount() << "}";
        return stream.str();
    }

private:
    std::vector<PolyhedronFace3d> faces_{};
};
} // namespace geometry::sdk
