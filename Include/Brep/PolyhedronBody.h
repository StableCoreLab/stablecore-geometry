#pragma once

#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

#include "Types/Geometry3d/SCGeometryTypes3d.h"
#include "Brep/PolyhedronFace3d.h"
#include "Export/GeometryExport.h"
#include "Types/Geometry3d/SCBox3.h"

namespace Geometry
{
    class GEOMETRY_API PolyhedronBody
    {
    public:
        PolyhedronBody() = default;
        explicit PolyhedronBody(std::vector<PolyhedronFace3d> faces) : faces_(std::move(faces))
        {
        }

        [[nodiscard]] bool IsEmpty() const
        {
            return faces_.empty();
        }

        [[deprecated("Use IsValid(const SCGeometryTolerance3d&)")]]
        [[nodiscard]] bool IsValid(double eps) const
        {
            SCGeometryTolerance3d tolerance;
            tolerance.distanceEpsilon = eps;
            return IsValid(tolerance);
        }

        [[nodiscard]] bool IsValid(const SCGeometryTolerance3d& tolerance = {}) const
        {
            if (!tolerance.IsValid() || faces_.empty())
            {
                return false;
            }
            for (const PolyhedronFace3d& face : faces_)
            {
                if (!face.IsValid(tolerance))
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

        [[nodiscard]] const PolyhedronFace3d& FaceAtRef(std::size_t index) const
        {
            return faces_.at(index);
        }

        [[nodiscard]] const std::vector<PolyhedronFace3d>& Faces() const
        {
            return faces_;
        }

        [[nodiscard]] SCBox3d Bounds() const
        {
            SCBox3d bounds{};
            for (const PolyhedronFace3d& face : faces_)
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
            stream << "PolyhedronBody{faceCount=" << FaceCount() << "}";
            return stream.str();
        }

    private:
        std::vector<PolyhedronFace3d> faces_{};
    };
}  // namespace Geometry
