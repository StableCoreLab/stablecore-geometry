#pragma once
#include "Brep/SCBrepBody.h"
#include "Brep/PolyhedronBody.h"
#include "Export/GeometryExport.h"
#include "Support/Epsilon.h"

namespace Geometry
{
    enum class BrepConversionIssue3d
    {
        None,
        InvalidFace,
        InvalidBody,
        UnsupportedSurface,
        InvalidTrim
    };

    struct GEOMETRY_API BrepFaceConversion3d
    {
        bool success{false};
        BrepConversionIssue3d issue{BrepConversionIssue3d::None};
        PolyhedronFace3d face{};

        [[nodiscard]] bool IsValid(double eps = Geometry::kDefaultEpsilon) const
        {
            return !success || face.IsValid(eps);
        }
    };

    struct GEOMETRY_API BrepBodyConversion3d
    {
        bool success{false};
        BrepConversionIssue3d issue{BrepConversionIssue3d::None};
        std::size_t faceIndex{0};
        PolyhedronBody body{};

        [[nodiscard]] bool IsValid(double eps = Geometry::kDefaultEpsilon) const
        {
            return !success || body.IsValid(eps);
        }
    };

    struct GEOMETRY_API PolyhedronBrepBodyConversion3d
    {
        bool success{false};
        BrepConversionIssue3d issue{BrepConversionIssue3d::None};
        std::size_t faceIndex{0};
        SCBrepBody body{};

        [[nodiscard]] bool IsValid(const SCGeometryTolerance3d& tolerance = {}) const
        {
            return !success || body.IsValid(tolerance);
        }
    };

    [[nodiscard]] GEOMETRY_API BrepFaceConversion3d ConvertToPolyhedronFace(const SCBrepFace& face,
                                                                            double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API BrepBodyConversion3d ConvertToPolyhedronBody(const SCBrepBody& body,
                                                                            double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API PolyhedronBrepBodyConversion3d ConvertToBrepBody(const PolyhedronBody& body,
                                                                                double eps = Geometry::kDefaultEpsilon);
}  // namespace Geometry


