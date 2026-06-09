#pragma once
#include "Brep/SCBrepBody.h"
#include "Brep/PolyhedronBody.h"
#include "Export/GeometryExport.h"
#include "Geometry3d/TriangleMesh.h"
#include "Support/Epsilon.h"

namespace Geometry
{
    enum class MeshConversionIssue3d
    {
        None,
        InvalidFace,
        InvalidBody,
        UnsupportedSurface,
        UnsupportedHoles,
        TriangulationFailed
    };

    struct GEOMETRY_API PolyhedronMeshConversion3d
    {
        bool success{false};
        MeshConversionIssue3d issue{MeshConversionIssue3d::None};
        std::size_t faceIndex{0};
        TriangleMesh mesh{};

        [[nodiscard]] bool IsValid() const
        {
            return !success || mesh.IsValid();
        }
    };

    [[nodiscard]] GEOMETRY_API PolyhedronMeshConversion3d ConvertToTriangleMesh(const PolyhedronFace3d& face,
                                                                                double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API PolyhedronMeshConversion3d ConvertToTriangleMesh(const PolyhedronBody& body,
                                                                                double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API PolyhedronMeshConversion3d ConvertToTriangleMesh(const SCBrepFace& face,
                                                                                double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API PolyhedronMeshConversion3d ConvertToTriangleMesh(const SCBrepBody& body,
                                                                                double eps = Geometry::kDefaultEpsilon);
}  // namespace Geometry

