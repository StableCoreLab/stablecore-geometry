#pragma once

#include <cstddef>

#include "sdk/BrepBody.h"
#include "export/GeometryExport.h"
#include "sdk/PolyhedronBody.h"
#include "sdk/TriangleMesh.h"

namespace geometry::sdk
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

[[nodiscard]] GEOMETRY_API PolyhedronMeshConversion3d ConvertToTriangleMesh(
    const PolyhedronFace3d& face,
    double eps = 1e-9);

[[nodiscard]] GEOMETRY_API PolyhedronMeshConversion3d ConvertToTriangleMesh(
    const PolyhedronBody& body,
    double eps = 1e-9);

[[nodiscard]] GEOMETRY_API PolyhedronMeshConversion3d ConvertToTriangleMesh(
    const BrepBody& body,
    double eps = 1e-9);
} // namespace geometry::sdk
