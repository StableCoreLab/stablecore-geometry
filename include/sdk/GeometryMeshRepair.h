#pragma once

#include "export/GeometryExport.h"
#include "sdk/TriangleMesh.h"

namespace geometry::sdk
{
enum class MeshRepairIssue3d
{
    None,
    InvalidMesh,
    NonManifold,
    NonOrientable,
    UnsupportedBoundaryTopology,
    NonPlanarBoundary
};

struct GEOMETRY_API TriangleMeshRepair3d
{
    bool success{false};
    MeshRepairIssue3d issue{MeshRepairIssue3d::None};
    TriangleMesh mesh{};

    [[nodiscard]] bool IsValid() const
    {
        return !success || mesh.IsValid();
    }
};

[[nodiscard]] GEOMETRY_API TriangleMeshRepair3d OrientTriangleMeshConsistently(
    const TriangleMesh& mesh,
    double eps = 1e-9);

[[nodiscard]] GEOMETRY_API TriangleMeshRepair3d CloseSinglePlanarBoundaryLoop(
    const TriangleMesh& mesh,
    double eps = 1e-9);

[[nodiscard]] GEOMETRY_API TriangleMeshRepair3d ClosePlanarBoundaryLoops(
    const TriangleMesh& mesh,
    double eps = 1e-9);
} // namespace geometry::sdk
