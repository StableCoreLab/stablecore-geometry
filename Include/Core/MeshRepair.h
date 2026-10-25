#pragma once
#include "Export/GeometryExport.h"
#include "Geometry3d/TriangleMesh.h"
#include "Support/Epsilon.h"

namespace Geometry
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

    [[nodiscard]] GEOMETRY_API TriangleMeshRepair3d
    OrientTriangleMeshConsistently(const TriangleMesh& mesh, double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API TriangleMeshRepair3d
    CloseSinglePlanarBoundaryLoop(const TriangleMesh& mesh, double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API TriangleMeshRepair3d ClosePlanarBoundaryLoops(const TriangleMesh& mesh,
                                                                             double eps = Geometry::kDefaultEpsilon);
}  // namespace Geometry
