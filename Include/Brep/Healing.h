#pragma once
#include "Brep/SCBrepBody.h"
#include "Brep/PolyhedronBody.h"
#include "Core/GeometryTypes.h"
#include "Export/GeometryExport.h"
#include "Geometry3d/TriangleMesh.h"
#include "Support/Epsilon.h"

namespace Geometry
{
    enum class HealingIssue3d
    {
        None,
        InvalidMesh,
        InvalidPolyhedron,
        InvalidBrep,
        RepairFailed
    };

    enum class HealingPolicy3d
    {
        Conservative,
        Aggressive
    };

    struct GEOMETRY_API MeshHealing3d
    {
        bool success{false};
        HealingIssue3d issue{HealingIssue3d::None};
        TriangleMesh mesh{};
    };

    struct GEOMETRY_API PolyhedronHealing3d
    {
        bool success{false};
        HealingIssue3d issue{HealingIssue3d::None};
        PolyhedronBody body{};
    };

    struct GEOMETRY_API BrepHealing3d
    {
        bool success{false};
        HealingIssue3d issue{HealingIssue3d::None};
        SCBrepBody body{};
    };

    [[nodiscard]] GEOMETRY_API MeshHealing3d Heal(const TriangleMesh& mesh, double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API PolyhedronHealing3d Heal(const PolyhedronBody& body,
                                                        double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API BrepHealing3d Heal(const SCBrepBody& body, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API BrepHealing3d Heal(const SCBrepBody& body,
                                                  const SCGeometryTolerance3d& tolerance,
                                                  HealingPolicy3d policy);
}  // namespace Geometry


