#include <gtest/gtest.h>

#include "Geometry.h"
#include "Support/Fixtures3d.h"

using namespace Geometry;

TEST(GeometryTest, CoversTriangleMeshCapabilities)
{
    const TriangleMesh mesh(
        {
            SCPoint3d{0.0, 0.0, 0.0},
            SCPoint3d{1.0, 0.0, 0.0},
            SCPoint3d{0.0, 1.0, 0.0},
            SCPoint3d{0.0, 0.0, 1.0},
        },
        {
            TriangleMesh::TriangleIndices{0, 1, 2},
            TriangleMesh::TriangleIndices{0, 1, 3},
        });
    ASSERT_FALSE(mesh.IsEmpty());
    ASSERT_TRUE(mesh.IsValid());
    ASSERT_EQ(mesh.VertexCount(), 4);
    ASSERT_EQ(mesh.TriangleCount(), 2);
    ASSERT_TRUE(mesh.VertexAt(3).AlmostEquals(SCPoint3d{0.0, 0.0, 1.0}, 1e-12));
    const SCTriangle3d firstTriangle = mesh.TriangleAt(0);
    ASSERT_TRUE(firstTriangle.IsValid());
    EXPECT_NEAR(firstTriangle.Area(), 0.5, 1e-12);
    const SCBox3d meshBounds = mesh.Bounds();
    ASSERT_TRUE(meshBounds.IsValid());
    ASSERT_TRUE(meshBounds.MinPoint().AlmostEquals(SCPoint3d{0.0, 0.0, 0.0}, 1e-12));
    ASSERT_TRUE(meshBounds.MaxPoint().AlmostEquals(SCPoint3d{1.0, 1.0, 1.0}, 1e-12));
    EXPECT_NEAR(mesh.SurfaceArea(), 1.0, 1e-12);
    const auto meshValidation = Validate(mesh);
    ASSERT_TRUE(meshValidation.valid);
    ASSERT_EQ(meshValidation.issue, SCMeshValidationIssue3d::None);
    const auto meshProjection = ProjectPointToTriangleMesh(SCPoint3d{1.0, 1.0, 3.0}, mesh);
    ASSERT_TRUE(meshProjection.success);
    ASSERT_TRUE(meshProjection.IsValid());
    ASSERT_TRUE(meshProjection.point.AlmostEquals(SCPoint3d{0.0, 0.0, 1.0}, 1e-12));
    EXPECT_NEAR(Distance(SCPoint3d{1.0, 1.0, 3.0}, mesh), std::sqrt(6.0), 1e-12);
    ASSERT_EQ(LocatePoint(SCPoint3d{1.0, 1.0, 0.0}, mesh), SCPointContainment3d::Outside);
    const auto meshLineIntersection =
        Intersect(SCLine3d::FromOriginAndDirection(SCPoint3d{1.0, 1.0, 3.0}, SCVector3d{0.0, 0.0, -1.0}), mesh);
    ASSERT_FALSE(meshLineIntersection.intersects);
    ASSERT_TRUE(meshLineIntersection.IsValid());
    ASSERT_TRUE(meshLineIntersection.triangleIndices.empty());
    ASSERT_TRUE(meshLineIntersection.points.empty());
    const SCVector3d firstTriangleNormal = TriangleNormal(mesh, 0);
    ASSERT_TRUE(firstTriangleNormal.AlmostEquals(SCVector3d{0.0, 0.0, 1.0}, 1e-12));
    const auto triangleNormals = ComputeTriangleNormals(mesh);
    ASSERT_EQ(triangleNormals.size(), mesh.TriangleCount());
    ASSERT_TRUE(triangleNormals[1].AlmostEquals(SCVector3d{0.0, -1.0, 0.0}, 1e-12));
    const SCVector3d sharedVertexNormal = VertexNormal(mesh, 0);
    EXPECT_NEAR(sharedVertexNormal.Length(), 1.0, 1e-12);
    EXPECT_NEAR(sharedVertexNormal.x, 0.0, 1e-12);
    EXPECT_NEAR(sharedVertexNormal.y, -std::sqrt(0.5), 1e-12);
    EXPECT_NEAR(sharedVertexNormal.z, std::sqrt(0.5), 1e-12);
    const auto vertexNormals = ComputeVertexNormals(mesh);
    ASSERT_EQ(vertexNormals.size(), mesh.VertexCount());
    ASSERT_TRUE(vertexNormals[2].AlmostEquals(SCVector3d{0.0, 0.0, 1.0}, 1e-12));
    const MeshTriangleAdjacency3d firstAdjacency = TriangleAdjacency(mesh, 0);
    ASSERT_TRUE(firstAdjacency.HasNeighbor(0));
    ASSERT_EQ(firstAdjacency.adjacentTriangles[0], 1);
    ASSERT_FALSE(firstAdjacency.HasNeighbor(1));
    ASSERT_FALSE(firstAdjacency.HasNeighbor(2));
    const auto adjacency = ComputeTriangleAdjacency(mesh);
    ASSERT_EQ(adjacency.size(), mesh.TriangleCount());
    ASSERT_TRUE(adjacency[1].HasNeighbor(0));
    ASSERT_EQ(adjacency[1].adjacentTriangles[0], 0);
    const std::vector<MeshBoundaryEdge3d> boundaryEdges = ExtractBoundaryEdges(mesh);
    ASSERT_EQ(boundaryEdges.size(), 4);
    ASSERT_TRUE(boundaryEdges[0].IsValid());
    const std::vector<MeshBoundaryLoop3d> boundaryLoops = ExtractBoundaryLoops(mesh);
    ASSERT_EQ(boundaryLoops.size(), 2);
    for (const MeshBoundaryLoop3d& loop : boundaryLoops)
    {
        ASSERT_TRUE(loop.IsValid());
        ASSERT_FALSE(loop.closed);
        ASSERT_EQ(loop.vertexIndices.size(), 3);
    }
    ASSERT_FALSE(IsClosedTriangleMesh(mesh));
    ASSERT_TRUE(IsManifoldTriangleMesh(mesh));
    ASSERT_FALSE(IsConsistentlyOrientedTriangleMesh(mesh));
    const TriangleMeshRepair3d repairedOpenMesh = OrientTriangleMeshConsistently(mesh);
    ASSERT_TRUE(repairedOpenMesh.success);
    ASSERT_EQ(repairedOpenMesh.issue, MeshRepairIssue3d::None);
    ASSERT_TRUE(repairedOpenMesh.mesh.IsValid());
    ASSERT_TRUE(IsConsistentlyOrientedTriangleMesh(repairedOpenMesh.mesh));
    ASSERT_TRUE(IsManifoldTriangleMesh(repairedOpenMesh.mesh));
    EXPECT_NEAR(repairedOpenMesh.mesh.SurfaceArea(), mesh.SurfaceArea(), 1e-12);
    const TriangleMeshRepair3d closedNonPlanarMesh = CloseSinglePlanarBoundaryLoop(mesh);
    ASSERT_FALSE(closedNonPlanarMesh.success);
    ASSERT_EQ(closedNonPlanarMesh.issue, MeshRepairIssue3d::UnsupportedBoundaryTopology);
    const auto meshComponents = ComputeTriangleConnectedComponents(mesh);
    ASSERT_EQ(meshComponents.size(), 1);
    ASSERT_EQ(meshComponents[0].size(), 2);
    const std::vector<MeshShell3d> openShells = ComputeMeshShells(mesh);
    ASSERT_EQ(openShells.size(), 1);
    ASSERT_TRUE(openShells[0].IsValid());
    ASSERT_FALSE(openShells[0].closed);
    ASSERT_TRUE(openShells[0].manifold);
    ASSERT_FALSE(openShells[0].consistentlyOriented);
    const TriangleMesh tetraMesh(
        {
            SCPoint3d{0.0, 0.0, 0.0},
            SCPoint3d{1.0, 0.0, 0.0},
            SCPoint3d{0.0, 1.0, 0.0},
            SCPoint3d{0.0, 0.0, 1.0},
        },
        {
            TriangleMesh::TriangleIndices{0, 2, 1},
            TriangleMesh::TriangleIndices{0, 1, 3},
            TriangleMesh::TriangleIndices{1, 2, 3},
            TriangleMesh::TriangleIndices{2, 0, 3},
        });
    ASSERT_TRUE(tetraMesh.IsValid());
    ASSERT_TRUE(ExtractBoundaryEdges(tetraMesh).empty());
    ASSERT_TRUE(ExtractBoundaryLoops(tetraMesh).empty());
    ASSERT_TRUE(IsClosedTriangleMesh(tetraMesh));
    ASSERT_TRUE(IsManifoldTriangleMesh(tetraMesh));
    ASSERT_TRUE(IsConsistentlyOrientedTriangleMesh(tetraMesh));
    EXPECT_NEAR(Volume(tetraMesh), 1.0 / 6.0, 1e-12);
    ASSERT_EQ(LocatePoint(SCPoint3d{0.1, 0.1, 0.1}, tetraMesh), SCPointContainment3d::Inside);
    ASSERT_EQ(LocatePoint(SCPoint3d{1.0, 1.0, 1.0}, tetraMesh), SCPointContainment3d::Outside);
    ASSERT_EQ(LocatePoint(SCPoint3d{0.0, 0.2, 0.2}, tetraMesh), SCPointContainment3d::OnBoundary);
    const TriangleMeshRepair3d repairedTetraMesh = OrientTriangleMeshConsistently(tetraMesh);
    ASSERT_TRUE(repairedTetraMesh.success);
    ASSERT_EQ(repairedTetraMesh.issue, MeshRepairIssue3d::None);
    ASSERT_TRUE(repairedTetraMesh.mesh.IsValid());
    ASSERT_TRUE(IsConsistentlyOrientedTriangleMesh(repairedTetraMesh.mesh));
    const std::vector<MeshShell3d> tetraShells = ComputeMeshShells(tetraMesh);
    ASSERT_EQ(tetraShells.size(), 1);
    ASSERT_TRUE(tetraShells[0].closed);
    ASSERT_TRUE(tetraShells[0].manifold);
    ASSERT_TRUE(tetraShells[0].consistentlyOriented);
    const TriangleMesh openTetraMesh(
        {
            SCPoint3d{0.0, 0.0, 0.0},
            SCPoint3d{1.0, 0.0, 0.0},
            SCPoint3d{0.0, 1.0, 0.0},
            SCPoint3d{0.0, 0.0, 1.0},
        },
        {
            TriangleMesh::TriangleIndices{0, 1, 3},
            TriangleMesh::TriangleIndices{1, 2, 3},
            TriangleMesh::TriangleIndices{2, 0, 3},
        });
    ASSERT_TRUE(openTetraMesh.IsValid());
    ASSERT_FALSE(IsClosedTriangleMesh(openTetraMesh));
    const TriangleMeshRepair3d closedOpenTetraMesh = CloseSinglePlanarBoundaryLoop(openTetraMesh);
    ASSERT_TRUE(closedOpenTetraMesh.success);
    ASSERT_EQ(closedOpenTetraMesh.issue, MeshRepairIssue3d::None);
    ASSERT_TRUE(closedOpenTetraMesh.mesh.IsValid());
    ASSERT_TRUE(IsClosedTriangleMesh(closedOpenTetraMesh.mesh));
    ASSERT_TRUE(IsManifoldTriangleMesh(closedOpenTetraMesh.mesh));
    ASSERT_TRUE(IsConsistentlyOrientedTriangleMesh(closedOpenTetraMesh.mesh));
    const MeshHealing3d healedOpenTetra = Heal(openTetraMesh);
    ASSERT_TRUE(healedOpenTetra.success);
    ASSERT_EQ(healedOpenTetra.issue, HealingIssue3d::None);
    ASSERT_TRUE(healedOpenTetra.mesh.IsValid());
    ASSERT_TRUE(IsClosedTriangleMesh(healedOpenTetra.mesh));
    const TriangleMesh disconnectedMesh(
        {
            SCPoint3d{0.0, 0.0, 0.0},
            SCPoint3d{1.0, 0.0, 0.0},
            SCPoint3d{0.0, 1.0, 0.0},
            SCPoint3d{3.0, 0.0, 0.0},
            SCPoint3d{4.0, 0.0, 0.0},
            SCPoint3d{3.0, 1.0, 0.0},
        },
        {
            TriangleMesh::TriangleIndices{0, 1, 2},
            TriangleMesh::TriangleIndices{3, 4, 5},
        });
    ASSERT_TRUE(disconnectedMesh.IsValid());
    const auto disconnectedComponents = ComputeTriangleConnectedComponents(disconnectedMesh);
    ASSERT_EQ(disconnectedComponents.size(), 2);
    ASSERT_EQ(disconnectedComponents[0].size(), 1);
    ASSERT_EQ(disconnectedComponents[1].size(), 1);
    const TriangleMeshRepair3d closedDisconnectedMesh = ClosePlanarBoundaryLoops(disconnectedMesh);
    ASSERT_TRUE(closedDisconnectedMesh.success);
    ASSERT_EQ(closedDisconnectedMesh.issue, MeshRepairIssue3d::None);
    ASSERT_TRUE(closedDisconnectedMesh.mesh.IsValid());
    ASSERT_TRUE(IsClosedTriangleMesh(closedDisconnectedMesh.mesh));
    ASSERT_TRUE(IsManifoldTriangleMesh(closedDisconnectedMesh.mesh));
    ASSERT_TRUE(IsConsistentlyOrientedTriangleMesh(closedDisconnectedMesh.mesh));
    const std::vector<MeshBoundaryLoop3d> disconnectedBoundaryLoops = ExtractBoundaryLoops(disconnectedMesh);
    ASSERT_EQ(disconnectedBoundaryLoops.size(), 2);
    ASSERT_TRUE(disconnectedBoundaryLoops[0].closed);
    ASSERT_TRUE(disconnectedBoundaryLoops[1].closed);
    const std::vector<MeshShell3d> disconnectedShells = ComputeMeshShells(disconnectedMesh);
    ASSERT_EQ(disconnectedShells.size(), 2);
    ASSERT_FALSE(disconnectedShells[0].closed);
    ASSERT_FALSE(disconnectedShells[1].closed);
    const TriangleMesh nonManifoldMesh(
        {
            SCPoint3d{0.0, 0.0, 0.0},
            SCPoint3d{1.0, 0.0, 0.0},
            SCPoint3d{0.0, 1.0, 0.0},
            SCPoint3d{0.0, 0.0, 1.0},
            SCPoint3d{0.0, -1.0, 0.0},
        },
        {
            TriangleMesh::TriangleIndices{0, 1, 2},
            TriangleMesh::TriangleIndices{0, 3, 1},
            TriangleMesh::TriangleIndices{0, 1, 4},
        });
    ASSERT_TRUE(nonManifoldMesh.IsValid());
    const auto nonManifoldEdges = ExtractNonManifoldEdges(nonManifoldMesh);
    ASSERT_EQ(nonManifoldEdges.size(), 1);
    ASSERT_TRUE(nonManifoldEdges[0].IsValid());
    ASSERT_EQ(nonManifoldEdges[0].incidentTriangles.size(), 3);
    ASSERT_FALSE(IsManifoldTriangleMesh(nonManifoldMesh));
    ASSERT_FALSE(IsConsistentlyOrientedTriangleMesh(nonManifoldMesh));
    const TriangleMeshRepair3d repairedNonManifoldMesh = OrientTriangleMeshConsistently(nonManifoldMesh);
    ASSERT_FALSE(repairedNonManifoldMesh.success);
    ASSERT_EQ(repairedNonManifoldMesh.issue, MeshRepairIssue3d::NonManifold);
    const auto nonManifoldComponents = ComputeTriangleConnectedComponents(nonManifoldMesh);
    ASSERT_EQ(nonManifoldComponents.size(), 1);
    ASSERT_EQ(nonManifoldComponents[0].size(), 3);
    const std::vector<MeshShell3d> nonManifoldShells = ComputeMeshShells(nonManifoldMesh);
    ASSERT_EQ(nonManifoldShells.size(), 1);
    ASSERT_FALSE(nonManifoldShells[0].closed);
    ASSERT_FALSE(nonManifoldShells[0].manifold);
    ASSERT_FALSE(nonManifoldShells[0].consistentlyOriented);
    const TriangleMesh movedMesh = mesh.Transformed(SCTransform3d::Translation(SCVector3d{2.0, -1.0, 3.0}));
    ASSERT_TRUE(movedMesh.IsValid());
    ASSERT_TRUE(movedMesh.VertexAt(0).AlmostEquals(SCPoint3d{2.0, -1.0, 3.0}, 1e-12));
    ASSERT_TRUE(movedMesh.VertexAt(3).AlmostEquals(SCPoint3d{2.0, -1.0, 4.0}, 1e-12));
    EXPECT_NEAR(movedMesh.SurfaceArea(), mesh.SurfaceArea(), 1e-12);

    const TriangleMesh invalidMesh(
        {
            SCPoint3d{0.0, 0.0, 0.0},
            SCPoint3d{1.0, 0.0, 0.0},
            SCPoint3d{1.0, 0.0, 0.0},
        },
        {
            TriangleMesh::TriangleIndices{0, 1, 2},
        });
    const auto invalidMeshValidation = Validate(invalidMesh);
    ASSERT_FALSE(invalidMeshValidation.valid);
    ASSERT_EQ(invalidMeshValidation.issue, SCMeshValidationIssue3d::DegenerateTriangle);

}

TEST(TriangleMeshToleranceTest, SmallOpenTriangleBoundaryUsesCallerTolerance)
{
    constexpr double side = 1e-10;
    constexpr double eps = 1e-15;
    const TriangleMesh mesh(
        {{0, 0, 0}, {side, 0, 0}, {0, side, 0}},
        {TriangleMesh::TriangleIndices{0, 1, 2}});
    ASSERT_FALSE(mesh.IsValid());
    ASSERT_TRUE(mesh.IsValid(eps));
    EXPECT_TRUE(ExtractBoundaryLoops(mesh).empty());
    EXPECT_EQ(ExtractBoundaryEdges(mesh, eps).size(), 3U);
    const auto loops = ExtractBoundaryLoops(mesh, eps);
    ASSERT_EQ(loops.size(), 1U);
    EXPECT_TRUE(loops.front().closed);
    EXPECT_EQ(loops.front().vertexIndices.size(), 3U);
}

TEST(TriangleMeshToleranceTest, SmallClosedTetrahedronTopologyUsesCallerTolerance)
{
    constexpr double side = 1e-10;
    constexpr double eps = 1e-15;
    const TriangleMesh mesh(
        {{0, 0, 0}, {side, 0, 0}, {0, side, 0}, {0, 0, side}},
        {{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {2, 0, 3}});
    ASSERT_FALSE(mesh.IsValid());
    ASSERT_TRUE(mesh.IsValid(eps));
    EXPECT_FALSE(IsManifoldTriangleMesh(mesh));
    EXPECT_TRUE(IsManifoldTriangleMesh(mesh, eps));
    EXPECT_TRUE(IsConsistentlyOrientedTriangleMesh(mesh, eps));
    const auto adjacency = ComputeTriangleAdjacency(mesh, eps);
    ASSERT_EQ(adjacency.size(), 4U);
    for (const auto& triangle : adjacency)
    {
        for (std::size_t edge = 0; edge < 3; ++edge)
        {
            EXPECT_TRUE(triangle.HasNeighbor(edge));
        }
    }
    EXPECT_TRUE(TriangleAdjacency(mesh, 0, eps).HasNeighbor(0));
    EXPECT_TRUE(ExtractNonManifoldEdges(mesh, eps).empty());
    const auto components = ComputeTriangleConnectedComponents(mesh, eps);
    ASSERT_EQ(components.size(), 1U);
    EXPECT_EQ(components.front().size(), 4U);
    const auto shells = ComputeMeshShells(mesh, eps);
    ASSERT_EQ(shells.size(), 1U);
    EXPECT_TRUE(shells.front().closed);
    EXPECT_TRUE(shells.front().manifold);
    EXPECT_TRUE(shells.front().consistentlyOriented);
}

TEST(TriangleMeshToleranceTest, SmallNonManifoldEdgeUsesCallerTolerance)
{
    constexpr double side = 1e-10;
    constexpr double eps = 1e-15;
    const TriangleMesh mesh(
        {{0, 0, 0}, {side, 0, 0}, {0, side, 0}, {0, 0, side}, {0, -side, 0}},
        {{0, 1, 2}, {1, 0, 3}, {0, 1, 4}});
    ASSERT_TRUE(mesh.IsValid(eps));
    EXPECT_TRUE(ExtractNonManifoldEdges(mesh).empty());
    const auto edges = ExtractNonManifoldEdges(mesh, eps);
    ASSERT_EQ(edges.size(), 1U);
    EXPECT_EQ(edges.front().incidentTriangles.size(), 3U);
    EXPECT_FALSE(IsManifoldTriangleMesh(mesh, eps));
}

TEST(TriangleMeshToleranceTest, SmallOpenTetrahedronRepairUsesCallerTolerance)
{
    constexpr double side = 1e-10;
    constexpr double eps = 1e-15;
    const TriangleMesh mesh(
        {{0, 0, 0}, {side, 0, 0}, {0, side, 0}, {0, 0, side}},
        {{0, 1, 3}, {1, 2, 3}, {2, 0, 3}});
    ASSERT_TRUE(mesh.IsValid(eps));
    ASSERT_FALSE(mesh.IsValid());
    ASSERT_TRUE(IsManifoldTriangleMesh(mesh, eps));
    ASSERT_EQ(ExtractBoundaryLoops(mesh, eps).size(), 1U);

    const auto oriented = OrientTriangleMeshConsistently(mesh, eps);
    EXPECT_TRUE(oriented.success);
    EXPECT_EQ(oriented.issue, MeshRepairIssue3d::None);
    EXPECT_TRUE(oriented.IsValid(eps));
    const auto single = CloseSinglePlanarBoundaryLoop(mesh, eps);
    EXPECT_TRUE(single.success);
    EXPECT_EQ(single.issue, MeshRepairIssue3d::None);
    if (single.success)
    {
        EXPECT_FALSE(single.IsValid());
        EXPECT_TRUE(single.IsValid(eps));
        EXPECT_TRUE(IsClosedTriangleMesh(single.mesh, eps));
    }
    const auto all = ClosePlanarBoundaryLoops(mesh, eps);
    EXPECT_TRUE(all.success);
    EXPECT_EQ(all.issue, MeshRepairIssue3d::None);
    if (all.success)
    {
        EXPECT_TRUE(all.IsValid(eps));
        EXPECT_TRUE(IsClosedTriangleMesh(all.mesh, eps));
    }
    const auto healed = Heal(mesh, eps);
    ASSERT_TRUE(healed.success);
    EXPECT_EQ(healed.issue, HealingIssue3d::None);
    EXPECT_TRUE(IsClosedTriangleMesh(healed.mesh, eps));
}
