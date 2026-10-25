#include <gtest/gtest.h>

#include "Core/Healing.h"
#include "Brep/MeshOps.h"

TEST(MeshHealingContractTest, ClosingBoundaryAlsoOrientsResult)
{
    const Geometry::TriangleMesh openTetrahedron(
        {
            Geometry::SCPoint3d{0.0, 0.0, 0.0},
            Geometry::SCPoint3d{1.0, 0.0, 0.0},
            Geometry::SCPoint3d{0.0, 1.0, 0.0},
            Geometry::SCPoint3d{0.0, 0.0, 1.0},
        },
        {
            Geometry::TriangleMesh::TriangleIndices{0, 1, 3},
            Geometry::TriangleMesh::TriangleIndices{1, 3, 2},
            Geometry::TriangleMesh::TriangleIndices{2, 0, 3},
        });
    ASSERT_TRUE(openTetrahedron.IsValid());
    ASSERT_FALSE(Geometry::IsClosedTriangleMesh(openTetrahedron));
    ASSERT_FALSE(Geometry::IsConsistentlyOrientedTriangleMesh(openTetrahedron));

    const Geometry::MeshHealing3d healed = Geometry::Heal(openTetrahedron);
    ASSERT_TRUE(healed.success);
    EXPECT_EQ(healed.issue, Geometry::HealingIssue3d::None);
    EXPECT_TRUE(Geometry::IsClosedTriangleMesh(healed.mesh));
    EXPECT_TRUE(Geometry::IsConsistentlyOrientedTriangleMesh(healed.mesh));
}
