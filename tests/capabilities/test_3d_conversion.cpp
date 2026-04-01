#include <cassert>
#include <cmath>

#include <gtest/gtest.h>

#include "sdk/Geometry.h"
#include "support/Fixtures3d.h"

using geometry::sdk::ConvertToTriangleMesh;
using geometry::sdk::MeshConversionIssue3d;
using geometry::sdk::PolyhedronBody;
using geometry::sdk::PolyhedronMeshConversion3d;

// Demonstrates that a closed PolyhedronBody (unit cube, 6 quad faces) converts
// to a TriangleMesh with the correct triangle count and total surface area.
// High-fidelity feature-preserving conversion and non-planar polyhedron→Brep
// repair remain open gaps.
TEST(Conversion3dCapabilityTest, UnitCubePolyhedronBodyConvertsToTriangleMesh)
{
    const PolyhedronBody cubeBody = geometry::test::BuildUnitCubeBody();
    assert(cubeBody.IsValid());
    assert(cubeBody.FaceCount() == 6);

    const PolyhedronMeshConversion3d result = ConvertToTriangleMesh(cubeBody);
    assert(result.success);
    assert(result.issue == MeshConversionIssue3d::None);
    assert(result.mesh.IsValid());
    // Each of the 6 quad faces is split into 2 triangles → 12 total
    assert(result.mesh.TriangleCount() == 12);
    // Total surface area of a unit cube = 6 × (1×1) = 6.0
    assert(std::abs(result.mesh.SurfaceArea() - 6.0) < 1e-10);
}
