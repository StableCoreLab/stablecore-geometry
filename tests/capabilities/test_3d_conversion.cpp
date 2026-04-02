#include <cassert>
#include <cmath>

#include <gtest/gtest.h>

#include "sdk/Geometry.h"
#include "support/Fixtures3d.h"

using geometry::sdk::ConvertToTriangleMesh;
using geometry::sdk::ConvertToBrepBody;
using geometry::sdk::BrepConversionIssue3d;
using geometry::sdk::MeshConversionIssue3d;
using geometry::sdk::PolyhedronBody;
using geometry::sdk::PolyhedronFace3d;
using geometry::sdk::PolyhedronLoop3d;
using geometry::sdk::PolyhedronMeshConversion3d;
using geometry::sdk::PolyhedronBrepBodyConversion3d;

namespace
{
Point3d SkewPoint(const Point3d& point)
{
    // Affine skew keeps each face planar while making the body non-axis-aligned.
    return Point3d{
        point.x + 0.3 * point.z,
        point.y + 0.2 * point.x,
        point.z};
}

PolyhedronLoop3d SkewLoop(const PolyhedronLoop3d& loop)
{
    std::vector<Point3d> vertices;
    vertices.reserve(loop.VertexCount());
    for (std::size_t i = 0; i < loop.VertexCount(); ++i)
    {
        vertices.push_back(SkewPoint(loop.VertexAt(i)));
    }
    return PolyhedronLoop3d(std::move(vertices));
}

Plane SupportPlaneFromLoop(const PolyhedronLoop3d& loop)
{
    const Point3d p0 = loop.VertexAt(0);
    const Point3d p1 = loop.VertexAt(1);
    const Point3d p2 = loop.VertexAt(2);
    return Plane::FromPointAndNormal(p0, Cross(p1 - p0, p2 - p0));
}

PolyhedronFace3d SkewFace(const PolyhedronFace3d& face)
{
    PolyhedronLoop3d outer = SkewLoop(face.OuterLoop());
    std::vector<PolyhedronLoop3d> holes;
    holes.reserve(face.HoleCount());
    for (std::size_t i = 0; i < face.HoleCount(); ++i)
    {
        holes.push_back(SkewLoop(face.HoleAt(i)));
    }

    return PolyhedronFace3d(SupportPlaneFromLoop(outer), std::move(outer), std::move(holes));
}

PolyhedronBody BuildSkewedUnitCubeBody()
{
    const PolyhedronBody cube = geometry::test::BuildUnitCubeBody();
    std::vector<PolyhedronFace3d> faces;
    faces.reserve(cube.FaceCount());
    for (const PolyhedronFace3d& face : cube.Faces())
    {
        faces.push_back(SkewFace(face));
    }
    return PolyhedronBody(std::move(faces));
}

PolyhedronBody BuildSupportPlaneMismatchedCubeBody()
{
    const PolyhedronBody cube = geometry::test::BuildUnitCubeBody();
    std::vector<PolyhedronFace3d> faces = cube.Faces();

    const PolyhedronFace3d first = faces.front();
    faces.front() = PolyhedronFace3d(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.1}, Vector3d{0.0, 0.0, 1.0}),
        first.OuterLoop());

    return PolyhedronBody(std::move(faces));
}
} // namespace

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

// Demonstrates that a valid PolyhedronBody can be lifted into a planar-line
// BrepBody representation while preserving top-level face cardinality.
TEST(Conversion3dCapabilityTest, UnitCubePolyhedronBodyConvertsToBrepBody)
{
    const PolyhedronBody cubeBody = geometry::test::BuildUnitCubeBody();
    assert(cubeBody.IsValid());
    assert(cubeBody.FaceCount() == 6);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(cubeBody);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 6);
}

// Demonstrates a non-axis-aligned (affine-skewed) polyhedron subset can be
// converted to BrepBody without requiring robust non-planar repair.
TEST(Conversion3dCapabilityTest, SkewedCubePolyhedronBodyConvertsToBrepBody)
{
    const PolyhedronBody skewedBody = BuildSkewedUnitCubeBody();
    assert(skewedBody.IsValid());
    assert(skewedBody.FaceCount() == 6);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(skewedBody);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 6);
}

// Demonstrates the conversion can repair face support-plane mismatch and still
// produce a valid BrepBody for a recoverable polyhedron input.
TEST(Conversion3dCapabilityTest, SupportPlaneMismatchedCubeCanBeRepairedToBrepBody)
{
    const PolyhedronBody mismatchedBody = BuildSupportPlaneMismatchedCubeBody();
    assert(!mismatchedBody.IsValid());

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(mismatchedBody);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 6);
}
