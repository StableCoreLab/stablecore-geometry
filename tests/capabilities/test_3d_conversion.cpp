#include <cassert>
#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include "sdk/Geometry.h"
#include "support/Fixtures3d.h"

using geometry::sdk::ConvertToTriangleMesh;
using geometry::sdk::ConvertToBrepBody;
using geometry::sdk::BrepConversionIssue3d;
using geometry::sdk::BrepBody;
using geometry::sdk::BrepCoedge;
using geometry::sdk::BrepEdge;
using geometry::sdk::BrepFace;
using geometry::sdk::BrepLoop;
using geometry::sdk::BrepShell;
using geometry::sdk::BrepVertex;
using geometry::sdk::CurveOnSurface;
using geometry::sdk::Intervald;
using geometry::sdk::Line3d;
using geometry::sdk::LineCurve3d;
using geometry::sdk::MeshConversionIssue3d;
using geometry::sdk::Plane;
using geometry::sdk::PlaneSurface;
using geometry::sdk::PolyhedronBody;
using geometry::sdk::PolyhedronFace3d;
using geometry::sdk::PolyhedronLoop3d;
using geometry::sdk::PolyhedronMeshConversion3d;
using geometry::sdk::PolyhedronBrepBodyConversion3d;
using geometry::sdk::Polyline2d;
using geometry::sdk::PolylineClosure;
using geometry::sdk::Surface;

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

PolyhedronBody BuildMildlyNonPlanarCubeFaceBody()
{
    const PolyhedronBody cube = geometry::test::BuildUnitCubeBody();
    std::vector<PolyhedronFace3d> faces = cube.Faces();

    const std::size_t topFaceIndex = 1;
    const PolyhedronFace3d top = faces[topFaceIndex];
    std::vector<Point3d> topVertices;
    topVertices.reserve(top.OuterLoop().VertexCount());
    for (std::size_t i = 0; i < top.OuterLoop().VertexCount(); ++i)
    {
        topVertices.push_back(top.OuterLoop().VertexAt(i));
    }

    // Introduce mild non-coplanarity on one vertex.
    topVertices[2].z += 0.05;
    faces[topFaceIndex] = PolyhedronFace3d(top.SupportPlane(), PolyhedronLoop3d(std::move(topVertices)));

    return PolyhedronBody(std::move(faces));
}

PolyhedronBody BuildMildlyNonPlanarHoledFaceBody()
{
    const PolyhedronLoop3d outer(
        {
            Point3d{0.0, 0.0, 0.0},
            Point3d{4.0, 0.0, 0.0},
            Point3d{4.0, 4.0, 0.0},
            Point3d{0.0, 4.0, 0.0},
        });
    std::vector<Point3d> holeVertices{
        Point3d{1.0, 1.0, 0.0},
        Point3d{3.0, 1.0, 0.0},
        Point3d{3.0, 3.0, 0.0},
        Point3d{1.0, 3.0, 0.0}};
    holeVertices[2].z += 0.03;

    const PolyhedronFace3d face(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}),
        outer,
        {PolyhedronLoop3d(std::move(holeVertices))});
    return PolyhedronBody({face});
}

PolyhedronBody BuildSupportPlaneMismatchedCollinearLeadingLoopBody()
{
    const PolyhedronFace3d face(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.2}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d(
            {
                Point3d{0.0, 0.0, 0.0},
                Point3d{1.0, 0.0, 0.0},
                Point3d{2.0, 0.0, 0.0},
                Point3d{2.0, 1.0, 0.0},
                Point3d{0.0, 1.0, 0.0},
            }));
    return PolyhedronBody({face});
}

PolyhedronBody BuildDuplicateVertexLoopBody()
{
    const PolyhedronFace3d face(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.2}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d(
            {
                Point3d{0.0, 0.0, 0.0},
                Point3d{2.0, 0.0, 0.0},
                Point3d{2.0, 0.0, 0.0},
                Point3d{2.0, 1.0, 0.0},
                Point3d{0.0, 1.0, 0.0},
            }));
    return PolyhedronBody({face});
}

PolyhedronBody BuildDuplicateVertexHoleLoopBody()
{
    const PolyhedronFace3d face(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.2}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d(
            {
                Point3d{0.0, 0.0, 0.0},
                Point3d{4.0, 0.0, 0.0},
                Point3d{4.0, 4.0, 0.0},
                Point3d{0.0, 4.0, 0.0},
            }),
        {
            PolyhedronLoop3d(
                {
                    Point3d{1.0, 1.0, 0.0},
                    Point3d{3.0, 1.0, 0.0},
                    Point3d{3.0, 1.0, 0.0},
                    Point3d{3.0, 3.0, 0.0},
                    Point3d{1.0, 3.0, 0.0},
                }),
        });
    return PolyhedronBody({face});
}

PolyhedronBody BuildCompositeRepairStressFaceBody()
{
    std::vector<Point3d> outer{
        Point3d{0.0, 0.0, 0.0},
        Point3d{1.0, 0.0, 0.0},
        Point3d{2.0, 0.0, 0.0},
        Point3d{4.0, 0.0, 0.0},
        Point3d{4.0, 4.0, 0.0},
        Point3d{0.0, 4.0, 0.0},
        Point3d{0.0, 4.0, 0.0},
    };
    // Mildly non-planar disturbance on outer loop.
    outer[4].z += 0.04;

    std::vector<Point3d> hole{
        Point3d{1.0, 1.0, 0.0},
        Point3d{3.0, 1.0, 0.0},
        Point3d{3.0, 1.0, 0.0},
        Point3d{3.0, 3.0, 0.0},
        Point3d{1.0, 3.0, 0.0},
    };
    // Mildly non-planar disturbance on hole loop.
    hole[3].z += 0.03;

    const PolyhedronFace3d face(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.25}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d(std::move(outer)),
        {PolyhedronLoop3d(std::move(hole))});
    return PolyhedronBody({face});
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

// Demonstrates conversion can repair mildly non-planar face loops by
// projecting vertices onto a refit support plane.
TEST(Conversion3dCapabilityTest, MildlyNonPlanarCubeFaceCanBeRepairedToBrepBody)
{
    const PolyhedronBody nonPlanarBody = BuildMildlyNonPlanarCubeFaceBody();
    assert(!nonPlanarBody.IsValid());

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(nonPlanarBody);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 6);
}

// Demonstrates refit-plane projection repair also handles mildly non-planar
// hole loops in a planar-holed face.
TEST(Conversion3dCapabilityTest, MildlyNonPlanarHoleLoopCanBeRepairedToBrepBody)
{
    const PolyhedronBody nonPlanarHoledBody = BuildMildlyNonPlanarHoledFaceBody();
    assert(!nonPlanarHoledBody.IsValid());

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(nonPlanarHoledBody);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 1);
}

// Demonstrates refit-plane repair remains robust when leading loop vertices
// are collinear and cannot directly define a support normal.
TEST(Conversion3dCapabilityTest, CollinearLeadingLoopStillRepairsToBrepBody)
{
    const PolyhedronBody body = BuildSupportPlaneMismatchedCollinearLeadingLoopBody();
    assert(!body.IsValid());

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 1);
}

// Demonstrates conversion repair can normalize consecutive duplicate loop
// vertices before support-plane refit and Brep reconstruction.
TEST(Conversion3dCapabilityTest, DuplicateVertexLoopStillRepairsToBrepBody)
{
    const PolyhedronBody body = BuildDuplicateVertexLoopBody();
    assert(!body.IsValid());

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 1);
}

// Demonstrates duplicate vertices in a hole loop are normalized before
// refit-plane conversion to BrepBody.
TEST(Conversion3dCapabilityTest, DuplicateVertexHoleLoopStillRepairsToBrepBody)
{
    const PolyhedronBody body = BuildDuplicateVertexHoleLoopBody();
    assert(!body.IsValid());

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 1);
}

// Demonstrates repair sub-strategies compose deterministically on one face:
// support-plane mismatch + collinear leading outer points + duplicate points +
// mild non-planar disturbances on outer and hole loops.
TEST(Conversion3dCapabilityTest, CompositeRepairStressFaceStillConvertsToBrepBody)
{
    const PolyhedronBody body = BuildCompositeRepairStressFaceBody();
    assert(!body.IsValid());

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 1);
}

// Demonstrates planar holed BrepBody conversion keeps representative area by
// honoring hole trims in mesh triangulation.
TEST(Conversion3dCapabilityTest, PlanarHoledBrepBodyConvertsToMeshWithExpectedArea)
{
    std::vector<BrepVertex> vertices{
        BrepVertex(Point3d{0.0, 0.0, 0.0}),
        BrepVertex(Point3d{4.0, 0.0, 0.0}),
        BrepVertex(Point3d{4.0, 4.0, 0.0}),
        BrepVertex(Point3d{0.0, 4.0, 0.0}),
        BrepVertex(Point3d{1.0, 1.0, 0.0}),
        BrepVertex(Point3d{3.0, 1.0, 0.0}),
        BrepVertex(Point3d{3.0, 3.0, 0.0}),
        BrepVertex(Point3d{1.0, 3.0, 0.0})};

    std::vector<BrepEdge> edges;
    auto addEdge = [&](std::size_t start, std::size_t end) {
        const Point3d first = vertices[start].Point();
        const Point3d second = vertices[end].Point();
        edges.emplace_back(
            std::make_shared<LineCurve3d>(LineCurve3d::FromLine(
                Line3d::FromOriginAndDirection(first, second - first),
                Intervald{0.0, 1.0})),
            start,
            end);
    };
    addEdge(0, 1);
    addEdge(1, 2);
    addEdge(2, 3);
    addEdge(3, 0);
    addEdge(4, 5);
    addEdge(5, 6);
    addEdge(6, 7);
    addEdge(7, 4);

    const BrepLoop outerLoop({BrepCoedge(0, false), BrepCoedge(1, false), BrepCoedge(2, false), BrepCoedge(3, false)});
    const BrepLoop holeLoop({BrepCoedge(4, false), BrepCoedge(5, false), BrepCoedge(6, false), BrepCoedge(7, false)});

    const PlaneSurface planeSurface = PlaneSurface::FromPlane(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}));
    auto supportSurface = std::shared_ptr<Surface>(planeSurface.Clone().release());
    const CurveOnSurface outerTrim(
        supportSurface,
        Polyline2d(
            {
                Point2d{0.0, 0.0},
                Point2d{4.0, 0.0},
                Point2d{4.0, 4.0},
                Point2d{0.0, 4.0},
            },
            PolylineClosure::Closed));
    const CurveOnSurface holeTrim(
        supportSurface,
        Polyline2d(
            {
                Point2d{1.0, 1.0},
                Point2d{3.0, 1.0},
                Point2d{3.0, 3.0},
                Point2d{1.0, 3.0},
            },
            PolylineClosure::Closed));

    const BrepFace face(
        std::shared_ptr<Surface>(planeSurface.Clone().release()),
        outerLoop,
        {holeLoop},
        outerTrim,
        {holeTrim});
    const BrepBody body(vertices, edges, {BrepShell({face}, false)});
    assert(body.IsValid());

    const PolyhedronMeshConversion3d mesh = ConvertToTriangleMesh(body);
    assert(mesh.success);
    assert(mesh.mesh.IsValid());
    // Expected area: 4x4 outer minus 2x2 hole = 12.
    assert(std::abs(mesh.mesh.SurfaceArea() - 12.0) < 1e-8);
}

// Demonstrates planar multi-face BrepBody conversion keeps representative
// aggregate area across disconnected faces.
TEST(Conversion3dCapabilityTest, PlanarMultiFaceBrepBodyConvertsToMeshWithExpectedArea)
{
    std::vector<BrepVertex> vertices{
        // Face A square: area 4
        BrepVertex(Point3d{0.0, 0.0, 0.0}),
        BrepVertex(Point3d{2.0, 0.0, 0.0}),
        BrepVertex(Point3d{2.0, 2.0, 0.0}),
        BrepVertex(Point3d{0.0, 2.0, 0.0}),
        // Face B rectangle: area 2
        BrepVertex(Point3d{3.0, 0.0, 0.0}),
        BrepVertex(Point3d{4.0, 0.0, 0.0}),
        BrepVertex(Point3d{4.0, 2.0, 0.0}),
        BrepVertex(Point3d{3.0, 2.0, 0.0})};

    std::vector<BrepEdge> edges;
    auto addEdge = [&](std::size_t start, std::size_t end) {
        const Point3d first = vertices[start].Point();
        const Point3d second = vertices[end].Point();
        edges.emplace_back(
            std::make_shared<LineCurve3d>(LineCurve3d::FromLine(
                Line3d::FromOriginAndDirection(first, second - first),
                Intervald{0.0, 1.0})),
            start,
            end);
    };

    // Face A
    addEdge(0, 1);
    addEdge(1, 2);
    addEdge(2, 3);
    addEdge(3, 0);
    // Face B
    addEdge(4, 5);
    addEdge(5, 6);
    addEdge(6, 7);
    addEdge(7, 4);

    const BrepLoop outerA({BrepCoedge(0, false), BrepCoedge(1, false), BrepCoedge(2, false), BrepCoedge(3, false)});
    const BrepLoop outerB({BrepCoedge(4, false), BrepCoedge(5, false), BrepCoedge(6, false), BrepCoedge(7, false)});

    const PlaneSurface planeSurface = PlaneSurface::FromPlane(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}));
    auto supportA = std::shared_ptr<Surface>(planeSurface.Clone().release());
    auto supportB = std::shared_ptr<Surface>(planeSurface.Clone().release());

    const CurveOnSurface trimA(
        supportA,
        Polyline2d(
            {
                Point2d{0.0, 0.0},
                Point2d{2.0, 0.0},
                Point2d{2.0, 2.0},
                Point2d{0.0, 2.0},
            },
            PolylineClosure::Closed));
    const CurveOnSurface trimB(
        supportB,
        Polyline2d(
            {
                Point2d{3.0, 0.0},
                Point2d{4.0, 0.0},
                Point2d{4.0, 2.0},
                Point2d{3.0, 2.0},
            },
            PolylineClosure::Closed));

    const BrepFace faceA(
        std::shared_ptr<Surface>(planeSurface.Clone().release()),
        outerA,
        {},
        trimA,
        {});
    const BrepFace faceB(
        std::shared_ptr<Surface>(planeSurface.Clone().release()),
        outerB,
        {},
        trimB,
        {});
    const BrepBody body(vertices, edges, {BrepShell({faceA, faceB}, false)});
    assert(body.IsValid());

    const PolyhedronMeshConversion3d mesh = ConvertToTriangleMesh(body);
    assert(mesh.success);
    assert(mesh.mesh.IsValid());
    // Expected area: 4 + 2 = 6.
    assert(std::abs(mesh.mesh.SurfaceArea() - 6.0) < 1e-8);
}
