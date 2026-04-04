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
using geometry::sdk::ComputeTriangleConnectedComponents;
using geometry::sdk::CurveOnSurface;
using geometry::sdk::Intervald;
using geometry::sdk::Line3d;
using geometry::sdk::LineCurve3d;
using geometry::sdk::MeshConversionIssue3d;
using geometry::sdk::Plane;
using geometry::sdk::PlaneSurface;
using geometry::sdk::Point3d;
using geometry::sdk::PolyhedronBody;
using geometry::sdk::PolyhedronFace3d;
using geometry::sdk::PolyhedronLoop3d;
using geometry::sdk::PolyhedronMeshConversion3d;
using geometry::sdk::PolyhedronBrepBodyConversion3d;
using geometry::sdk::Point2d;
using geometry::sdk::Polyline2d;
using geometry::sdk::PolylineClosure;
using geometry::sdk::Surface;
using geometry::sdk::Vector3d;

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

Plane SkewPlane(const Plane& plane)
{
    // For p' = A p (with A defined by SkewPoint), normal transforms as n' = (A^{-1})^T n.
    const Vector3d n = plane.normal;
    const Vector3d transformedNormal{
        n.x - 0.2 * n.y,
        n.y,
        -0.3 * n.x + 0.06 * n.y + n.z};

    return Plane::FromPointAndNormal(SkewPoint(plane.origin), transformedNormal);
}

PolyhedronLoop3d SkewLoop(const PolyhedronLoop3d& loop)
{
    std::vector<Point3d> vertices;
    const std::vector<Point3d>& source = loop.Vertices();
    vertices.reserve(source.size());
    for (const Point3d& point : source)
    {
        vertices.push_back(SkewPoint(point));
    }
    return PolyhedronLoop3d(std::move(vertices));
}

Plane SupportPlaneFromLoop(const PolyhedronLoop3d& loop)
{
    const std::vector<Point3d>& vertices = loop.Vertices();
    if (vertices.size() < 3)
    {
        return Plane{};
    }

    const Point3d& p0 = vertices[0];
    const Point3d& p1 = vertices[1];
    const Point3d& p2 = vertices[2];
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

    return PolyhedronFace3d(SkewPlane(face.SupportPlane()), std::move(outer), std::move(holes));
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
    try
    {
        const PolyhedronBody cube = geometry::test::BuildUnitCubeBody();
        std::vector<PolyhedronFace3d> faces = cube.Faces();

        if (faces.empty())
        {
            return PolyhedronBody();
        }

        const PolyhedronFace3d first = faces.front();
        faces.front() = PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.1}, Vector3d{0.0, 0.0, 1.0}),
            first.OuterLoop());

        return PolyhedronBody(std::move(faces));
    }
    catch (const std::exception&)
    {
        return PolyhedronBody();
    }
}

PolyhedronLoop3d TranslateLoop3d(const PolyhedronLoop3d& loop, const Vector3d& delta)
{
    try
    {
        std::vector<Point3d> vertices;
        vertices.reserve(loop.VertexCount());
        for (std::size_t i = 0; i < loop.VertexCount(); ++i)
        {
            vertices.push_back(loop.VertexAt(i) + delta);
        }
        return PolyhedronLoop3d(std::move(vertices));
    }
    catch (const std::exception&)
    {
        return PolyhedronLoop3d();
    }
}

PolyhedronFace3d TranslateFace3d(const PolyhedronFace3d& face, const Vector3d& delta)
{
    try
    {
        const Plane translatedPlane = Plane::FromPointAndNormal(
            face.SupportPlane().origin + delta,
            face.SupportPlane().normal);

        PolyhedronLoop3d outer = TranslateLoop3d(face.OuterLoop(), delta);
        std::vector<PolyhedronLoop3d> holes;
        holes.reserve(face.HoleCount());
        for (std::size_t i = 0; i < face.HoleCount(); ++i)
        {
            holes.push_back(TranslateLoop3d(face.HoleAt(i), delta));
        }

        return PolyhedronFace3d(translatedPlane, std::move(outer), std::move(holes));
    }
    catch (const std::exception&)
    {
        return PolyhedronFace3d();
    }
}

PolyhedronBody BuildTwoSeparatedUnitCubeBody()
{
    try
    {
        const PolyhedronBody first = geometry::test::BuildUnitCubeBody();
        std::vector<PolyhedronFace3d> faces = first.Faces();

        const Vector3d delta{3.0, 0.0, 0.0};
        for (const PolyhedronFace3d& face : first.Faces())
        {
            faces.push_back(TranslateFace3d(face, delta));
        }

        return PolyhedronBody(std::move(faces));
    }
    catch (const std::exception&)
    {
        return PolyhedronBody();
    }
}

PolyhedronBody BuildDeformedUnitCubeBody()
{
    // V0 is the origin corner shared by bottom / front / left faces.
    // Displacing it makes those three faces non-planar.
    const Point3d v0{0.1, 0.1, -0.1}; // displaced from (0,0,0)
    const Point3d v1{1.0, 0.0,  0.0};
    const Point3d v2{1.0, 1.0,  0.0};
    const Point3d v3{0.0, 1.0,  0.0};
    const Point3d v4{0.0, 0.0,  1.0};
    const Point3d v5{1.0, 0.0,  1.0};
    const Point3d v6{1.0, 1.0,  1.0};
    const Point3d v7{0.0, 1.0,  1.0};

    // Support planes are the original axis-aligned planes of the unit cube —
    // intentionally wrong for the three faces that contain v0.
    const Plane bottom{Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, -1.0}};
    const Plane top   {Point3d{0.0, 0.0, 1.0}, Vector3d{0.0, 0.0,  1.0}};
    const Plane front {Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, -1.0, 0.0}};
    const Plane back  {Point3d{0.0, 1.0, 0.0}, Vector3d{0.0,  1.0, 0.0}};
    const Plane left  {Point3d{0.0, 0.0, 0.0}, Vector3d{-1.0, 0.0, 0.0}};
    const Plane right {Point3d{1.0, 0.0, 0.0}, Vector3d{ 1.0, 0.0, 0.0}};

    return PolyhedronBody({
        PolyhedronFace3d(bottom, PolyhedronLoop3d({v0, v3, v2, v1}), {}),
        PolyhedronFace3d(top,    PolyhedronLoop3d({v4, v5, v6, v7}), {}),
        PolyhedronFace3d(front,  PolyhedronLoop3d({v0, v1, v5, v4}), {}),
        PolyhedronFace3d(back,   PolyhedronLoop3d({v3, v7, v6, v2}), {}),
        PolyhedronFace3d(left,   PolyhedronLoop3d({v0, v4, v7, v3}), {}),
        PolyhedronFace3d(right,  PolyhedronLoop3d({v1, v2, v6, v5}), {}),
    });
}

// Displaces two opposite cube corners so that all six quad faces become
// non-planar against their original support planes. This is a stronger variant
// of the single-displaced-vertex deformed cube.
PolyhedronBody BuildDualDeformedUnitCubeBody()
{
    const Point3d v0{0.1, 0.1, -0.1}; // displaced from (0,0,0)
    const Point3d v1{1.0, 0.0, 0.0};
    const Point3d v2{1.0, 1.0, 0.0};
    const Point3d v3{0.0, 1.0, 0.0};
    const Point3d v4{0.0, 0.0, 1.0};
    const Point3d v5{1.0, 0.0, 1.0};
    const Point3d v6{0.9, 0.9, 1.1}; // displaced from (1,1,1)
    const Point3d v7{0.0, 1.0, 1.0};

    const Plane bottom{Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, -1.0}};
    const Plane top   {Point3d{0.0, 0.0, 1.0}, Vector3d{0.0, 0.0,  1.0}};
    const Plane front {Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, -1.0, 0.0}};
    const Plane back  {Point3d{0.0, 1.0, 0.0}, Vector3d{0.0,  1.0, 0.0}};
    const Plane left  {Point3d{0.0, 0.0, 0.0}, Vector3d{-1.0, 0.0, 0.0}};
    const Plane right {Point3d{1.0, 0.0, 0.0}, Vector3d{ 1.0, 0.0, 0.0}};

    return PolyhedronBody({
        PolyhedronFace3d(bottom, PolyhedronLoop3d({v0, v3, v2, v1}), {}),
        PolyhedronFace3d(top,    PolyhedronLoop3d({v4, v5, v6, v7}), {}),
        PolyhedronFace3d(front,  PolyhedronLoop3d({v0, v1, v5, v4}), {}),
        PolyhedronFace3d(back,   PolyhedronLoop3d({v3, v7, v6, v2}), {}),
        PolyhedronFace3d(left,   PolyhedronLoop3d({v0, v4, v7, v3}), {}),
        PolyhedronFace3d(right,  PolyhedronLoop3d({v1, v2, v6, v5}), {}),
    });
}

PolyhedronBody BuildDeformedUnitCubeWithDuplicateLoopBody()
{
    const PolyhedronBody base = BuildDeformedUnitCubeBody();
    std::vector<PolyhedronFace3d> faces = base.Faces();
    if (faces.size() != 6)
    {
        return PolyhedronBody();
    }

    const PolyhedronFace3d leftFace = faces[4];
    std::vector<Point3d> loop = leftFace.OuterLoop().Vertices();
    if (loop.empty())
    {
        return PolyhedronBody();
    }

    loop.insert(loop.begin(), loop.front());
    faces[4] = PolyhedronFace3d(leftFace.SupportPlane(), PolyhedronLoop3d(std::move(loop)), leftFace.Holes());
    return PolyhedronBody(std::move(faces));
}

PolyhedronBody BuildDeformedUnitCubeWithDualDuplicateLoopBody()
{
    const PolyhedronBody base = BuildDeformedUnitCubeWithDuplicateLoopBody();
    std::vector<PolyhedronFace3d> faces = base.Faces();
    if (faces.size() != 6)
    {
        return PolyhedronBody();
    }

    const PolyhedronFace3d frontFace = faces[2];
    std::vector<Point3d> loop = frontFace.OuterLoop().Vertices();
    if (loop.empty())
    {
        return PolyhedronBody();
    }

    loop.insert(loop.begin(), loop.front());
    faces[2] = PolyhedronFace3d(frontFace.SupportPlane(), PolyhedronLoop3d(std::move(loop)), frontFace.Holes());
    return PolyhedronBody(std::move(faces));
}

PolyhedronBody BuildDualDeformedUnitCubeWithDuplicateLoopBody()
{
    const PolyhedronBody base = BuildDualDeformedUnitCubeBody();
    std::vector<PolyhedronFace3d> faces = base.Faces();
    if (faces.size() != 6)
    {
        return PolyhedronBody();
    }

    const PolyhedronFace3d leftFace = faces[4];
    std::vector<Point3d> loop = leftFace.OuterLoop().Vertices();
    if (loop.empty())
    {
        return PolyhedronBody();
    }

    loop.insert(loop.begin(), loop.front());
    faces[4] = PolyhedronFace3d(leftFace.SupportPlane(), PolyhedronLoop3d(std::move(loop)), leftFace.Holes());
    return PolyhedronBody(std::move(faces));
}

PolyhedronBody BuildDualDeformedUnitCubeWithDualDuplicateLoopBody()
{
    const PolyhedronBody base = BuildDualDeformedUnitCubeWithDuplicateLoopBody();
    std::vector<PolyhedronFace3d> faces = base.Faces();
    if (faces.size() != 6)
    {
        return PolyhedronBody();
    }

    const PolyhedronFace3d rightFace = faces[5];
    std::vector<Point3d> loop = rightFace.OuterLoop().Vertices();
    if (loop.empty())
    {
        return PolyhedronBody();
    }

    loop.insert(loop.begin(), loop.front());
    faces[5] = PolyhedronFace3d(rightFace.SupportPlane(), PolyhedronLoop3d(std::move(loop)), rightFace.Holes());
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

PolyhedronBody BuildHoleDominatedNonPlanarHoledFaceBody()
{
    std::vector<Point3d> outer{
        Point3d{0.0, 0.0, 0.12},
        Point3d{6.0, 0.0, 0.00},
        Point3d{6.0, 6.0, 0.10},
        Point3d{0.0, 6.0, 0.00}};
    std::vector<Point3d> hole{
        Point3d{1.0, 1.0, 0.0},
        Point3d{5.0, 1.0, 0.0},
        Point3d{5.0, 5.0, 0.0},
        Point3d{1.0, 5.0, 0.0}};

    const PolyhedronFace3d face(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.2}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d(std::move(outer)),
        {PolyhedronLoop3d(std::move(hole))});
    return PolyhedronBody({face});
}

PolyhedronBody BuildSharedEdgeHoleDominatedMixedContentBody()
{
    const Plane mismatchedPlane =
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.2}, Vector3d{0.0, 0.0, 1.0});

    const Point3d a{0.0, 0.0, 0.0};
    const Point3d b{2.0, 0.0, 0.0};
    const Point3d c{2.0, 2.0, 0.0};
    const Point3d d{0.0, 2.0, 0.0};

    const Point3d e{6.0, 0.0, 0.12};
    const Point3d f{6.0, 2.0, 0.10};

    std::vector<Point3d> hole{
        Point3d{2.6, 0.4, 0.0},
        Point3d{5.4, 0.4, 0.0},
        Point3d{5.4, 1.6, 0.0},
        Point3d{2.6, 1.6, 0.0}};

    const PolyhedronFace3d faceA(
        mismatchedPlane,
        PolyhedronLoop3d({a, b, c, d}));
    const PolyhedronFace3d faceB(
        mismatchedPlane,
        PolyhedronLoop3d({b, e, f, c}),
        {PolyhedronLoop3d(std::move(hole))});
    return PolyhedronBody({faceA, faceB});
}

PolyhedronBody BuildSharedChainHoleDominatedMixedContentBody()
{
    const Plane mismatchedPlane =
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.2}, Vector3d{0.0, 0.0, 1.0});

    const Point3d a{0.0, 0.0, 0.0};
    const Point3d b{2.0, 0.0, 0.0};
    const Point3d c{2.0, 2.0, 0.0};
    const Point3d d{0.0, 2.0, 0.0};

    const Point3d e{6.0, 0.0, 0.12};
    const Point3d f{6.0, 2.0, 0.10};

    const Point3d g{8.0, 0.0, 0.0};
    const Point3d h{8.0, 2.0, 0.0};

    std::vector<Point3d> hole{
        Point3d{2.6, 0.4, 0.0},
        Point3d{5.4, 0.4, 0.0},
        Point3d{5.4, 1.6, 0.0},
        Point3d{2.6, 1.6, 0.0}};

    const PolyhedronFace3d faceA(
        mismatchedPlane,
        PolyhedronLoop3d({a, b, c, d}));
    const PolyhedronFace3d faceB(
        mismatchedPlane,
        PolyhedronLoop3d({b, e, f, c}),
        {PolyhedronLoop3d(std::move(hole))});
    const PolyhedronFace3d faceC(
        mismatchedPlane,
        PolyhedronLoop3d({e, g, h, f}));
    return PolyhedronBody({faceA, faceB, faceC});
}

PolyhedronBody BuildSupportMismatchNearEqualSharedChainHoleDominatedMixedContentBody()
{
    const Plane mismatchedPlane =
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.2}, Vector3d{0.0, 0.0, 1.0});

    const double dx = 2.0e-7;

    const Point3d a{0.0, 0.0, 0.0};
    const Point3d b{2.0, 0.0, 0.0};
    const Point3d c{2.0, 2.0, 0.0};
    const Point3d d{0.0, 2.0, 0.0};

    const Point3d b2{2.0 + dx, 0.0, 0.0};
    const Point3d c2{2.0 + dx, 2.0, 0.0};
    const Point3d e{6.0 + dx, 0.0, 0.12};
    const Point3d f{6.0 + dx, 2.0, 0.10};

    const Point3d e2{6.0, 0.0, 0.0};
    const Point3d f2{6.0, 2.0, 0.0};
    const Point3d g{8.0, 0.0, 0.0};
    const Point3d h{8.0, 2.0, 0.0};

    std::vector<Point3d> hole{
        Point3d{2.6, 0.4, 0.0},
        Point3d{5.4, 0.4, 0.0},
        Point3d{5.4, 1.6, 0.0},
        Point3d{2.6, 1.6, 0.0}};

    const PolyhedronFace3d faceA(
        mismatchedPlane,
        PolyhedronLoop3d({a, b, c, d}));
    const PolyhedronFace3d faceB(
        mismatchedPlane,
        PolyhedronLoop3d({b2, e, f, c2}),
        {PolyhedronLoop3d(std::move(hole))});
    const PolyhedronFace3d faceC(
        mismatchedPlane,
        PolyhedronLoop3d({e2, g, h, f2}));
    return PolyhedronBody({faceA, faceB, faceC});
}

PolyhedronBody BuildSupportMismatchNearEqualSharedChainHoleDominatedMixedContentWithDuplicateHoleBody()
{
    const Plane mismatchedPlane =
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.2}, Vector3d{0.0, 0.0, 1.0});

    const double dx = 2.0e-7;

    const Point3d a{0.0, 0.0, 0.0};
    const Point3d b{2.0, 0.0, 0.0};
    const Point3d c{2.0, 2.0, 0.0};
    const Point3d d{0.0, 2.0, 0.0};

    const Point3d b2{2.0 + dx, 0.0, 0.0};
    const Point3d c2{2.0 + dx, 2.0, 0.0};
    const Point3d e{6.0 + dx, 0.0, 0.12};
    const Point3d f{6.0 + dx, 2.0, 0.10};

    const Point3d e2{6.0, 0.0, 0.0};
    const Point3d f2{6.0, 2.0, 0.0};
    const Point3d g{8.0, 0.0, 0.0};
    const Point3d h{8.0, 2.0, 0.0};

    std::vector<Point3d> hole{
        Point3d{2.6, 0.4, 0.0},
        Point3d{5.4, 0.4, 0.0},
        Point3d{5.4, 0.4, 0.0},
        Point3d{5.4, 1.6, 0.0},
        Point3d{2.6, 1.6, 0.0}};

    const PolyhedronFace3d faceA(
        mismatchedPlane,
        PolyhedronLoop3d({a, b, c, d}));
    const PolyhedronFace3d faceB(
        mismatchedPlane,
        PolyhedronLoop3d({b2, e, f, c2}),
        {PolyhedronLoop3d(std::move(hole))});
    const PolyhedronFace3d faceC(
        mismatchedPlane,
        PolyhedronLoop3d({e2, g, h, f2}));
    return PolyhedronBody({faceA, faceB, faceC});
}

PolyhedronBody BuildSupportMismatchNearEqualSharedChainHoleDominatedFullCompositionBody()
{
    const Plane mismatchedPlane =
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.2}, Vector3d{0.0, 0.0, 1.0});

    const double dx = 2.0e-7;

    const Point3d a{0.0, 0.0, 0.0};
    const Point3d b{2.0, 0.0, 0.0};
    const Point3d c{2.0, 2.0, 0.0};
    const Point3d d{0.0, 2.0, 0.0};

    const Point3d b2{2.0 + dx, 0.0, 0.0};
    const Point3d c2{2.0 + dx, 2.0, 0.0};
    const Point3d m{4.0 + dx, 0.0, 0.0};
    const Point3d e{6.0 + dx, 0.0, 0.12};
    const Point3d f{6.0 + dx, 2.0, 0.10};

    const Point3d e2{6.0, 0.0, 0.0};
    const Point3d f2{6.0, 2.0, 0.0};
    const Point3d g{8.0, 0.0, 0.0};
    const Point3d h{8.0, 2.0, 0.0};

    std::vector<Point3d> hole{
        Point3d{2.6, 0.4, 0.0},
        Point3d{5.4, 0.4, 0.0},
        Point3d{5.4, 0.4, 0.0},
        Point3d{5.4, 1.6, 0.0},
        Point3d{2.6, 1.6, 0.0}};

    const PolyhedronFace3d faceA(
        mismatchedPlane,
        PolyhedronLoop3d({a, b, c, d}));
    const PolyhedronFace3d faceB(
        mismatchedPlane,
        PolyhedronLoop3d({b2, m, e, f, c2}),
        {PolyhedronLoop3d(std::move(hole))});
    const PolyhedronFace3d faceC(
        mismatchedPlane,
        PolyhedronLoop3d({e2, g, h, f2}));
    return PolyhedronBody({faceA, faceB, faceC});
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

PolyhedronBody BuildTinyScaleNonPlanarRepairBody()
{
    const double s = 1e-5;
    std::vector<Point3d> outer{
        Point3d{0.0, 0.0, 0.0},
        Point3d{s, 0.0, 0.0},
        Point3d{s, s, 0.0},
        Point3d{0.0, s, 0.0}};
    // Small-scale non-planar disturbance that still describes a recoverable loop.
    outer[2].z += 2e-6;

    const PolyhedronFace3d face(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 3e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d(std::move(outer)));
    return PolyhedronBody({face});
}

PolyhedronBody BuildTinyScaleNonPlanarHoledRepairBody()
{
    const double s = 1e-5;
    std::vector<Point3d> outer{
        Point3d{0.0, 0.0, 0.0},
        Point3d{s, 0.0, 0.0},
        Point3d{s, s, 0.0},
        Point3d{0.0, s, 0.0}};
    std::vector<Point3d> hole{
        Point3d{0.3 * s, 0.3 * s, 0.0},
        Point3d{0.7 * s, 0.3 * s, 0.0},
        Point3d{0.7 * s, 0.7 * s, 0.0},
        Point3d{0.3 * s, 0.7 * s, 0.0}};

    outer[2].z += 2e-6;
    hole[1].z += 1.5e-6;

    const PolyhedronFace3d face(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 2e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d(std::move(outer)),
        {PolyhedronLoop3d(std::move(hole))});
    return PolyhedronBody({face});
}

PolyhedronBody BuildTinyScaleNonPlanarMultiFaceRepairBody()
{
    const double s = 1e-5;
    std::vector<Point3d> faceA{
        Point3d{0.0, 0.0, 0.0},
        Point3d{s, 0.0, 0.0},
        Point3d{s, s, 0.0},
        Point3d{0.0, s, 0.0}};
    std::vector<Point3d> faceB{
        Point3d{2.0 * s, 0.0, 0.0},
        Point3d{3.0 * s, 0.0, 0.0},
        Point3d{3.0 * s, s, 0.0},
        Point3d{2.0 * s, s, 0.0}};

    faceA[2].z += 2e-6;
    faceB[1].z += 1.4e-6;

    return PolyhedronBody({
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{0.0, 0.0, 2e-6}, Vector3d{0.0, 0.0, 1.0}),
            PolyhedronLoop3d(std::move(faceA))),
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{2.0 * s, 0.0, 2e-6}, Vector3d{0.0, 0.0, 1.0}),
            PolyhedronLoop3d(std::move(faceB)))});
}

PolyhedronBody BuildTinyScaleNonPlanarMixedContentBody()
{
    const double s = 1e-5;
    std::vector<Point3d> outer{
        Point3d{0.0, 0.0, 0.0},
        Point3d{s, 0.0, 0.0},
        Point3d{s, s, 0.0},
        Point3d{0.0, s, 0.0}};
    std::vector<Point3d> hole{
        Point3d{0.3 * s, 0.3 * s, 0.0},
        Point3d{0.7 * s, 0.3 * s, 0.0},
        Point3d{0.7 * s, 0.7 * s, 0.0},
        Point3d{0.3 * s, 0.7 * s, 0.0}};
    std::vector<Point3d> plain{
        Point3d{2.0 * s, 0.0, 0.0},
        Point3d{3.0 * s, 0.0, 0.0},
        Point3d{3.0 * s, s, 0.0},
        Point3d{2.0 * s, s, 0.0}};

    outer[2].z += 2e-6;
    hole[1].z += 1.3e-6;
    plain[0].z += 1.1e-6;

    return PolyhedronBody({
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{0.0, 0.0, 2e-6}, Vector3d{0.0, 0.0, 1.0}),
            PolyhedronLoop3d(std::move(outer)),
            {PolyhedronLoop3d(std::move(hole))}),
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{2.0 * s, 0.0, 2e-6}, Vector3d{0.0, 0.0, 1.0}),
            PolyhedronLoop3d(std::move(plain)))});
}

PolyhedronBody BuildTinyScaleNonPlanarSharedEdgeBody()
{
    const double s = 1e-5;
    const Point3d a{0.0, 0.0, 0.0};
    const Point3d b{s, 0.0, 0.0};
    const Point3d c{s, s, 0.0};
    const Point3d d{0.0, s, 1.8e-6};
    const Point3d e{2.0 * s, 0.0, 1.4e-6};
    const Point3d f{2.0 * s, s, 0.0};

    // Face A and Face B share edge (b, c), while each has tiny non-planar drift.
    const PolyhedronFace3d faceA(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 2e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({a, b, c, d}));
    const PolyhedronFace3d faceB(
        Plane::FromPointAndNormal(Point3d{s, 0.0, 2e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({b, e, f, c}));
    return PolyhedronBody({faceA, faceB});
}

PolyhedronBody BuildTinyScaleNonPlanarSharedEdgeChainBody()
{
    const double s = 1e-5;
    const Point3d a{0.0, 0.0, 0.0};
    const Point3d b{s, 0.0, 0.0};
    const Point3d c{s, s, 0.0};
    const Point3d d{0.0, s, 1.5e-6};
    const Point3d e{2.0 * s, 0.0, 1.1e-6};
    const Point3d f{2.0 * s, s, 0.0};
    const Point3d g{3.0 * s, 0.0, 1.4e-6};
    const Point3d h{3.0 * s, s, 0.0};

    // Face A shares (b,c) with face B; face B shares (e,f) with face C.
    const PolyhedronFace3d faceA(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 2e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({a, b, c, d}));
    const PolyhedronFace3d faceB(
        Plane::FromPointAndNormal(Point3d{s, 0.0, 2e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({b, e, f, c}));
    const PolyhedronFace3d faceC(
        Plane::FromPointAndNormal(Point3d{2.0 * s, 0.0, 2e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({e, g, h, f}));
    return PolyhedronBody({faceA, faceB, faceC});
}

PolyhedronBody BuildTinyScaleSupportMismatchSharedEdgeChainBody()
{
    const double s = 1e-5;
    const Point3d a{0.0, 0.0, 0.0};
    const Point3d b{s, 0.0, 0.0};
    const Point3d c{s, s, 0.0};
    const Point3d d{0.0, s, 1.6e-6};
    const Point3d e{2.0 * s, 0.0, 1.1e-6};
    const Point3d f{2.0 * s, s, 0.0};
    const Point3d g{3.0 * s, 0.0, 1.4e-6};
    const Point3d h{3.0 * s, s, 0.0};

    const Plane mismatchedPlane =
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 4e-6}, Vector3d{0.0, 0.0, 1.0});

    const PolyhedronFace3d faceA(mismatchedPlane, PolyhedronLoop3d({a, b, c, d}));
    const PolyhedronFace3d faceB(mismatchedPlane, PolyhedronLoop3d({b, e, f, c}));
    const PolyhedronFace3d faceC(mismatchedPlane, PolyhedronLoop3d({e, g, h, f}));
    return PolyhedronBody({faceA, faceB, faceC});
}

PolyhedronBody BuildTinyScaleSupportMismatchSharedEdgeChainWithDuplicateBody()
{
    const double s = 1e-5;
    const Point3d a{0.0, 0.0, 0.0};
    const Point3d b{s, 0.0, 0.0};
    const Point3d c{s, s, 0.0};
    const Point3d d{0.0, s, 1.6e-6};
    const Point3d e{2.0 * s, 0.0, 1.1e-6};
    const Point3d f{2.0 * s, s, 0.0};
    const Point3d g{3.0 * s, 0.0, 1.4e-6};
    const Point3d h{3.0 * s, s, 0.0};

    const Plane mismatchedPlane =
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 4e-6}, Vector3d{0.0, 0.0, 1.0});

    const PolyhedronFace3d faceA(mismatchedPlane, PolyhedronLoop3d({a, b, c, d}));
    const PolyhedronFace3d faceB(mismatchedPlane, PolyhedronLoop3d({b, b, e, f, c}));
    const PolyhedronFace3d faceC(mismatchedPlane, PolyhedronLoop3d({e, g, h, f}));
    return PolyhedronBody({faceA, faceB, faceC});
}

PolyhedronBody BuildTinyScaleNonPlanarSharedEdgeChainMixedContentBody()
{
    const double s = 1e-5;
    const Point3d a{0.0, 0.0, 0.0};
    const Point3d b{s, 0.0, 0.0};
    const Point3d c{s, s, 0.0};
    const Point3d d{0.0, s, 1.6e-6};

    const Point3d e{2.0 * s, 0.0, 1.1e-6};
    const Point3d f{2.0 * s, s, 0.0};

    const Point3d g{3.0 * s, 0.0, 1.3e-6};
    const Point3d h{3.0 * s, s, 0.0};

    std::vector<Point3d> hole{
        Point3d{1.2 * s, 0.3 * s, 0.0},
        Point3d{1.8 * s, 0.3 * s, 1.0e-6},
        Point3d{1.8 * s, 0.7 * s, 0.0},
        Point3d{1.2 * s, 0.7 * s, 0.0}};

    // Face A shares (b,c) with face B; face B shares (e,f) with face C.
    const PolyhedronFace3d faceA(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 2e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({a, b, c, d}));
    const PolyhedronFace3d faceB(
        Plane::FromPointAndNormal(Point3d{s, 0.0, 2e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({b, e, f, c}),
        {PolyhedronLoop3d(std::move(hole))});
    const PolyhedronFace3d faceC(
        Plane::FromPointAndNormal(Point3d{2.0 * s, 0.0, 2e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({e, g, h, f}));
    return PolyhedronBody({faceA, faceB, faceC});
}

PolyhedronBody BuildTinyScaleNonPlanarSharedEdgeChainWithDuplicateLoopBody()
{
    const double s = 1e-5;
    const Point3d a{0.0, 0.0, 0.0};
    const Point3d b{s, 0.0, 0.0};
    const Point3d c{s, s, 0.0};
    const Point3d d{0.0, s, 1.4e-6};

    const Point3d e{2.0 * s, 0.0, 1.2e-6};
    const Point3d f{2.0 * s, s, 0.0};

    const Point3d g{3.0 * s, 0.0, 1.1e-6};
    const Point3d h{3.0 * s, s, 0.0};

    // Duplicate vertex in middle face loop to exercise normalization + refit.
    const PolyhedronFace3d faceA(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 2e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({a, b, c, d}));
    const PolyhedronFace3d faceB(
        Plane::FromPointAndNormal(Point3d{s, 0.0, 2e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({b, e, e, f, c}));
    const PolyhedronFace3d faceC(
        Plane::FromPointAndNormal(Point3d{2.0 * s, 0.0, 2e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({e, g, h, f}));

    return PolyhedronBody({faceA, faceB, faceC});
}

PolyhedronBody BuildTinyScaleSharedEdgeChainMixedContentWithDuplicateHoleBody()
{
    const double s = 1e-5;
    const Point3d a{0.0, 0.0, 0.0};
    const Point3d b{s, 0.0, 0.0};
    const Point3d c{s, s, 0.0};
    const Point3d d{0.0, s, 1.5e-6};

    const Point3d e{2.0 * s, 0.0, 1.2e-6};
    const Point3d f{2.0 * s, s, 0.0};

    const Point3d g{3.0 * s, 0.0, 1.1e-6};
    const Point3d h{3.0 * s, s, 0.0};

    std::vector<Point3d> hole{
        Point3d{1.2 * s, 0.3 * s, 0.0},
        Point3d{1.8 * s, 0.3 * s, 1.0e-6},
        Point3d{1.8 * s, 0.3 * s, 1.0e-6},
        Point3d{1.8 * s, 0.7 * s, 0.0},
        Point3d{1.2 * s, 0.7 * s, 0.0}};

    const PolyhedronFace3d faceA(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 2e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({a, b, c, d}));
    const PolyhedronFace3d faceB(
        Plane::FromPointAndNormal(Point3d{s, 0.0, 2e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({b, e, f, c}),
        {PolyhedronLoop3d(std::move(hole))});
    const PolyhedronFace3d faceC(
        Plane::FromPointAndNormal(Point3d{2.0 * s, 0.0, 2e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({e, g, h, f}));

    return PolyhedronBody({faceA, faceB, faceC});
}

PolyhedronBody BuildTinyScaleSharedChainMixedContentWithCollinearLeadingBody()
{
    const double s = 1e-5;
    const Point3d a{0.0, 0.0, 0.0};
    const Point3d b{s, 0.0, 0.0};
    const Point3d c{s, s, 0.0};
    const Point3d d{0.0, s, 1.6e-6};

    const Point3d e{2.0 * s, 0.0, 0.0};
    const Point3d f{2.0 * s, s, 1.3e-6};
    const Point3d g{3.0 * s, 0.0, 1.1e-6};
    const Point3d h{3.0 * s, s, 0.0};

    // Leading points b -> m -> e are collinear to exercise robust normal fallback
    // while the loop remains mildly non-planar at tiny scale.
    const Point3d m{1.5 * s, 0.0, 0.0};

    std::vector<Point3d> hole{
        Point3d{1.2 * s, 0.3 * s, 0.0},
        Point3d{1.8 * s, 0.3 * s, 1.0e-6},
        Point3d{1.8 * s, 0.7 * s, 0.0},
        Point3d{1.2 * s, 0.7 * s, 0.0}};

    const PolyhedronFace3d faceA(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 2e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({a, b, c, d}));
    const PolyhedronFace3d faceB(
        Plane::FromPointAndNormal(Point3d{s, 0.0, 2e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({b, m, e, f, c}),
        {PolyhedronLoop3d(std::move(hole))});
    const PolyhedronFace3d faceC(
        Plane::FromPointAndNormal(Point3d{2.0 * s, 0.0, 2e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({e, g, h, f}));

    return PolyhedronBody({faceA, faceB, faceC});
}

PolyhedronBody BuildTinyScaleSharedChainMixedContentSupportPlaneMismatchBody()
{
    const double s = 1e-5;
    const Point3d a{0.0, 0.0, 0.0};
    const Point3d b{s, 0.0, 0.0};
    const Point3d c{s, s, 0.0};
    const Point3d d{0.0, s, 1.5e-6};

    const Point3d e{2.0 * s, 0.0, 1.2e-6};
    const Point3d f{2.0 * s, s, 0.0};
    const Point3d g{3.0 * s, 0.0, 1.0e-6};
    const Point3d h{3.0 * s, s, 0.0};

    std::vector<Point3d> hole{
        Point3d{1.2 * s, 0.3 * s, 0.0},
        Point3d{1.8 * s, 0.3 * s, 8e-7},
        Point3d{1.8 * s, 0.7 * s, 0.0},
        Point3d{1.2 * s, 0.7 * s, 0.0}};

    // Deliberately offset support planes to require refit in combination with
    // shared-chain mixed-content tiny-scale projection repair.
    const PolyhedronFace3d faceA(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 4e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({a, b, c, d}));
    const PolyhedronFace3d faceB(
        Plane::FromPointAndNormal(Point3d{s, 0.0, 4e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({b, e, f, c}),
        {PolyhedronLoop3d(std::move(hole))});
    const PolyhedronFace3d faceC(
        Plane::FromPointAndNormal(Point3d{2.0 * s, 0.0, 4e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({e, g, h, f}));

    return PolyhedronBody({faceA, faceB, faceC});
}

PolyhedronBody BuildTinyScaleSharedChainMixedContentSupportMismatchWithDuplicateHoleBody()
{
    const double s = 1e-5;
    const Point3d a{0.0, 0.0, 0.0};
    const Point3d b{s, 0.0, 0.0};
    const Point3d c{s, s, 0.0};
    const Point3d d{0.0, s, 1.5e-6};

    const Point3d e{2.0 * s, 0.0, 1.2e-6};
    const Point3d f{2.0 * s, s, 0.0};
    const Point3d g{3.0 * s, 0.0, 1.0e-6};
    const Point3d h{3.0 * s, s, 0.0};

    std::vector<Point3d> hole{
        Point3d{1.2 * s, 0.3 * s, 0.0},
        Point3d{1.8 * s, 0.3 * s, 8e-7},
        Point3d{1.8 * s, 0.3 * s, 8e-7},
        Point3d{1.8 * s, 0.7 * s, 0.0},
        Point3d{1.2 * s, 0.7 * s, 0.0}};

    const PolyhedronFace3d faceA(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 4e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({a, b, c, d}));
    const PolyhedronFace3d faceB(
        Plane::FromPointAndNormal(Point3d{s, 0.0, 4e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({b, e, f, c}),
        {PolyhedronLoop3d(std::move(hole))});
    const PolyhedronFace3d faceC(
        Plane::FromPointAndNormal(Point3d{2.0 * s, 0.0, 4e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({e, g, h, f}));

    return PolyhedronBody({faceA, faceB, faceC});
}

PolyhedronBody BuildTinyScaleSharedChainSupportMismatchAndCollinearBody()
{
    const double s = 1e-5;
    const Point3d a{0.0, 0.0, 0.0};
    const Point3d b{s, 0.0, 0.0};
    const Point3d c{s, s, 0.0};
    const Point3d d{0.0, s, 1.5e-6};

    const Point3d e{2.0 * s, 0.0, 0.0};
    const Point3d f{2.0 * s, s, 1.2e-6};
    const Point3d g{3.0 * s, 0.0, 1.1e-6};
    const Point3d h{3.0 * s, s, 0.0};
    const Point3d m{1.5 * s, 0.0, 0.0};

    std::vector<Point3d> hole{
        Point3d{1.2 * s, 0.3 * s, 0.0},
        Point3d{1.8 * s, 0.3 * s, 8e-7},
        Point3d{1.8 * s, 0.7 * s, 0.0},
        Point3d{1.2 * s, 0.7 * s, 0.0}};

    const PolyhedronFace3d faceA(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 4e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({a, b, c, d}));
    const PolyhedronFace3d faceB(
        Plane::FromPointAndNormal(Point3d{s, 0.0, 4e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({b, m, e, f, c}),
        {PolyhedronLoop3d(std::move(hole))});
    const PolyhedronFace3d faceC(
        Plane::FromPointAndNormal(Point3d{2.0 * s, 0.0, 4e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({e, g, h, f}));

    return PolyhedronBody({faceA, faceB, faceC});
}

PolyhedronBody BuildTinyScaleSharedChainFullCompositionBody()
{
    const double s = 1e-5;
    const Point3d a{0.0, 0.0, 0.0};
    const Point3d b{s, 0.0, 0.0};
    const Point3d c{s, s, 0.0};
    const Point3d d{0.0, s, 1.5e-6};

    const Point3d e{2.0 * s, 0.0, 0.0};
    const Point3d f{2.0 * s, s, 1.2e-6};
    const Point3d g{3.0 * s, 0.0, 1.1e-6};
    const Point3d h{3.0 * s, s, 0.0};
    const Point3d m{1.5 * s, 0.0, 0.0};

    std::vector<Point3d> hole{
        Point3d{1.2 * s, 0.3 * s, 0.0},
        Point3d{1.8 * s, 0.3 * s, 8e-7},
        Point3d{1.8 * s, 0.3 * s, 8e-7},
        Point3d{1.8 * s, 0.7 * s, 0.0},
        Point3d{1.2 * s, 0.7 * s, 0.0}};

    const PolyhedronFace3d faceA(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 4e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({a, b, c, d}));
    const PolyhedronFace3d faceB(
        Plane::FromPointAndNormal(Point3d{s, 0.0, 4e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({b, m, e, f, c}),
        {PolyhedronLoop3d(std::move(hole))});
    const PolyhedronFace3d faceC(
        Plane::FromPointAndNormal(Point3d{2.0 * s, 0.0, 4e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({e, g, h, f}));

    return PolyhedronBody({faceA, faceB, faceC});
}

PolyhedronBody BuildTinyScaleSharedChainDualDuplicateFullCompositionBody()
{
    const double s = 1e-5;
    const Point3d a{0.0, 0.0, 0.0};
    const Point3d b{s, 0.0, 0.0};
    const Point3d c{s, s, 0.0};
    const Point3d d{0.0, s, 1.6e-6};

    const Point3d e{2.0 * s, 0.0, 0.0};
    const Point3d f{2.0 * s, s, 1.2e-6};
    const Point3d g{3.0 * s, 0.0, 1.0e-6};
    const Point3d h{3.0 * s, s, 0.0};
    const Point3d m{1.5 * s, 0.0, 0.0};

    std::vector<Point3d> hole{
        Point3d{1.2 * s, 0.3 * s, 0.0},
        Point3d{1.8 * s, 0.3 * s, 8e-7},
        Point3d{1.8 * s, 0.3 * s, 8e-7},
        Point3d{1.8 * s, 0.7 * s, 0.0},
        Point3d{1.2 * s, 0.7 * s, 0.0}};

    const PolyhedronFace3d faceA(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 4e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({a, b, b, c, d}));
    const PolyhedronFace3d faceB(
        Plane::FromPointAndNormal(Point3d{s, 0.0, 4e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({b, m, e, f, c}),
        {PolyhedronLoop3d(std::move(hole))});
    const PolyhedronFace3d faceC(
        Plane::FromPointAndNormal(Point3d{2.0 * s, 0.0, 4e-6}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d({e, g, h, f}));

    return PolyhedronBody({faceA, faceB, faceC});
}

PolyhedronBody BuildTinyScaleClosedTetrahedronBody()
{
    // A tiny-scale tetrahedron (4 triangular faces, 6 edges, 4 vertices) where
    // every support plane is identical and mismatched to force per-face refit.
    // All 6 edges are each shared by exactly two faces, so after repair the
    // resulting BrepBody should form a deterministically closed shell.
    const double s = 1e-5;
    const Point3d v0{0.0,       0.0,        0.0};
    const Point3d v1{s,         0.0,        1.2e-6};
    const Point3d v2{0.5 * s,  0.866 * s,  0.8e-6};
    const Point3d v3{0.5 * s,  0.289 * s,  0.816 * s};

    // Deliberately flat support plane (z=2e-6, normal +z) while actual face
    // normals vary — the per-face refit must compute the correct orientation.
    const Plane mismatchedPlane =
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 2e-6}, Vector3d{0.0, 0.0, 1.0});

    const PolyhedronFace3d f0(mismatchedPlane, PolyhedronLoop3d({v0, v1, v2}));
    const PolyhedronFace3d f1(mismatchedPlane, PolyhedronLoop3d({v0, v3, v1}));
    const PolyhedronFace3d f2(mismatchedPlane, PolyhedronLoop3d({v1, v3, v2}));
    const PolyhedronFace3d f3(mismatchedPlane, PolyhedronLoop3d({v0, v2, v3}));

    return PolyhedronBody({f0, f1, f2, f3});
}

PolyhedronBody BuildTinyScaleTriangularFaceChainBody()
{
    // Three triangular faces sharing edges in a strip:
    //   T1=(a,b,c), T2=(b,d,c), T3=(c,d,e)
    //   T1/T2 share edge b-c; T2/T3 share edge c-d (traversed reversed).
    // Key property: every shared vertex is one of the 3 defining vertices of
    // EVERY triangle that contains it, so after per-face refit its distance to
    // that face's refit-plane is exactly 0 — no projection occurs and the
    // original 3D position is preserved unchanged. FindOrAddBrepVertex therefore
    // merges all shared vertices exactly, giving VertexCount=5, EdgeCount=7.
    const double s = 1e-5;
    const Point3d a{0.0,       0.0,       0.0};
    const Point3d b{s,         0.0,       1.3e-6};
    const Point3d c{2.0 * s,   0.0,       0.9e-6};
    const Point3d d{1.5 * s,   s,         1.1e-6};
    const Point3d e{3.0 * s,   0.5 * s,   0.7e-6};

    const Plane mismatchedPlane =
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 2e-6}, Vector3d{0.0, 0.0, 1.0});

    return PolyhedronBody({
        PolyhedronFace3d(mismatchedPlane, PolyhedronLoop3d({a, b, c})),
        PolyhedronFace3d(mismatchedPlane, PolyhedronLoop3d({b, d, c})),
        PolyhedronFace3d(mismatchedPlane, PolyhedronLoop3d({c, d, e}))});
}

PolyhedronBody BuildTinyScaleTriangularFanBody()
{
    // Four triangular faces sharing a common apex vertex, arranged like a
    // square pyramid cap (open base). All support planes mismatched.
    //   T0=(apex,v0,v1), T1=(apex,v1,v2), T2=(apex,v2,v3), T3=(apex,v3,v0)
    // Each radial edge (apex-vk) is shared by exactly two triangles, the four
    // outer edges (vk-v(k+1 mod 4)) are each used once.
    // Key: apex is one of the 3 defining vertices of all 4 triangles → after
    // per-face refit its distance to each refit-plane is exactly 0 → apex is
    // never projected → it is preserved at its original position in all faces
    // → FindOrAddBrepVertex merges it exactly once.
    // Expected: VertexCount=5, EdgeCount=8 (4 radial + 4 outer).
    const double s = 1e-5;
    const Point3d apex{0.0,  0.0,  s};
    const Point3d v0  {s,    0.0,  0.0};
    const Point3d v1  {0.0,  s,    0.0};
    const Point3d v2  {-s,   0.0,  0.0};
    const Point3d v3  {0.0,  -s,   0.0};

    const Plane mismatchedPlane =
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 2e-6}, Vector3d{0.0, 0.0, 1.0});

    return PolyhedronBody({
        PolyhedronFace3d(mismatchedPlane, PolyhedronLoop3d({apex, v0, v1})),
        PolyhedronFace3d(mismatchedPlane, PolyhedronLoop3d({apex, v1, v2})),
        PolyhedronFace3d(mismatchedPlane, PolyhedronLoop3d({apex, v2, v3})),
        PolyhedronFace3d(mismatchedPlane, PolyhedronLoop3d({apex, v3, v0}))});
}

PolyhedronBody BuildNearEqualSharedEdgeQuadPairBody()
{
    // Two planar quads share one edge geometrically, but one face stores
    // near-equal (<eps) perturbed coordinates for shared vertices.
    // Conversion should keep shared topology and place shared Brep vertices at
    // representative-id global average points instead of first-face points.
    const double s = 1e-5;
    const double dx = 2e-7;

    const Point3d a{0.0, 0.0, 0.0};
    const Point3d b{s, 0.0, 0.0};
    const Point3d c{s, s, 0.0};
    const Point3d d{0.0, s, 0.0};

    const Point3d b2{s + dx, 0.0, 0.0};
    const Point3d c2{s + dx, s, 0.0};
    const Point3d e{2.0 * s, 0.0, 0.0};
    const Point3d f{2.0 * s, s, 0.0};

    const Plane plane = Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0});
    return PolyhedronBody({
        PolyhedronFace3d(plane, PolyhedronLoop3d({a, b, c, d})),
        PolyhedronFace3d(plane, PolyhedronLoop3d({b2, e, f, c2}))});
}

PolyhedronBody BuildSupportMismatchNearEqualSharedEdgeQuadPairBody()
{
    // Same near-equal shared-edge setup as BuildNearEqualSharedEdgeQuadPairBody,
    // but with intentionally mismatched support planes to force repair path.
    const double s = 1e-5;
    const double dx = 2e-7;

    const Point3d a{0.0, 0.0, 0.0};
    const Point3d b{s, 0.0, 0.0};
    const Point3d c{s, s, 0.0};
    const Point3d d{0.0, s, 0.0};

    const Point3d b2{s + dx, 0.0, 0.0};
    const Point3d c2{s + dx, s, 0.0};
    const Point3d e{2.0 * s, 0.0, 0.0};
    const Point3d f{2.0 * s, s, 0.0};

    const Plane mismatched = Plane::FromPointAndNormal(Point3d{0.0, 0.0, 3e-6}, Vector3d{0.0, 0.0, 1.0});
    return PolyhedronBody({
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({a, b, c, d})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({b2, e, f, c2}))});
}

PolyhedronBody BuildSupportMismatchNearEqualApexTriangularFanBody()
{
    // Four triangles share a near-equal apex (<eps perturbations), with
    // mismatched support planes forcing repair before Brep conversion.
    // Representative-id averaging should converge the shared apex to the
    // cross-face average target point.
    const double s = 1e-5;
    const Point3d apex0{0.0,    0.0, s};
    const Point3d apex1{1.0e-7, 0.0, s};
    const Point3d apex2{2.0e-7, 0.0, s};
    const Point3d apex3{-1.0e-7, 0.0, s};

    const Point3d v0{s, 0.0, 0.0};
    const Point3d v1{0.0, s, 0.0};
    const Point3d v2{-s, 0.0, 0.0};
    const Point3d v3{0.0, -s, 0.0};

    const Plane mismatched =
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 3e-6}, Vector3d{0.0, 0.0, 1.0});

    return PolyhedronBody({
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({apex0, v0, v1})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({apex1, v1, v2})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({apex2, v2, v3})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({apex3, v3, v0}))});
}

PolyhedronBody BuildSupportMismatchNearEqualSharedEdgeChainBody()
{
    // Three quads in a shared-edge chain with near-equal (<eps) perturbations
    // on middle-face shared vertices. Mismatched support planes force repair.
    const double s = 1e-5;
    const double dx = 2e-7;

    const Point3d a{0.0, 0.0, 0.0};
    const Point3d b{s, 0.0, 0.0};
    const Point3d c{s, s, 0.0};
    const Point3d d{0.0, s, 0.0};

    const Point3d b2{s + dx, 0.0, 0.0};
    const Point3d c2{s + dx, s, 0.0};
    const Point3d e{2.0 * s + dx, 0.0, 0.0};
    const Point3d f{2.0 * s + dx, s, 0.0};

    const Point3d e2{2.0 * s, 0.0, 0.0};
    const Point3d f2{2.0 * s, s, 0.0};
    const Point3d g{3.0 * s, 0.0, 0.0};
    const Point3d h{3.0 * s, s, 0.0};

    const Plane mismatched =
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 4e-6}, Vector3d{0.0, 0.0, 1.0});

    return PolyhedronBody({
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({a, b, c, d})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({b2, e, f, c2})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({e2, g, h, f2}))});
}

PolyhedronBody BuildSupportMismatchNearEqualSharedCornerQuadFanBody()
{
    // Three quads only share one near-equal corner (<eps), without shared
    // edges. Mismatched support planes force conversion repair.
    const double s = 1e-5;

    const Point3d p0a{0.0, 0.0, 0.0};
    const Point3d p0b{1.0e-7, 0.0, 0.0};
    const Point3d p0c{-1.0e-7, 0.0, 0.0};

    const Point3d a1{s, 0.0, 0.0};
    const Point3d a2{s, s, 0.0};
    const Point3d a3{0.0, s, 0.0};

    const Point3d b1{-s, 0.0, 0.0};
    const Point3d b2{-s, s, 0.0};
    const Point3d b3{0.0, s, 0.0};

    const Point3d c1{0.0, -s, 0.0};
    const Point3d c2{s, -s, 0.0};
    const Point3d c3{s, 0.0, 0.0};

    const Plane mismatched =
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 4e-6}, Vector3d{0.0, 0.0, 1.0});

    return PolyhedronBody({
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({p0a, a1, a2, a3})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({p0b, b1, b2, b3})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({p0c, c1, c2, c3}))});
}

PolyhedronBody BuildSupportMismatchNearEqualClosedTetrahedronBody()
{
    // Closed tetrahedron where one logical vertex appears with near-equal
    // (<eps) perturbations across faces. Mismatched support planes force repair
    // before conversion. Representative averaging should merge the perturbed
    // variants into one shared Brep vertex.
    const double s = 1e-5;
    const Point3d v0a{0.0, 0.0, 0.0};
    const Point3d v0b{1.0e-7, 0.0, 0.0};
    const Point3d v0c{-1.0e-7, 0.0, 0.0};

    const Point3d v1{s, 0.0, 1.2e-6};
    const Point3d v2{0.5 * s, 0.866 * s, 0.8e-6};
    const Point3d v3{0.5 * s, 0.289 * s, 0.816 * s};

    const Plane mismatched =
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 3e-6}, Vector3d{0.0, 0.0, 1.0});

    return PolyhedronBody({
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0a, v1, v2})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0b, v3, v1})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v1, v3, v2})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0c, v2, v3}))});
}

PolyhedronBody BuildSupportMismatchNearEqualSharedEdgeChainWithDuplicateBody()
{
    // Three-face shared-edge chain with near-equal perturbation on shared
    // vertices and duplicate leading vertex in middle face to force loop
    // normalization together with representative-average placement.
    const double s = 1e-5;
    const double dx = 2e-7;

    const Point3d a{0.0, 0.0, 0.0};
    const Point3d b{s, 0.0, 0.0};
    const Point3d c{s, s, 0.0};
    const Point3d d{0.0, s, 0.0};

    const Point3d b2{s + dx, 0.0, 0.0};
    const Point3d c2{s + dx, s, 0.0};
    const Point3d e{2.0 * s + dx, 0.0, 0.0};
    const Point3d f{2.0 * s + dx, s, 0.0};

    const Point3d e2{2.0 * s, 0.0, 0.0};
    const Point3d f2{2.0 * s, s, 0.0};
    const Point3d g{3.0 * s, 0.0, 0.0};
    const Point3d h{3.0 * s, s, 0.0};

    const Plane mismatched =
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 4e-6}, Vector3d{0.0, 0.0, 1.0});

    return PolyhedronBody({
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({a, b, c, d})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({b2, b2, e, f, c2})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({e2, g, h, f2}))});
}

PolyhedronBody BuildSupportMismatchNearEqualSharedChainMixedContentWithDuplicateHoleBody()
{
    // Shared-edge chain mixed-content variant with near-equal perturbations on
    // shared vertices and duplicate hole vertices on middle face.
    const double s = 1e-5;
    const double dx = 2e-7;

    const Point3d a{0.0, 0.0, 0.0};
    const Point3d b{s, 0.0, 0.0};
    const Point3d c{s, s, 0.0};
    const Point3d d{0.0, s, 1.5e-6};

    const Point3d e{2.0 * s + dx, 0.0, 1.2e-6};
    const Point3d f{2.0 * s + dx, s, 0.0};
    const Point3d g{3.0 * s, 0.0, 1.0e-6};
    const Point3d h{3.0 * s, s, 0.0};

    const Point3d e2{2.0 * s, 0.0, 1.2e-6};
    const Point3d f2{2.0 * s, s, 0.0};

    std::vector<Point3d> hole{
        Point3d{1.2 * s, 0.3 * s, 0.0},
        Point3d{1.8 * s, 0.3 * s, 8e-7},
        Point3d{1.8 * s, 0.3 * s, 8e-7},
        Point3d{1.8 * s, 0.7 * s, 0.0},
        Point3d{1.2 * s, 0.7 * s, 0.0}};

    const Plane mismatched =
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 4e-6}, Vector3d{0.0, 0.0, 1.0});

    return PolyhedronBody({
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({a, b, c, d})),
        PolyhedronFace3d(
            mismatched,
            PolyhedronLoop3d({b + Vector3d{dx, 0.0, 0.0}, e, f, c + Vector3d{dx, 0.0, 0.0}}),
            {PolyhedronLoop3d(std::move(hole))}),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({e2, g, h, f2}))});
}

PolyhedronBody BuildSupportMismatchNearEqualSharedChainFullCompositionBody()
{
    // Shared-chain full composition: support-plane mismatch + collinear-leading
    // + duplicate-hole normalization + near-equal shared-edge perturbations.
    const double s = 1e-5;
    const double dx = 2e-7;

    const Point3d a{0.0, 0.0, 0.0};
    const Point3d b{s, 0.0, 0.0};
    const Point3d c{s, s, 0.0};
    const Point3d d{0.0, s, 1.5e-6};

    const Point3d e{2.0 * s + dx, 0.0, 0.0};
    const Point3d f{2.0 * s + dx, s, 1.2e-6};
    const Point3d g{3.0 * s, 0.0, 1.1e-6};
    const Point3d h{3.0 * s, s, 0.0};
    const Point3d m{1.5 * s + dx, 0.0, 0.0};

    const Point3d e2{2.0 * s, 0.0, 0.0};
    const Point3d f2{2.0 * s, s, 1.2e-6};

    std::vector<Point3d> hole{
        Point3d{1.2 * s, 0.3 * s, 0.0},
        Point3d{1.8 * s, 0.3 * s, 8e-7},
        Point3d{1.8 * s, 0.3 * s, 8e-7},
        Point3d{1.8 * s, 0.7 * s, 0.0},
        Point3d{1.2 * s, 0.7 * s, 0.0}};

    const Plane mismatched =
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 4e-6}, Vector3d{0.0, 0.0, 1.0});

    return PolyhedronBody({
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({a, b, c, d})),
        PolyhedronFace3d(
            mismatched,
            PolyhedronLoop3d({b + Vector3d{dx, 0.0, 0.0}, m, e, f, c + Vector3d{dx, 0.0, 0.0}}),
            {PolyhedronLoop3d(std::move(hole))}),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({e2, g, h, f2}))});
}

PolyhedronBody BuildSupportMismatchNearEqualSharedChainDualDuplicateFullCompositionBody()
{
    // Shared-chain full composition with both outer and hole duplicate
    // normalization + near-equal shared-edge perturbations.
    const double s = 1e-5;
    const double dx = 2e-7;

    const Point3d a{0.0, 0.0, 0.0};
    const Point3d b{s, 0.0, 0.0};
    const Point3d c{s, s, 0.0};
    const Point3d d{0.0, s, 1.6e-6};

    const Point3d e{2.0 * s + dx, 0.0, 0.0};
    const Point3d f{2.0 * s + dx, s, 1.2e-6};
    const Point3d g{3.0 * s, 0.0, 1.0e-6};
    const Point3d h{3.0 * s, s, 0.0};
    const Point3d m{1.5 * s + dx, 0.0, 0.0};

    const Point3d e2{2.0 * s, 0.0, 0.0};
    const Point3d f2{2.0 * s, s, 1.2e-6};

    std::vector<Point3d> hole{
        Point3d{1.2 * s, 0.3 * s, 0.0},
        Point3d{1.8 * s, 0.3 * s, 8e-7},
        Point3d{1.8 * s, 0.3 * s, 8e-7},
        Point3d{1.8 * s, 0.7 * s, 0.0},
        Point3d{1.2 * s, 0.7 * s, 0.0}};

    const Plane mismatched =
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 4e-6}, Vector3d{0.0, 0.0, 1.0});

    return PolyhedronBody({
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({a, b, b, c, d})),
        PolyhedronFace3d(
            mismatched,
            PolyhedronLoop3d({b + Vector3d{dx, 0.0, 0.0}, m, e, f, c + Vector3d{dx, 0.0, 0.0}}),
            {PolyhedronLoop3d(std::move(hole))}),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({e2, g, h, f2}))});
}

PolyhedronBody BuildSupportMismatchNearEqualClosedTetrahedronDualVerticesBody()
{
    // Closed tetrahedron where two logical shared vertices (v0, v1) are each
    // represented by near-equal (<eps) variants across faces.
    const double s = 1e-5;

    const Point3d v0a{0.0, 0.0, 0.0};
    const Point3d v0b{1.0e-7, 0.0, 0.0};
    const Point3d v0c{-1.0e-7, 0.0, 0.0};

    const Point3d v1a{s + 2.0e-7, 0.0, 1.2e-6};
    const Point3d v1b{s - 1.0e-7, 0.0, 1.2e-6};
    const Point3d v1c{s, 0.0, 1.2e-6};

    const Point3d v2{0.5 * s, 0.866 * s, 0.8e-6};
    const Point3d v3{0.5 * s, 0.289 * s, 0.816 * s};

    const Plane mismatched =
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 3e-6}, Vector3d{0.0, 0.0, 1.0});

    return PolyhedronBody({
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0a, v1a, v2})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0b, v3, v1b})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v1c, v3, v2})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0c, v2, v3}))});
}

PolyhedronBody BuildSupportMismatchNearEqualClosedTetrahedronDualVerticesWithDuplicateLoopBody()
{
    // Reuse dual-vertices near-equal closed-tetra and inject one duplicate
    // leading vertex on a triangular face.
    const PolyhedronBody base = BuildSupportMismatchNearEqualClosedTetrahedronDualVerticesBody();
    std::vector<PolyhedronFace3d> faces = base.Faces();
    if (faces.size() != 4)
    {
        return PolyhedronBody();
    }

    const PolyhedronFace3d face = faces[2];
    std::vector<Point3d> loop = face.OuterLoop().Vertices();
    if (loop.empty())
    {
        return PolyhedronBody();
    }

    loop.insert(loop.begin(), loop.front());
    faces[2] = PolyhedronFace3d(face.SupportPlane(), PolyhedronLoop3d(std::move(loop)));
    return PolyhedronBody(std::move(faces));
}
PolyhedronBody BuildSupportMismatchNearEqualClosedTetrahedronDualVerticesWithDualDuplicateLoopBody()
{
    // Start from the single-duplicate dual-vertices near-equal closed-tetra
    // variant and inject another duplicate leading vertex on a second face.
    const PolyhedronBody base = BuildSupportMismatchNearEqualClosedTetrahedronDualVerticesWithDuplicateLoopBody();
    std::vector<PolyhedronFace3d> faces = base.Faces();
    if (faces.size() != 4)
    {
        return PolyhedronBody();
    }
    const PolyhedronFace3d face = faces[1];
    std::vector<Point3d> loop = face.OuterLoop().Vertices();
    if (loop.empty())
    {
        return PolyhedronBody();
    }
    loop.insert(loop.begin(), loop.front());
    faces[1] = PolyhedronFace3d(face.SupportPlane(), PolyhedronLoop3d(std::move(loop)));
    return PolyhedronBody(std::move(faces));
}
PolyhedronBody BuildSupportMismatchNearEqualClosedTetrahedronAllVerticesBody()
{
    // Closed tetrahedron where all four logical shared vertices each have
    // near-equal (<eps) position variants across their three incident faces,
    // exercising simultaneous representative-average placement for every
    // shared vertex at once.
    const double s = 1e-5;

    // v0 near {0,0,0}: faces 0,1,3
    const Point3d v0a{0.0, 0.0, 0.0};
    const Point3d v0b{1.0e-7, 0.0, 0.0};
    const Point3d v0c{-1.0e-7, 0.0, 0.0};
    // avg x = 0

    // v1 near {s,0,1.2e-6}: faces 0,1,2
    const Point3d v1a{s + 2.0e-7, 0.0, 1.2e-6};
    const Point3d v1b{s - 1.0e-7, 0.0, 1.2e-6};
    const Point3d v1c{s, 0.0, 1.2e-6};
    // avg x = s + 1e-7/3

    // v2 near {0.5*s, 0.866*s, 0.8e-6}: faces 0,2,3
    const Point3d v2a{0.5 * s + 1.0e-7, 0.866 * s, 0.8e-6};
    const Point3d v2b{0.5 * s - 1.0e-7, 0.866 * s, 0.8e-6};
    const Point3d v2c{0.5 * s, 0.866 * s + 1.0e-7, 0.8e-6};
    // avg x = 0.5*s, avg y = 0.866*s + 1e-7/3

    // v3 near {0.5*s, 0.289*s, 0.816*s}: faces 1,2,3
    const Point3d v3a{0.5 * s + 2.0e-7, 0.289 * s, 0.816 * s};
    const Point3d v3b{0.5 * s - 1.0e-7, 0.289 * s, 0.816 * s};
    const Point3d v3c{0.5 * s, 0.289 * s, 0.816 * s};
    // avg x = 0.5*s + 1e-7/3

    const Plane mismatched =
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 3e-6}, Vector3d{0.0, 0.0, 1.0});

    return PolyhedronBody({
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0a, v1a, v2a})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0b, v3a, v1b})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v1c, v3b, v2b})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0c, v2c, v3c}))});
}

PolyhedronBody BuildSupportMismatchNearEqualClosedTetrahedronAllVerticesWithDuplicateLoopBody()
{
    // Reuse all-vertices near-equal closed-tetra and inject a duplicate
    // leading vertex on one triangular face.
    const PolyhedronBody base = BuildSupportMismatchNearEqualClosedTetrahedronAllVerticesBody();
    std::vector<PolyhedronFace3d> faces = base.Faces();
    if (faces.size() != 4)
    {
        return PolyhedronBody();
    }

    const PolyhedronFace3d face = faces[2];
    std::vector<Point3d> loop = face.OuterLoop().Vertices();
    if (loop.empty())
    {
        return PolyhedronBody();
    }

    loop.insert(loop.begin(), loop.front());
    faces[2] = PolyhedronFace3d(face.SupportPlane(), PolyhedronLoop3d(std::move(loop)));
    return PolyhedronBody(std::move(faces));
}
PolyhedronBody BuildSupportMismatchNearEqualClosedTetrahedronAllVerticesWithDualDuplicateLoopBody()
{
    // Start from the single-duplicate all-vertices near-equal closed-tetra
    // variant and inject another duplicate leading vertex on a second face.
    const PolyhedronBody base = BuildSupportMismatchNearEqualClosedTetrahedronAllVerticesWithDuplicateLoopBody();
    std::vector<PolyhedronFace3d> faces = base.Faces();
    if (faces.size() != 4)
    {
        return PolyhedronBody();
    }
    const PolyhedronFace3d face = faces[1];
    std::vector<Point3d> loop = face.OuterLoop().Vertices();
    if (loop.empty())
    {
        return PolyhedronBody();
    }
    loop.insert(loop.begin(), loop.front());
    faces[1] = PolyhedronFace3d(face.SupportPlane(), PolyhedronLoop3d(std::move(loop)));
    return PolyhedronBody(std::move(faces));
}
PolyhedronBody BuildSupportMismatchNearEqualClosedPrismDualVerticesBody()
{
    // Closed triangular prism where two non-adjacent shared vertices (v0 at
    // bottom-origin and v4 at top-right) each appear with near-equal (<eps)
    // position variants across their three incident faces. Tests that
    // representative-average placement scales to prism topology.
    // Expected: FaceCount=5 / VertexCount=6 / EdgeCount=9 / closed-shell.
    const double s = 1e-5;

    // v0 near {0,0,0}: bottom + front-side + left-side
    const Point3d v0a{1.0e-7, 0.0, 0.0};
    const Point3d v0b{-1.0e-7, 0.0, 0.0};
    const Point3d v0c{0.0, 0.0, 0.0};
    // avg x = 0

    const Point3d v1{s, 0.0, 0.0};
    const Point3d v2{0.5 * s, 0.866 * s, 0.0};
    const Point3d v3{0.0, 0.0, s};

    // v4 near {s,0,s}: top + front-side + right-side
    const Point3d v4a{s + 2.0e-7, 0.0, s};
    const Point3d v4b{s - 1.0e-7, 0.0, s};
    const Point3d v4c{s, 0.0, s};
    // avg x = s + 1e-7/3

    const Point3d v5{0.5 * s, 0.866 * s, s};

    const Plane mismatched =
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 3e-6}, Vector3d{0.0, 0.0, 1.0});

    return PolyhedronBody({
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0a, v1, v2})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v3, v4a, v5})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0b, v1, v4b, v3})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v1, v2, v5, v4c})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v2, v0c, v3, v5}))});
}

PolyhedronBody BuildSupportMismatchNearEqualClosedPrismDualVerticesWithDuplicateLoopBody()
{
    // Reuse dual-vertices near-equal closed-prism and inject one consecutive
    // duplicate leading vertex on a quad side face.
    const PolyhedronBody base = BuildSupportMismatchNearEqualClosedPrismDualVerticesBody();
    std::vector<PolyhedronFace3d> faces = base.Faces();
    if (faces.size() != 5)
    {
        return PolyhedronBody();
    }

    const PolyhedronFace3d sideFace = faces[2];
    std::vector<Point3d> loop = sideFace.OuterLoop().Vertices();
    if (loop.empty())
    {
        return PolyhedronBody();
    }

    loop.insert(loop.begin(), loop.front());
    faces[2] = PolyhedronFace3d(sideFace.SupportPlane(), PolyhedronLoop3d(std::move(loop)));
    return PolyhedronBody(std::move(faces));
}
PolyhedronBody BuildSupportMismatchNearEqualClosedPrismDualVerticesWithDualDuplicateLoopBody()
{
    // Start from the single-duplicate dual-vertices near-equal closed-prism
    // variant and inject another duplicate leading vertex on a second side face.
    const PolyhedronBody base = BuildSupportMismatchNearEqualClosedPrismDualVerticesWithDuplicateLoopBody();
    std::vector<PolyhedronFace3d> faces = base.Faces();
    if (faces.size() != 5)
    {
        return PolyhedronBody();
    }
    const PolyhedronFace3d sideFace = faces[3];
    std::vector<Point3d> loop = sideFace.OuterLoop().Vertices();
    if (loop.empty())
    {
        return PolyhedronBody();
    }
    loop.insert(loop.begin(), loop.front());
    faces[3] = PolyhedronFace3d(sideFace.SupportPlane(), PolyhedronLoop3d(std::move(loop)));
    return PolyhedronBody(std::move(faces));
}
PolyhedronBody BuildSupportMismatchNearEqualClosedPrismAllVerticesBody()
{
    // Closed triangular prism where all six logical shared vertices each have
    // near-equal variants across their incident faces.
    const double s = 1e-5;

    const Point3d v0a{1.0e-7, 0.0, 0.0};
    const Point3d v0b{-1.0e-7, 0.0, 0.0};
    const Point3d v0c{0.0, 0.0, 0.0};

    const Point3d v1a{s + 2.0e-7, 0.0, 0.0};
    const Point3d v1b{s - 1.0e-7, 0.0, 0.0};
    const Point3d v1c{s, 0.0, 0.0};

    const Point3d v2a{0.5 * s + 1.0e-7, 0.866 * s, 0.0};
    const Point3d v2b{0.5 * s - 1.0e-7, 0.866 * s, 0.0};
    const Point3d v2c{0.5 * s, 0.866 * s + 1.0e-7, 0.0};

    const Point3d v3a{0.0, 0.0, s};
    const Point3d v3b{1.0e-7, 0.0, s};
    const Point3d v3c{-1.0e-7, 0.0, s};

    const Point3d v4a{s + 3.0e-7, 0.0, s};
    const Point3d v4b{s - 2.0e-7, 0.0, s};
    const Point3d v4c{s + 1.0e-7, 0.0, s};

    const Point3d v5a{0.5 * s, 0.866 * s, s};
    const Point3d v5b{0.5 * s + 1.0e-7, 0.866 * s, s};
    const Point3d v5c{0.5 * s - 1.0e-7, 0.866 * s, s};

    const Plane mismatched =
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 3e-6}, Vector3d{0.0, 0.0, 1.0});

    return PolyhedronBody({
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0a, v1a, v2a})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v3a, v4a, v5a})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0b, v1b, v4b, v3b})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v1c, v2b, v5b, v4c})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v2c, v0c, v3c, v5c}))});
}

PolyhedronBody BuildSupportMismatchNearEqualClosedPrismAllVerticesWithDuplicateLoopBody()
{
    // Reuse the all-vertices near-equal closed-prism and inject one
    // consecutive duplicate leading vertex on a quad side face.
    const PolyhedronBody base = BuildSupportMismatchNearEqualClosedPrismAllVerticesBody();
    std::vector<PolyhedronFace3d> faces = base.Faces();
    if (faces.size() != 5)
    {
        return PolyhedronBody();
    }

    const PolyhedronFace3d sideFace = faces[2];
    std::vector<Point3d> vertices = sideFace.OuterLoop().Vertices();
    if (vertices.empty())
    {
        return PolyhedronBody();
    }

    vertices.insert(vertices.begin(), vertices.front());
    faces[2] = PolyhedronFace3d(sideFace.SupportPlane(), PolyhedronLoop3d(std::move(vertices)));
    return PolyhedronBody(std::move(faces));
}
PolyhedronBody BuildSupportMismatchNearEqualClosedPrismAllVerticesWithDualDuplicateLoopBody()
{
    // Start from the single-duplicate all-vertices near-equal closed-prism
    // variant and inject another duplicate leading vertex on a second side face.
    const PolyhedronBody base = BuildSupportMismatchNearEqualClosedPrismAllVerticesWithDuplicateLoopBody();
    std::vector<PolyhedronFace3d> faces = base.Faces();
    if (faces.size() != 5)
    {
        return PolyhedronBody();
    }
    const PolyhedronFace3d sideFace = faces[3];
    std::vector<Point3d> loop = sideFace.OuterLoop().Vertices();
    if (loop.empty())
    {
        return PolyhedronBody();
    }
    loop.insert(loop.begin(), loop.front());
    faces[3] = PolyhedronFace3d(sideFace.SupportPlane(), PolyhedronLoop3d(std::move(loop)));
    return PolyhedronBody(std::move(faces));
}
PolyhedronBody BuildSupportMismatchNearEqualClosedCuboidAllVerticesBody()
{
    // Tiny-scale 2×1×1 rectangular box (pure-quad closed shell) where all 8
    // logical shared vertices have near-equal variants across their 3 incident
    // faces. All faces use a single mismatched support plane. Tests that
    // representative-average vertex placement works for a pure-quad closed-shell
    // topology with non-equal side lengths.
    // Expected after repair: FaceCount=6 / VertexCount=8 / EdgeCount=12 / IsClosed=true.
    const double s = 1e-5;
    const double w = 2.0 * s; // width in x-direction (twice the other dimensions)

    // v0 ≈ (0,0,0): bottom + front + left
    const Point3d v0a{1.0e-7, 0.0, 0.0};
    const Point3d v0b{-1.0e-7, 0.0, 0.0};
    const Point3d v0c{0.0, 0.0, 0.0};

    // v1 ≈ (w,0,0): bottom + front + right
    const Point3d v1a{w + 2.0e-7, 0.0, 0.0};
    const Point3d v1b{w - 1.0e-7, 0.0, 0.0};
    const Point3d v1c{w + 1.0e-7, 0.0, 0.0};

    // v2 ≈ (w,s,0): bottom + back + right
    const Point3d v2a{w + 1.0e-7, s, 0.0};
    const Point3d v2b{w, s + 1.0e-7, 0.0};
    const Point3d v2c{w, s, 0.0};

    // v3 ≈ (0,s,0): bottom + back + left
    const Point3d v3a{0.0, s + 2.0e-7, 0.0};
    const Point3d v3b{0.0, s, 0.0};
    const Point3d v3c{-1.0e-7, s, 0.0};

    // v4 ≈ (0,0,s): top + front + left
    const Point3d v4a{0.0, 0.0, s};
    const Point3d v4b{1.0e-7, 0.0, s};
    const Point3d v4c{0.0, 0.0, s + 1.0e-7};

    // v5 ≈ (w,0,s): top + front + right
    const Point3d v5a{w + 1.0e-7, 0.0, s};
    const Point3d v5b{w, 0.0, s - 1.0e-7};
    const Point3d v5c{w, 0.0, s};

    // v6 ≈ (w,s,s): top + back + right
    const Point3d v6a{w, s, s + 1.0e-7};
    const Point3d v6b{w + 1.0e-7, s, s};
    const Point3d v6c{w, s + 1.0e-7, s};

    // v7 ≈ (0,s,s): top + back + left
    const Point3d v7a{0.0, s, s};
    const Point3d v7b{-1.0e-7, s, s};
    const Point3d v7c{0.0, s, s + 1.0e-7};

    const Plane mismatched =
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 3e-6}, Vector3d{0.0, 0.0, 1.0});

    return PolyhedronBody({
        // Bottom (z≈0, normal -z): v0, v3, v2, v1
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0a, v3a, v2a, v1a})),
        // Top (z≈s, normal +z): v4, v5, v6, v7
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v4a, v5a, v6a, v7a})),
        // Front (y≈0, normal -y): v0, v1, v5, v4
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0b, v1b, v5b, v4b})),
        // Back (y≈s, normal +y): v3, v7, v6, v2
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v3b, v7b, v6b, v2b})),
        // Left (x≈0, normal -x): v0, v4, v7, v3
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0c, v4c, v7c, v3c})),
        // Right (x≈w, normal +x): v1, v2, v6, v5
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v1c, v2c, v6c, v5c}))});
}

PolyhedronBody BuildSupportMismatchNearEqualClosedCuboidAllVerticesWithDuplicateLoopBody()
{
    // Reuse the all-vertices near-equal closed-cuboid and inject a
    // consecutive duplicate leading vertex into one face loop.
    const PolyhedronBody base = BuildSupportMismatchNearEqualClosedCuboidAllVerticesBody();
    std::vector<PolyhedronFace3d> faces = base.Faces();
    if (faces.size() != 6)
    {
        return PolyhedronBody();
    }

    const PolyhedronFace3d front = faces[2];
    std::vector<Point3d> loop = front.OuterLoop().Vertices();
    if (loop.empty())
    {
        return PolyhedronBody();
    }

    loop.insert(loop.begin(), loop.front());
    faces[2] = PolyhedronFace3d(front.SupportPlane(), PolyhedronLoop3d(std::move(loop)));
    return PolyhedronBody(std::move(faces));
}

PolyhedronBody BuildSupportMismatchNearEqualClosedCuboidAllVerticesWithDualDuplicateLoopBody()
{
    // Start from the single-duplicate variant and add another duplicate leading
    // vertex on a second face, to stress duplicate-loop normalization
    // composition in a closed all-vertices near-equal cuboid input.
    const PolyhedronBody base = BuildSupportMismatchNearEqualClosedCuboidAllVerticesWithDuplicateLoopBody();
    std::vector<PolyhedronFace3d> faces = base.Faces();
    if (faces.size() != 6)
    {
        return PolyhedronBody();
    }

    const PolyhedronFace3d back = faces[3];
    std::vector<Point3d> loop = back.OuterLoop().Vertices();
    if (loop.empty())
    {
        return PolyhedronBody();
    }

    loop.insert(loop.begin(), loop.front());
    faces[3] = PolyhedronFace3d(back.SupportPlane(), PolyhedronLoop3d(std::move(loop)));
    return PolyhedronBody(std::move(faces));
}

PolyhedronBody BuildSupportMismatchNearEqualClosedCuboidDualVerticesWithDuplicateLoopBody()
{
    // Tiny-scale 2x1x1 closed cuboid where two non-adjacent shared vertices
    // (v0 and v6) have near-equal variants across their 3 incident faces,
    // and one face also needs duplicate-loop normalization.
    const double s = 1e-5;
    const double w = 2.0 * s;

    // v0 variants: bottom/front/left
    const Point3d v0a{1.0e-7, 0.0, 0.0};
    const Point3d v0b{-1.0e-7, 0.0, 0.0};
    const Point3d v0c{0.0, 0.0, 0.0};

    // Stable vertices.
    const Point3d v1{w, 0.0, 0.0};
    const Point3d v2{w, s, 0.0};
    const Point3d v3{0.0, s, 0.0};
    const Point3d v4{0.0, 0.0, s};
    const Point3d v5{w, 0.0, s};
    const Point3d v7{0.0, s, s};

    // v6 variants: top/back/right
    const Point3d v6a{w + 2.0e-7, s, s};
    const Point3d v6b{w - 1.0e-7, s, s};
    const Point3d v6c{w, s, s};

    const Plane mismatched =
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 3e-6}, Vector3d{0.0, 0.0, 1.0});

    std::vector<Point3d> leftLoop{v0c, v4, v7, v3};
    leftLoop.insert(leftLoop.begin(), leftLoop.front());

    return PolyhedronBody({
        // Bottom
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0a, v3, v2, v1})),
        // Top
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v4, v5, v6a, v7})),
        // Front
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0b, v1, v5, v4})),
        // Back
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v3, v7, v6b, v2})),
        // Left (with duplicate leading vertex)
        PolyhedronFace3d(mismatched, PolyhedronLoop3d(std::move(leftLoop))),
        // Right
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v1, v2, v6c, v5}))});
}

PolyhedronBody BuildSupportMismatchNearEqualClosedCuboidDualVerticesWithDualDuplicateLoopBody()
{
    // Start from the single-duplicate dual-vertices variant and inject a
    // second duplicate leading vertex on another face.
    const PolyhedronBody base = BuildSupportMismatchNearEqualClosedCuboidDualVerticesWithDuplicateLoopBody();
    std::vector<PolyhedronFace3d> faces = base.Faces();
    if (faces.size() != 6)
    {
        return PolyhedronBody();
    }

    const PolyhedronFace3d back = faces[3];
    std::vector<Point3d> loop = back.OuterLoop().Vertices();
    if (loop.empty())
    {
        return PolyhedronBody();
    }

    loop.insert(loop.begin(), loop.front());
    faces[3] = PolyhedronFace3d(back.SupportPlane(), PolyhedronLoop3d(std::move(loop)));
    return PolyhedronBody(std::move(faces));
}

PolyhedronBody BuildSupportMismatchNearEqualClosedCuboidDualVerticesBody()
{
    // Same as the dual-vertices+duplicate case but without duplicate loop
    // normalization input, to isolate representative-average behavior.
    const double s = 1e-5;
    const double w = 2.0 * s;

    const Point3d v0a{1.0e-7, 0.0, 0.0};
    const Point3d v0b{-1.0e-7, 0.0, 0.0};
    const Point3d v0c{0.0, 0.0, 0.0};

    const Point3d v1{w, 0.0, 0.0};
    const Point3d v2{w, s, 0.0};
    const Point3d v3{0.0, s, 0.0};
    const Point3d v4{0.0, 0.0, s};
    const Point3d v5{w, 0.0, s};
    const Point3d v7{0.0, s, s};

    const Point3d v6a{w + 2.0e-7, s, s};
    const Point3d v6b{w - 1.0e-7, s, s};
    const Point3d v6c{w, s, s};

    const Plane mismatched =
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 3e-6}, Vector3d{0.0, 0.0, 1.0});

    return PolyhedronBody({
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0a, v3, v2, v1})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v4, v5, v6a, v7})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0b, v1, v5, v4})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v3, v7, v6b, v2})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0c, v4, v7, v3})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v1, v2, v6c, v5}))});
}

// Three shared vertices with near-equal perturbations: v0, v1, v6.
// Extends dual-vertices coverage to an intermediate triple-vertex case.
PolyhedronBody BuildSupportMismatchNearEqualClosedCuboidTripleVerticesBody()
{
    const double s = 1e-5;
    const double w = 2.0 * s;

    // v0 cluster (3 faces share this corner)
    const Point3d v0a{1.0e-7, 0.0, 0.0};
    const Point3d v0b{-1.0e-7, 0.0, 0.0};
    const Point3d v0c{0.0, 0.0, 0.0};

    // v1 cluster (3 faces share this corner)
    const Point3d v1a{w + 1.0e-7, 0.0, 0.0};
    const Point3d v1b{w - 1.0e-7, 0.0, 0.0};
    const Point3d v1c{w, 0.0, 0.0};

    // v6 cluster (3 faces share this corner, diagonal to v0)
    const Point3d v6a{w + 2.0e-7, s, s};
    const Point3d v6b{w - 1.0e-7, s, s};
    const Point3d v6c{w, s, s};

    const Point3d v2{w, s, 0.0};
    const Point3d v3{0.0, s, 0.0};
    const Point3d v4{0.0, 0.0, s};
    const Point3d v5{w, 0.0, s};
    const Point3d v7{0.0, s, s};

    const Plane mismatched =
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 3e-6}, Vector3d{0.0, 0.0, 1.0});

    return PolyhedronBody({
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0a, v3, v2, v1a})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v4, v5, v6a, v7})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0b, v1b, v5, v4})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v3, v7, v6b, v2})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0c, v4, v7, v3})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v1c, v2, v6c, v5}))});
}

// Same triple-vertices near-equal perturbation, but the left face also has a
// duplicate leading vertex injected for duplicate-loop-normalization stress.
PolyhedronBody BuildSupportMismatchNearEqualClosedCuboidTripleVerticesWithDuplicateLoopBody()
{
    const double s = 1e-5;
    const double w = 2.0 * s;

    const Point3d v0a{1.0e-7, 0.0, 0.0};
    const Point3d v0b{-1.0e-7, 0.0, 0.0};
    const Point3d v0c{0.0, 0.0, 0.0};

    const Point3d v1a{w + 1.0e-7, 0.0, 0.0};
    const Point3d v1b{w - 1.0e-7, 0.0, 0.0};
    const Point3d v1c{w, 0.0, 0.0};

    const Point3d v6a{w + 2.0e-7, s, s};
    const Point3d v6b{w - 1.0e-7, s, s};
    const Point3d v6c{w, s, s};

    const Point3d v2{w, s, 0.0};
    const Point3d v3{0.0, s, 0.0};
    const Point3d v4{0.0, 0.0, s};
    const Point3d v5{w, 0.0, s};
    const Point3d v7{0.0, s, s};

    const Plane mismatched =
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 3e-6}, Vector3d{0.0, 0.0, 1.0});

    // Left face with duplicate leading vertex
    std::vector<Point3d> leftLoop{v0c, v4, v7, v3};
    leftLoop.insert(leftLoop.begin(), leftLoop.front());

    return PolyhedronBody({
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0a, v3, v2, v1a})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v4, v5, v6a, v7})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0b, v1b, v5, v4})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v3, v7, v6b, v2})),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d(std::move(leftLoop))),
        PolyhedronFace3d(mismatched, PolyhedronLoop3d({v1c, v2, v6c, v5}))});
}

PolyhedronBody BuildSupportMismatchNearEqualClosedCuboidTripleVerticesWithDualDuplicateLoopBody()
{
    // Start from the single-duplicate triple-vertices variant and add a second
    // duplicate leading vertex on another face to stress composition behavior.
    const PolyhedronBody base = BuildSupportMismatchNearEqualClosedCuboidTripleVerticesWithDuplicateLoopBody();
    std::vector<PolyhedronFace3d> faces = base.Faces();
    if (faces.size() != 6)
    {
        return PolyhedronBody();
    }

    const PolyhedronFace3d back = faces[3];
    std::vector<Point3d> loop = back.OuterLoop().Vertices();
    if (loop.empty())
    {
        return PolyhedronBody();
    }

    loop.insert(loop.begin(), loop.front());
    faces[3] = PolyhedronFace3d(back.SupportPlane(), PolyhedronLoop3d(std::move(loop)));
    return PolyhedronBody(std::move(faces));
}

void AssertClosedCuboidAllVerticesRepresentativeTargets(const BrepBody& body)
{
    const double s = 1e-5;
    const double w = 2.0 * s;
    const double third = 1.0e-7 / 3.0;
    const std::vector<Point3d> expectedPoints{
        Point3d{0.0, 0.0, 0.0},
        Point3d{w + 2.0 * third, 0.0, 0.0},
        Point3d{w + third, s + third, 0.0},
        Point3d{-third, s + 2.0 * third, 0.0},
        Point3d{third, 0.0, s + third},
        Point3d{w + third, 0.0, s - third},
        Point3d{w + third, s + third, s + third},
        Point3d{-third, s, s + third}};
    std::vector<std::size_t> expectedMatchCounts(expectedPoints.size(), 0);

    assert(body.VertexCount() == expectedPoints.size());
    for (std::size_t i = 0; i < body.VertexCount(); ++i)
    {
        const Point3d point = body.VertexAt(i).Point();
        bool matched = false;
        for (std::size_t expectedIndex = 0; expectedIndex < expectedPoints.size(); ++expectedIndex)
        {
            const Point3d& expected = expectedPoints[expectedIndex];
            if (std::abs(point.x - expected.x) < 1e-10 &&
                std::abs(point.y - expected.y) < 1e-10 &&
                std::abs(point.z - expected.z) < 1e-10)
            {
                ++expectedMatchCounts[expectedIndex];
                matched = true;
                break;
            }
        }

        assert(matched);
    }

    for (const std::size_t count : expectedMatchCounts)
    {
        assert(count == 1);
    }
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
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 8);
    assert(result.body.EdgeCount() == 12);
}

// Demonstrates representative-id global target averaging participates in final
// shared-vertex placement: near-equal shared-edge inputs no longer pin to the
// first encountered face point.
TEST(Conversion3dCapabilityTest, NearEqualSharedEdgeVerticesUseRepresentativeAverageTarget)
{
    const PolyhedronBody body = BuildNearEqualSharedEdgeQuadPairBody();
    assert(body.IsValid());
    assert(body.FaceCount() == 2);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 2);
    assert(result.body.ShellCount() == 1);
    assert(!result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 6);
    assert(result.body.EdgeCount() == 7);

    const double s = 1e-5;
    const double dx = 2e-7;
    const double expectedSharedX = s + 0.5 * dx;

    std::size_t sharedBLikeCount = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12 && point.x > 0.5 * s && point.x < 1.5 * s)
        {
            ++sharedBLikeCount;
            assert(std::abs(point.x - expectedSharedX) < 1e-12);
            assert(std::abs(point.x - s) > 5e-8);
        }
    }

    assert(sharedBLikeCount == 1);
}

// Demonstrates representative-average shared-vertex placement also holds when
// conversion first performs support-plane refit repair.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualSharedEdgeRepairsWithRepresentativeAverageTarget)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualSharedEdgeQuadPairBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 2);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 2);
    assert(result.body.ShellCount() == 1);
    assert(!result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 6);
    assert(result.body.EdgeCount() == 7);

    const double s = 1e-5;
    const double dx = 2e-7;
    const double expectedSharedX = s + 0.5 * dx;

    std::size_t sharedBLikeCount = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12 && point.x > 0.5 * s && point.x < 1.5 * s)
        {
            ++sharedBLikeCount;
            assert(std::abs(point.x - expectedSharedX) < 1e-12);
            assert(std::abs(point.x - s) > 5e-8);
        }
    }

    assert(sharedBLikeCount == 1);
}

// Demonstrates representative-id global averaging with support-plane refit on
// a multi-face shared-apex fan: near-equal apex variants merge to one shared
// Brep vertex at the expected average position.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualApexFanRepairsWithRepresentativeAverageTarget)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualApexTriangularFanBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 4);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 4);
    assert(result.body.ShellCount() == 1);
    assert(!result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 5);
    assert(result.body.EdgeCount() == 8);

    const double s = 1e-5;
    const double expectedApexX = 5e-8;

    std::size_t apexLikeCount = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        if (point.z > 0.5 * s)
        {
            ++apexLikeCount;
            assert(std::abs(point.x - expectedApexX) < 1e-12);
            assert(std::abs(point.y) < 1e-12);
        }
    }

    assert(apexLikeCount == 1);
}
// Demonstrates representative-average shared-vertex placement scales to a
// support-mismatch shared-edge chain (three faces) with near-equal middle-face
// perturbations on both shared edges.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualSharedEdgeChainRepairsWithRepresentativeAverageTargets)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualSharedEdgeChainBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 3);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 3);
    assert(result.body.ShellCount() == 1);
    assert(!result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 8);
    assert(result.body.EdgeCount() == 10);

    const double s = 1e-5;
    const double dx = 2e-7;
    const double expectedXLeft = s + 0.5 * dx;
    const double expectedXRight = 2.0 * s + 0.5 * dx;

    std::size_t leftCount = 0;
    std::size_t rightCount = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.y) < 1e-12 || std::abs(point.y - s) < 1e-12)
        {
            if (std::abs(point.x - expectedXLeft) < 1e-12)
            {
                ++leftCount;
            }
            if (std::abs(point.x - expectedXRight) < 1e-12)
            {
                ++rightCount;
            }
        }
    }

    assert(leftCount == 2);
    assert(rightCount == 2);
}

// Demonstrates representative-average shared-vertex placement also works for
// a shared-corner fan where faces only meet at a single near-equal corner.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualSharedCornerFanRepairsWithRepresentativeAverageTarget)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualSharedCornerQuadFanBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 3);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 3);
    assert(result.body.ShellCount() == 1);
    assert(!result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 8);
    assert(result.body.EdgeCount() == 10);

    std::size_t sharedCornerCount = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.x) < 1e-12 && std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12)
        {
            ++sharedCornerCount;
        }
    }

    assert(sharedCornerCount == 1);
}

// Demonstrates representative-average shared-vertex placement remains stable
// on a support-mismatch closed-shell input (tetrahedron).
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualClosedTetrahedronRepairsWithRepresentativeAverageTarget)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualClosedTetrahedronBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 4);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 4);
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 4);
    assert(result.body.EdgeCount() == 6);

    std::size_t sharedV0Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.x) < 1e-12 && std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12)
        {
            ++sharedV0Count;
        }
    }

    assert(sharedV0Count == 1);
}

// Demonstrates representative-average placement remains deterministic when
// support-mismatch shared-edge chain also requires duplicate-loop normalization
// on the middle face.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualSharedEdgeChainWithDuplicateRepairsWithRepresentativeAverageTargets)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualSharedEdgeChainWithDuplicateBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 3);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 3);
    assert(result.body.ShellCount() == 1);
    assert(!result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 8);
    assert(result.body.EdgeCount() == 10);

    const double s = 1e-5;
    const double dx = 2e-7;
    const double expectedXLeft = s + 0.5 * dx;
    const double expectedXRight = 2.0 * s + 0.5 * dx;

    std::size_t leftCount = 0;
    std::size_t rightCount = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.y) < 1e-12 || std::abs(point.y - s) < 1e-12)
        {
            if (std::abs(point.x - expectedXLeft) < 1e-12)
            {
                ++leftCount;
            }
            if (std::abs(point.x - expectedXRight) < 1e-12)
            {
                ++rightCount;
            }
        }
    }

    assert(leftCount == 2);
    assert(rightCount == 2);
}

// Demonstrates representative-average placement remains stable when
// support-mismatch shared-chain mixed-content also requires duplicate-hole
// normalization on middle face.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualSharedChainMixedContentWithDuplicateHoleRepairsToBrepBody)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualSharedChainMixedContentWithDuplicateHoleBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 3);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 3);
    assert(result.body.ShellCount() == 1);
    assert(!result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 12);
    assert(result.body.EdgeCount() == 14);

    const double s = 1e-5;
    const double dx = 2e-7;
    const double expectedXLeft = s + 0.5 * dx;
    const double expectedXRight = 2.0 * s + 0.5 * dx;

    std::size_t leftCount = 0;
    std::size_t rightCount = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.y) < 1e-12 || std::abs(point.y - s) < 1e-12)
        {
            if (std::abs(point.x - expectedXLeft) < 1e-12)
            {
                ++leftCount;
            }
            if (std::abs(point.x - expectedXRight) < 1e-12)
            {
                ++rightCount;
            }
        }
    }

    assert(leftCount == 2);
    assert(rightCount == 2);
}

// Demonstrates representative-average placement remains deterministic for
// support-mismatch shared-chain full-composition with near-equal shared-edge
// perturbations.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualSharedChainFullCompositionRepairsToBrepBody)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualSharedChainFullCompositionBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 3);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 3);
    assert(result.body.ShellCount() == 1);
    assert(!result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 13);
    assert(result.body.EdgeCount() == 15);

    const double s = 1e-5;
    const double dx = 2e-7;
    const double expectedXLeft = s + 0.5 * dx;
    const double expectedXRight = 2.0 * s + 0.5 * dx;

    std::size_t leftCount = 0;
    std::size_t rightCount = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.y) < 1e-12 || std::abs(point.y - s) < 1e-12)
        {
            if (std::abs(point.x - expectedXLeft) < 1e-12)
            {
                ++leftCount;
            }
            if (std::abs(point.x - expectedXRight) < 1e-12)
            {
                ++rightCount;
            }
        }
    }

    assert(leftCount == 2);
    assert(rightCount == 2);
}

// Demonstrates representative-average placement stays deterministic for
// support-mismatch shared-chain dual-duplicate full-composition.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualSharedChainDualDuplicateFullCompositionRepairsToBrepBody)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualSharedChainDualDuplicateFullCompositionBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 3);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 3);
    assert(result.body.ShellCount() == 1);
    assert(!result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 13);
    assert(result.body.EdgeCount() == 15);

    const double s = 1e-5;
    const double dx = 2e-7;
    const double expectedXLeft = s + 0.5 * dx;
    const double expectedXRight = 2.0 * s + 0.5 * dx;

    std::size_t leftCount = 0;
    std::size_t rightCount = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.y) < 1e-12 || std::abs(point.y - s) < 1e-12)
        {
            if (std::abs(point.x - expectedXLeft) < 1e-12)
            {
                ++leftCount;
            }
            if (std::abs(point.x - expectedXRight) < 1e-12)
            {
                ++rightCount;
            }
        }
    }

    assert(leftCount == 2);
    assert(rightCount == 2);
}

// Demonstrates representative-average placement can simultaneously stabilize
// multiple shared vertices on a support-mismatch closed tetrahedron.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualClosedTetrahedronDualVerticesRepairsWithRepresentativeAverageTarget)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualClosedTetrahedronDualVerticesBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 4);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 4);
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 4);
    assert(result.body.EdgeCount() == 6);

    const double s = 1e-5;
    const double expectedV1X = s + (2e-7 - 1e-7 + 0.0) / 3.0;

    std::size_t sharedV0Count = 0;
    std::size_t sharedV1Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.x) < 1e-12 && std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12)
        {
            ++sharedV0Count;
        }
        if (std::abs(point.y) < 1e-12 && std::abs(point.z - 1.2e-6) < 1e-12 && point.x > 0.5 * s && point.x < 1.5 * s)
        {
            ++sharedV1Count;
            assert(std::abs(point.x - expectedV1X) < 1e-10);
        }
    }

    assert(sharedV0Count == 1);
    assert(sharedV1Count == 1);
}

// Demonstrates representative-average placement on near-equal
// closed-tetra dual-shared-vertices remains stable when one face also needs
// duplicate-loop normalization.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualClosedTetrahedronDualVerticesWithDuplicateLoopRepairsWithRepresentativeAverageTarget)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualClosedTetrahedronDualVerticesWithDuplicateLoopBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 4);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 4);
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 4);
    assert(result.body.EdgeCount() == 6);

    const double s = 1e-5;
    const double expectedV1X = s + (2e-7 - 1e-7 + 0.0) / 3.0;

    std::size_t sharedV0Count = 0;
    std::size_t sharedV1Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.x) < 1e-12 && std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12)
        {
            ++sharedV0Count;
        }
        if (std::abs(point.y) < 1e-12 && std::abs(point.z - 1.2e-6) < 1e-12 && point.x > 0.5 * s && point.x < 1.5 * s)
        {
            ++sharedV1Count;
            assert(std::abs(point.x - expectedV1X) < 1e-10);
        }
    }

    assert(sharedV0Count == 1);
    assert(sharedV1Count == 1);
}
// Demonstrates representative-average placement on near-equal
// closed-tetra dual-shared-vertices remains stable when duplicate-loop
// normalization is required on two faces simultaneously.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualClosedTetrahedronDualVerticesWithDualDuplicateLoopRepairsWithRepresentativeAverageTarget)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualClosedTetrahedronDualVerticesWithDualDuplicateLoopBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 4);
    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 4);
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 4);
    assert(result.body.EdgeCount() == 6);
    const double s = 1e-5;
    const double expectedV1X = s + (2e-7 - 1e-7 + 0.0) / 3.0;
    std::size_t sharedV0Count = 0;
    std::size_t sharedV1Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.x) < 1e-12 && std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12)
        {
            ++sharedV0Count;
        }
        if (std::abs(point.y) < 1e-12 && std::abs(point.z - 1.2e-6) < 1e-12 && point.x > 0.5 * s && point.x < 1.5 * s)
        {
            ++sharedV1Count;
            assert(std::abs(point.x - expectedV1X) < 1e-10);
        }
    }
    assert(sharedV0Count == 1);
    assert(sharedV1Count == 1);
}
// Demonstrates representative-average placement scales to all four shared
// vertices simultaneously on a support-mismatch closed tetrahedron.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualClosedTetrahedronAllVerticesRepairsWithRepresentativeAverageTarget)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualClosedTetrahedronAllVerticesBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 4);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 4);
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 4);
    assert(result.body.EdgeCount() == 6);

    const double s = 1e-5;
    const double expectedV1X = s + 1.0e-7 / 3.0;
    const double expectedV3X = 0.5 * s + 1.0e-7 / 3.0;

    std::size_t sharedV1Count = 0;
    std::size_t sharedV3Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.y) < 1e-12 && point.z > 0.5e-6 && point.z < 2.0e-6 && point.x > 0.5 * s)
        {
            ++sharedV1Count;
            assert(std::abs(point.x - expectedV1X) < 1e-10);
        }
        if (point.z > 0.7 * s && point.x > 0.3 * s && point.x < 0.7 * s)
        {
            ++sharedV3Count;
            assert(std::abs(point.x - expectedV3X) < 1e-10);
        }
    }

    assert(sharedV1Count == 1);
    assert(sharedV3Count == 1);
}

// Demonstrates representative-average placement on the near-equal closed-tetra
// all-vertices subset remains stable when one face also requires
// duplicate-loop normalization.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualClosedTetrahedronAllVerticesWithDuplicateLoopRepairsWithRepresentativeAverageTarget)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualClosedTetrahedronAllVerticesWithDuplicateLoopBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 4);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 4);
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 4);
    assert(result.body.EdgeCount() == 6);

    const double s = 1e-5;
    const double expectedV1X = s + 1.0e-7 / 3.0;
    const double expectedV3X = 0.5 * s + 1.0e-7 / 3.0;

    std::size_t sharedV1Count = 0;
    std::size_t sharedV3Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.y) < 1e-12 && point.z > 0.5e-6 && point.z < 2.0e-6 && point.x > 0.5 * s)
        {
            ++sharedV1Count;
            assert(std::abs(point.x - expectedV1X) < 1e-10);
        }
        if (point.z > 0.7 * s && point.x > 0.3 * s && point.x < 0.7 * s)
        {
            ++sharedV3Count;
            assert(std::abs(point.x - expectedV3X) < 1e-10);
        }
    }

    assert(sharedV1Count == 1);
    assert(sharedV3Count == 1);
}

// Demonstrates representative-average placement on the near-equal closed-tetra
// all-vertices subset remains stable when duplicate-loop normalization is
// required on two faces simultaneously.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualClosedTetrahedronAllVerticesWithDualDuplicateLoopRepairsWithRepresentativeAverageTarget)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualClosedTetrahedronAllVerticesWithDualDuplicateLoopBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 4);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 4);
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 4);
    assert(result.body.EdgeCount() == 6);

    const double s = 1e-5;
    const double expectedV1X = s + 1.0e-7 / 3.0;
    const double expectedV3X = 0.5 * s + 1.0e-7 / 3.0;

    std::size_t sharedV1Count = 0;
    std::size_t sharedV3Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.y) < 1e-12 && point.z > 0.5e-6 && point.z < 2.0e-6 && point.x > 0.5 * s)
        {
            ++sharedV1Count;
            assert(std::abs(point.x - expectedV1X) < 1e-10);
        }
        if (point.z > 0.7 * s && point.x > 0.3 * s && point.x < 0.7 * s)
        {
            ++sharedV3Count;
            assert(std::abs(point.x - expectedV3X) < 1e-10);
        }
    }

    assert(sharedV1Count == 1);
    assert(sharedV3Count == 1);
}
// Demonstrates representative-average shared-vertex placement scales to a
// closed triangular prism topology: two non-adjacent near-equal shared vertices
// across quad and triangle faces simultaneously converge to stable average targets.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualClosedPrismDualVerticesRepairsWithRepresentativeAverageTarget)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualClosedPrismDualVerticesBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 5);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 5);
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 6);
    assert(result.body.EdgeCount() == 9);

    const double s = 1e-5;
    const double expectedV4X = s + 1.0e-7 / 3.0;

    std::size_t sharedV0Count = 0;
    std::size_t sharedV4Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.x) < 1e-12 && std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12)
        {
            ++sharedV0Count;
        }
        if (std::abs(point.y) < 1e-12 && point.x > 0.5 * s && point.x < 1.5 * s
            && point.z > 0.5 * s && point.z < 1.5 * s)
        {
            ++sharedV4Count;
            assert(std::abs(point.x - expectedV4X) < 1e-10);
        }
    }

    assert(sharedV0Count == 1);
    assert(sharedV4Count == 1);
}

// Demonstrates representative-average placement on the near-equal
// closed-prism dual-vertices subset remains stable when one side face also
// requires duplicate-loop normalization.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualClosedPrismDualVerticesWithDuplicateLoopRepairsWithRepresentativeAverageTarget)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualClosedPrismDualVerticesWithDuplicateLoopBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 5);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 5);
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 6);
    assert(result.body.EdgeCount() == 9);

    const double s = 1e-5;
    const double expectedV4X = s + 1.0e-7 / 3.0;

    std::size_t sharedV0Count = 0;
    std::size_t sharedV4Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.x) < 1e-12 && std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12)
        {
            ++sharedV0Count;
        }
        if (std::abs(point.y) < 1e-12 && point.x > 0.5 * s && point.x < 1.5 * s
            && point.z > 0.5 * s && point.z < 1.5 * s)
        {
            ++sharedV4Count;
            assert(std::abs(point.x - expectedV4X) < 1e-10);
        }
    }

    assert(sharedV0Count == 1);
    assert(sharedV4Count == 1);
}
// Demonstrates representative-average placement on the near-equal
// closed-prism dual-vertices subset remains stable when duplicate-loop
// normalization is required on two side faces simultaneously.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualClosedPrismDualVerticesWithDualDuplicateLoopRepairsWithRepresentativeAverageTarget)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualClosedPrismDualVerticesWithDualDuplicateLoopBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 5);
    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 5);
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 6);
    assert(result.body.EdgeCount() == 9);
    const double s = 1e-5;
    const double expectedV4X = s + 1.0e-7 / 3.0;
    std::size_t sharedV0Count = 0;
    std::size_t sharedV4Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.x) < 1e-12 && std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12)
        {
            ++sharedV0Count;
        }
        if (std::abs(point.y) < 1e-12 && point.x > 0.5 * s && point.x < 1.5 * s
            && point.z > 0.5 * s && point.z < 1.5 * s)
        {
            ++sharedV4Count;
            assert(std::abs(point.x - expectedV4X) < 1e-10);
        }
    }
    assert(sharedV0Count == 1);
    assert(sharedV4Count == 1);
}
// Demonstrates representative-average placement also stabilizes all shared
// vertices on a closed triangular prism when every logical shared vertex is
// near-equal perturbed across incident faces.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualClosedPrismAllVerticesRepairsWithRepresentativeAverageTarget)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualClosedPrismAllVerticesBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 5);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 5);
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 6);
    assert(result.body.EdgeCount() == 9);

    const double s = 1e-5;
    const double expectedV1X = s + 1.0e-7 / 3.0;
    const double expectedV4X = s + 2.0e-7 / 3.0;

    std::size_t sharedV1Count = 0;
    std::size_t sharedV4Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12 && point.x > 0.5 * s && point.x < 1.5 * s)
        {
            ++sharedV1Count;
            assert(std::abs(point.x - expectedV1X) < 1e-10);
        }
        if (std::abs(point.y) < 1e-12 && point.x > 0.5 * s && point.x < 1.5 * s
            && point.z > 0.5 * s && point.z < 1.5 * s)
        {
            ++sharedV4Count;
            assert(std::abs(point.x - expectedV4X) < 1e-10);
        }
    }

    assert(sharedV1Count == 1);
    assert(sharedV4Count == 1);
}

// Demonstrates representative-average placement on the near-equal
// closed-prism all-vertices subset remains stable when one side face also
// requires duplicate-loop normalization.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualClosedPrismAllVerticesWithDuplicateLoopRepairsWithRepresentativeAverageTarget)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualClosedPrismAllVerticesWithDuplicateLoopBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 5);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 5);
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 6);
    assert(result.body.EdgeCount() == 9);

    const double s = 1e-5;
    const double expectedV1X = s + 1.0e-7 / 3.0;
    const double expectedV4X = s + 2.0e-7 / 3.0;

    std::size_t sharedV1Count = 0;
    std::size_t sharedV4Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12 && point.x > 0.5 * s && point.x < 1.5 * s)
        {
            ++sharedV1Count;
            assert(std::abs(point.x - expectedV1X) < 1e-10);
        }
        if (std::abs(point.y) < 1e-12 && point.x > 0.5 * s && point.x < 1.5 * s
            && point.z > 0.5 * s && point.z < 1.5 * s)
        {
            ++sharedV4Count;
            assert(std::abs(point.x - expectedV4X) < 1e-10);
        }
    }

    assert(sharedV1Count == 1);
    assert(sharedV4Count == 1);
}
// Demonstrates representative-average placement on the near-equal
// closed-prism all-vertices subset remains stable when duplicate-loop
// normalization is required on two side faces simultaneously.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualClosedPrismAllVerticesWithDualDuplicateLoopRepairsWithRepresentativeAverageTarget)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualClosedPrismAllVerticesWithDualDuplicateLoopBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 5);
    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 5);
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 6);
    assert(result.body.EdgeCount() == 9);
    const double s = 1e-5;
    const double expectedV1X = s + 1.0e-7 / 3.0;
    const double expectedV4X = s + 2.0e-7 / 3.0;
    std::size_t sharedV1Count = 0;
    std::size_t sharedV4Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12 && point.x > 0.5 * s && point.x < 1.5 * s)
        {
            ++sharedV1Count;
            assert(std::abs(point.x - expectedV1X) < 1e-10);
        }
        if (std::abs(point.y) < 1e-12 && point.x > 0.5 * s && point.x < 1.5 * s
            && point.z > 0.5 * s && point.z < 1.5 * s)
        {
            ++sharedV4Count;
            assert(std::abs(point.x - expectedV4X) < 1e-10);
        }
    }
    assert(sharedV1Count == 1);
    assert(sharedV4Count == 1);
}
// Demonstrates a non-axis-aligned (affine-skewed) polyhedron subset can be
// converted to BrepBody without requiring robust non-planar repair.
TEST(Conversion3dCapabilityTest, SkewedCubePolyhedronBodyConvertsToBrepBody)
{
    PolyhedronBody skewedBody;
    ASSERT_NO_THROW(skewedBody = BuildSkewedUnitCubeBody());
    ASSERT_TRUE(skewedBody.IsValid());
    ASSERT_EQ(skewedBody.FaceCount(), 6);

    PolyhedronBrepBodyConversion3d result{};
    ASSERT_NO_THROW(result = ConvertToBrepBody(skewedBody));
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);

    bool bodyValid = false;
    ASSERT_NO_THROW(bodyValid = result.body.IsValid());
    ASSERT_TRUE(bodyValid);

    ASSERT_EQ(result.body.FaceCount(), 6);
    ASSERT_EQ(result.body.VertexCount(), 8);
    ASSERT_EQ(result.body.EdgeCount(), 12);

    std::size_t shellCount = 0;
    ASSERT_NO_THROW(shellCount = result.body.ShellCount());
    ASSERT_EQ(shellCount, 1u);
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
    assert(result.body.VertexCount() == 8);
    assert(result.body.EdgeCount() == 12);
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
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
    assert(result.body.VertexCount() == 8);
    assert(result.body.EdgeCount() == 12);
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
}

// Demonstrates that when a single shared vertex is displaced (making three
// adjacent faces non-planar simultaneously), ConvertToBrepBody can still
// recover via per-face refit and produce a topologically valid closed
// BrepBody. The area-based triplet selection naturally ignores the displaced
// vertex when the other three vertices define a larger-area (more stable)
// support plane.
TEST(Conversion3dCapabilityTest, MultipleNonPlanarFacesFromDisplacedVertexRepairsToBrepBody)
{
    const PolyhedronBody deformedBody = BuildDeformedUnitCubeBody();
    assert(!deformedBody.IsValid());

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(deformedBody);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 6);
    assert(result.body.VertexCount() == 8);
    assert(result.body.EdgeCount() == 12);
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
}

// Demonstrates the deformed-cube multi-face non-planar repair path remains
// stable when one adjacent face also needs duplicate-loop normalization.
TEST(Conversion3dCapabilityTest, DeformedCubeWithDuplicateLoopRepairsToClosedSharedTopologyBrepBody)
{
    const PolyhedronBody deformedBody = BuildDeformedUnitCubeWithDuplicateLoopBody();
    assert(!deformedBody.IsValid());

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(deformedBody);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 6);
    assert(result.body.VertexCount() == 8);
    assert(result.body.EdgeCount() == 12);
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
}

// Demonstrates the deformed-cube multi-face non-planar repair path remains
// stable when duplicate-loop normalization is required on two faces.
TEST(Conversion3dCapabilityTest, DeformedCubeWithDualDuplicateLoopRepairsToClosedSharedTopologyBrepBody)
{
    const PolyhedronBody deformedBody = BuildDeformedUnitCubeWithDualDuplicateLoopBody();
    assert(!deformedBody.IsValid());

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(deformedBody);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 6);
    assert(result.body.VertexCount() == 8);
    assert(result.body.EdgeCount() == 12);
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
}

// Demonstrates a stronger multi-face non-planar case: two opposite displaced
// vertices make all six cube faces non-planar against original support planes,
// and conversion still repairs to a closed shared-topology BrepBody.
TEST(Conversion3dCapabilityTest, DualDeformedCubeRepairsToClosedSharedTopologyBrepBody)
{
    const PolyhedronBody deformedBody = BuildDualDeformedUnitCubeBody();
    assert(!deformedBody.IsValid());

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(deformedBody);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 6);
    assert(result.body.VertexCount() == 8);
    assert(result.body.EdgeCount() == 12);
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
}

// Demonstrates the dual-deformed-cube multi-face non-planar repair path remains
// stable when one adjacent face also needs duplicate-loop normalization.
TEST(Conversion3dCapabilityTest, DualDeformedCubeWithDuplicateLoopRepairsToClosedSharedTopologyBrepBody)
{
    const PolyhedronBody deformedBody = BuildDualDeformedUnitCubeWithDuplicateLoopBody();
    assert(!deformedBody.IsValid());

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(deformedBody);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 6);
    assert(result.body.VertexCount() == 8);
    assert(result.body.EdgeCount() == 12);
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
}

// Demonstrates the dual-deformed-cube non-planar repair path remains stable
// when duplicate-loop normalization is required on two faces simultaneously.
TEST(Conversion3dCapabilityTest, DualDeformedCubeWithDualDuplicateLoopRepairsToClosedSharedTopologyBrepBody)
{
    const PolyhedronBody deformedBody = BuildDualDeformedUnitCubeWithDualDuplicateLoopBody();
    assert(!deformedBody.IsValid());

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(deformedBody);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 6);
    assert(result.body.VertexCount() == 8);
    assert(result.body.EdgeCount() == 12);
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
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
    assert(result.body.VertexCount() == 8);
    assert(result.body.EdgeCount() == 8);
}

// Demonstrates refit support-plane selection now scores all loop vertices, so
// a holed face can converge to the lower-error shared plane even when the outer
// loop alone would suggest a worse tilted support plane.
TEST(Conversion3dCapabilityTest, HoleDominatedNonPlanarHoledFaceRepairsToPlanarBrepBody)
{
    const PolyhedronBody body = BuildHoleDominatedNonPlanarHoledFaceBody();
    assert(!body.IsValid());

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 1);
    assert(result.body.VertexCount() == 8);
    assert(result.body.EdgeCount() == 8);

    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        assert(std::abs(point.z) < 1e-10);
    }
}

// Demonstrates the all-loop support-plane scoring also composes with
// shared-edge mixed-content repair: a holed face can choose the lower-error
// plane dictated by its hole loop while still preserving shared topology with
// its adjacent plain face.
TEST(Conversion3dCapabilityTest, SharedEdgeHoleDominatedMixedContentRepairsToPlanarSharedTopologyBrepBody)
{
    const PolyhedronBody body = BuildSharedEdgeHoleDominatedMixedContentBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 2);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 2);
    assert(result.body.VertexCount() == 10);
    assert(result.body.EdgeCount() == 11);
    assert(result.body.ShellCount() == 1);
    assert(!result.body.ShellAt(0).IsClosed());

    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        assert(std::abs(point.z) < 1e-10);
    }
}

// Demonstrates the all-loop support-plane scoring scales from one shared edge
// to a minimal three-face chain: the middle holed face still chooses the
// lower-error plane dictated by its hole loop while preserving shared topology
// with both adjacent plain faces.
TEST(Conversion3dCapabilityTest, SharedChainHoleDominatedMixedContentRepairsToPlanarSharedTopologyBrepBody)
{
    const PolyhedronBody body = BuildSharedChainHoleDominatedMixedContentBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 3);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 3);
    assert(result.body.VertexCount() == 12);
    assert(result.body.EdgeCount() == 14);
    assert(result.body.ShellCount() == 1);
    assert(!result.body.ShellAt(0).IsClosed());

    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        assert(std::abs(point.z) < 1e-10);
    }
}

// Demonstrates all-loop support-plane scoring also composes with
// representative-average placement on a minimal three-face shared chain: the
// middle holed face picks the lower-error plane dictated by its hole loop
// while near-equal shared-edge vertices still converge to deterministic
// averaged targets across both adjacent plain faces.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualSharedChainHoleDominatedMixedContentRepairsWithRepresentativeAverageTarget)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualSharedChainHoleDominatedMixedContentBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 3);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 3);
    assert(result.body.VertexCount() == 12);
    assert(result.body.EdgeCount() == 14);
    assert(result.body.ShellCount() == 1);
    assert(!result.body.ShellAt(0).IsClosed());

    const double expectedLeftX = 2.0 + 0.5 * 2.0e-7;
    const double expectedRightX = 6.0 + 0.5 * 2.0e-7;
    std::size_t leftCount = 0;
    std::size_t rightCount = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        assert(std::abs(point.z) < 1e-10);

        if (std::abs(point.y) < 1e-12 || std::abs(point.y - 2.0) < 1e-12)
        {
            if (std::abs(point.x - expectedLeftX) < 1e-10)
            {
                ++leftCount;
            }
            if (std::abs(point.x - expectedRightX) < 1e-10)
            {
                ++rightCount;
            }
        }
    }

    assert(leftCount == 2);
    assert(rightCount == 2);
}

// Demonstrates duplicate-hole normalization composes with the
// hole-dominated shared-chain representative-average path: the middle holed
// face still picks the lower-error plane while near-equal shared-edge
// vertices converge to deterministic average targets.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualSharedChainHoleDominatedMixedContentWithDuplicateHoleRepairsWithRepresentativeAverageTarget)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualSharedChainHoleDominatedMixedContentWithDuplicateHoleBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 3);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 3);
    assert(result.body.VertexCount() == 12);
    assert(result.body.EdgeCount() == 14);
    assert(result.body.ShellCount() == 1);
    assert(!result.body.ShellAt(0).IsClosed());

    const double expectedLeftX = 2.0 + 0.5 * 2.0e-7;
    const double expectedRightX = 6.0 + 0.5 * 2.0e-7;
    std::size_t leftCount = 0;
    std::size_t rightCount = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        assert(std::abs(point.z) < 1e-10);

        if (std::abs(point.y) < 1e-12 || std::abs(point.y - 2.0) < 1e-12)
        {
            if (std::abs(point.x - expectedLeftX) < 1e-10)
            {
                ++leftCount;
            }
            if (std::abs(point.x - expectedRightX) < 1e-10)
            {
                ++rightCount;
            }
        }
    }

    assert(leftCount == 2);
    assert(rightCount == 2);
}

// Demonstrates collinear-leading fallback also composes with the
// hole-dominated shared-chain representative-average path after duplicate-hole
// normalization: the middle holed face still picks the lower-error plane while
// shared-edge targets remain deterministic.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualSharedChainHoleDominatedFullCompositionRepairsWithRepresentativeAverageTarget)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualSharedChainHoleDominatedFullCompositionBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 3);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 3);
    assert(result.body.VertexCount() == 13);
    assert(result.body.EdgeCount() == 15);
    assert(result.body.ShellCount() == 1);
    assert(!result.body.ShellAt(0).IsClosed());

    const double expectedLeftX = 2.0 + 0.5 * 2.0e-7;
    const double expectedRightX = 6.0 + 0.5 * 2.0e-7;
    std::size_t leftCount = 0;
    std::size_t rightCount = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        assert(std::abs(point.z) < 1e-10);

        if (std::abs(point.y) < 1e-12 || std::abs(point.y - 2.0) < 1e-12)
        {
            if (std::abs(point.x - expectedLeftX) < 1e-10)
            {
                ++leftCount;
            }
            if (std::abs(point.x - expectedRightX) < 1e-10)
            {
                ++rightCount;
            }
        }
    }

    assert(leftCount == 2);
    assert(rightCount == 2);
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
    assert(result.body.VertexCount() == 5);
    assert(result.body.EdgeCount() == 5);
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
    assert(result.body.VertexCount() == 4);
    assert(result.body.EdgeCount() == 4);
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
    assert(result.body.VertexCount() == 8);
    assert(result.body.EdgeCount() == 8);
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
    assert(result.body.VertexCount() == 10);
    assert(result.body.EdgeCount() == 10);
}

// Demonstrates robust refit can recover a tiny-scale non-planar loop even
// when absolute-normal thresholds would otherwise reject support estimation.
TEST(Conversion3dCapabilityTest, TinyScaleNonPlanarFaceStillRepairsToBrepBody)
{
    const PolyhedronBody body = BuildTinyScaleNonPlanarRepairBody();
    assert(!body.IsValid());

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 1);
    assert(result.body.VertexCount() == 4);
    assert(result.body.EdgeCount() == 4);
}

// Demonstrates tiny-scale refit/projection repair also stabilizes holed
// non-planar inputs in the same face.
TEST(Conversion3dCapabilityTest, TinyScaleNonPlanarHoledFaceStillRepairsToBrepBody)
{
    const PolyhedronBody body = BuildTinyScaleNonPlanarHoledRepairBody();
    assert(!body.IsValid());

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 1);
    assert(result.body.VertexCount() == 8);
    assert(result.body.EdgeCount() == 8);
}

// Demonstrates tiny-scale non-planar repair remains stable for multi-face
// body inputs where each face needs local support-plane refit.
TEST(Conversion3dCapabilityTest, TinyScaleNonPlanarMultiFaceStillRepairsToBrepBody)
{
    const PolyhedronBody body = BuildTinyScaleNonPlanarMultiFaceRepairBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 2);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 2);
    assert(result.body.VertexCount() == 8);
    assert(result.body.EdgeCount() == 8);
}

// Demonstrates tiny-scale non-planar repair remains stable for mixed-content
// multi-face inputs (holed + plain) in one conversion.
TEST(Conversion3dCapabilityTest, TinyScaleNonPlanarMixedContentStillRepairsToBrepBody)
{
    const PolyhedronBody body = BuildTinyScaleNonPlanarMixedContentBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 2);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 2);
    assert(result.body.VertexCount() == 12);
    assert(result.body.EdgeCount() == 12);
}

// Demonstrates tiny-scale non-planar repair remains stable when adjacent faces
// share an edge and each face needs local refit/projection.
TEST(Conversion3dCapabilityTest, TinyScaleNonPlanarSharedEdgeFacesStillRepairToBrepBody)
{
    const PolyhedronBody body = BuildTinyScaleNonPlanarSharedEdgeBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 2);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 2);
    assert(result.body.VertexCount() == 6);
    assert(result.body.EdgeCount() == 7);
}

// Demonstrates tiny-scale non-planar repair remains stable on a shared-edge
// adjacency chain where local refit is applied across multiple neighboring faces.
// The conversion now preserves source representative vertex identities across
// repaired faces, so shared-edge chain topology remains deterministic even when
// per-face projection introduces small geometric drift.
TEST(Conversion3dCapabilityTest, TinyScaleNonPlanarSharedEdgeChainStillRepairsToBrepBody)
{
    const PolyhedronBody body = BuildTinyScaleNonPlanarSharedEdgeChainBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 3);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 3);
    assert(result.body.VertexCount() == 8);
    assert(result.body.EdgeCount() == 10);
}

// Demonstrates support-plane mismatch across a tiny-scale quad shared-edge
// chain can still recover deterministic shared topology through source
// representative-id-driven Brep vertex reuse.
TEST(Conversion3dCapabilityTest, TinyScaleSupportMismatchSharedEdgeChainRepairsWithSharedTopology)
{
    const PolyhedronBody body = BuildTinyScaleSupportMismatchSharedEdgeChainBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 3);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 3);
    assert(result.body.VertexCount() == 8);
    assert(result.body.EdgeCount() == 10);
    assert(result.body.ShellCount() == 1);
    assert(!result.body.ShellAt(0).IsClosed());
}

// Demonstrates representative-id propagation through normalization can keep
// deterministic shared topology even when support-plane mismatch and duplicate
// middle-face leading vertices appear together in a quad shared-edge chain.
TEST(Conversion3dCapabilityTest, TinyScaleSupportMismatchSharedEdgeChainWithDuplicateRepairsWithSharedTopology)
{
    const PolyhedronBody body = BuildTinyScaleSupportMismatchSharedEdgeChainWithDuplicateBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 3);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 3);
    assert(result.body.VertexCount() == 8);
    assert(result.body.EdgeCount() == 10);
    assert(result.body.ShellCount() == 1);
    assert(!result.body.ShellAt(0).IsClosed());
}

// Demonstrates tiny-scale shared-edge adjacency chain repair also supports
// mixed-content faces (including holed faces) in one conversion.
TEST(Conversion3dCapabilityTest, TinyScaleNonPlanarSharedEdgeChainMixedContentRepairsToBrepBody)
{
    const PolyhedronBody body = BuildTinyScaleNonPlanarSharedEdgeChainMixedContentBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 3);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 3);
    assert(result.body.VertexCount() == 12);
    assert(result.body.EdgeCount() == 14);
}

// Demonstrates shared-edge chain tiny-scale repair remains stable when one
// adjacent face also needs duplicate-loop normalization.
TEST(Conversion3dCapabilityTest, TinyScaleSharedEdgeChainWithDuplicateLoopRepairsToBrepBody)
{
    const PolyhedronBody body = BuildTinyScaleNonPlanarSharedEdgeChainWithDuplicateLoopBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 3);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 3);
    assert(result.body.VertexCount() == 8);
    assert(result.body.EdgeCount() == 10);
}

// Demonstrates duplicate-hole-loop normalization composes with shared-edge
// chain mixed-content tiny-scale refit/projection repair.
TEST(Conversion3dCapabilityTest, TinyScaleSharedChainMixedContentDuplicateHoleRepairsToBrepBody)
{
    const PolyhedronBody body = BuildTinyScaleSharedEdgeChainMixedContentWithDuplicateHoleBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 3);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 3);
    assert(result.body.VertexCount() == 12);
    assert(result.body.EdgeCount() == 14);
}

// Demonstrates shared-chain mixed-content tiny-scale repair remains stable
// with collinear-leading outer vertices on one adjacent face.
TEST(Conversion3dCapabilityTest, TinyScaleSharedChainMixedContentCollinearLeadingRepairsToBrepBody)
{
    const PolyhedronBody body = BuildTinyScaleSharedChainMixedContentWithCollinearLeadingBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 3);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 3);
    assert(result.body.VertexCount() == 13);
    assert(result.body.EdgeCount() == 15);
}

// Demonstrates support-plane mismatch composes with shared-chain mixed-content
// tiny-scale repair and still yields stable Brep conversion.
TEST(Conversion3dCapabilityTest, TinyScaleSharedChainMixedContentSupportPlaneMismatchRepairsToBrepBody)
{
    const PolyhedronBody body = BuildTinyScaleSharedChainMixedContentSupportPlaneMismatchBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 3);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 3);
    assert(result.body.VertexCount() == 12);
    assert(result.body.EdgeCount() == 14);
}

// Demonstrates support-plane mismatch and duplicate-hole normalization can
// compose on shared-chain mixed-content input while preserving deterministic
// shared topology counts.
TEST(Conversion3dCapabilityTest, TinyScaleSharedChainMixedContentSupportMismatchWithDuplicateHoleRepairsToBrepBody)
{
    const PolyhedronBody body = BuildTinyScaleSharedChainMixedContentSupportMismatchWithDuplicateHoleBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 3);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 3);
    assert(result.body.VertexCount() == 12);
    assert(result.body.EdgeCount() == 14);
    assert(result.body.ShellCount() == 1);
    assert(!result.body.ShellAt(0).IsClosed());
}

// Demonstrates support-plane mismatch and collinear-leading fallback can
// compose under shared-chain mixed-content tiny-scale repair.
TEST(Conversion3dCapabilityTest, TinyScaleSharedChainSupportMismatchAndCollinearRepairsToBrepBody)
{
    const PolyhedronBody body = BuildTinyScaleSharedChainSupportMismatchAndCollinearBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 3);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 3);
    assert(result.body.VertexCount() == 13);
    assert(result.body.EdgeCount() == 15);
}

// Demonstrates shared-chain mixed-content tiny-scale conversion remains stable
// when support-plane mismatch, collinear-leading fallback, and duplicate-hole
// normalization are all present together.
TEST(Conversion3dCapabilityTest, TinyScaleSharedChainFullCompositionRepairsToBrepBody)
{
    const PolyhedronBody body = BuildTinyScaleSharedChainFullCompositionBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 3);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 3);
    assert(result.body.VertexCount() == 13);
    assert(result.body.EdgeCount() == 15);
}

// Demonstrates shared-chain mixed-content tiny-scale conversion remains stable
// when full-composition repair also normalizes duplicate vertices on both an
// outer loop and a hole loop.
TEST(Conversion3dCapabilityTest, TinyScaleSharedChainDualDuplicateFullCompositionRepairsToBrepBody)
{
    const PolyhedronBody body = BuildTinyScaleSharedChainDualDuplicateFullCompositionBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 3);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 3);
    assert(result.body.VertexCount() == 13);
    assert(result.body.EdgeCount() == 15);
}

// Demonstrates that a tiny-scale closed tetrahedron polyhedron (4 triangular
// faces, all 6 edges shared by exactly two faces each) with identical
// mismatched support planes can be fully repaired and converted to a valid
// closed BrepBody with globally shared topology (VertexCount=4, EdgeCount=6,
// IsClosed=true). This is the representative closed-shell shared-topology
// conversion sub-scenario for the non-planar repair capability.
TEST(Conversion3dCapabilityTest, TinyScaleClosedTetrahedronConvertsToBrepBodyWithClosedShell)
{
    const PolyhedronBody body = BuildTinyScaleClosedTetrahedronBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 4);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 4);
    assert(result.body.VertexCount() == 4);
    assert(result.body.EdgeCount() == 6);
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
}

// Demonstrates that for a strip of triangular faces sharing edges (all with
// mismatched support planes), independent per-face refit achieves provably
// correct shared-edge vertex consistency.  Each shared vertex is one of the
// 3 defining points of every triangle that contains it, so its distance to
// that face's refit-plane is exactly 0 — no projection divergence.  The
// result is VertexCount=5 and EdgeCount=7 (2 shared edges properly reused).
TEST(Conversion3dCapabilityTest, TinyScaleTriangularFaceChainRepairsToBrepBodyWithSharedEdgeConsistency)
{
    const PolyhedronBody body = BuildTinyScaleTriangularFaceChainBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 3);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 3);
    assert(result.body.VertexCount() == 5);
    assert(result.body.EdgeCount() == 7);
}

// Demonstrates planar holed BrepBody conversion keeps representative area by
// honoring hole trims in mesh triangulation.
// Demonstrates that four triangular faces sharing a common apex vertex (square
// pyramid cap without base), all with mismatched support planes, produce a
// BrepBody with exactly VertexCount=5 and EdgeCount=8. The shared apex is a
// defining vertex of all four triangles → distance 0 from each refit-plane →
// never projected → exact merge.  Four radial edges are each used twice;
// four outer edges are each used once — the open boundary is deterministic.
TEST(Conversion3dCapabilityTest, TinyScaleTriangularFanRepairsToBrepBodyWithSharedApex)
{
    const PolyhedronBody body = BuildTinyScaleTriangularFanBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 4);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 4);
    assert(result.body.VertexCount() == 5);
    assert(result.body.EdgeCount() == 8);
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

// Demonstrates representative shared-edge Brep->mesh conversion can preserve
// shared geometric vertices globally instead of duplicating each face mesh.
TEST(Conversion3dCapabilityTest, PlanarSharedEdgeBrepBodyConvertsToMeshWithSharedVertices)
{
    std::vector<BrepVertex> vertices{
        BrepVertex(Point3d{0.0, 0.0, 0.0}),
        BrepVertex(Point3d{1.0, 0.0, 0.0}),
        BrepVertex(Point3d{1.0, 1.0, 0.0}),
        BrepVertex(Point3d{0.0, 1.0, 0.0}),
        BrepVertex(Point3d{2.0, 0.0, 0.0}),
        BrepVertex(Point3d{2.0, 1.0, 0.0})};

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
    addEdge(1, 4);
    addEdge(4, 5);
    addEdge(5, 2);

    const BrepLoop outerA({BrepCoedge(0, false), BrepCoedge(1, false), BrepCoedge(2, false), BrepCoedge(3, false)});
    const BrepLoop outerB({BrepCoedge(4, false), BrepCoedge(5, false), BrepCoedge(6, false), BrepCoedge(1, true)});

    const PlaneSurface planeSurface = PlaneSurface::FromPlane(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}));

    auto makeTrim = [&](const std::vector<Point2d>& points) {
        return CurveOnSurface(
            std::shared_ptr<Surface>(planeSurface.Clone().release()),
            Polyline2d(points, PolylineClosure::Closed));
    };

    const BrepFace faceA(
        std::shared_ptr<Surface>(planeSurface.Clone().release()),
        outerA,
        {},
        makeTrim({
            Point2d{0.0, 0.0},
            Point2d{1.0, 0.0},
            Point2d{1.0, 1.0},
            Point2d{0.0, 1.0},
        }),
        {});
    const BrepFace faceB(
        std::shared_ptr<Surface>(planeSurface.Clone().release()),
        outerB,
        {},
        makeTrim({
            Point2d{1.0, 0.0},
            Point2d{2.0, 0.0},
            Point2d{2.0, 1.0},
            Point2d{1.0, 1.0},
        }),
        {});

    const BrepBody body(vertices, edges, {BrepShell({faceA, faceB}, false)});
    assert(body.IsValid());

    const PolyhedronMeshConversion3d mesh = ConvertToTriangleMesh(body);
    assert(mesh.success);
    assert(mesh.mesh.IsValid());
    assert(mesh.mesh.TriangleCount() == 4);
    assert(mesh.mesh.VertexCount() == 6);
    assert(std::abs(mesh.mesh.SurfaceArea() - 2.0) < 1e-8);
}

// Demonstrates planar mixed content conversion: one holed face plus one
// disjoint face preserve aggregate representative area.
TEST(Conversion3dCapabilityTest, PlanarHoledAndMultiFaceBrepBodyConvertsToMeshWithExpectedArea)
{
    std::vector<BrepVertex> vertices{
        // Face A outer square (area 16)
        BrepVertex(Point3d{0.0, 0.0, 0.0}),
        BrepVertex(Point3d{4.0, 0.0, 0.0}),
        BrepVertex(Point3d{4.0, 4.0, 0.0}),
        BrepVertex(Point3d{0.0, 4.0, 0.0}),
        // Face A hole square (area 4)
        BrepVertex(Point3d{1.0, 1.0, 0.0}),
        BrepVertex(Point3d{3.0, 1.0, 0.0}),
        BrepVertex(Point3d{3.0, 3.0, 0.0}),
        BrepVertex(Point3d{1.0, 3.0, 0.0}),
        // Face B rectangle (area 2)
        BrepVertex(Point3d{5.0, 0.0, 0.0}),
        BrepVertex(Point3d{6.0, 0.0, 0.0}),
        BrepVertex(Point3d{6.0, 2.0, 0.0}),
        BrepVertex(Point3d{5.0, 2.0, 0.0})};

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

    // Face A outer
    addEdge(0, 1);
    addEdge(1, 2);
    addEdge(2, 3);
    addEdge(3, 0);
    // Face A hole
    addEdge(4, 5);
    addEdge(5, 6);
    addEdge(6, 7);
    addEdge(7, 4);
    // Face B
    addEdge(8, 9);
    addEdge(9, 10);
    addEdge(10, 11);
    addEdge(11, 8);

    const BrepLoop outerA({BrepCoedge(0, false), BrepCoedge(1, false), BrepCoedge(2, false), BrepCoedge(3, false)});
    const BrepLoop holeA({BrepCoedge(4, false), BrepCoedge(5, false), BrepCoedge(6, false), BrepCoedge(7, false)});
    const BrepLoop outerB({BrepCoedge(8, false), BrepCoedge(9, false), BrepCoedge(10, false), BrepCoedge(11, false)});

    const PlaneSurface planeSurface = PlaneSurface::FromPlane(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}));

    auto makeTrim = [&](const std::vector<Point2d>& points) {
        return CurveOnSurface(
            std::shared_ptr<Surface>(planeSurface.Clone().release()),
            Polyline2d(points, PolylineClosure::Closed));
    };

    const CurveOnSurface outerTrimA = makeTrim({
        Point2d{0.0, 0.0},
        Point2d{4.0, 0.0},
        Point2d{4.0, 4.0},
        Point2d{0.0, 4.0},
    });
    const CurveOnSurface holeTrimA = makeTrim({
        Point2d{1.0, 1.0},
        Point2d{3.0, 1.0},
        Point2d{3.0, 3.0},
        Point2d{1.0, 3.0},
    });
    const CurveOnSurface outerTrimB = makeTrim({
        Point2d{5.0, 0.0},
        Point2d{6.0, 0.0},
        Point2d{6.0, 2.0},
        Point2d{5.0, 2.0},
    });

    const BrepFace faceA(
        std::shared_ptr<Surface>(planeSurface.Clone().release()),
        outerA,
        {holeA},
        outerTrimA,
        {holeTrimA});
    const BrepFace faceB(
        std::shared_ptr<Surface>(planeSurface.Clone().release()),
        outerB,
        {},
        outerTrimB,
        {});
    const BrepBody body(vertices, edges, {BrepShell({faceA, faceB}, false)});
    assert(body.IsValid());

    const PolyhedronMeshConversion3d mesh = ConvertToTriangleMesh(body);
    assert(mesh.success);
    assert(mesh.mesh.IsValid());
    // Expected area: (16 - 4) + 2 = 14.
    assert(std::abs(mesh.mesh.SurfaceArea() - 14.0) < 1e-8);
}

// Demonstrates disconnected closed-shell Brep->mesh conversion preserves
// component separation (feature-level connectivity) in addition to geometry.
TEST(Conversion3dCapabilityTest, TwoSeparatedCubeBrepBodyConvertsToMeshWithTwoComponents)
{
    const PolyhedronBody twoCubes = BuildTwoSeparatedUnitCubeBody();
    assert(twoCubes.IsValid());
    assert(twoCubes.FaceCount() == 12);

    const PolyhedronBrepBodyConversion3d brep = ConvertToBrepBody(twoCubes);
    assert(brep.success);
    assert(brep.issue == BrepConversionIssue3d::None);
    assert(brep.body.IsValid());
    assert(brep.body.FaceCount() == 12);

    const PolyhedronMeshConversion3d mesh = ConvertToTriangleMesh(brep.body);
    assert(mesh.success);
    assert(mesh.mesh.IsValid());
    assert(mesh.mesh.TriangleCount() == 24);
    assert(std::abs(mesh.mesh.SurfaceArea() - 12.0) < 1e-8);

    const auto components = ComputeTriangleConnectedComponents(mesh.mesh);
    assert(components.size() == 2);
    assert(components[0].size() == 12);
    assert(components[1].size() == 12);
}

// Demonstrates the pure-quad closed-shell analogue of the triangular-prism
// all-vertices test: a tiny-scale 2x1x1 cuboid with all 8 logical shared
// vertices near-equal perturbed across 3 incident faces still converges to the
// deterministic representative-average targets for every shared vertex.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualClosedCuboidAllVerticesRepairsWithRepresentativeAverageTarget)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualClosedCuboidAllVerticesBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 6);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 6);
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 8);
    assert(result.body.EdgeCount() == 12);
    AssertClosedCuboidAllVerticesRepresentativeTargets(result.body);
}

// Demonstrates representative-average shared-vertex placement and closed-shell
// topology recovery remain stable when the near-equal closed-cuboid input also
// requires duplicate-loop normalization on one face.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualClosedCuboidAllVerticesWithDuplicateLoopRepairsWithRepresentativeAverageTarget)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualClosedCuboidAllVerticesWithDuplicateLoopBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 6);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 6);
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 8);
    assert(result.body.EdgeCount() == 12);
    AssertClosedCuboidAllVerticesRepresentativeTargets(result.body);
}

// Demonstrates closed-shell representative-average recovery remains stable when
// the near-equal closed-cuboid all-vertices input requires duplicate-loop
// normalization on two faces simultaneously.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualClosedCuboidAllVerticesWithDualDuplicateLoopRepairsWithRepresentativeAverageTarget)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualClosedCuboidAllVerticesWithDualDuplicateLoopBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 6);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 6);
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 8);
    assert(result.body.EdgeCount() == 12);
    AssertClosedCuboidAllVerticesRepresentativeTargets(result.body);
}

// Demonstrates representative-average placement and closed-shell topology
// recovery remain stable on a near-equal closed-cuboid dual-shared-vertices
// subset when one face also requires duplicate-loop normalization.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualClosedCuboidDualVerticesWithDuplicateLoopRepairsToValidBrepBody)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualClosedCuboidDualVerticesWithDuplicateLoopBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 6);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 6);
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 8);
    assert(result.body.EdgeCount() == 12);

    const double s = 1e-5;
    const double w = 2.0 * s;
    const double expectedV6X = w + 1.0e-7 / 3.0;

    std::size_t sharedV0Count = 0;
    std::size_t sharedV6Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.x) < 1e-12 && std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12)
        {
            ++sharedV0Count;
        }
        if (point.x > 1.5 * s && point.x < 2.5 * s && std::abs(point.y - s) < 1e-12 && std::abs(point.z - s) < 1e-12)
        {
            ++sharedV6Count;
            assert(std::abs(point.x - expectedV6X) < 1e-10);
        }
    }

    assert(sharedV0Count == 1);
    assert(sharedV6Count == 1);
}

// Demonstrates representative-average placement and closed-shell topology
// recovery remain stable on near-equal closed-cuboid dual-shared-vertices
// when duplicate-loop-normalization is required on two faces.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualClosedCuboidDualVerticesWithDualDuplicateLoopRepairsToValidBrepBody)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualClosedCuboidDualVerticesWithDualDuplicateLoopBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 6);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 6);
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 8);
    assert(result.body.EdgeCount() == 12);

    const double s = 1e-5;
    const double w = 2.0 * s;
    const double expectedV6X = w + 1.0e-7 / 3.0;

    std::size_t sharedV0Count = 0;
    std::size_t sharedV6Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.x) < 1e-12 && std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12)
        {
            ++sharedV0Count;
        }
        if (point.x > 1.5 * s && point.x < 2.5 * s && std::abs(point.y - s) < 1e-12 && std::abs(point.z - s) < 1e-12)
        {
            ++sharedV6Count;
            assert(std::abs(point.x - expectedV6X) < 1e-10);
        }
    }

    assert(sharedV0Count == 1);
    assert(sharedV6Count == 1);
}

// Demonstrates representative-average placement and closed-shell topology also
// stay deterministic on near-equal closed-cuboid dual-shared-vertices without
// duplicate-loop normalization noise.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualClosedCuboidDualVerticesRepairsToValidBrepBody)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualClosedCuboidDualVerticesBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 6);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 6);
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 8);
    assert(result.body.EdgeCount() == 12);

    const double s = 1e-5;
    const double w = 2.0 * s;
    const double expectedV6X = w + 1.0e-7 / 3.0;

    std::size_t sharedV0Count = 0;
    std::size_t sharedV6Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.x) < 1e-12 && std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12)
        {
            ++sharedV0Count;
        }
        if (point.x > 1.5 * s && point.x < 2.5 * s && std::abs(point.y - s) < 1e-12 && std::abs(point.z - s) < 1e-12)
        {
            ++sharedV6Count;
            assert(std::abs(point.x - expectedV6X) < 1e-10);
        }
    }

    assert(sharedV0Count == 1);
    assert(sharedV6Count == 1);
}

// Demonstrates representative-average placement covers an intermediate
// triple-shared-vertices case on a near-equal closed cuboid: v0, v1, and v6
// each perturbed across 3 incident faces, without duplicate-loop noise.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualClosedCuboidTripleVerticesRepairsToValidBrepBody)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualClosedCuboidTripleVerticesBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 6);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 6);
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 8);
    assert(result.body.EdgeCount() == 12);

    const double s = 1e-5;
    const double w = 2.0 * s;
    const double expectedV1X = w;
    const double expectedV6X = w + 1.0e-7 / 3.0;

    std::size_t sharedV0Count = 0;
    std::size_t sharedV1Count = 0;
    std::size_t sharedV6Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.x) < 1e-10 && std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12)
        {
            ++sharedV0Count;
        }
        if (point.x > 1.5 * s && point.x < 2.5 * s && std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12)
        {
            ++sharedV1Count;
            assert(std::abs(point.x - expectedV1X) < 1e-10);
        }
        if (point.x > 1.5 * s && point.x < 2.5 * s && std::abs(point.y - s) < 1e-12 && std::abs(point.z - s) < 1e-12)
        {
            ++sharedV6Count;
            assert(std::abs(point.x - expectedV6X) < 1e-10);
        }
    }

    assert(sharedV0Count == 1);
    assert(sharedV1Count == 1);
    assert(sharedV6Count == 1);
}

// Demonstrates that triple-shared-vertices near-equal perturbation paired with
// duplicate-loop-normalization on one face also converges: the left face has a
// duplicate leading vertex, while v0, v1, v6 each span 3 incident faces.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualClosedCuboidTripleVerticesWithDuplicateLoopRepairsToValidBrepBody)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualClosedCuboidTripleVerticesWithDuplicateLoopBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 6);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 6);
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 8);
    assert(result.body.EdgeCount() == 12);

    const double s = 1e-5;
    const double w = 2.0 * s;
    const double expectedV1X = w;
    const double expectedV6X = w + 1.0e-7 / 3.0;

    std::size_t sharedV0Count = 0;
    std::size_t sharedV1Count = 0;
    std::size_t sharedV6Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.x) < 1e-10 && std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12)
        {
            ++sharedV0Count;
        }
        if (point.x > 1.5 * s && point.x < 2.5 * s && std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12)
        {
            ++sharedV1Count;
            assert(std::abs(point.x - expectedV1X) < 1e-10);
        }
        if (point.x > 1.5 * s && point.x < 2.5 * s && std::abs(point.y - s) < 1e-12 && std::abs(point.z - s) < 1e-12)
        {
            ++sharedV6Count;
            assert(std::abs(point.x - expectedV6X) < 1e-10);
        }
    }

    assert(sharedV0Count == 1);
    assert(sharedV1Count == 1);
    assert(sharedV6Count == 1);
}

// Demonstrates triple-shared-vertices near-equal perturbation remains stable
// when duplicate-loop-normalization is required on two faces simultaneously.
TEST(Conversion3dCapabilityTest, SupportMismatchNearEqualClosedCuboidTripleVerticesWithDualDuplicateLoopRepairsToValidBrepBody)
{
    const PolyhedronBody body = BuildSupportMismatchNearEqualClosedCuboidTripleVerticesWithDualDuplicateLoopBody();
    assert(!body.IsValid());
    assert(body.FaceCount() == 6);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    assert(result.success);
    assert(result.issue == BrepConversionIssue3d::None);
    assert(result.body.IsValid());
    assert(result.body.FaceCount() == 6);
    assert(result.body.ShellCount() == 1);
    assert(result.body.ShellAt(0).IsClosed());
    assert(result.body.VertexCount() == 8);
    assert(result.body.EdgeCount() == 12);

    const double s = 1e-5;
    const double w = 2.0 * s;
    const double expectedV1X = w;
    const double expectedV6X = w + 1.0e-7 / 3.0;

    std::size_t sharedV0Count = 0;
    std::size_t sharedV1Count = 0;
    std::size_t sharedV6Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const Point3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.x) < 1e-10 && std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12)
        {
            ++sharedV0Count;
        }
        if (point.x > 1.5 * s && point.x < 2.5 * s && std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12)
        {
            ++sharedV1Count;
            assert(std::abs(point.x - expectedV1X) < 1e-10);
        }
        if (point.x > 1.5 * s && point.x < 2.5 * s && std::abs(point.y - s) < 1e-12 && std::abs(point.z - s) < 1e-12)
        {
            ++sharedV6Count;
            assert(std::abs(point.x - expectedV6X) < 1e-10);
        }
    }

    assert(sharedV0Count == 1);
    assert(sharedV1Count == 1);
    assert(sharedV6Count == 1);
}




