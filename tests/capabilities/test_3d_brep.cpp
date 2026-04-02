#include <cassert>

#include <gtest/gtest.h>

#include "sdk/Geometry.h"
#include "support/Fixtures3d.h"

using geometry::sdk::BrepFace;
using geometry::sdk::BrepFaceEdit3d;
using geometry::sdk::BrepShell;
using geometry::sdk::BrepBody;
using geometry::sdk::BrepConversionIssue3d;
using geometry::sdk::BrepBodyEdit3d;
using geometry::sdk::BrepLoopEdit3d;
using geometry::sdk::BrepLoopEditIssue3d;
using geometry::sdk::BrepShellEdit3d;
using geometry::sdk::ConvertToBrepBody;
using geometry::sdk::FlipCoedgeDirection;
using geometry::sdk::InsertCoedge;
using geometry::sdk::Plane;
using geometry::sdk::Point3d;
using geometry::sdk::PolyhedronFace3d;
using geometry::sdk::PolyhedronLoop3d;
using geometry::sdk::PolyhedronBody;
using geometry::sdk::ReplaceFace;
using geometry::sdk::ReplaceOuterLoop;
using geometry::sdk::ReplaceShell;
using geometry::sdk::RemoveCoedge;
using geometry::sdk::RebuildSectionBrepBodies;
using geometry::sdk::RebuildSectionBrepBody;
using geometry::sdk::RebuildSectionBodies;
using geometry::sdk::RebuildSectionBody;
using geometry::sdk::Section;
using geometry::sdk::SectionBrepBodySetRebuild3d;
using geometry::sdk::SectionBrepBodyRebuild3d;
using geometry::sdk::SectionBodySetRebuild3d;
using geometry::sdk::SectionBodyRebuild3d;
using geometry::sdk::Vector3d;

namespace
{
geometry::sdk::PolyhedronLoop3d TranslateLoop(
    const geometry::sdk::PolyhedronLoop3d& loop,
    const geometry::sdk::Vector3d& delta)
{
    std::vector<Point3d> vertices;
    vertices.reserve(loop.VertexCount());
    for (std::size_t i = 0; i < loop.VertexCount(); ++i)
    {
        vertices.push_back(loop.VertexAt(i) + delta);
    }
    return geometry::sdk::PolyhedronLoop3d(std::move(vertices));
}

geometry::sdk::PolyhedronFace3d TranslateFace(
    const geometry::sdk::PolyhedronFace3d& face,
    const geometry::sdk::Vector3d& delta)
{
    const Plane plane = Plane::FromPointAndNormal(
        face.SupportPlane().origin + delta,
        face.SupportPlane().normal);
    PolyhedronLoop3d outer = TranslateLoop(face.OuterLoop(), delta);

    std::vector<PolyhedronLoop3d> holes;
    holes.reserve(face.HoleCount());
    for (std::size_t i = 0; i < face.HoleCount(); ++i)
    {
        holes.push_back(TranslateLoop(face.HoleAt(i), delta));
    }

    return PolyhedronFace3d(plane, std::move(outer), std::move(holes));
}

geometry::sdk::PolyhedronBody BuildTwoSeparatedUnitCubes()
{
    const PolyhedronBody first = geometry::test::BuildUnitCubeBody();
    std::vector<PolyhedronFace3d> faces = first.Faces();

    const Vector3d delta{3.0, 0.0, 0.0};
    for (const PolyhedronFace3d& face : first.Faces())
    {
        faces.push_back(TranslateFace(face, delta));
    }

    return PolyhedronBody(std::move(faces));
}
} // namespace

// Demonstrates that a non-axis-aligned (slanted) cube section can be rebuilt
// into a valid single-face BrepBody with a correct outer coedge loop.
// Covers the "read-only topology inspection" subset of the brep workflow, i.e.
// the part that is verifiably working while coedge-editing APIs remain open.
TEST(Brep3dCapabilityTest, SlantedCubeRebuildsSingleFaceBrepWithValidOuterLoop)
{
    const PolyhedronBody cubeBody = geometry::test::BuildUnitCubeBody();
    assert(cubeBody.IsValid());

    // plane: x + z = 0.5 — oblique cut yielding a quadrilateral section
    const Plane slantedCut = Plane::FromPointAndNormal(
        Point3d{0.0, 0.0, 0.5},
        Vector3d{1.0, 0.0, 1.0});
    const auto section = Section(cubeBody, slantedCut);
    assert(section.success);
    assert(section.IsValid());
    assert(!section.polygons.empty());

    const SectionBrepBodyRebuild3d rebuilt = RebuildSectionBrepBody(section);
    assert(rebuilt.success);
    assert(rebuilt.body.IsValid());
    assert(rebuilt.body.ShellCount() == 1);
    assert(!rebuilt.body.ShellAt(0).IsClosed());
    assert(rebuilt.body.FaceCount() == 1);

    const BrepShell firstShell = rebuilt.body.ShellAt(0);
    const BrepFace firstFace = firstShell.FaceAt(0);
    assert(firstFace.OuterLoop().IsValid());
    // The slanted plane cuts 4 faces of the cube → quadrilateral section → 4 coedges
    assert(firstFace.OuterLoop().CoedgeCount() == 4);
}

// Demonstrates the Polyhedron rebuild path from section is also stable:
// slanted cube section can be rebuilt into a valid single-face PolyhedronBody.
TEST(Brep3dCapabilityTest, SlantedCubeRebuildsSingleFacePolyhedronBody)
{
    const PolyhedronBody cubeBody = geometry::test::BuildUnitCubeBody();
    assert(cubeBody.IsValid());

    const Plane slantedCut = Plane::FromPointAndNormal(
        Point3d{0.0, 0.0, 0.5},
        Vector3d{1.0, 0.0, 1.0});
    const auto section = Section(cubeBody, slantedCut);
    assert(section.success);
    assert(section.IsValid());
    assert(!section.polygons.empty());

    const SectionBodyRebuild3d rebuilt = RebuildSectionBody(section);
    assert(rebuilt.success);
    assert(rebuilt.body.IsValid());
    assert(rebuilt.body.FaceCount() == 1);

    const PolyhedronFace3d face = rebuilt.body.FaceAt(0);
    assert(face.OuterLoop().IsValid());
    assert(face.OuterLoop().VertexCount() == 4);
}

// Demonstrates multi-component section rebuild: a horizontal cut through two
// separated cubes yields two independent area components and two Brep bodies.
TEST(Brep3dCapabilityTest, TwoSeparatedCubeSectionsRebuildIntoTwoBrepBodies)
{
    const PolyhedronBody body = BuildTwoSeparatedUnitCubes();
    assert(body.IsValid());
    assert(body.FaceCount() == 12);

    const Plane cutPlane = Plane::FromPointAndNormal(
        Point3d{0.0, 0.0, 0.5},
        Vector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cutPlane);
    assert(section.success);
    assert(section.IsValid());
    assert(section.polygons.size() == 2);

    const SectionBrepBodySetRebuild3d rebuilt = RebuildSectionBrepBodies(section);
    assert(rebuilt.success);
    assert(rebuilt.bodies.size() == 2);
    assert(rebuilt.rootPolygonIndices.size() == 2);
    for (const BrepBody& rebuiltBody : rebuilt.bodies)
    {
        assert(rebuiltBody.IsValid());
        assert(rebuiltBody.ShellCount() == 1);
        assert(!rebuiltBody.ShellAt(0).IsClosed());
        assert(rebuiltBody.FaceCount() == 1);
    }
}

// Demonstrates multi-component Polyhedron rebuild path:
// two separated cube section components can be rebuilt into two bodies.
TEST(Brep3dCapabilityTest, TwoSeparatedCubeSectionsRebuildIntoTwoPolyhedronBodies)
{
    const PolyhedronBody body = BuildTwoSeparatedUnitCubes();
    assert(body.IsValid());
    assert(body.FaceCount() == 12);

    const Plane cutPlane = Plane::FromPointAndNormal(
        Point3d{0.0, 0.0, 0.5},
        Vector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cutPlane);
    assert(section.success);
    assert(section.IsValid());
    assert(section.polygons.size() == 2);

    const SectionBodySetRebuild3d rebuilt = RebuildSectionBodies(section);
    assert(rebuilt.success);
    assert(rebuilt.bodies.size() == 2);
    assert(rebuilt.rootPolygonIndices.size() == 2);
    for (const PolyhedronBody& rebuiltBody : rebuilt.bodies)
    {
        assert(rebuiltBody.IsValid());
        assert(rebuiltBody.FaceCount() == 1);
    }
}

// Demonstrates a minimal non-trivial coedge-loop edit chain:
// insert -> flip orientation -> remove, with deterministic recovery.
TEST(Brep3dCapabilityTest, CoedgeLoopEditingInsertFlipRemoveRoundTrips)
{
    const PolyhedronBody cubeBody = geometry::test::BuildUnitCubeBody();
    const Plane slantedCut = Plane::FromPointAndNormal(
        Point3d{0.0, 0.0, 0.5},
        Vector3d{1.0, 0.0, 1.0});
    const auto section = Section(cubeBody, slantedCut);
    assert(section.success);

    const SectionBrepBodyRebuild3d rebuilt = RebuildSectionBrepBody(section);
    assert(rebuilt.success);
    const auto originalLoop = rebuilt.body.ShellAt(0).FaceAt(0).OuterLoop();
    assert(originalLoop.IsValid());
    assert(originalLoop.CoedgeCount() == 4);

    const auto seedCoedge = originalLoop.CoedgeAt(2);
    const BrepLoopEdit3d inserted = InsertCoedge(
        originalLoop,
        2,
        geometry::sdk::BrepCoedge(seedCoedge.EdgeIndex(), true));
    assert(inserted.success);
    assert(inserted.issue == BrepLoopEditIssue3d::None);
    assert(inserted.loop.IsValid());
    assert(inserted.loop.CoedgeCount() == 5);
    assert(inserted.loop.CoedgeAt(2).Reversed());

    const BrepLoopEdit3d flipped = FlipCoedgeDirection(inserted.loop, 2);
    assert(flipped.success);
    assert(flipped.issue == BrepLoopEditIssue3d::None);
    assert(flipped.loop.IsValid());
    assert(!flipped.loop.CoedgeAt(2).Reversed());

    const BrepLoopEdit3d removed = RemoveCoedge(flipped.loop, 2);
    assert(removed.success);
    assert(removed.issue == BrepLoopEditIssue3d::None);
    assert(removed.loop.IsValid());
    assert(removed.loop.CoedgeCount() == originalLoop.CoedgeCount());

    for (std::size_t i = 0; i < originalLoop.CoedgeCount(); ++i)
    {
        assert(removed.loop.CoedgeAt(i).EdgeIndex() == originalLoop.CoedgeAt(i).EdgeIndex());
        assert(removed.loop.CoedgeAt(i).Reversed() == originalLoop.CoedgeAt(i).Reversed());
    }
}

// Demonstrates a minimal ownership-consistent editing workflow:
// loop edit -> face replacement -> shell replacement -> body replacement.
TEST(Brep3dCapabilityTest, OwnershipConsistentEditingWorkflowRoundTripsIntoBody)
{
    const PolyhedronBody cubeBody = geometry::test::BuildUnitCubeBody();
    const Plane slantedCut = Plane::FromPointAndNormal(
        Point3d{0.0, 0.0, 0.5},
        Vector3d{1.0, 0.0, 1.0});
    const auto section = Section(cubeBody, slantedCut);
    assert(section.success);

    const SectionBrepBodyRebuild3d rebuilt = RebuildSectionBrepBody(section);
    assert(rebuilt.success);
    assert(rebuilt.body.IsValid());

    const BrepBody originalBody = rebuilt.body;
    const BrepShell originalShell = originalBody.ShellAt(0);
    const BrepFace originalFace = originalShell.FaceAt(0);
    const BrepLoop originalLoop = originalFace.OuterLoop();

    const BrepLoopEdit3d inserted = InsertCoedge(
        originalLoop,
        1,
        geometry::sdk::BrepCoedge(originalLoop.CoedgeAt(1).EdgeIndex(), true));
    assert(inserted.success);

    const BrepLoopEdit3d removed = RemoveCoedge(inserted.loop, 1);
    assert(removed.success);
    assert(removed.loop.CoedgeCount() == originalLoop.CoedgeCount());

    const BrepFaceEdit3d editedFace = ReplaceOuterLoop(originalFace, removed.loop);
    assert(editedFace.success);
    assert(editedFace.face.IsValid());

    const BrepShellEdit3d editedShell = ReplaceFace(originalShell, 0, editedFace.face);
    assert(editedShell.success);
    assert(editedShell.shell.IsValid());
    assert(editedShell.shell.FaceCount() == 1);

    const BrepBodyEdit3d editedBody = ReplaceShell(originalBody, 0, editedShell.shell);
    assert(editedBody.success);
    assert(editedBody.body.IsValid());
    assert(editedBody.body.ShellCount() == 1);
    assert(editedBody.body.FaceCount() == 1);

    const BrepLoop roundTrippedLoop = editedBody.body.ShellAt(0).FaceAt(0).OuterLoop();
    assert(roundTrippedLoop.CoedgeCount() == originalLoop.CoedgeCount());
    for (std::size_t i = 0; i < originalLoop.CoedgeCount(); ++i)
    {
        assert(roundTrippedLoop.CoedgeAt(i).EdgeIndex() == originalLoop.CoedgeAt(i).EdgeIndex());
        assert(roundTrippedLoop.CoedgeAt(i).Reversed() == originalLoop.CoedgeAt(i).Reversed());
    }
}

// Demonstrates the ownership-consistent replacement workflow is also stable on
// a non-trivial multi-face closed shell (unit-cube Brep), not only on a
// single-face rebuilt section body.
TEST(Brep3dCapabilityTest, OwnershipReplacementWorkflowOnMultiFaceClosedShell)
{
    const PolyhedronBody cubeBody = geometry::test::BuildUnitCubeBody();
    assert(cubeBody.IsValid());

    const auto converted = ConvertToBrepBody(cubeBody);
    assert(converted.success);
    assert(converted.issue == BrepConversionIssue3d::None);
    assert(converted.body.IsValid());
    assert(converted.body.ShellCount() == 1);
    assert(converted.body.ShellAt(0).IsClosed());
    assert(converted.body.FaceCount() == 6);

    const BrepBody originalBody = converted.body;
    const BrepShell originalShell = originalBody.ShellAt(0);
    const BrepFace firstFace = originalShell.FaceAt(0);
    const BrepLoop firstOuter = firstFace.OuterLoop();
    assert(firstOuter.IsValid());

    // No-op outer-loop replacement still exercises loop->face->shell->body
    // ownership propagation in a multi-face shell.
    const BrepFaceEdit3d sameFace = ReplaceOuterLoop(firstFace, firstOuter);
    assert(sameFace.success);
    assert(sameFace.face.IsValid());

    const BrepShellEdit3d shellEdited = ReplaceFace(originalShell, 0, sameFace.face);
    assert(shellEdited.success);
    assert(shellEdited.shell.IsValid());
    assert(shellEdited.shell.FaceCount() == originalShell.FaceCount());
    assert(shellEdited.shell.IsClosed() == originalShell.IsClosed());

    const BrepBodyEdit3d bodyEdited = ReplaceShell(originalBody, 0, shellEdited.shell);
    assert(bodyEdited.success);
    assert(bodyEdited.body.IsValid());
    assert(bodyEdited.body.ShellCount() == 1);
    assert(bodyEdited.body.ShellAt(0).IsClosed());
    assert(bodyEdited.body.FaceCount() == 6);
}
