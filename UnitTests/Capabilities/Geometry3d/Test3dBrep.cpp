
#include <gtest/gtest.h>

#include "Geometry.h"
#include "Support/Fixtures3d.h"

using Geometry::SCBrepBody;
using Geometry::BrepBodyEdit3d;
using Geometry::SCBrepCoedge;
using Geometry::BrepConversionIssue3d;
using Geometry::SCBrepFace;
using Geometry::BrepFaceEdit3d;
using Geometry::SCBrepLoop;
using Geometry::BrepLoopEdit3d;
using Geometry::BrepLoopEditIssue3d;
using Geometry::SCBrepShell;
using Geometry::BrepShellEdit3d;
using Geometry::ConvertToBrepBody;
using Geometry::FlipCoedgeDirection;
using Geometry::InsertCoedge;
using Geometry::SCPlane;
using Geometry::SCPoint3d;
using Geometry::PolyhedronBody;
using Geometry::PolyhedronFace3d;
using Geometry::PolyhedronLoop3d;
using Geometry::RebuildSectionBodies;
using Geometry::RebuildSectionBody;
using Geometry::RebuildSectionBrepBodies;
using Geometry::RebuildSectionBrepBody;
using Geometry::RemoveCoedge;
using Geometry::ReplaceFace;
using Geometry::ReplaceOuterLoop;
using Geometry::ReplaceShell;
using Geometry::Section;
using SectionBodyRebuild3d = Geometry::SCSectionBodyRebuild3d;
using SectionBodySetRebuild3d = Geometry::SCSectionBodySetRebuild3d;
using SectionBrepBodyRebuild3d = Geometry::SCSectionBrepBodyRebuild3d;
using SectionBrepBodySetRebuild3d = Geometry::SCSectionBrepBodySetRebuild3d;
using Geometry::SCVector3d;

namespace
{
    PolyhedronLoop3d TranslateLoop(const PolyhedronLoop3d& loop, const SCVector3d& delta)
    {
        std::vector<SCPoint3d> vertices;
        vertices.reserve(loop.VertexCount());
        for (std::size_t i = 0; i < loop.VertexCount(); ++i)
        {
            vertices.push_back(loop.VertexAt(i) + delta);
        }
        return PolyhedronLoop3d(std::move(vertices));
    }

    PolyhedronFace3d TranslateFace(const PolyhedronFace3d& face, const SCVector3d& delta)
    {
        const SCPlane plane = SCPlane::FromPointAndNormal(face.SupportPlane().origin + delta, face.SupportPlane().normal);
        PolyhedronLoop3d outer = TranslateLoop(face.OuterLoop(), delta);

        std::vector<PolyhedronLoop3d> holes;
        holes.reserve(face.HoleCount());
        for (std::size_t i = 0; i < face.HoleCount(); ++i)
        {
            holes.push_back(TranslateLoop(face.HoleAt(i), delta));
        }

        return PolyhedronFace3d(plane, std::move(outer), std::move(holes));
    }

    PolyhedronBody BuildTwoSeparatedUnitCubes()
    {
        const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
        std::vector<PolyhedronFace3d> faces = first.Faces();

        const SCVector3d delta{3.0, 0.0, 0.0};
        for (const PolyhedronFace3d& face : first.Faces())
        {
            faces.push_back(TranslateFace(face, delta));
        }

        return PolyhedronBody(std::move(faces));
    }
}  // namespace

// Demonstrates that a non-axis-aligned (slanted) cube section can be rebuilt
// into a valid single-face SCBrepBody with a correct outer coedge loop.
// Covers the "read-only topology inspection" subset of the brep workflow, i.e.
// the part that is verifiably working while coedge-editing APIs remain open.
TEST(Brep3dCapabilityTest, SlantedCubeRebuildsSingleFaceBrepWithValidOuterLoop)
{
    const PolyhedronBody cubeBody = Geometry::Test::BuildUnitCubeBody();
    ASSERT_TRUE(cubeBody.IsValid());

    // plane: x + z = 0.5 芒锟?oblique cut yielding a quadrilateral section
    const SCPlane slantedCut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{1.0, 0.0, 1.0});
    const auto section = Section(cubeBody, slantedCut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());
    ASSERT_FALSE(section.polygons.empty());

    const SectionBrepBodyRebuild3d rebuilt = RebuildSectionBrepBody(section);
    ASSERT_TRUE(rebuilt.success);
    ASSERT_TRUE(rebuilt.body.IsValid());
    ASSERT_EQ(rebuilt.body.ShellCount(), 1);
    ASSERT_FALSE(rebuilt.body.ShellAt(0).IsClosed());
    ASSERT_EQ(rebuilt.body.FaceCount(), 1);

    const SCBrepShell firstShell = rebuilt.body.ShellAt(0);
    const SCBrepFace firstFace = firstShell.FaceAt(0);
    ASSERT_TRUE(firstFace.OuterLoop().IsValid());
    // The slanted plane cuts 4 faces of the cube 芒锟?quadrilateral section
    // 芒锟?4 coedges
    ASSERT_EQ(firstFace.OuterLoop().CoedgeCount(), 4);
}

// Demonstrates the Polyhedron rebuild path from section is also stable:
// slanted cube section can be rebuilt into a valid single-face PolyhedronBody.
TEST(Brep3dCapabilityTest, SlantedCubeRebuildsSingleFacePolyhedronBody)
{
    const PolyhedronBody cubeBody = Geometry::Test::BuildUnitCubeBody();
    ASSERT_TRUE(cubeBody.IsValid());

    const SCPlane slantedCut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{1.0, 0.0, 1.0});
    const auto section = Section(cubeBody, slantedCut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());
    ASSERT_FALSE(section.polygons.empty());

    const SectionBodyRebuild3d rebuilt = RebuildSectionBody(section);
    ASSERT_TRUE(rebuilt.success);
    ASSERT_TRUE(rebuilt.body.IsValid());
    ASSERT_EQ(rebuilt.body.FaceCount(), 1);

    const PolyhedronFace3d face = rebuilt.body.FaceAt(0);
    ASSERT_TRUE(face.OuterLoop().IsValid());
    ASSERT_EQ(face.OuterLoop().VertexCount(), 4);
}

// Demonstrates multi-component section rebuild: a horizontal cut through two
// separated cubes yields two independent area components and two Brep bodies.
TEST(Brep3dCapabilityTest, TwoSeparatedCubeSectionsRebuildIntoTwoBrepBodies)
{
    const PolyhedronBody body = BuildTwoSeparatedUnitCubes();
    ASSERT_TRUE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 12);

    const SCPlane cutPlane = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cutPlane);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());
    ASSERT_EQ(section.polygons.size(), 2);

    const SectionBrepBodySetRebuild3d rebuilt = RebuildSectionBrepBodies(section);
    ASSERT_TRUE(rebuilt.success);
    ASSERT_EQ(rebuilt.bodies.size(), 2);
    ASSERT_EQ(rebuilt.rootPolygonIndices.size(), 2);
    for (const SCBrepBody& rebuiltBody : rebuilt.bodies)
    {
        ASSERT_TRUE(rebuiltBody.IsValid());
        ASSERT_EQ(rebuiltBody.ShellCount(), 1);
        ASSERT_FALSE(rebuiltBody.ShellAt(0).IsClosed());
        ASSERT_EQ(rebuiltBody.FaceCount(), 1);
    }
}

// Demonstrates multi-component Polyhedron rebuild path:
// two separated cube section components can be rebuilt into two bodies.
TEST(Brep3dCapabilityTest, TwoSeparatedCubeSectionsRebuildIntoTwoPolyhedronBodies)
{
    const PolyhedronBody body = BuildTwoSeparatedUnitCubes();
    ASSERT_TRUE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 12);

    const SCPlane cutPlane = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cutPlane);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());
    ASSERT_EQ(section.polygons.size(), 2);

    const SectionBodySetRebuild3d rebuilt = RebuildSectionBodies(section);
    ASSERT_TRUE(rebuilt.success);
    ASSERT_EQ(rebuilt.bodies.size(), 2);
    ASSERT_EQ(rebuilt.rootPolygonIndices.size(), 2);
    for (const PolyhedronBody& rebuiltBody : rebuilt.bodies)
    {
        ASSERT_TRUE(rebuiltBody.IsValid());
        ASSERT_EQ(rebuiltBody.FaceCount(), 1);
    }
}

// Demonstrates end-to-end Brep path stability on multi-component section:
// convert polyhedron to Brep, section the Brep body, then rebuild two Brep
// section bodies from the section result.
TEST(Brep3dCapabilityTest, TwoSeparatedCubeBrepSectionRebuildsIntoTwoBrepBodies)
{
    const PolyhedronBody source = BuildTwoSeparatedUnitCubes();
    ASSERT_TRUE(source.IsValid());
    ASSERT_EQ(source.FaceCount(), 12);

    const auto converted = ConvertToBrepBody(source);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());
    ASSERT_EQ(converted.body.FaceCount(), 12);

    const SCPlane cutPlane = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cutPlane);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());
    ASSERT_EQ(section.polygons.size(), 2);

    const SectionBrepBodySetRebuild3d rebuilt = RebuildSectionBrepBodies(section);
    ASSERT_TRUE(rebuilt.success);
    ASSERT_EQ(rebuilt.bodies.size(), 2);
    ASSERT_EQ(rebuilt.rootPolygonIndices.size(), 2);
    for (const SCBrepBody& rebuiltBody : rebuilt.bodies)
    {
        ASSERT_TRUE(rebuiltBody.IsValid());
        ASSERT_EQ(rebuiltBody.ShellCount(), 1);
        ASSERT_FALSE(rebuiltBody.ShellAt(0).IsClosed());
        ASSERT_EQ(rebuiltBody.FaceCount(), 1);
    }
}

// Demonstrates a minimal non-trivial coedge-loop edit chain:
// insert -> flip orientation -> remove, with deterministic recovery.
TEST(Brep3dCapabilityTest, CoedgeLoopEditingInsertFlipRemoveRoundTrips)
{
    const PolyhedronBody cubeBody = Geometry::Test::BuildUnitCubeBody();
    const SCPlane slantedCut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{1.0, 0.0, 1.0});
    const auto section = Section(cubeBody, slantedCut);
    ASSERT_TRUE(section.success);

    const SectionBrepBodyRebuild3d rebuilt = RebuildSectionBrepBody(section);
    ASSERT_TRUE(rebuilt.success);
    const auto originalLoop = rebuilt.body.ShellAt(0).FaceAt(0).OuterLoop();
    ASSERT_TRUE(originalLoop.IsValid());
    ASSERT_EQ(originalLoop.CoedgeCount(), 4);

    const auto seedCoedge = originalLoop.CoedgeAt(2);
    const BrepLoopEdit3d inserted = InsertCoedge(originalLoop, 2, SCBrepCoedge(seedCoedge.EdgeIndex(), true));
    ASSERT_TRUE(inserted.success);
    ASSERT_EQ(inserted.issue, BrepLoopEditIssue3d::None);
    ASSERT_TRUE(inserted.loop.IsValid());
    ASSERT_EQ(inserted.loop.CoedgeCount(), 5);
    ASSERT_TRUE(inserted.loop.CoedgeAt(2).Reversed());

    const BrepLoopEdit3d flipped = FlipCoedgeDirection(inserted.loop, 2);
    ASSERT_TRUE(flipped.success);
    ASSERT_EQ(flipped.issue, BrepLoopEditIssue3d::None);
    ASSERT_TRUE(flipped.loop.IsValid());
    ASSERT_FALSE(flipped.loop.CoedgeAt(2).Reversed());

    const BrepLoopEdit3d removed = RemoveCoedge(flipped.loop, 2);
    ASSERT_TRUE(removed.success);
    ASSERT_EQ(removed.issue, BrepLoopEditIssue3d::None);
    ASSERT_TRUE(removed.loop.IsValid());
    ASSERT_EQ(removed.loop.CoedgeCount(), originalLoop.CoedgeCount());

    for (std::size_t i = 0; i < originalLoop.CoedgeCount(); ++i)
    {
        ASSERT_EQ(removed.loop.CoedgeAt(i).EdgeIndex(), originalLoop.CoedgeAt(i).EdgeIndex());
        ASSERT_EQ(removed.loop.CoedgeAt(i).Reversed(), originalLoop.CoedgeAt(i).Reversed());
    }
}

// Demonstrates a minimal ownership-consistent editing workflow:
// loop edit -> face replacement -> shell replacement -> body replacement.
TEST(Brep3dCapabilityTest, OwnershipConsistentEditingWorkflowRoundTripsIntoBody)
{
    const PolyhedronBody cubeBody = Geometry::Test::BuildUnitCubeBody();
    const SCPlane slantedCut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{1.0, 0.0, 1.0});
    const auto section = Section(cubeBody, slantedCut);
    ASSERT_TRUE(section.success);

    const SectionBrepBodyRebuild3d rebuilt = RebuildSectionBrepBody(section);
    ASSERT_TRUE(rebuilt.success);
    ASSERT_TRUE(rebuilt.body.IsValid());

    const SCBrepBody originalBody = rebuilt.body;
    const SCBrepShell originalShell = originalBody.ShellAt(0);
    const SCBrepFace originalFace = originalShell.FaceAt(0);
    const SCBrepLoop originalLoop = originalFace.OuterLoop();

    const BrepLoopEdit3d inserted =
        InsertCoedge(originalLoop, 1, SCBrepCoedge(originalLoop.CoedgeAt(1).EdgeIndex(), true));
    ASSERT_TRUE(inserted.success);

    const BrepLoopEdit3d removed = RemoveCoedge(inserted.loop, 1);
    ASSERT_TRUE(removed.success);
    ASSERT_EQ(removed.loop.CoedgeCount(), originalLoop.CoedgeCount());

    const BrepFaceEdit3d editedFace = ReplaceOuterLoop(originalFace, removed.loop);
    ASSERT_TRUE(editedFace.success);
    ASSERT_TRUE(editedFace.face.IsValid());

    const BrepShellEdit3d editedShell = ReplaceFace(originalShell, 0, editedFace.face);
    ASSERT_TRUE(editedShell.success);
    ASSERT_TRUE(editedShell.shell.IsValid());
    ASSERT_EQ(editedShell.shell.FaceCount(), 1);

    const BrepBodyEdit3d editedBody = ReplaceShell(originalBody, 0, editedShell.shell);
    ASSERT_TRUE(editedBody.success);
    ASSERT_TRUE(editedBody.body.IsValid());
    ASSERT_EQ(editedBody.body.ShellCount(), 1);
    ASSERT_EQ(editedBody.body.FaceCount(), 1);

    const SCBrepLoop roundTrippedLoop = editedBody.body.ShellAt(0).FaceAt(0).OuterLoop();
    ASSERT_EQ(roundTrippedLoop.CoedgeCount(), originalLoop.CoedgeCount());
    for (std::size_t i = 0; i < originalLoop.CoedgeCount(); ++i)
    {
        ASSERT_EQ(roundTrippedLoop.CoedgeAt(i).EdgeIndex(), originalLoop.CoedgeAt(i).EdgeIndex());
        ASSERT_EQ(roundTrippedLoop.CoedgeAt(i).Reversed(), originalLoop.CoedgeAt(i).Reversed());
    }
}

// Demonstrates the ownership-consistent replacement workflow is also stable on
// a non-trivial multi-face closed shell (unit-cube Brep), not only on a
// single-face rebuilt section body.
TEST(Brep3dCapabilityTest, OwnershipReplacementWorkflowOnMultiFaceClosedShell)
{
    const PolyhedronBody cubeBody = Geometry::Test::BuildUnitCubeBody();
    ASSERT_TRUE(cubeBody.IsValid());

    const auto converted = ConvertToBrepBody(cubeBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());
    ASSERT_EQ(converted.body.ShellCount(), 1);
    ASSERT_TRUE(converted.body.ShellAt(0).IsClosed());
    ASSERT_EQ(converted.body.FaceCount(), 6);

    const SCBrepBody originalBody = converted.body;
    const SCBrepShell originalShell = originalBody.ShellAt(0);
    const SCBrepFace firstFace = originalShell.FaceAt(0);
    const SCBrepLoop firstOuter = firstFace.OuterLoop();
    ASSERT_TRUE(firstOuter.IsValid());

    // No-op outer-loop replacement still exercises loop->face->shell->body
    // ownership propagation in a multi-face shell.
    const BrepFaceEdit3d sameFace = ReplaceOuterLoop(firstFace, firstOuter);
    ASSERT_TRUE(sameFace.success);
    ASSERT_TRUE(sameFace.face.IsValid());

    const BrepShellEdit3d shellEdited = ReplaceFace(originalShell, 0, sameFace.face);
    ASSERT_TRUE(shellEdited.success);
    ASSERT_TRUE(shellEdited.shell.IsValid());
    ASSERT_EQ(shellEdited.shell.FaceCount(), originalShell.FaceCount());
    ASSERT_EQ(shellEdited.shell.IsClosed(), originalShell.IsClosed());

    const BrepBodyEdit3d bodyEdited = ReplaceShell(originalBody, 0, shellEdited.shell);
    ASSERT_TRUE(bodyEdited.success);
    ASSERT_TRUE(bodyEdited.body.IsValid());
    ASSERT_EQ(bodyEdited.body.ShellCount(), 1);
    ASSERT_TRUE(bodyEdited.body.ShellAt(0).IsClosed());
    ASSERT_EQ(bodyEdited.body.FaceCount(), 6);
}
