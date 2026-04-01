#include <cassert>

#include <gtest/gtest.h>

#include "sdk/Geometry.h"
#include "support/Fixtures3d.h"

using geometry::sdk::BrepFace;
using geometry::sdk::BrepShell;
using geometry::sdk::Plane;
using geometry::sdk::Point3d;
using geometry::sdk::PolyhedronBody;
using geometry::sdk::RebuildSectionBrepBody;
using geometry::sdk::Section;
using geometry::sdk::SectionBrepBodyRebuild3d;
using geometry::sdk::Vector3d;

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
    assert(rebuilt.body.FaceCount() == 1);

    const BrepShell firstShell = rebuilt.body.ShellAt(0);
    const BrepFace firstFace = firstShell.FaceAt(0);
    assert(firstFace.OuterLoop().IsValid());
    // The slanted plane cuts 4 faces of the cube → quadrilateral section → 4 coedges
    assert(firstFace.OuterLoop().CoedgeCount() == 4);
}
