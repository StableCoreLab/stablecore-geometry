#include <cassert>

#include <cmath>

#include <gtest/gtest.h>

#include "sdk/Geometry.h"
#include "support/Fixtures3d.h"

using geometry::sdk::BuildSectionComponents;
using geometry::sdk::BuildSectionTopology;
using geometry::sdk::ClassifySectionContent;
using geometry::sdk::Plane;
using geometry::sdk::Point3d;
using geometry::sdk::PolyhedronBody;
using geometry::sdk::PolyhedronFace3d;
using geometry::sdk::PolyhedronLoop3d;
using geometry::sdk::Section;
using geometry::sdk::SectionContentKind3d;
using geometry::sdk::Vector3d;

namespace
{
PolyhedronBody BuildAdjacentCoplanarFaceBody()
{
    return PolyhedronBody({
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}),
            PolyhedronLoop3d({
                Point3d{0.0, 0.0, 0.0},
                Point3d{1.0, 0.0, 0.0},
                Point3d{1.0, 1.0, 0.0},
                Point3d{0.0, 1.0, 0.0},
            })),
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{1.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}),
            PolyhedronLoop3d({
                Point3d{1.0, 0.0, 0.0},
                Point3d{2.0, 0.0, 0.0},
                Point3d{2.0, 1.0, 0.0},
                Point3d{1.0, 1.0, 0.0},
            }))});
}
} // namespace

TEST(Section3dCapabilityTest, SlantedCubeSectionBuildsSingleAreaComponent)
{
    const PolyhedronBody cubeBody = geometry::test::BuildUnitCubeBody();
    assert(cubeBody.IsValid());

    const Plane slantedCut = Plane::FromPointAndNormal(
        Point3d{0.0, 0.0, 0.5},
        Vector3d{1.0, 0.0, 1.0});
    const auto section = Section(cubeBody, slantedCut);
    assert(section.success);
    assert(section.IsValid());
    assert(!section.segments.empty());
    assert(!section.polygons.empty());

    const auto topology = BuildSectionTopology(section);
    assert(topology.IsValid());
    assert(topology.Count() == 1);
    assert(topology.Roots().size() == 1);

    const auto components = BuildSectionComponents(section);
    assert(components.IsValid());
    assert(components.components.size() == 1);
    assert(components.components.front().polygonIndices.size() == 1);
    assert(components.components.front().faceIndices.size() == 1);

    assert(ClassifySectionContent(section) == SectionContentKind3d::Area);
}


// Demonstrates non-axis-aligned multi-face sectioning remains stable on a
// generic cube cut: contour/polygon counts stay deterministic.
TEST(Section3dCapabilityTest, NonAxisAlignedCubeSectionHasSingleHexLikeContour)
{
    const PolyhedronBody cubeBody = geometry::test::BuildUnitCubeBody();
    assert(cubeBody.IsValid());

    const Plane cut = Plane::FromPointAndNormal(
        Point3d{0.5, 0.5, 0.5},
        Vector3d{1.0, 1.0, 1.0});
    const auto section = Section(cubeBody, cut);
    assert(section.success);
    assert(section.IsValid());

    assert(section.polygons.size() == 1);
    assert(section.contours.size() == 1);
    assert(section.contours[0].closed);
    // For x+y+z=1.5 slicing a unit cube, the expected closed contour has 6 corners.
    assert(section.contours[0].points.size() == 6);

    const auto topology = BuildSectionTopology(section);
    assert(topology.IsValid());
    assert(topology.Roots().size() == 1);
    assert(ClassifySectionContent(section) == SectionContentKind3d::Area);
}

// Demonstrates coplanar adjacent face fragments are merged into one area
// polygon instead of remaining as two disjoint coplanar pieces.
TEST(Section3dCapabilityTest, AdjacentCoplanarFacesMergeIntoSingleSectionPolygon)
{
    const PolyhedronBody body = BuildAdjacentCoplanarFaceBody();
    assert(body.IsValid());
    assert(body.FaceCount() == 2);

    const Plane cut = Plane::FromPointAndNormal(
        Point3d{0.0, 0.0, 0.0},
        Vector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    assert(section.success);
    assert(section.IsValid());
    assert(section.polygons.size() == 1);
    assert(section.contours.size() == 1);
    assert(section.contours[0].closed);
    assert(section.contours[0].points.size() == 4);

    const auto topology = BuildSectionTopology(section);
    assert(topology.IsValid());
    assert(topology.Roots().size() == 1);

    const auto components = BuildSectionComponents(section);
    assert(components.IsValid());
    assert(components.components.size() == 1);
    assert(components.components[0].faceIndices.size() == 1);
    assert(ClassifySectionContent(section) == SectionContentKind3d::Area);
    assert(std::abs(geometry::sdk::Area(section.polygons[0]) - 2.0) < 1e-12);
}