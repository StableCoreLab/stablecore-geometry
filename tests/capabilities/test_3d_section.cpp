#include <cassert>

#include <cmath>

#include <gtest/gtest.h>

#include "sdk/Geometry.h"
#include "support/Fixtures3d.h"

using geometry::sdk::BuildSectionComponents;
using geometry::sdk::BuildSectionTopology;
using geometry::sdk::ClassifySectionContent;
using geometry::sdk::ConvertToBrepBody;
using geometry::sdk::BrepConversionIssue3d;
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

PolyhedronBody BuildThreeCoplanarStripFacesBody()
{
    return PolyhedronBody({
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}),
            PolyhedronLoop3d({
                Point3d{0.0, 0.0, 0.0}, Point3d{1.0, 0.0, 0.0},
                Point3d{1.0, 1.0, 0.0}, Point3d{0.0, 1.0, 0.0}})),
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{1.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}),
            PolyhedronLoop3d({
                Point3d{1.0, 0.0, 0.0}, Point3d{2.0, 0.0, 0.0},
                Point3d{2.0, 1.0, 0.0}, Point3d{1.0, 1.0, 0.0}})),
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{2.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}),
            PolyhedronLoop3d({
                Point3d{2.0, 0.0, 0.0}, Point3d{3.0, 0.0, 0.0},
                Point3d{3.0, 1.0, 0.0}, Point3d{2.0, 1.0, 0.0}}))});
}

PolyhedronLoop3d TranslateLoop(const PolyhedronLoop3d& loop, const Vector3d& delta)
{
    std::vector<Point3d> vertices;
    vertices.reserve(loop.VertexCount());
    for (std::size_t i = 0; i < loop.VertexCount(); ++i)
    {
        vertices.push_back(loop.VertexAt(i) + delta);
    }
    return PolyhedronLoop3d(std::move(vertices));
}

PolyhedronFace3d TranslateFace(const PolyhedronFace3d& face, const Vector3d& delta)
{
    const Plane translatedPlane = Plane::FromPointAndNormal(
        face.SupportPlane().origin + delta,
        face.SupportPlane().normal);

    PolyhedronLoop3d outer = TranslateLoop(face.OuterLoop(), delta);
    std::vector<PolyhedronLoop3d> holes;
    holes.reserve(face.HoleCount());
    for (std::size_t i = 0; i < face.HoleCount(); ++i)
    {
        holes.push_back(TranslateLoop(face.HoleAt(i), delta));
    }

    return PolyhedronFace3d(translatedPlane, std::move(outer), std::move(holes));
}

PolyhedronBody BuildTwoSeparatedUnitCubesBody()
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

// Demonstrates Section(BrepBody, Plane) remains stable on an oblique cut:
// after Polyhedron->Brep conversion, sectioning still yields one closed area
// contour with deterministic vertex count.
TEST(Section3dCapabilityTest, BrepBodyObliqueSectionHasSingleHexLikeContour)
{
    const PolyhedronBody cubeBody = geometry::test::BuildUnitCubeBody();
    assert(cubeBody.IsValid());

    const auto converted = ConvertToBrepBody(cubeBody);
    assert(converted.success);
    assert(converted.issue == BrepConversionIssue3d::None);
    assert(converted.body.IsValid());

    const Plane cut = Plane::FromPointAndNormal(
        Point3d{0.5, 0.5, 0.5},
        Vector3d{1.0, 1.0, 1.0});
    const auto section = Section(converted.body, cut);
    assert(section.success);
    assert(section.IsValid());

    assert(section.polygons.size() == 1);
    assert(section.contours.size() == 1);
    assert(section.contours[0].closed);
    assert(section.contours[0].points.size() == 6);

    const auto topology = BuildSectionTopology(section);
    assert(topology.IsValid());
    assert(topology.Roots().size() == 1);

    const auto components = BuildSectionComponents(section);
    assert(components.IsValid());
    assert(components.components.size() == 1);
    assert(ClassifySectionContent(section) == SectionContentKind3d::Area);
}

// Demonstrates coplanar fragment merge also holds for Section(BrepBody,
// Plane): after Polyhedron->Brep conversion, adjacent coplanar faces are still
// merged into one area polygon instead of staying split.
TEST(Section3dCapabilityTest, BrepBodyAdjacentCoplanarFacesMergeIntoSinglePolygon)
{
    const PolyhedronBody polyBody = BuildAdjacentCoplanarFaceBody();
    assert(polyBody.IsValid());
    assert(polyBody.FaceCount() == 2);

    const auto converted = ConvertToBrepBody(polyBody);
    assert(converted.success);
    assert(converted.issue == BrepConversionIssue3d::None);
    assert(converted.body.IsValid());

    const Plane cut = Plane::FromPointAndNormal(
        Point3d{0.0, 0.0, 0.0},
        Vector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
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
    assert(ClassifySectionContent(section) == SectionContentKind3d::Area);
    assert(std::abs(geometry::sdk::Area(section.polygons[0]) - 2.0) < 1e-12);
}

// Demonstrates Section(BrepBody, Plane) also supports multi-component area
// output: two separated cubes sectioned at z=0.5 produce two independent
// section polygons/components.
TEST(Section3dCapabilityTest, BrepBodySectionBuildsTwoAreaComponents)
{
    const PolyhedronBody polyBody = BuildTwoSeparatedUnitCubesBody();
    assert(polyBody.IsValid());
    assert(polyBody.FaceCount() == 12);

    const auto converted = ConvertToBrepBody(polyBody);
    assert(converted.success);
    assert(converted.issue == BrepConversionIssue3d::None);
    assert(converted.body.IsValid());

    const Plane cut = Plane::FromPointAndNormal(
        Point3d{0.0, 0.0, 0.5},
        Vector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    assert(section.success);
    assert(section.IsValid());
    assert(section.polygons.size() == 2);
    assert(section.contours.size() == 2);
    assert(section.contours[0].closed);
    assert(section.contours[1].closed);

    const auto topology = BuildSectionTopology(section);
    assert(topology.IsValid());
    assert(topology.Roots().size() == 2);

    const auto components = BuildSectionComponents(section);
    assert(components.IsValid());
    assert(components.components.size() == 2);
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

// Demonstrates that the face-merge subset extends beyond two adjacent faces:
// three coplanar faces arranged in an L-strip (each pair sharing an edge)
// are all merged into a single section polygon at the coplanar cut.
// Validates that coplanar fragment merge works for chains of 3+ faces, not
// just a single adjacent pair.
TEST(Section3dCapabilityTest, ThreeCoplanarFacesInLStripMergeIntoSinglePolygon)
{
    const PolyhedronBody body = BuildThreeCoplanarStripFacesBody();
    assert(body.IsValid());
    assert(body.FaceCount() == 3);

    const Plane cut = Plane::FromPointAndNormal(
        Point3d{0.0, 0.0, 0.0},
        Vector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    assert(section.success);
    assert(section.IsValid());
    // Expected: 3 coplanar adjacent faces merge into 1 polygon (area = 3.0 x 1)
    assert(section.polygons.size() == 1);
    assert(section.contours.size() == 1);
    assert(section.contours[0].closed);
    // Rectangle 3×1 → 4 corners
    assert(section.contours[0].points.size() == 4);

    const auto topology = BuildSectionTopology(section);
    assert(topology.IsValid());
    assert(topology.Roots().size() == 1);

    const auto components = BuildSectionComponents(section);
    assert(components.IsValid());
    assert(components.components.size() == 1);
    assert(components.components[0].faceIndices.size() == 1);

    assert(ClassifySectionContent(section) == SectionContentKind3d::Area);
    assert(std::abs(geometry::sdk::Area(section.polygons[0]) - 3.0) < 1e-12);
}

// Demonstrates that the same three-face coplanar strip merge semantics are
// preserved on the Brep path: Polyhedron->Brep conversion does not fragment
// the section result into multiple polygons.
TEST(Section3dCapabilityTest, BrepThreeCoplanarFacesInStripMergeIntoSinglePolygon)
{
    const PolyhedronBody polyBody = BuildThreeCoplanarStripFacesBody();
    assert(polyBody.IsValid());
    assert(polyBody.FaceCount() == 3);

    const auto converted = ConvertToBrepBody(polyBody);
    assert(converted.success);
    assert(converted.issue == BrepConversionIssue3d::None);
    assert(converted.body.IsValid());

    const Plane cut = Plane::FromPointAndNormal(
        Point3d{0.0, 0.0, 0.0},
        Vector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    assert(section.success);
    assert(section.IsValid());
    assert(section.polygons.size() == 1);
    assert(section.contours.size() == 1);
    assert(section.contours[0].closed);
    assert(section.contours[0].points.size() == 4);
    assert(section.segments.size() == 4);

    const auto topology = BuildSectionTopology(section);
    assert(topology.IsValid());
    assert(topology.Roots().size() == 1);

    const auto components = BuildSectionComponents(section);
    assert(components.IsValid());
    assert(components.components.size() == 1);
    assert(ClassifySectionContent(section) == SectionContentKind3d::Area);
    assert(std::abs(geometry::sdk::Area(section.polygons[0]) - 3.0) < 1e-12);
}

// Demonstrates that a mid-plane cut through a unit cube (whose 4 intersected
// faces are mutually non-coplanar) stitches into exactly one closed contour
// with exactly 4 corner points — proving that segment stitching across
// non-coplanar face pairs is deterministic for convex closed bodies.
// This narrows the NonPlanarDominantSectionGraphRemainsOpen gap to the
// specific subset: convex-body oblique-face-to-face stitching.
TEST(Section3dCapabilityTest, UnitCubeMidPlaneSectionYieldsFourSegmentClosedContour)
{
    const PolyhedronBody cubeBody = geometry::test::BuildUnitCubeBody();
    assert(cubeBody.IsValid());

    // y=0.5 mid-plane cuts left/right/bottom/top faces (all mutually non-coplanar).
    const Plane midCut = Plane::FromPointAndNormal(
        Point3d{0.5, 0.5, 0.5},
        Vector3d{0.0, 1.0, 0.0});
    const auto section = Section(cubeBody, midCut);
    assert(section.success);
    assert(section.IsValid());

    // Exactly one closed rectangular contour with 4 corner points.
    assert(section.contours.size() == 1);
    assert(section.contours[0].closed);
    assert(section.contours[0].points.size() == 4);

    // Exactly 4 segments (one per intersected face).
    assert(section.segments.size() == 4);

    assert(ClassifySectionContent(section) == SectionContentKind3d::Area);
    // Area of the 1×1 rectangle in the xz plane.
    assert(section.polygons.size() == 1);
    assert(std::abs(geometry::sdk::Area(section.polygons[0]) - 1.0) < 1e-12);

    const auto topology = BuildSectionTopology(section);
    assert(topology.IsValid());
    assert(topology.Count() == 1);
    assert(topology.Roots().size() == 1);

    const auto components = BuildSectionComponents(section);
    assert(components.IsValid());
    assert(components.components.size() == 1);
}

// Demonstrates that Section() on a closed prism-like polyhedron where the
// cut plane is oblique to all faces produces a deterministic closed contour
// and that the sum of contour edge lengths (total rebar perimeter) is stable.
TEST(Section3dCapabilityTest, ObliquePrismSectionYieldsDeterministicContourLength)
{
    // Right triangular prism: two triangular faces (top/bottom) + three quad
    // side faces, all z-aligned, base triangle in z=0 plane.
    const Point3d a0{0.0, 0.0, 0.0};
    const Point3d b0{1.0, 0.0, 0.0};
    const Point3d c0{0.5, 0.866, 0.0};
    const Point3d a1{0.0, 0.0, 1.0};
    const Point3d b1{1.0, 0.0, 1.0};
    const Point3d c1{0.5, 0.866, 1.0};

    const PolyhedronBody prism({
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{0.5, 0.289, 0.0}, Vector3d{0.0, 0.0, -1.0}),
            PolyhedronLoop3d({a0, c0, b0})),
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{0.5, 0.289, 1.0}, Vector3d{0.0, 0.0, 1.0}),
            PolyhedronLoop3d({a1, b1, c1})),
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{0.5, 0.0, 0.5}, Vector3d{0.0, -1.0, 0.0}),
            PolyhedronLoop3d({a0, b0, b1, a1})),
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{0.75, 0.433, 0.5}, Vector3d{0.866, 0.5, 0.0}),
            PolyhedronLoop3d({b0, c0, c1, b1})),
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{0.25, 0.433, 0.5}, Vector3d{-0.866, 0.5, 0.0}),
            PolyhedronLoop3d({c0, a0, a1, c1}))});
    assert(prism.IsValid());
    assert(prism.FaceCount() == 5);

    const Plane cut = Plane::FromPointAndNormal(
        Point3d{0.5, 0.289, 0.5},
        Vector3d{0.0, 0.0, 1.0});
    const auto section = Section(prism, cut);
    assert(section.success);
    assert(section.IsValid());

    // Horizontal mid-section of a triangular prism = triangle → closed contour
    assert(!section.contours.empty());
    assert(section.contours[0].closed);
    assert(section.segments.size() == 3);

    // Perimeter of the cross-section triangle should equal side-length sum.
    // Equilateral triangle of side 1: perimeter = 3.0 (approximately, since
    // c0 uses 0.866 ≈ sqrt(3)/2).
    double totalLength = 0.0;
    const auto& pts = section.contours[0].points;
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        const Point3d& p0 = pts[i];
        const Point3d& p1 = pts[(i + 1) % pts.size()];
        const double dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
        totalLength += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    // Perimeter ≈ 3.0 (equilateral triangle side=1)
    assert(totalLength > 2.5 && totalLength < 3.5);
    assert(ClassifySectionContent(section) == SectionContentKind3d::Area);
}

// Demonstrates the same triangular-prism deterministic contour-length subset
// also holds on the Brep path after Polyhedron->Brep conversion.
TEST(Section3dCapabilityTest, BrepObliquePrismSectionYieldsDeterministicContourLength)
{
    const Point3d a0{0.0, 0.0, 0.0};
    const Point3d b0{1.0, 0.0, 0.0};
    const Point3d c0{0.5, 0.866, 0.0};
    const Point3d a1{0.0, 0.0, 1.0};
    const Point3d b1{1.0, 0.0, 1.0};
    const Point3d c1{0.5, 0.866, 1.0};

    const PolyhedronBody prism({
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{0.5, 0.289, 0.0}, Vector3d{0.0, 0.0, -1.0}),
            PolyhedronLoop3d({a0, c0, b0})),
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{0.5, 0.289, 1.0}, Vector3d{0.0, 0.0, 1.0}),
            PolyhedronLoop3d({a1, b1, c1})),
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{0.5, 0.0, 0.5}, Vector3d{0.0, -1.0, 0.0}),
            PolyhedronLoop3d({a0, b0, b1, a1})),
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{0.75, 0.433, 0.5}, Vector3d{0.866, 0.5, 0.0}),
            PolyhedronLoop3d({b0, c0, c1, b1})),
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{0.25, 0.433, 0.5}, Vector3d{-0.866, 0.5, 0.0}),
            PolyhedronLoop3d({c0, a0, a1, c1}))});
    assert(prism.IsValid());

    const auto converted = ConvertToBrepBody(prism);
    assert(converted.success);
    assert(converted.issue == BrepConversionIssue3d::None);
    assert(converted.body.IsValid());

    const Plane cut = Plane::FromPointAndNormal(
        Point3d{0.5, 0.289, 0.5},
        Vector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    assert(section.success);
    assert(section.IsValid());

    assert(!section.contours.empty());
    assert(section.contours[0].closed);
    assert(section.segments.size() == 3);

    double totalLength = 0.0;
    const auto& pts = section.contours[0].points;
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        const Point3d& p0 = pts[i];
        const Point3d& p1 = pts[(i + 1) % pts.size()];
        const double dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
        totalLength += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    assert(totalLength > 2.5 && totalLength < 3.5);
    assert(ClassifySectionContent(section) == SectionContentKind3d::Area);
}

// Demonstrates that cutting the unit cube with a plane perpendicular to the
// x-axis (x=0.5) produces a deterministic 1×1 square cross-section with
// known perimeter (4.0) and area (1.0). Extends rebar-length coverage to
// a third axis direction, complementing the y-axis mid-plane and the
// triangular-prism tests already present.
TEST(Section3dCapabilityTest, UnitCubeXAxisSectionYieldsDeterministicRebarPerimeter)
{
    const PolyhedronBody cubeBody = geometry::test::BuildUnitCubeBody();
    assert(cubeBody.IsValid());

    // x=0.5 plane intersects the four faces whose vertex x-coordinates span
    // the range (front/back/bottom/top), leaving left (x=0) and right (x=1)
    // parallel and uncut.
    const Plane cut = Plane::FromPointAndNormal(
        Point3d{0.5, 0.5, 0.5},
        Vector3d{1.0, 0.0, 0.0});
    const auto section = Section(cubeBody, cut);
    assert(section.success);
    assert(section.IsValid());

    // Exactly four segments (one per intersected face).
    assert(section.segments.size() == 4);

    // Exactly one closed contour with 4 corner points.
    assert(section.contours.size() == 1);
    assert(section.contours[0].closed);
    assert(section.contours[0].points.size() == 4);

    // Perimeter of the 1×1 square cross-section in the yz-plane = 4.0.
    double totalLength = 0.0;
    const auto& pts = section.contours[0].points;
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        const Point3d& p0 = pts[i];
        const Point3d& p1 = pts[(i + 1) % pts.size()];
        const double dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
        totalLength += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    assert(totalLength > 3.5 && totalLength < 4.5);

    assert(ClassifySectionContent(section) == SectionContentKind3d::Area);
    assert(section.polygons.size() == 1);
    assert(std::abs(geometry::sdk::Area(section.polygons[0]) - 1.0) < 1e-12);

    const auto topology = BuildSectionTopology(section);
    assert(topology.IsValid());
    assert(topology.Roots().size() == 1);

    const auto components = BuildSectionComponents(section);
    assert(components.IsValid());
    assert(components.components.size() == 1);
}

// Demonstrates the same x-axis deterministic rebar perimeter subset also
// holds on the Brep path after Polyhedron->Brep conversion.
TEST(Section3dCapabilityTest, BrepUnitCubeXAxisSectionYieldsDeterministicRebarPerimeter)
{
    const PolyhedronBody cubeBody = geometry::test::BuildUnitCubeBody();
    assert(cubeBody.IsValid());

    const auto converted = ConvertToBrepBody(cubeBody);
    assert(converted.success);
    assert(converted.issue == BrepConversionIssue3d::None);
    assert(converted.body.IsValid());

    const Plane cut = Plane::FromPointAndNormal(
        Point3d{0.5, 0.5, 0.5},
        Vector3d{1.0, 0.0, 0.0});
    const auto section = Section(converted.body, cut);
    assert(section.success);
    assert(section.IsValid());

    assert(section.segments.size() == 4);
    assert(section.contours.size() == 1);
    assert(section.contours[0].closed);
    assert(section.contours[0].points.size() == 4);

    double totalLength = 0.0;
    const auto& pts = section.contours[0].points;
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        const Point3d& p0 = pts[i];
        const Point3d& p1 = pts[(i + 1) % pts.size()];
        const double dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
        totalLength += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    assert(totalLength > 3.5 && totalLength < 4.5);

    assert(ClassifySectionContent(section) == SectionContentKind3d::Area);
    assert(section.polygons.size() == 1);
    assert(std::abs(geometry::sdk::Area(section.polygons[0]) - 1.0) < 1e-12);

    const auto topology = BuildSectionTopology(section);
    assert(topology.IsValid());
    assert(topology.Roots().size() == 1);

    const auto components = BuildSectionComponents(section);
    assert(components.IsValid());
    assert(components.components.size() == 1);
}

// Demonstrates that cutting a 2×2×1 rectangular prism (box) at its z mid-plane
// (z=0.5) yields a deterministic 2×2 square cross-section with perimeter 8.0
// and area 4.0. Validates that the rebar-line perimeter assertion scales to
// non-square cross-sections, extending 必需-6 rebar coverage beyond unit
// triangular-prism and unit-cube subsets.
TEST(Section3dCapabilityTest, RectangularPrismMidSectionYieldsDeterministicRebarPerimeter)
{
    // 2×2×1 rectangular box: x in [0,2], y in [0,2], z in [0,1].
    const PolyhedronBody prism({
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, -1.0}),
            PolyhedronLoop3d({
                Point3d{0.0, 0.0, 0.0}, Point3d{0.0, 2.0, 0.0},
                Point3d{2.0, 2.0, 0.0}, Point3d{2.0, 0.0, 0.0}})),
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{0.0, 0.0, 1.0}, Vector3d{0.0, 0.0, 1.0}),
            PolyhedronLoop3d({
                Point3d{0.0, 0.0, 1.0}, Point3d{2.0, 0.0, 1.0},
                Point3d{2.0, 2.0, 1.0}, Point3d{0.0, 2.0, 1.0}})),
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, -1.0, 0.0}),
            PolyhedronLoop3d({
                Point3d{0.0, 0.0, 0.0}, Point3d{2.0, 0.0, 0.0},
                Point3d{2.0, 0.0, 1.0}, Point3d{0.0, 0.0, 1.0}})),
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{0.0, 2.0, 0.0}, Vector3d{0.0, 1.0, 0.0}),
            PolyhedronLoop3d({
                Point3d{0.0, 2.0, 0.0}, Point3d{0.0, 2.0, 1.0},
                Point3d{2.0, 2.0, 1.0}, Point3d{2.0, 2.0, 0.0}})),
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{-1.0, 0.0, 0.0}),
            PolyhedronLoop3d({
                Point3d{0.0, 0.0, 0.0}, Point3d{0.0, 0.0, 1.0},
                Point3d{0.0, 2.0, 1.0}, Point3d{0.0, 2.0, 0.0}})),
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{2.0, 0.0, 0.0}, Vector3d{1.0, 0.0, 0.0}),
            PolyhedronLoop3d({
                Point3d{2.0, 0.0, 0.0}, Point3d{2.0, 2.0, 0.0},
                Point3d{2.0, 2.0, 1.0}, Point3d{2.0, 0.0, 1.0}}))});
    assert(prism.IsValid());
    assert(prism.FaceCount() == 6);

    // z=0.5 mid-plane cuts all four side faces (front/back/left/right), yielding
    // a 2×2 square cross-section; bottom and top remain parallel and uncut.
    const Plane cut = Plane::FromPointAndNormal(
        Point3d{1.0, 1.0, 0.5},
        Vector3d{0.0, 0.0, 1.0});
    const auto section = Section(prism, cut);
    assert(section.success);
    assert(section.IsValid());

    assert(section.contours.size() == 1);
    assert(section.contours[0].closed);
    assert(section.contours[0].points.size() == 4);
    assert(section.segments.size() == 4);

    // Perimeter of 2×2 square cross-section = 8.0.
    double totalLength = 0.0;
    const auto& pts = section.contours[0].points;
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        const Point3d& p0 = pts[i];
        const Point3d& p1 = pts[(i + 1) % pts.size()];
        const double dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
        totalLength += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    assert(totalLength > 7.0 && totalLength < 9.0);

    assert(ClassifySectionContent(section) == SectionContentKind3d::Area);
    assert(section.polygons.size() == 1);
    assert(std::abs(geometry::sdk::Area(section.polygons[0]) - 4.0) < 1e-12);

    const auto topology = BuildSectionTopology(section);
    assert(topology.IsValid());
    assert(topology.Roots().size() == 1);

    const auto components = BuildSectionComponents(section);
    assert(components.IsValid());
    assert(components.components.size() == 1);
}

// Demonstrates the same rectangular-prism deterministic perimeter subset also
// holds on the Brep path after Polyhedron->Brep conversion.
TEST(Section3dCapabilityTest, BrepRectangularPrismMidSectionYieldsDeterministicRebarPerimeter)
{
    const PolyhedronBody prism({
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, -1.0}),
            PolyhedronLoop3d({
                Point3d{0.0, 0.0, 0.0}, Point3d{0.0, 2.0, 0.0},
                Point3d{2.0, 2.0, 0.0}, Point3d{2.0, 0.0, 0.0}})),
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{0.0, 0.0, 1.0}, Vector3d{0.0, 0.0, 1.0}),
            PolyhedronLoop3d({
                Point3d{0.0, 0.0, 1.0}, Point3d{2.0, 0.0, 1.0},
                Point3d{2.0, 2.0, 1.0}, Point3d{0.0, 2.0, 1.0}})),
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, -1.0, 0.0}),
            PolyhedronLoop3d({
                Point3d{0.0, 0.0, 0.0}, Point3d{2.0, 0.0, 0.0},
                Point3d{2.0, 0.0, 1.0}, Point3d{0.0, 0.0, 1.0}})),
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{0.0, 2.0, 0.0}, Vector3d{0.0, 1.0, 0.0}),
            PolyhedronLoop3d({
                Point3d{0.0, 2.0, 0.0}, Point3d{0.0, 2.0, 1.0},
                Point3d{2.0, 2.0, 1.0}, Point3d{2.0, 2.0, 0.0}})),
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{-1.0, 0.0, 0.0}),
            PolyhedronLoop3d({
                Point3d{0.0, 0.0, 0.0}, Point3d{0.0, 0.0, 1.0},
                Point3d{0.0, 2.0, 1.0}, Point3d{0.0, 2.0, 0.0}})),
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{2.0, 0.0, 0.0}, Vector3d{1.0, 0.0, 0.0}),
            PolyhedronLoop3d({
                Point3d{2.0, 0.0, 0.0}, Point3d{2.0, 2.0, 0.0},
                Point3d{2.0, 2.0, 1.0}, Point3d{2.0, 0.0, 1.0}}))});
    assert(prism.IsValid());
    assert(prism.FaceCount() == 6);

    const auto converted = ConvertToBrepBody(prism);
    assert(converted.success);
    assert(converted.issue == BrepConversionIssue3d::None);
    assert(converted.body.IsValid());

    const Plane cut = Plane::FromPointAndNormal(
        Point3d{1.0, 1.0, 0.5},
        Vector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    assert(section.success);
    assert(section.IsValid());

    assert(section.contours.size() == 1);
    assert(section.contours[0].closed);
    assert(section.contours[0].points.size() == 4);
    assert(section.segments.size() == 4);

    double totalLength = 0.0;
    const auto& pts = section.contours[0].points;
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        const Point3d& p0 = pts[i];
        const Point3d& p1 = pts[(i + 1) % pts.size()];
        const double dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
        totalLength += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    assert(totalLength > 7.0 && totalLength < 9.0);

    assert(ClassifySectionContent(section) == SectionContentKind3d::Area);
    assert(section.polygons.size() == 1);
    assert(std::abs(geometry::sdk::Area(section.polygons[0]) - 4.0) < 1e-12);

    const auto topology = BuildSectionTopology(section);
    assert(topology.IsValid());
    assert(topology.Roots().size() == 1);

    const auto components = BuildSectionComponents(section);
    assert(components.IsValid());
    assert(components.components.size() == 1);
}

// Demonstrates rectangular-prism deterministic perimeter subset also covers an
// orthogonal axis on the Brep path: x=1 mid-cut yields a 2x1 rectangle with
// stable segments/perimeter/area.
TEST(Section3dCapabilityTest, BrepRectangularPrismXAxisSectionYieldsDeterministicRebarPerimeter)
{
    const PolyhedronBody prism({
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, -1.0}),
            PolyhedronLoop3d({
                Point3d{0.0, 0.0, 0.0}, Point3d{0.0, 2.0, 0.0},
                Point3d{2.0, 2.0, 0.0}, Point3d{2.0, 0.0, 0.0}})),
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{0.0, 0.0, 1.0}, Vector3d{0.0, 0.0, 1.0}),
            PolyhedronLoop3d({
                Point3d{0.0, 0.0, 1.0}, Point3d{2.0, 0.0, 1.0},
                Point3d{2.0, 2.0, 1.0}, Point3d{0.0, 2.0, 1.0}})),
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, -1.0, 0.0}),
            PolyhedronLoop3d({
                Point3d{0.0, 0.0, 0.0}, Point3d{2.0, 0.0, 0.0},
                Point3d{2.0, 0.0, 1.0}, Point3d{0.0, 0.0, 1.0}})),
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{0.0, 2.0, 0.0}, Vector3d{0.0, 1.0, 0.0}),
            PolyhedronLoop3d({
                Point3d{0.0, 2.0, 0.0}, Point3d{0.0, 2.0, 1.0},
                Point3d{2.0, 2.0, 1.0}, Point3d{2.0, 2.0, 0.0}})),
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{-1.0, 0.0, 0.0}),
            PolyhedronLoop3d({
                Point3d{0.0, 0.0, 0.0}, Point3d{0.0, 0.0, 1.0},
                Point3d{0.0, 2.0, 1.0}, Point3d{0.0, 2.0, 0.0}})),
        PolyhedronFace3d(
            Plane::FromPointAndNormal(Point3d{2.0, 0.0, 0.0}, Vector3d{1.0, 0.0, 0.0}),
            PolyhedronLoop3d({
                Point3d{2.0, 0.0, 0.0}, Point3d{2.0, 2.0, 0.0},
                Point3d{2.0, 2.0, 1.0}, Point3d{2.0, 0.0, 1.0}}))});
    assert(prism.IsValid());
    assert(prism.FaceCount() == 6);

    const auto converted = ConvertToBrepBody(prism);
    assert(converted.success);
    assert(converted.issue == BrepConversionIssue3d::None);
    assert(converted.body.IsValid());

    const Plane cut = Plane::FromPointAndNormal(
        Point3d{1.0, 1.0, 0.5},
        Vector3d{1.0, 0.0, 0.0});
    const auto section = Section(converted.body, cut);
    assert(section.success);
    assert(section.IsValid());

    assert(section.contours.size() == 1);
    assert(section.contours[0].closed);
    assert(section.contours[0].points.size() == 4);
    assert(section.segments.size() == 4);

    double totalLength = 0.0;
    const auto& pts = section.contours[0].points;
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        const Point3d& p0 = pts[i];
        const Point3d& p1 = pts[(i + 1) % pts.size()];
        const double dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
        totalLength += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    assert(totalLength > 5.0 && totalLength < 7.0);

    assert(ClassifySectionContent(section) == SectionContentKind3d::Area);
    assert(section.polygons.size() == 1);
    assert(std::abs(geometry::sdk::Area(section.polygons[0]) - 2.0) < 1e-12);

    const auto topology = BuildSectionTopology(section);
    assert(topology.IsValid());
    assert(topology.Roots().size() == 1);

    const auto components = BuildSectionComponents(section);
    assert(components.IsValid());
    assert(components.components.size() == 1);
}