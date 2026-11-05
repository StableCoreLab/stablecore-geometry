#include "Support/SectionFixtures3d.h"

// Demonstrates that a mid-plane cut through a unit cube (whose 4 intersected
// faces are mutually non-coplanar) stitches into exactly one closed contour
// with exactly 4 corner points, proving that segment stitching across
// non-coplanar face pairs is deterministic for convex closed bodies.
// This narrows the NonPlanarDominantSectionGraphRemainsOpen gap to the
// specific subset: convex-body oblique-face-to-face stitching.

TEST(Section3dCapabilityTest, UnitCubeMidPlaneSectionYieldsFourSegmentClosedContour)
{
    const PolyhedronBody cubeBody = Geometry::Test::BuildUnitCubeBody();
    ASSERT_TRUE(cubeBody.IsValid());

    // y=0.5 mid-plane cuts left/right/bottom/top faces (all mutually
    // non-coplanar).
    const SCPlane midCut = SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.5, 0.5}, SCVector3d{0.0, 1.0, 0.0});
    const auto section = Section(cubeBody, midCut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    // Exactly one closed rectangular contour with 4 corner points.
    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);

    // Exactly 4 segments (one per intersected face).
    ASSERT_EQ(section.segments.size(), 4);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
    // Area of the 1-by-1 rectangle in the xz plane.
    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Size(), 1);
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
}

// Demonstrates the same unit-cube y=0.5 mid-plane subset also holds on the
// Brep path after Polyhedron->Brep conversion.
TEST(Section3dCapabilityTest, BrepUnitCubeMidPlaneSectionYieldsFourSegmentClosedContour)
{
    const PolyhedronBody cubeBody = Geometry::Test::BuildUnitCubeBody();
    ASSERT_TRUE(cubeBody.IsValid());

    const auto converted = ConvertToBrepBody(cubeBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane midCut = SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.5, 0.5}, SCVector3d{0.0, 1.0, 0.0});
    const auto section = Section(converted.body, midCut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);
    ASSERT_EQ(section.segments.size(), 4);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Size(), 1);
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components2 = BuildSectionComponents(section);
    ASSERT_TRUE(components2.IsValid());
    ASSERT_EQ(components2.components.size(), 1);
}

// Demonstrates that Section() on a closed prism-like polyhedron where the
// cut plane is oblique to all faces produces a deterministic closed contour
// and that the sum of contour edge lengths (total rebar perimeter) is stable.
TEST(Section3dCapabilityTest, ObliquePrismSectionYieldsDeterministicContourLength)
{
    // Right triangular prism: two triangular faces (top/bottom) + three quad
    // side faces, all z-aligned, base triangle in z=0 plane.
    const SCPoint3d a0{0.0, 0.0, 0.0};
    const SCPoint3d b0{1.0, 0.0, 0.0};
    const SCPoint3d c0{0.5, 0.866, 0.0};
    const SCPoint3d a1{0.0, 0.0, 1.0};
    const SCPoint3d b1{1.0, 0.0, 1.0};
    const SCPoint3d c1{0.5, 0.866, 1.0};

    const PolyhedronBody prism(
        {PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.289, 0.0}, SCVector3d{0.0, 0.0, -1.0}),
                          PolyhedronLoop3d({a0, c0, b0})),
         PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.289, 1.0}, SCVector3d{0.0, 0.0, 1.0}),
                          PolyhedronLoop3d({a1, b1, c1})),
         PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.0, 0.5}, SCVector3d{0.0, -1.0, 0.0}),
                          PolyhedronLoop3d({a0, b0, b1, a1})),
         PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.75, 0.433, 0.5}, SCVector3d{0.866, 0.5, 0.0}),
                          PolyhedronLoop3d({b0, c0, c1, b1})),
         PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.25, 0.433, 0.5}, SCVector3d{-0.866, 0.5, 0.0}),
                          PolyhedronLoop3d({c0, a0, a1, c1}))});
    ASSERT_TRUE(prism.IsValid());
    ASSERT_EQ(prism.FaceCount(), 5);

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.289, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(prism, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    // Horizontal mid-section of a triangular prism forms a closed triangle.
    // contour
    ASSERT_FALSE(section.contours.empty());
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.segments.size(), 3);

    // Perimeter of the cross-section triangle should equal side-length sum.
    // Equilateral triangle of side 1: perimeter = 3.0 (approximately, since
    // c0 uses 0.866 (sqrt(3)/2).
    double totalLength = 0.0;
    const auto& pts = section.contours[0].points;
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        const SCPoint3d& p0 = pts[i];
        const SCPoint3d& p1 = pts[(i + 1) % pts.size()];
        const double dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
        totalLength += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    // Perimeter is 3.0 (equilateral triangle side=1).
    ASSERT_GT(totalLength, 2.5);
    ASSERT_LT(totalLength, 3.5);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
}

// Demonstrates the same triangular-prism deterministic contour-length subset
// also holds on the Brep path after Polyhedron->Brep conversion.
TEST(Section3dCapabilityTest, BrepObliquePrismSectionYieldsDeterministicContourLength)
{
    const SCPoint3d a0{0.0, 0.0, 0.0};
    const SCPoint3d b0{1.0, 0.0, 0.0};
    const SCPoint3d c0{0.5, 0.866, 0.0};
    const SCPoint3d a1{0.0, 0.0, 1.0};
    const SCPoint3d b1{1.0, 0.0, 1.0};
    const SCPoint3d c1{0.5, 0.866, 1.0};

    const PolyhedronBody prism(
        {PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.289, 0.0}, SCVector3d{0.0, 0.0, -1.0}),
                          PolyhedronLoop3d({a0, c0, b0})),
         PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.289, 1.0}, SCVector3d{0.0, 0.0, 1.0}),
                          PolyhedronLoop3d({a1, b1, c1})),
         PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.0, 0.5}, SCVector3d{0.0, -1.0, 0.0}),
                          PolyhedronLoop3d({a0, b0, b1, a1})),
         PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.75, 0.433, 0.5}, SCVector3d{0.866, 0.5, 0.0}),
                          PolyhedronLoop3d({b0, c0, c1, b1})),
         PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.25, 0.433, 0.5}, SCVector3d{-0.866, 0.5, 0.0}),
                          PolyhedronLoop3d({c0, a0, a1, c1}))});
    ASSERT_TRUE(prism.IsValid());

    const auto converted = ConvertToBrepBody(prism);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.289, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_FALSE(section.contours.empty());
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.segments.size(), 3);

    double totalLength = 0.0;
    const auto& pts = section.contours[0].points;
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        const SCPoint3d& p0 = pts[i];
        const SCPoint3d& p1 = pts[(i + 1) % pts.size()];
        const double dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
        totalLength += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    ASSERT_GT(totalLength, 2.5);
    ASSERT_LT(totalLength, 3.5);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
}
