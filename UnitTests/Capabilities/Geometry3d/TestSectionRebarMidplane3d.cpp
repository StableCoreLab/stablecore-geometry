#include "Support/SectionFixtures3d.h"

// Demonstrates that cutting the unit cube with a plane perpendicular to the
// x-axis (x=0.5) produces a deterministic 1-by-1 square cross-section with
// known perimeter (4.0) and area (1.0). Extends rebar-length coverage to
// a third axis direction, complementing the y-axis mid-plane and the
// triangular-prism tests already present.
TEST(Section3dCapabilityTest, UnitCubeXAxisSectionYieldsDeterministicRebarPerimeter)
{
    const PolyhedronBody cubeBody = Geometry::Test::BuildUnitCubeBody();
    ASSERT_TRUE(cubeBody.IsValid());

    // x=0.5 plane intersects the four faces whose vertex x-coordinates span
    // the range (front/back/bottom/top), leaving left (x=0) and right (x=1)
    // parallel and uncut.
    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.5, 0.5}, SCVector3d{1.0, 0.0, 0.0});
    const auto section = Section(cubeBody, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    // Exactly four segments (one per intersected face).
    ASSERT_EQ(section.segments.size(), 4);

    // Exactly one closed contour with 4 corner points.
    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);

    // Perimeter of the 1-by-1 square cross-section in the yz-plane = 4.0.
    double totalLength = 0.0;
    const auto& pts = section.contours[0].points;
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        const SCPoint3d& p0 = pts[i];
        const SCPoint3d& p1 = pts[(i + 1) % pts.size()];
        const double dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
        totalLength += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    ASSERT_GT(totalLength, 3.5);
    ASSERT_LT(totalLength, 4.5);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
}

// Demonstrates the same x-axis deterministic rebar perimeter subset also
// holds on the Brep path after Polyhedron->Brep conversion.
TEST(Section3dCapabilityTest, BrepUnitCubeXAxisSectionYieldsDeterministicRebarPerimeter)
{
    const PolyhedronBody cubeBody = Geometry::Test::BuildUnitCubeBody();
    ASSERT_TRUE(cubeBody.IsValid());

    const auto converted = ConvertToBrepBody(cubeBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.5, 0.5}, SCVector3d{1.0, 0.0, 0.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.segments.size(), 4);
    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);

    double totalLength = 0.0;
    const auto& pts = section.contours[0].points;
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        const SCPoint3d& p0 = pts[i];
        const SCPoint3d& p1 = pts[(i + 1) % pts.size()];
        const double dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
        totalLength += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    ASSERT_GT(totalLength, 3.5);
    ASSERT_LT(totalLength, 4.5);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
}

// Demonstrates that cutting a 2-by-2-by-1 rectangular prism (box) at its z
// mid-plane (z=0.5) yields a deterministic 2-by-2 square cross-section with
// perimeter 8.0 and area 4.0. Validates that the rebar-line perimeter assertion
// scales to a larger cross-section, extending the rebar
// coverage beyond unit triangular-prism and unit-cube subsets.
TEST(Section3dCapabilityTest, RectangularPrismMidSectionYieldsDeterministicRebarPerimeter)
{
    // 2-by-2-by-1 rectangular box: x in [0,2], y in [0,2], z in [0,1].
    const PolyhedronBody prism(
        {PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, -1.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{0.0, 2.0, 0.0}, SCPoint3d{2.0, 2.0, 0.0}, SCPoint3d{2.0, 0.0, 0.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 1.0}, SCVector3d{0.0, 0.0, 1.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 1.0}, SCPoint3d{2.0, 0.0, 1.0}, SCPoint3d{2.0, 2.0, 1.0}, SCPoint3d{0.0, 2.0, 1.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, -1.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{2.0, 0.0, 0.0}, SCPoint3d{2.0, 0.0, 1.0}, SCPoint3d{0.0, 0.0, 1.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 2.0, 0.0}, SCVector3d{0.0, 1.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 2.0, 0.0}, SCPoint3d{0.0, 2.0, 1.0}, SCPoint3d{2.0, 2.0, 1.0}, SCPoint3d{2.0, 2.0, 0.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{-1.0, 0.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{0.0, 0.0, 1.0}, SCPoint3d{0.0, 2.0, 1.0}, SCPoint3d{0.0, 2.0, 0.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{2.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{2.0, 0.0, 0.0}, SCPoint3d{2.0, 2.0, 0.0}, SCPoint3d{2.0, 2.0, 1.0}, SCPoint3d{2.0, 0.0, 1.0}}))});
    ASSERT_TRUE(prism.IsValid());
    ASSERT_EQ(prism.FaceCount(), 6);

    // z=0.5 mid-plane cuts all four side faces (front/back/left/right),
    // yielding a 2-by-2 square cross-section; bottom and top remain parallel
    // and uncut.
    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{1.0, 1.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(prism, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);
    ASSERT_EQ(section.segments.size(), 4);

    // Perimeter of 2-by-2 square cross-section = 8.0.
    double totalLength = 0.0;
    const auto& pts = section.contours[0].points;
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        const SCPoint3d& p0 = pts[i];
        const SCPoint3d& p1 = pts[(i + 1) % pts.size()];
        const double dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
        totalLength += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    ASSERT_GT(totalLength, 7.0);
    ASSERT_LT(totalLength, 9.0);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 4.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
}

// Demonstrates the same rectangular-prism deterministic perimeter subset also
// holds on the Brep path after Polyhedron->Brep conversion.
TEST(Section3dCapabilityTest, BrepRectangularPrismMidSectionYieldsDeterministicRebarPerimeter)
{
    const PolyhedronBody prism(
        {PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, -1.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{0.0, 2.0, 0.0}, SCPoint3d{2.0, 2.0, 0.0}, SCPoint3d{2.0, 0.0, 0.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 1.0}, SCVector3d{0.0, 0.0, 1.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 1.0}, SCPoint3d{2.0, 0.0, 1.0}, SCPoint3d{2.0, 2.0, 1.0}, SCPoint3d{0.0, 2.0, 1.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, -1.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{2.0, 0.0, 0.0}, SCPoint3d{2.0, 0.0, 1.0}, SCPoint3d{0.0, 0.0, 1.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 2.0, 0.0}, SCVector3d{0.0, 1.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 2.0, 0.0}, SCPoint3d{0.0, 2.0, 1.0}, SCPoint3d{2.0, 2.0, 1.0}, SCPoint3d{2.0, 2.0, 0.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{-1.0, 0.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{0.0, 0.0, 1.0}, SCPoint3d{0.0, 2.0, 1.0}, SCPoint3d{0.0, 2.0, 0.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{2.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{2.0, 0.0, 0.0}, SCPoint3d{2.0, 2.0, 0.0}, SCPoint3d{2.0, 2.0, 1.0}, SCPoint3d{2.0, 0.0, 1.0}}))});
    ASSERT_TRUE(prism.IsValid());
    ASSERT_EQ(prism.FaceCount(), 6);

    const auto converted = ConvertToBrepBody(prism);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{1.0, 1.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);
    ASSERT_EQ(section.segments.size(), 4);

    double totalLength = 0.0;
    const auto& pts = section.contours[0].points;
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        const SCPoint3d& p0 = pts[i];
        const SCPoint3d& p1 = pts[(i + 1) % pts.size()];
        const double dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
        totalLength += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    ASSERT_GT(totalLength, 7.0);
    ASSERT_LT(totalLength, 9.0);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 4.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
}
