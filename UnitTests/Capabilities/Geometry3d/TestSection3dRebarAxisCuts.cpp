#include "Support/SectionFixtures3d.h"

// Demonstrates rectangular-prism deterministic perimeter subset covers an
// orthogonal axis on the Polyhedron path: x=1 mid-cut yields a 2-by-1 rectangle
// (y茅藛颅氓艙录0,2], z茅藛颅氓艙录0,1]) with stable segments/perimeter/area.
TEST(Section3dCapabilityTest, RectangularPrismXAxisSectionYieldsDeterministicRebarPerimeter)
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

    // x=1 cuts front(y=0)/back(y=2)/bottom(z=0)/top(z=1): 4 faces, leaving
    // the two x-normal caps parallel and uncut; a 2-by-1 rectangle in the
    // yz-plane.
    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{1.0, 1.0, 0.5}, SCVector3d{1.0, 0.0, 0.0});
    const auto section = Section(prism, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);
    ASSERT_EQ(section.segments.size(), 4);

    // Perimeter of 2-by-1 rectangle = 6.0.
    double totalLength = 0.0;
    const auto& pts = section.contours[0].points;
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        const SCPoint3d& p0 = pts[i];
        const SCPoint3d& p1 = pts[(i + 1) % pts.size()];
        const double dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
        totalLength += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    ASSERT_GT(totalLength, 5.0);
    ASSERT_LT(totalLength, 7.0);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 2.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
}

// Demonstrates rectangular-prism deterministic perimeter subset also covers an
// orthogonal axis on the Brep path: x=1 mid-cut yields a 2x1 rectangle with
// stable segments/perimeter/area.
TEST(Section3dCapabilityTest, BrepRectangularPrismXAxisSectionYieldsDeterministicRebarPerimeter)
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

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{1.0, 1.0, 0.5}, SCVector3d{1.0, 0.0, 0.0});
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
    ASSERT_GT(totalLength, 5.0);
    ASSERT_LT(totalLength, 7.0);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 2.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
}

// Demonstrates rectangular-prism deterministic perimeter subset covers the
// y-axis on the Polyhedron path: y=1 mid-cut yields a 2-by-1 rectangle
// (x茅藛颅氓艙录0,2], z茅藛颅氓艙录0,1]) with stable segments/perimeter/area.
TEST(Section3dCapabilityTest, RectangularPrismYAxisSectionYieldsDeterministicRebarPerimeter)
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

    // y=1 cuts left(x=0)/right(x=2)/bottom(z=0)/top(z=1): a 2-by-1 rectangle
    // in xz.
    const SCPlane cut2 = SCPlane::FromPointAndNormal(SCPoint3d{1.0, 1.0, 0.5}, SCVector3d{0.0, 1.0, 0.0});
    const auto section2 = Section(prism, cut2);
    ASSERT_TRUE(section2.success);
    ASSERT_TRUE(section2.IsValid());

    ASSERT_EQ(section2.contours.size(), 1);
    ASSERT_TRUE(section2.contours[0].closed);
    ASSERT_EQ(section2.contours[0].points.size(), 4);
    ASSERT_EQ(section2.segments.size(), 4);

    double totalLength2 = 0.0;
    const auto& pts2 = section2.contours[0].points;
    for (std::size_t i = 0; i < pts2.size(); ++i)
    {
        const SCPoint3d& p0 = pts2[i];
        const SCPoint3d& p1 = pts2[(i + 1) % pts2.size()];
        const double dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
        totalLength2 += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    ASSERT_GT(totalLength2, 5.0);
    ASSERT_LT(totalLength2, 7.0);

    ASSERT_EQ(ClassifySectionContent(section2), SectionContentKind3d::Area);
    ASSERT_EQ(section2.polygons.size(), 1);
    ASSERT_LT(std::abs(section2.polygons[0].Area() - 2.0), 1e-12);

    const auto topology2 = BuildSectionTopology(section2);
    ASSERT_TRUE(topology2.IsValid());
    ASSERT_EQ(topology2.Roots().size(), 1);

    const auto components2 = BuildSectionComponents(section2);
    ASSERT_TRUE(components2.IsValid());
    ASSERT_EQ(components2.components.size(), 1);
}

// Demonstrates the same rectangular-prism y-axis subset also holds on the
// Brep path after Polyhedron->Brep conversion.
TEST(Section3dCapabilityTest, BrepRectangularPrismYAxisSectionYieldsDeterministicRebarPerimeter)
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

    const SCPlane cut3 = SCPlane::FromPointAndNormal(SCPoint3d{1.0, 1.0, 0.5}, SCVector3d{0.0, 1.0, 0.0});
    const auto section3 = Section(converted.body, cut3);
    ASSERT_TRUE(section3.success);
    ASSERT_TRUE(section3.IsValid());

    ASSERT_EQ(section3.contours.size(), 1);
    ASSERT_TRUE(section3.contours[0].closed);
    ASSERT_EQ(section3.contours[0].points.size(), 4);
    ASSERT_EQ(section3.segments.size(), 4);

    double totalLength3 = 0.0;
    const auto& pts3 = section3.contours[0].points;
    for (std::size_t i = 0; i < pts3.size(); ++i)
    {
        const SCPoint3d& p0 = pts3[i];
        const SCPoint3d& p1 = pts3[(i + 1) % pts3.size()];
        const double dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
        totalLength3 += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    ASSERT_GT(totalLength3, 5.0);
    ASSERT_LT(totalLength3, 7.0);

    ASSERT_EQ(ClassifySectionContent(section3), SectionContentKind3d::Area);
    ASSERT_EQ(section3.polygons.size(), 1);
    ASSERT_LT(std::abs(section3.polygons[0].Area() - 2.0), 1e-12);

    const auto topology3 = BuildSectionTopology(section3);
    ASSERT_TRUE(topology3.IsValid());
    ASSERT_EQ(topology3.Roots().size(), 1);

    const auto components3 = BuildSectionComponents(section3);
    ASSERT_TRUE(components3.IsValid());
    ASSERT_EQ(components3.components.size(), 1);
}

// Demonstrates that cutting a unit cube (x,y,z in [0,1]) at z=0.5 (horizontal
// mid-plane) yields a deterministic 1-by-1 square cross-section with
// perimeter 4.0 and area 1.0, extending the rebar-line coverage to the z-axis
// direction on the PolyhedronBody path.
TEST(Section3dCapabilityTest, UnitCubeZAxisSectionYieldsDeterministicRebarPerimeter)
{
    const PolyhedronBody cubeBody = Geometry::Test::BuildUnitCubeBody();
    ASSERT_TRUE(cubeBody.IsValid());

    // z=0.5 plane intersects the four lateral faces whose vertex z-coordinates
    // span the range, leaving bottom (z=0) and top (z=1) uncut.
    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.5, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(cubeBody, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    // Exactly four segments (one per intersected lateral face).
    ASSERT_EQ(section.segments.size(), 4);

    // Exactly one closed contour with 4 corner points.
    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);

    // Perimeter of the 1-by-1 square cross-section in the xy-plane = 4.0.
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

// Demonstrates the same z-axis deterministic rebar perimeter subset also
// holds on the Brep path after Polyhedron->Brep conversion.
TEST(Section3dCapabilityTest, BrepUnitCubeZAxisSectionYieldsDeterministicRebarPerimeter)
{
    const PolyhedronBody cubeBody = Geometry::Test::BuildUnitCubeBody();
    ASSERT_TRUE(cubeBody.IsValid());

    const auto converted = ConvertToBrepBody(cubeBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.5, 0.5}, SCVector3d{0.0, 0.0, 1.0});
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
