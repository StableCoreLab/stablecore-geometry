#include <limits>

#include "Support/SectionFixtures3d.h"

TEST(Section3dCapabilityTest, ValidationAcceptsShortIndependentAxes)
{
    Geometry::SCPolyhedronSection3d section{};
    section.success = true;
    section.uAxis = SCVector3d{1e-5, 0.0, 0.0};
    section.vAxis = SCVector3d{0.0, 1e-5, 0.0};
    EXPECT_TRUE(Geometry::Validate(section).valid);

    section.uAxis = SCVector3d{1.0, 0.0, 0.0};
    section.vAxis = SCVector3d{1.0, 5e-10, 0.0};
    EXPECT_EQ(Geometry::Validate(section).issue,
              Geometry::SCSectionValidationIssue3d::InvalidBasis);
}

TEST(Section3dCapabilityTest, ValidationPassesCallerToleranceToSectionPolygons)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    Geometry::SCSectionPolyline3d contour{false, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}};
    EXPECT_FALSE(contour.IsValid(-1.0));
    EXPECT_FALSE(contour.IsValid(nan));
    Geometry::SCSectionBodySetRebuildResult3d bodySet{};
    bodySet.success = true;
    EXPECT_FALSE(bodySet.IsValid(-1.0));
    EXPECT_FALSE(bodySet.IsValid(nan));

    Geometry::SCPolyhedronSection3d section{};
    section.success = true;
    section.uAxis = SCVector3d{1.0, 0.0, 0.0};
    section.vAxis = SCVector3d{0.0, 1.0, 0.0};
    EXPECT_FALSE(section.IsValid(-1.0));
    EXPECT_FALSE(section.IsValid(nan));
    EXPECT_FALSE(Geometry::BuildSectionTopology(section, -1.0).IsValid());
    EXPECT_FALSE(Geometry::BuildSectionTopology(section, nan).IsValid());
    section.polygons.emplace_back(Geometry::SCPolyline2d(
        {{0.0, 0.0}, {1e-10, 0.0}, {1e-10, 1e-10}, {0.0, 1e-10}},
        Geometry::SCPolylineClosure::Closed));

    ASSERT_FALSE(section.polygons.front().IsValid());
    ASSERT_TRUE(section.polygons.front().IsValid(1e-15));
    EXPECT_TRUE(section.IsValid(1e-15));
    EXPECT_TRUE(Geometry::Validate(section, 1e-15).valid);
    EXPECT_TRUE(Geometry::BuildSectionTopology(section, 1e-15).IsValid());
    EXPECT_TRUE(Geometry::RebuildSectionFaces(section, 1e-15).success);

    section.polygons.clear();
    section.polygons.emplace_back(Geometry::SCPolyline2d(
        {{0.0, 0.0}, {0.0, 1e-10}, {1e-10, 1e-10}, {1e-10, 0.0}},
        Geometry::SCPolylineClosure::Closed));
    const auto rebuiltClockwise = Geometry::RebuildSectionFaces(section, 1e-15);
    ASSERT_TRUE(rebuiltClockwise.success);
    ASSERT_EQ(rebuiltClockwise.faces.size(), 1U);
    const auto& vertices = rebuiltClockwise.faces.front().OuterLoopRef().Vertices();
    ASSERT_EQ(vertices.size(), 4U);
    EXPECT_GT(Geometry::Cross(vertices[1] - vertices[0], vertices[2] - vertices[1]).z, 0.0);
}

TEST(Section3dCapabilityTest, RebuildBrepFacePreservesUvWithShortSectionAxis)
{
    Geometry::SCPolyhedronSection3d section{};
    section.success = true;
    section.uAxis = SCVector3d{1e-6, 0.0, 0.0};
    section.vAxis = SCVector3d{0.0, 1.0, 0.0};
    section.polygons.emplace_back(Geometry::SCPolyline2d(
        {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}}, Geometry::SCPolylineClosure::Closed));

    const auto rebuilt = Geometry::RebuildSectionBrepFaces(section);
    ASSERT_TRUE(rebuilt.success);
    ASSERT_EQ(rebuilt.faces.size(), 1U);
    ASSERT_TRUE(rebuilt.faces.front().IsValid());
    const auto& trim = rebuilt.faces.front().OuterTrim();
    ASSERT_EQ(trim.PointCount(), 4U);
    double maximumU = 0.0;
    for (std::size_t i = 0; i < trim.PointCount(); ++i)
    {
        maximumU = std::max(maximumU, trim.UvPointAt(i).x);
    }
    EXPECT_NEAR(maximumU, 1.0, 1e-9);
}

TEST(Section3dCapabilityTest, RebuildBrepFacesRejectsSkewSectionAxes)
{
    Geometry::SCPolyhedronSection3d section{};
    section.success = true;
    section.uAxis = SCVector3d{1.0, 0.0, 0.0};
    section.vAxis = SCVector3d{1.0, 1.0, 0.0};
    section.polygons.emplace_back(Geometry::SCPolyline2d(
        {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}}, Geometry::SCPolylineClosure::Closed));

    const auto rebuilt = Geometry::RebuildSectionBrepFaces(section);
    EXPECT_FALSE(rebuilt.success);
    EXPECT_EQ(rebuilt.issue, Geometry::SCSectionFaceRebuildIssue3d::InvalidSection);
    EXPECT_TRUE(rebuilt.faces.empty());
}

TEST(Section3dCapabilityTest, TriangleCutHonorsSubDefaultDistanceTolerance)
{
    const SCPoint3d a{0.0, 0.0, -1e-11};
    const SCPoint3d b{1.0, 0.0, 1e-11};
    const SCPoint3d c{0.0, 1.0, 1e-11};
    const SCPlane support = SCPlane::FromPointAndNormal(a, Geometry::Cross(b - a, c - a));
    const PolyhedronBody body({PolyhedronFace3d(support, PolyhedronLoop3d({a, b, c}))});
    ASSERT_TRUE(body.IsValid());

    Geometry::SCGeometryTolerance3d tolerance{};
    tolerance.distanceEpsilon = 1e-14;
    const auto section = Section(body, SCPlane::FromPointAndNormal({}, {0.0, 0.0, 1.0}), tolerance);
    ASSERT_TRUE(section.success);
    ASSERT_FALSE(section.segments.empty());
    for (const auto& segment : section.segments)
    {
        EXPECT_NEAR(segment.startPoint.z, 0.0, tolerance.distanceEpsilon);
        EXPECT_NEAR(segment.endPoint.z, 0.0, tolerance.distanceEpsilon);
    }
}

TEST(Section3dCapabilityTest, FaceCoplanarityUsesAngularTolerance)
{
    const SCPoint3d a{0.0, 0.0, 0.0};
    const SCPoint3d b{100.0, 0.0, 0.05};
    const SCPoint3d c{-100.0, 100.0, -0.05};
    const SCPlane support = SCPlane::FromPointAndNormal(a, Geometry::Cross(b - a, c - a));
    const PolyhedronBody body({PolyhedronFace3d(support, PolyhedronLoop3d({a, b, c}))});
    Geometry::SCGeometryTolerance3d tolerance{};
    tolerance.distanceEpsilon = 1e-3;
    tolerance.angleEpsilon = 1e-8;
    ASSERT_TRUE(body.IsValid(tolerance));

    const auto section = Section(body, SCPlane::FromPointAndNormal({}, {0.0, 0.0, 1.0}), tolerance);
    ASSERT_TRUE(section.success);
    EXPECT_TRUE(section.polygons.empty());
    EXPECT_FALSE(section.segments.empty());
}

TEST(Section3dCapabilityTest, BrepFaceCoplanarityUsesAngularTolerance)
{
    const SCPoint3d a{0.0, 0.0, 0.0};
    const SCPoint3d b{1.0, 0.0, -std::tan(1.0)};
    const SCPoint3d c{0.0, 1.0, 0.0};
    const SCPlane support = SCPlane::FromPointAndNormal(a, Geometry::Cross(b - a, c - a));
    const PolyhedronBody body({PolyhedronFace3d(support, PolyhedronLoop3d({a, b, c}))});
    ASSERT_TRUE(body.IsValid());
    const auto converted = ConvertToBrepBody(body);
    ASSERT_TRUE(converted.success);

    Geometry::SCGeometryTolerance3d tolerance{};
    tolerance.angleEpsilon = 0.9;
    const auto section = Section(converted.body,
                                 SCPlane::FromPointAndNormal({}, {0.0, 0.0, 1.0}), tolerance);
    ASSERT_TRUE(section.success);
    EXPECT_TRUE(section.polygons.empty());
}

TEST(Section3dCapabilityTest, SlantedCubeSectionBuildsSingleAreaComponent)
{
    const PolyhedronBody cubeBody = Geometry::Test::BuildUnitCubeBody();
    ASSERT_TRUE(cubeBody.IsValid());

    const SCPlane slantedCut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{1.0, 0.0, 1.0});
    const auto section = Section(cubeBody, slantedCut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());
    ASSERT_FALSE(section.segments.empty());
    ASSERT_FALSE(section.polygons.empty());

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Count(), 1);
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
    ASSERT_EQ(components.components.front().polygonIndices.size(), 1);
    ASSERT_EQ(components.components.front().faceIndices.size(), 1);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
}

// Demonstrates non-axis-aligned multi-face sectioning remains stable on a
// generic cube cut: contour/polygon counts stay deterministic.
TEST(Section3dCapabilityTest, NonAxisAlignedCubeSectionHasSingleHexLikeContour)
{
    const PolyhedronBody cubeBody = Geometry::Test::BuildUnitCubeBody();
    ASSERT_TRUE(cubeBody.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.5, 0.5}, SCVector3d{1.0, 1.0, 1.0});
    const auto section = Section(cubeBody, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    // For x+y+z=1.5 slicing a unit cube, the expected closed contour has 6
    // corners.
    ASSERT_EQ(section.contours[0].points.size(), 6);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
}

// Demonstrates Section(SCBrepBody, SCPlane) remains stable on an oblique cut:
// after Polyhedron->Brep conversion, sectioning still yields one closed area
// contour with deterministic vertex count.
TEST(Section3dCapabilityTest, BrepBodyObliqueSectionHasSingleHexLikeContour)
{
    const PolyhedronBody cubeBody = Geometry::Test::BuildUnitCubeBody();
    ASSERT_TRUE(cubeBody.IsValid());

    const auto converted = ConvertToBrepBody(cubeBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.5, 0.5}, SCVector3d{1.0, 1.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 6);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
}
