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

// Demonstrates coplanar fragment merge also holds for Section(SCBrepBody,
// SCPlane): after Polyhedron->Brep conversion, adjacent coplanar faces are still
// merged into one area polygon instead of staying split.
TEST(Section3dCapabilityTest, BrepBodyAdjacentCoplanarFacesMergeIntoSinglePolygon)
{
    const PolyhedronBody polyBody = BuildAdjacentCoplanarFaceBody();
    ASSERT_TRUE(polyBody.IsValid());
    ASSERT_EQ(polyBody.FaceCount(), 2);

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());
    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 2.0), 1e-12);
}

TEST(Section3dCapabilityTest, TwoDisjointNonPlanarLoopsStaySeparatedInSameSectionPlane)
{
    const PolyhedronBody body = BuildTwoSeparatedUnitCubesBody();
    ASSERT_TRUE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 12);

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());
    ASSERT_EQ(section.polygons.size(), 2);
    ASSERT_EQ(section.contours.size(), 2);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_TRUE(section.contours[1].closed);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 2);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 2);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);

    double totalArea = 0.0;
    for (const auto& polygon : section.polygons)
    {
        totalArea += polygon.Area();
    }
    ASSERT_LT(std::abs(totalArea - 2.0), 1e-12);
}

// Demonstrates Section(SCBrepBody, SCPlane) also supports multi-component area
// output: two separated cubes sectioned at z=0.5 produce two independent
// section polygons/components.
TEST(Section3dCapabilityTest, BrepBodySectionBuildsTwoAreaComponents)
{
    const PolyhedronBody polyBody = BuildTwoSeparatedUnitCubesBody();
    ASSERT_TRUE(polyBody.IsValid());
    ASSERT_EQ(polyBody.FaceCount(), 12);

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());
    ASSERT_EQ(section.polygons.size(), 2);
    ASSERT_EQ(section.contours.size(), 2);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_TRUE(section.contours[1].closed);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 2);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 2);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
}

// Demonstrates coplanar adjacent face fragments are merged into one area
// polygon instead of remaining as two disjoint coplanar pieces.
TEST(Section3dCapabilityTest, AdjacentCoplanarFacesMergeIntoSingleSectionPolygon)
{
    const PolyhedronBody body = BuildAdjacentCoplanarFaceBody();
    ASSERT_TRUE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 2);

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());
    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
    ASSERT_EQ(components.components[0].faceIndices.size(), 1);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 2.0), 1e-12);
}

// Demonstrates that the face-merge subset extends beyond two adjacent faces:
// three coplanar faces arranged in an L-strip (each pair sharing an edge)
// are all merged into a single section polygon at the coplanar cut.
// Validates that coplanar fragment merge works for chains of 3+ faces, not
// just a single adjacent pair.
TEST(Section3dCapabilityTest, ThreeCoplanarFacesInLStripMergeIntoSinglePolygon)
{
    const PolyhedronBody body = BuildThreeCoplanarStripFacesBody();
    ASSERT_TRUE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 3);

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());
    // Expected: 3 coplanar adjacent faces merge into 1 polygon (area = 3.0 x 1)
    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    // Rectangle 3猫锟? 茅藛?4 corners
    ASSERT_EQ(section.contours[0].points.size(), 4);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
    ASSERT_EQ(components.components[0].faceIndices.size(), 1);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 3.0), 1e-12);
}

// Demonstrates that the same three-face coplanar strip merge semantics are
// preserved on the Brep path: Polyhedron->Brep conversion does not fragment
// the section result into multiple polygons.
TEST(Section3dCapabilityTest, BrepThreeCoplanarFacesInStripMergeIntoSinglePolygon)
{
    const PolyhedronBody polyBody = BuildThreeCoplanarStripFacesBody();
    ASSERT_TRUE(polyBody.IsValid());
    ASSERT_EQ(polyBody.FaceCount(), 3);

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());
    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);
    ASSERT_EQ(section.segments.size(), 4);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 3.0), 1e-12);
}

// Demonstrates higher-order coplanar fragment merge semantics beyond strips:
// four coplanar faces arranged as a rectangular frame merge into one polygon
// with one rectangular hole instead of remaining split into four fragments.
TEST(Section3dCapabilityTest, CoplanarFrameFacesMergeIntoSinglePolygonWithHole)
{
    const PolyhedronBody body = BuildCoplanarFrameFacesBody();
    ASSERT_TRUE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 4);

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.polygons[0].HoleCount(), 1);
    ASSERT_EQ(section.contours.size(), 2);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_TRUE(section.contours[1].closed);
    ASSERT_EQ(section.segments.size(), 9);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 8.0), 1e-12);
}

// Demonstrates mixed section semantics: a coplanar frame component and a
// separate non-planar cube mid-section can coexist in one result instead of
// the coplanar component short-circuiting the non-planar contour stitching.
TEST(Section3dCapabilityTest, MixedCoplanarAndNonPlanarSectionBuildsTwoAreaComponents)
{
    const PolyhedronBody body = BuildMixedCoplanarFrameAndIntersectingCubeBody();
    ASSERT_TRUE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 10);

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 2);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 12);

    std::size_t polygonsWithHole = 0;
    double totalArea = 0.0;
    for (const auto& polygon : section.polygons)
    {
        if (polygon.HoleCount() == 1)
        {
            ++polygonsWithHole;
        }
        totalArea += polygon.Area();
    }
    ASSERT_EQ(polygonsWithHole, 1);
    ASSERT_LT(std::abs(totalArea - 9.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 2);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 2);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
}

// Demonstrates the same mixed coplanar + non-planar section subset also
// holds on the Brep path after Polyhedron->Brep conversion.
TEST(Section3dCapabilityTest, BrepMixedCoplanarAndNonPlanarSectionBuildsTwoAreaComponents)
{
    const PolyhedronBody polyBody = BuildMixedCoplanarFrameAndIntersectingCubeBody();
    ASSERT_TRUE(polyBody.IsValid());
    ASSERT_EQ(polyBody.FaceCount(), 10);

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 2);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 12);

    std::size_t polygonsWithHole = 0;
    double totalArea = 0.0;
    for (const auto& polygon : section.polygons)
    {
        if (polygon.HoleCount() == 1)
        {
            ++polygonsWithHole;
        }
        totalArea += polygon.Area();
    }
    ASSERT_EQ(polygonsWithHole, 1);
    ASSERT_LT(std::abs(totalArea - 9.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 2);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 2);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
}
