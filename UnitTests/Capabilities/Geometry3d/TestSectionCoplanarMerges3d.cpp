#include <limits>

#include "Support/SectionFixtures3d.h"

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

TEST(Section3dCapabilityTest, SeparatedSmallCoplanarFacesRespectCallerTolerance)
{
    constexpr double side = 1e-10;
    constexpr double gap = 5e-13;
    const SCPlane cut = SCPlane::FromPointAndNormal({0.0, 0.0, 0.0}, {0.0, 0.0, 1.0});
    const PolyhedronBody body({
        PolyhedronFace3d(cut, PolyhedronLoop3d({{0.0, 0.0, 0.0}, {side, 0.0, 0.0},
                                                {side, side, 0.0}, {0.0, side, 0.0}})),
        PolyhedronFace3d(cut, PolyhedronLoop3d({{side + gap, 0.0, 0.0},
                                                {2.0 * side + gap, 0.0, 0.0},
                                                {2.0 * side + gap, side, 0.0},
                                                {side + gap, side, 0.0}}))});
    Geometry::SCGeometryTolerance3d tolerance{};
    tolerance.distanceEpsilon = 1e-15;
    ASSERT_TRUE(body.IsValid(tolerance));

    const auto section = Section(body, cut, tolerance);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid(tolerance.distanceEpsilon));
    ASSERT_EQ(section.polygons.size(), 2U);
    const auto firstBounds = section.polygons[0].Bounds(tolerance.distanceEpsilon);
    const auto secondBounds = section.polygons[1].Bounds(tolerance.distanceEpsilon);
    const double gapX = std::max(firstBounds.MinPoint().x, secondBounds.MinPoint().x) -
                        std::min(firstBounds.MaxPoint().x, secondBounds.MaxPoint().x);
    const double gapY = std::max(firstBounds.MinPoint().y, secondBounds.MinPoint().y) -
                        std::min(firstBounds.MaxPoint().y, secondBounds.MaxPoint().y);
    EXPECT_GT(std::max(gapX, gapY), tolerance.distanceEpsilon)
        << firstBounds.DebugString() << " " << secondBounds.DebugString();
}

TEST(Section3dCapabilityTest, SmallCoplanarCollinearVertexUsesCallerTolerance)
{
    constexpr double side = 1e-10;
    const SCPlane cut = SCPlane::FromPointAndNormal({0.0, 0.0, 0.0}, {0.0, 0.0, 1.0});
    const PolyhedronBody body({PolyhedronFace3d(cut, PolyhedronLoop3d(
        {{0.0, 0.0, 0.0}, {side * 0.5, 0.0, 0.0}, {side, 0.0, 0.0},
         {side, side, 0.0}, {0.0, side, 0.0}}))});
    Geometry::SCGeometryTolerance3d tolerance{};
    tolerance.distanceEpsilon = 1e-15;
    ASSERT_TRUE(body.IsValid(tolerance));

    const auto section = Section(body, cut, tolerance);
    ASSERT_TRUE(section.success);
    ASSERT_EQ(section.contours.size(), 1U);
    EXPECT_EQ(section.contours.front().points.size(), 4U);
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
