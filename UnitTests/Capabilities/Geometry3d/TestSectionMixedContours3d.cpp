#include "Support/SectionFixtures3d.h"

// Demonstrates mixed open-curve/area arbitration also keeps two representative
// edge-attached open contours on the same area result, with stable endpoint
// direction/order.
TEST(Section3dCapabilityTest, TwoEdgeAttachedOpenContoursBuildStableMixedContent)
{
    const PolyhedronBody body = BuildTwoEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 9);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 6);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_FALSE(section.contours[2].closed);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 2.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[2].points.front().x - 0.75), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y + 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 0.75), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 0.0), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, BrepTwoEdgeAttachedOpenContoursBuildStableMixedContent)
{
    const PolyhedronBody polyBody = BuildTwoEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(polyBody.IsValid());

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 8);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 6);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_FALSE(section.contours[2].closed);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 2.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[2].points.front().x - 0.75), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y + 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 0.75), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 0.0), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, VertexAndEdgeAttachedOpenContoursBuildStableMixedContent)
{
    const PolyhedronBody body = BuildVertexAndEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 7);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_FALSE(section.contours[2].closed);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 2.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[2].points.front().x - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 2.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, VertexTouchThenEdgeTouchOpenContoursDoNotCollapseIntoSinglePolyline)
{
    const PolyhedronBody body = BuildVertexAndEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 7);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_FALSE(section.contours[2].closed);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 2.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[2].points.front().x - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 2.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, BrepVertexAndEdgeAttachedOpenContoursBuildStableMixedContent)
{
    const PolyhedronBody polyBody = BuildVertexAndEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(polyBody.IsValid());

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 7);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_FALSE(section.contours[2].closed);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 2.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[2].points.front().x - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 2.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, BrepVertexTouchThenEdgeTouchOpenContoursDoNotCollapseIntoSinglePolyline)
{
    const PolyhedronBody polyBody = BuildVertexAndEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(polyBody.IsValid());

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 7);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_FALSE(section.contours[2].closed);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 2.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[2].points.front().x - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 2.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, NonPlanarLoopWithInteriorOpenSpurKeepsClosedContourAndOpenContourSeparate)
{
    const PolyhedronBody body = BuildNonPlanarLoopWithInteriorOpenSpurSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 2);
    ASSERT_EQ(section.segments.size(), 5);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_EQ(section.contours[1].points.size(), 2);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.5), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.5), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 0.25), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, BrepNonPlanarLoopWithInteriorOpenSpurKeepsClosedContourAndOpenContourSeparate)
{
    const PolyhedronBody polyBody = BuildNonPlanarLoopWithInteriorOpenSpurSectionBody();
    ASSERT_TRUE(polyBody.IsValid());

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 2);
    ASSERT_EQ(section.segments.size(), 5);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_EQ(section.contours[1].points.size(), 2);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.5), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.5), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 0.25), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, LCornerCoplanarPatchAndNonPlanarAreaMergeIntoSinglePolygon)
{
    const PolyhedronBody body = BuildLCornerCoplanarPatchAndNonPlanarSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_EQ(section.segments.size(), 6);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 6);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 3.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
}

TEST(Section3dCapabilityTest, BrepLCornerCoplanarPatchAndNonPlanarAreaMergeIntoSinglePolygon)
{
    const PolyhedronBody polyBody = BuildLCornerCoplanarPatchAndNonPlanarSectionBody();
    ASSERT_TRUE(polyBody.IsValid());

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_EQ(section.segments.size(), 6);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 6);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 3.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
}
