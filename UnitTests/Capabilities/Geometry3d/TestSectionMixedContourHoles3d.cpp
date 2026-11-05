#include "Support/SectionFixtures3d.h"

TEST(Section3dCapabilityTest, MixedMergedAreaWithInteriorHoleStaysSinglePolygonWithHole)
{
    const PolyhedronBody body = BuildMixedMergedHoleAndAdjacentNonPlanarSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.polygons[0].HoleCount(), 1);
    ASSERT_EQ(section.contours.size(), 2);
    ASSERT_EQ(section.segments.size(), 12);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 8);
    ASSERT_TRUE(section.contours[1].closed);
    ASSERT_EQ(section.contours[1].points.size(), 4);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 9.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
}

TEST(Section3dCapabilityTest, BrepMixedMergedAreaWithInteriorHoleStaysSinglePolygonWithHole)
{
    const PolyhedronBody polyBody = BuildMixedMergedHoleAndAdjacentNonPlanarSectionBody();
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
    ASSERT_EQ(section.polygons[0].HoleCount(), 1);
    ASSERT_EQ(section.contours.size(), 2);
    ASSERT_EQ(section.segments.size(), 12);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 8);
    ASSERT_TRUE(section.contours[1].closed);
    ASSERT_EQ(section.contours[1].points.size(), 4);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 9.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
}

TEST(Section3dCapabilityTest, MixedMergedAreaWithInteriorHoleAndDetachedOpenContourStaysMixed)
{
    const PolyhedronBody body = BuildMixedMergedHoleAndAdjacentNonPlanarWithDetachedOpenSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.polygons[0].HoleCount(), 1);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 13);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 8);
    ASSERT_TRUE(section.contours[1].closed);
    ASSERT_EQ(section.contours[1].points.size(), 4);
    ASSERT_FALSE(section.contours[2].closed);
    ASSERT_EQ(section.contours[2].points.size(), 2);
    ASSERT_LT(std::abs(section.contours[2].points.front().x - 5.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 0.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 5.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 9.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, BrepMixedMergedAreaWithInteriorHoleAndDetachedOpenContourStaysMixed)
{
    const PolyhedronBody polyBody = BuildMixedMergedHoleAndAdjacentNonPlanarWithDetachedOpenSectionBody();
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
    ASSERT_EQ(section.polygons[0].HoleCount(), 1);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 13);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 8);
    ASSERT_TRUE(section.contours[1].closed);
    ASSERT_EQ(section.contours[1].points.size(), 4);
    ASSERT_FALSE(section.contours[2].closed);
    ASSERT_EQ(section.contours[2].points.size(), 2);
    ASSERT_LT(std::abs(section.contours[2].points.front().x - 5.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 0.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 5.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 9.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, MixedMergedAreaWithInteriorHoleAndEdgeAttachedOpenContourStaysMixed)
{
    const PolyhedronBody body = BuildMixedMergedHoleAndAdjacentNonPlanarWithEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.polygons[0].HoleCount(), 1);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 13);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 8);
    ASSERT_TRUE(section.contours[1].closed);
    ASSERT_EQ(section.contours[1].points.size(), 4);
    ASSERT_FALSE(section.contours[2].closed);
    ASSERT_EQ(section.contours[2].points.size(), 2);
    ASSERT_LT(std::abs(section.contours[2].points.front().x - 3.5), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 2.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 3.5), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 3.0), 1e-12);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 9.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
}

TEST(Section3dCapabilityTest, BrepMixedMergedAreaWithInteriorHoleAndEdgeAttachedOpenContourStaysMixed)
{
    const PolyhedronBody polyBody = BuildMixedMergedHoleAndAdjacentNonPlanarWithEdgeAttachedOpenSectionBody();
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
    ASSERT_EQ(section.polygons[0].HoleCount(), 1);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 13);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 8);
    ASSERT_TRUE(section.contours[1].closed);
    ASSERT_EQ(section.contours[1].points.size(), 4);
    ASSERT_FALSE(section.contours[2].closed);
    ASSERT_EQ(section.contours[2].points.size(), 2);
    ASSERT_LT(std::abs(section.contours[2].points.front().x - 3.5), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 2.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 3.5), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 3.0), 1e-12);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 9.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, MixedMergedAreaWithInteriorHoleAndVertexAttachedOpenContourStaysMixed)
{
    const PolyhedronBody body = BuildMixedMergedHoleAndAdjacentNonPlanarWithVertexAttachedOpenSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.polygons[0].HoleCount(), 1);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 13);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 8);
    ASSERT_TRUE(section.contours[1].closed);
    ASSERT_EQ(section.contours[1].points.size(), 4);
    ASSERT_FALSE(section.contours[2].closed);
    ASSERT_EQ(section.contours[2].points.size(), 2);
    ASSERT_LT(std::abs(section.contours[2].points.front().x - 4.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 2.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 5.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 2.0), 1e-12);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 9.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, BrepMixedMergedAreaWithInteriorHoleAndVertexAttachedOpenContourStaysMixed)
{
    const PolyhedronBody polyBody = BuildMixedMergedHoleAndAdjacentNonPlanarWithVertexAttachedOpenSectionBody();
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
    ASSERT_EQ(section.polygons[0].HoleCount(), 1);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 13);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 8);
    ASSERT_TRUE(section.contours[1].closed);
    ASSERT_EQ(section.contours[1].points.size(), 4);
    ASSERT_FALSE(section.contours[2].closed);
    ASSERT_EQ(section.contours[2].points.size(), 2);
    ASSERT_LT(std::abs(section.contours[2].points.front().x - 4.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 2.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 5.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 2.0), 1e-12);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 9.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}
