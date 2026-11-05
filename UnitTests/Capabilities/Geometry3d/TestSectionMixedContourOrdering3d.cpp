#include "Support/SectionFixtures3d.h"

TEST(Section3dCapabilityTest, DetachedVertexAndEdgeAttachedOpenContoursBuildStableMixedContent)
{
    const PolyhedronBody body = BuildDetachedVertexAndEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 4);
    ASSERT_EQ(section.segments.size(), 8);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_FALSE(section.contours[2].closed);
    ASSERT_FALSE(section.contours[3].closed);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 2.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[2].points.front().x - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 2.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[3].points.front().x - 3.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.front().y - 0.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.back().x - 3.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, BrepDetachedVertexAndEdgeAttachedOpenContoursBuildStableMixedContent)
{
    const PolyhedronBody polyBody = BuildDetachedVertexAndEdgeAttachedOpenSectionBody();
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
    ASSERT_EQ(section.contours.size(), 4);
    ASSERT_EQ(section.segments.size(), 8);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_FALSE(section.contours[2].closed);
    ASSERT_FALSE(section.contours[3].closed);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 2.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[2].points.front().x - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 2.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[3].points.front().x - 3.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.front().y - 0.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.back().x - 3.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

// Demonstrates mixed open-curve/area arbitration now sorts boundary-attached
// open contours ahead of detached ones, even when the detached contour is
// lexicographically earlier in world coordinates.
TEST(Section3dCapabilityTest, BoundaryAttachedOpenContourSortsAheadOfDetachedContour)
{
    const PolyhedronBody body = BuildDetachedLeftAndEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 6);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_FALSE(section.contours[2].closed);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 2.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[2].points.front().x + 2.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 0.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x + 2.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, BrepBoundaryAttachedOpenContourSortsAheadOfDetachedContour)
{
    const PolyhedronBody polyBody = BuildDetachedLeftAndEdgeAttachedOpenSectionBody();
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
    ASSERT_EQ(section.segments.size(), 6);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_FALSE(section.contours[2].closed);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 2.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[2].points.front().x + 2.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 0.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x + 2.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, DetachedPlusTwoEdgeAttachedContoursRemainSeparateFromMergedArea)
{
    const PolyhedronBody body = BuildDetachedAndTwoEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 4);
    ASSERT_EQ(section.segments.size(), 8);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_FALSE(section.contours[2].closed);
    ASSERT_FALSE(section.contours[3].closed);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 2.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[2].points.front().x - 0.75), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y + 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 0.75), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 0.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[3].points.front().x - 3.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.front().y - 0.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.back().x - 3.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, BrepDetachedPlusTwoEdgeAttachedContoursRemainSeparateFromMergedArea)
{
    const PolyhedronBody polyBody = BuildDetachedAndTwoEdgeAttachedOpenSectionBody();
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
    ASSERT_EQ(section.contours.size(), 4);
    ASSERT_EQ(section.segments.size(), 8);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_FALSE(section.contours[2].closed);
    ASSERT_FALSE(section.contours[3].closed);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 2.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[2].points.front().x - 0.75), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y + 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 0.75), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 0.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[3].points.front().x - 3.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.front().y - 0.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.back().x - 3.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}
