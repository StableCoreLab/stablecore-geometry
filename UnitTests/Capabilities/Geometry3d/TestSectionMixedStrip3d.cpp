#include "Support/SectionFixtures3d.h"

TEST(Section3dCapabilityTest, MixedCoplanarStripWithVertexAndEdgeAttachedOpenContoursBuildsMixedContent)
{
    const PolyhedronBody body = BuildMixedCoplanarStripWithVertexAndEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 6);

    std::size_t closedContours = 0;
    std::size_t openContours = 0;
    bool sawEdgeAttached = false;
    bool sawVertexAttached = false;
    for (const auto& contour : section.contours)
    {
        if (contour.closed)
        {
            ++closedContours;
            ASSERT_EQ(contour.points.size(), 4);
            continue;
        }

        ++openContours;
        ASSERT_EQ(contour.points.size(), 2);
        if (std::abs(contour.points.front().x - 0.25) < 1e-12 && std::abs(contour.points.front().y - 1.0) < 1e-12 &&
            std::abs(contour.points.back().x - 0.25) < 1e-12 && std::abs(contour.points.back().y - 2.0) < 1e-12)
        {
            sawEdgeAttached = true;
            continue;
        }

        if (std::abs(contour.points.front().x - 3.0) < 1e-12 && std::abs(contour.points.front().y - 1.0) < 1e-12 &&
            std::abs(contour.points.back().x - 4.0) < 1e-12 && std::abs(contour.points.back().y - 1.0) < 1e-12)
        {
            sawVertexAttached = true;
            continue;
        }

        ASSERT_TRUE(false);
    }

    ASSERT_EQ(closedContours, 1);
    ASSERT_EQ(openContours, 2);
    ASSERT_TRUE(sawEdgeAttached);
    ASSERT_TRUE(sawVertexAttached);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 3.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, BrepMixedCoplanarStripWithVertexAndEdgeAttachedOpenContoursBuildsMixedContent)
{
    const PolyhedronBody polyBody = BuildMixedCoplanarStripWithVertexAndEdgeAttachedOpenSectionBody();
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

    std::size_t closedContours = 0;
    std::size_t openContours = 0;
    bool sawEdgeAttached = false;
    bool sawVertexAttached = false;
    for (const auto& contour : section.contours)
    {
        if (contour.closed)
        {
            ++closedContours;
            ASSERT_EQ(contour.points.size(), 4);
            continue;
        }

        ++openContours;
        ASSERT_EQ(contour.points.size(), 2);
        if (std::abs(contour.points.front().x - 0.25) < 1e-12 && std::abs(contour.points.front().y - 1.0) < 1e-12 &&
            std::abs(contour.points.back().x - 0.25) < 1e-12 && std::abs(contour.points.back().y - 2.0) < 1e-12)
        {
            sawEdgeAttached = true;
            continue;
        }

        if (std::abs(contour.points.front().x - 3.0) < 1e-12 && std::abs(contour.points.front().y - 1.0) < 1e-12 &&
            std::abs(contour.points.back().x - 4.0) < 1e-12 && std::abs(contour.points.back().y - 1.0) < 1e-12)
        {
            sawVertexAttached = true;
            continue;
        }

        ASSERT_TRUE(false);
    }

    ASSERT_EQ(closedContours, 1);
    ASSERT_EQ(openContours, 2);
    ASSERT_TRUE(sawEdgeAttached);
    ASSERT_TRUE(sawVertexAttached);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 3.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, MixedCoplanarStripWithDetachedVertexAndEdgeAttachedOpenContoursBuildsMixedContent)
{
    const PolyhedronBody body = BuildMixedCoplanarStripWithDetachedVertexAndEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 4);
    ASSERT_EQ(section.segments.size(), 7);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_FALSE(section.contours[2].closed);
    ASSERT_FALSE(section.contours[3].closed);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 2.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[2].points.front().x - 3.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 4.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[3].points.front().x - 5.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.front().y - 0.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.back().x - 5.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 3.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, BrepMixedCoplanarStripWithDetachedVertexAndEdgeAttachedOpenContoursBuildsMixedContent)
{
    const PolyhedronBody polyBody = BuildMixedCoplanarStripWithDetachedVertexAndEdgeAttachedOpenSectionBody();
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
    ASSERT_EQ(section.segments.size(), 7);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_FALSE(section.contours[2].closed);
    ASSERT_FALSE(section.contours[3].closed);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 2.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[2].points.front().x - 3.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 4.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[3].points.front().x - 5.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.front().y - 0.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.back().x - 5.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 3.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}
