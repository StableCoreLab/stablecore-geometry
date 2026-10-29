#include "Support/SectionFixtures3d.h"

// Demonstrates mixed section arbitration now keeps a representative detached
// open contour alongside a closed area component instead of forcing the result
// into area-only output.
TEST(Section3dCapabilityTest, MixedAreaAndOpenContourSectionBuildsMixedContent)
{
    const PolyhedronBody body = BuildMixedAreaAndOpenSectionBody();
    ASSERT_TRUE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 7);

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 2);
    ASSERT_EQ(section.segments.size(), 5);

    std::size_t closedContours = 0;
    std::size_t openContours = 0;
    for (const auto& contour : section.contours)
    {
        if (contour.closed)
        {
            ++closedContours;
            ASSERT_EQ(contour.points.size(), 4);
        } else
        {
            ++openContours;
            ASSERT_EQ(contour.points.size(), 2);
        }
    }

    ASSERT_EQ(closedContours, 1);
    ASSERT_EQ(openContours, 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

// Demonstrates the same representative mixed area/open arbitration also holds
// on the Brep path after Polyhedron->Brep conversion.
TEST(Section3dCapabilityTest, BrepMixedAreaAndOpenContourSectionBuildsMixedContent)
{
    const PolyhedronBody polyBody = BuildMixedAreaAndOpenSectionBody();
    ASSERT_TRUE(polyBody.IsValid());
    ASSERT_EQ(polyBody.FaceCount(), 7);

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

    std::size_t closedContours = 0;
    std::size_t openContours = 0;
    for (const auto& contour : section.contours)
    {
        if (contour.closed)
        {
            ++closedContours;
            ASSERT_EQ(contour.points.size(), 4);
        } else
        {
            ++openContours;
            ASSERT_EQ(contour.points.size(), 2);
        }
    }

    ASSERT_EQ(closedContours, 1);
    ASSERT_EQ(openContours, 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

// Demonstrates adjacency arbitration now keeps a representative open contour
// even when it touches the closed area at a single polygon vertex.
TEST(Section3dCapabilityTest, VertexAttachedOpenContourSectionBuildsMixedContent)
{
    const PolyhedronBody body = BuildVertexAttachedOpenSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 2);
    ASSERT_EQ(section.segments.size(), 5);

    std::size_t closedContours = 0;
    std::size_t openContours = 0;
    for (const auto& contour : section.contours)
    {
        if (contour.closed)
        {
            ++closedContours;
            ASSERT_EQ(contour.points.size(), 4);
        } else
        {
            ++openContours;
            ASSERT_EQ(contour.points.size(), 2);
            ASSERT_LT(std::abs(contour.points.front().x - 1.0), 1e-12);
            ASSERT_LT(std::abs(contour.points.front().y - 1.0), 1e-12);
            ASSERT_LT(std::abs(contour.points.back().x - 1.0), 1e-12);
            ASSERT_LT(std::abs(contour.points.back().y - 2.0), 1e-12);
        }
    }

    ASSERT_EQ(closedContours, 1);
    ASSERT_EQ(openContours, 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

// Demonstrates the same vertex-attached mixed area/open arbitration also
// holds on the Brep path after Polyhedron->Brep conversion.
TEST(Section3dCapabilityTest, BrepVertexAttachedOpenContourSectionBuildsMixedContent)
{
    const PolyhedronBody polyBody = BuildVertexAttachedOpenSectionBody();
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

    std::size_t closedContours = 0;
    std::size_t openContours = 0;
    for (const auto& contour : section.contours)
    {
        if (contour.closed)
        {
            ++closedContours;
        } else
        {
            ++openContours;
            ASSERT_EQ(contour.points.size(), 2);
            ASSERT_LT(std::abs(contour.points.front().x - 1.0), 1e-12);
            ASSERT_LT(std::abs(contour.points.front().y - 1.0), 1e-12);
            ASSERT_LT(std::abs(contour.points.back().x - 1.0), 1e-12);
            ASSERT_LT(std::abs(contour.points.back().y - 2.0), 1e-12);
        }
    }

    ASSERT_EQ(closedContours, 1);
    ASSERT_EQ(openContours, 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

// Demonstrates adjacency arbitration also keeps a representative open contour
// when it touches the closed area at the midpoint of a polygon edge.
TEST(Section3dCapabilityTest, EdgeAttachedOpenContourSectionBuildsMixedContent)
{
    const PolyhedronBody body = BuildEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 2);
    ASSERT_EQ(section.segments.size(), 6);

    std::size_t closedContours = 0;
    std::size_t openContours = 0;
    for (const auto& contour : section.contours)
    {
        if (contour.closed)
        {
            ++closedContours;
            ASSERT_EQ(contour.points.size(), 5);
        } else
        {
            ++openContours;
            ASSERT_EQ(contour.points.size(), 2);
            ASSERT_LT(std::abs(contour.points.front().x - 0.5), 1e-12);
            ASSERT_LT(std::abs(contour.points.front().y - 1.0), 1e-12);
            ASSERT_LT(std::abs(contour.points.back().x - 0.5), 1e-12);
            ASSERT_LT(std::abs(contour.points.back().y - 2.0), 1e-12);
        }
    }

    ASSERT_EQ(closedContours, 1);
    ASSERT_EQ(openContours, 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, BrepEdgeAttachedOpenContourSectionBuildsMixedContent)
{
    const PolyhedronBody polyBody = BuildEdgeAttachedOpenSectionBody();
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
    ASSERT_EQ(section.segments.size(), 6);

    std::size_t closedContours = 0;
    std::size_t openContours = 0;
    for (const auto& contour : section.contours)
    {
        if (contour.closed)
        {
            ++closedContours;
            ASSERT_EQ(contour.points.size(), 5);
        } else
        {
            ++openContours;
            ASSERT_EQ(contour.points.size(), 2);
            ASSERT_LT(std::abs(contour.points.front().x - 0.5), 1e-12);
            ASSERT_LT(std::abs(contour.points.front().y - 1.0), 1e-12);
            ASSERT_LT(std::abs(contour.points.back().x - 0.5), 1e-12);
            ASSERT_LT(std::abs(contour.points.back().y - 2.0), 1e-12);
        }
    }

    ASSERT_EQ(closedContours, 1);
    ASSERT_EQ(openContours, 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}
