#include <gtest/gtest.h>

#include <limits>

#include "Geometry.h"
#include "Support/Fixtures3d.h"

TEST(ValidationToleranceTest, RejectsInvalidScalarToleranceAcrossGeometryTypes)
{
    const Geometry::SCPolyline2d ring(
        {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}},
        Geometry::SCPolylineClosure::Closed);
    const Geometry::SCPolygon2d polygon(ring);
    const Geometry::TriangleMesh mesh(
        {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}},
        {Geometry::TriangleMesh::TriangleIndices{0, 1, 2}});
    const Geometry::PolyhedronBody body = Geometry::Test::BuildUnitCubeBody();
    Geometry::SCPolyhedronSection3d section{};
    section.success = true;
    section.uAxis = {1.0, 0.0, 0.0};
    section.vAxis = {0.0, 1.0, 0.0};
    ASSERT_TRUE(Geometry::Validate(ring).valid);
    ASSERT_TRUE(Geometry::Validate(polygon).valid);
    ASSERT_TRUE(Geometry::Validate(mesh).valid);
    ASSERT_TRUE(Geometry::Validate(body).valid);
    ASSERT_TRUE(Geometry::Validate(section).valid);

    for (double eps : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN()})
    {
        const auto ringResult = Geometry::Validate(ring, eps);
        const auto polygonResult = Geometry::Validate(polygon, eps);
        const auto meshResult = Geometry::Validate(mesh, eps);
        const auto bodyResult = Geometry::Validate(body, eps);
        const auto sectionResult = Geometry::Validate(section, eps);
        EXPECT_FALSE(ringResult.valid);
        EXPECT_EQ(ringResult.issue, Geometry::SCPolygonValidationIssue2d::InvalidTolerance);
        EXPECT_FALSE(polygonResult.valid);
        EXPECT_EQ(polygonResult.issue, Geometry::SCPolygonValidationIssue2d::InvalidTolerance);
        EXPECT_FALSE(meshResult.valid);
        EXPECT_EQ(meshResult.issue, Geometry::SCMeshValidationIssue3d::InvalidTolerance);
        EXPECT_FALSE(bodyResult.valid);
        EXPECT_EQ(bodyResult.issue, Geometry::SCPolyhedronValidationIssue3d::InvalidTolerance);
        EXPECT_FALSE(sectionResult.valid);
        EXPECT_EQ(sectionResult.issue, Geometry::SCSectionValidationIssue3d::InvalidTolerance);
    }
}

TEST(ValidationToleranceTest, RejectsInvalidBrepTolerance)
{
    const auto converted = Geometry::ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    ASSERT_TRUE(converted.success);
    ASSERT_TRUE(Geometry::Validate(converted.body).valid);

    for (double eps : {-1.0, std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN()})
    {
        Geometry::SCGeometryTolerance3d tolerance{};
        tolerance.distanceEpsilon = eps;
        const auto result = Geometry::Validate(converted.body, tolerance);
        EXPECT_FALSE(result.valid);
        EXPECT_EQ(result.issue, Geometry::SCBrepValidationIssue3d::InvalidTolerance);
    }
}
