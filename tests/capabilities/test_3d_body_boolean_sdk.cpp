#include <gtest/gtest.h>

#include "sdk/GeometryBodyBoolean.h"
#include "sdk/GeometryBrepConversion.h"
#include "support/Fixtures3d.h"

using geometry::sdk::BodyBooleanIssue3d;
using geometry::sdk::BodyBooleanOptions3d;
using geometry::sdk::DifferenceBodies;
using geometry::sdk::IntersectBodies;
using geometry::sdk::PolyhedronBody;
using geometry::sdk::Point3d;
using geometry::sdk::PolyhedronFace3d;
using geometry::sdk::PolyhedronLoop3d;
using geometry::sdk::UnionBodies;
using geometry::sdk::Plane;
using geometry::sdk::Vector3d;

namespace
{
[[nodiscard]] PolyhedronBody BuildAxisAlignedBoxBody(
    double minX,
    double minY,
    double minZ,
    double maxX,
    double maxY,
    double maxZ)
{
    return PolyhedronBody(
        {
            PolyhedronFace3d(
                Plane::FromPointAndNormal(Point3d{minX, minY, minZ}, Vector3d{0.0, 0.0, -1.0}),
                PolyhedronLoop3d(
                    {
                        Point3d{minX, minY, minZ},
                        Point3d{minX, maxY, minZ},
                        Point3d{maxX, maxY, minZ},
                        Point3d{maxX, minY, minZ},
                    })),
            PolyhedronFace3d(
                Plane::FromPointAndNormal(Point3d{minX, minY, maxZ}, Vector3d{0.0, 0.0, 1.0}),
                PolyhedronLoop3d(
                    {
                        Point3d{minX, minY, maxZ},
                        Point3d{maxX, minY, maxZ},
                        Point3d{maxX, maxY, maxZ},
                        Point3d{minX, maxY, maxZ},
                    })),
            PolyhedronFace3d(
                Plane::FromPointAndNormal(Point3d{minX, minY, minZ}, Vector3d{0.0, -1.0, 0.0}),
                PolyhedronLoop3d(
                    {
                        Point3d{minX, minY, minZ},
                        Point3d{maxX, minY, minZ},
                        Point3d{maxX, minY, maxZ},
                        Point3d{minX, minY, maxZ},
                    })),
            PolyhedronFace3d(
                Plane::FromPointAndNormal(Point3d{maxX, minY, minZ}, Vector3d{1.0, 0.0, 0.0}),
                PolyhedronLoop3d(
                    {
                        Point3d{maxX, minY, minZ},
                        Point3d{maxX, maxY, minZ},
                        Point3d{maxX, maxY, maxZ},
                        Point3d{maxX, minY, maxZ},
                    })),
            PolyhedronFace3d(
                Plane::FromPointAndNormal(Point3d{minX, maxY, minZ}, Vector3d{0.0, 1.0, 0.0}),
                PolyhedronLoop3d(
                    {
                        Point3d{minX, maxY, minZ},
                        Point3d{minX, maxY, maxZ},
                        Point3d{maxX, maxY, maxZ},
                        Point3d{maxX, maxY, minZ},
                    })),
            PolyhedronFace3d(
                Plane::FromPointAndNormal(Point3d{minX, minY, minZ}, Vector3d{-1.0, 0.0, 0.0}),
                PolyhedronLoop3d(
                    {
                        Point3d{minX, minY, minZ},
                        Point3d{minX, minY, maxZ},
                        Point3d{minX, maxY, maxZ},
                        Point3d{minX, maxY, minZ},
                    })),
        });
}

[[nodiscard]] PolyhedronBody BuildTranslatedUnitCubeBody(double dx, double dy, double dz)
{
    return BuildAxisAlignedBoxBody(dx + 0.0, dy + 0.0, dz + 0.0, dx + 1.0, dy + 1.0, dz + 1.0);
}
} // namespace

TEST(BodyBooleanSdkCapabilityTest, InvalidInputContractIsStableForEmptyBodies)
{
    const PolyhedronBody first;
    const PolyhedronBody second;
    const BodyBooleanOptions3d options{};

    const auto intersection = IntersectBodies(first, second, options);
    const auto united = UnionBodies(first, second, options);
    const auto difference = DifferenceBodies(first, second, options);

    EXPECT_EQ(intersection.issue, BodyBooleanIssue3d::InvalidInput);
    EXPECT_EQ(united.issue, BodyBooleanIssue3d::InvalidInput);
    EXPECT_EQ(difference.issue, BodyBooleanIssue3d::InvalidInput);
    EXPECT_FALSE(intersection.IsSuccess());
    EXPECT_FALSE(united.IsSuccess());
    EXPECT_FALSE(difference.IsSuccess());
}

TEST(BodyBooleanSdkCapabilityTest, IdenticalPolyhedronIntersectionReturnsClosedBody)
{
    const PolyhedronBody first = geometry::test::BuildUnitCubeBody();
    const PolyhedronBody second = geometry::test::BuildUnitCubeBody();

    const auto result = IntersectBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_EQ(result.body.VertexCount(), 8U);
    EXPECT_EQ(result.body.EdgeCount(), 12U);
    EXPECT_EQ(result.body.ShellCount(), 1U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
}

TEST(BodyBooleanSdkCapabilityTest, IdenticalPolyhedronUnionReturnsSingleClosedBody)
{
    const PolyhedronBody first = geometry::test::BuildUnitCubeBody();
    const PolyhedronBody second = geometry::test::BuildUnitCubeBody();

    const auto result = UnionBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    EXPECT_TRUE(result.bodies.empty());
}

TEST(BodyBooleanSdkCapabilityTest, DisjointPolyhedronUnionReturnsTwoBodies)
{
    const PolyhedronBody first = geometry::test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildTranslatedUnitCubeBody(3.0, 0.0, 0.0);

    const auto result = UnionBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_EQ(result.bodies.size(), 2U);
    EXPECT_EQ(result.body.FaceCount(), 0U);
    EXPECT_EQ(result.bodies[0].FaceCount(), 6U);
    EXPECT_EQ(result.bodies[1].FaceCount(), 6U);
}

TEST(BodyBooleanSdkCapabilityTest, DisjointPolyhedronDifferenceReturnsOriginalBody)
{
    const PolyhedronBody first = geometry::test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildTranslatedUnitCubeBody(3.0, 0.0, 0.0);

    const auto result = DifferenceBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_EQ(result.body.VertexCount(), 8U);
    EXPECT_EQ(result.body.EdgeCount(), 12U);
}

TEST(BodyBooleanSdkCapabilityTest, OverlappingPolyhedronIntersectionReturnsSingleOverlapBox)
{
    const PolyhedronBody first = geometry::test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildAxisAlignedBoxBody(0.5, 0.0, 0.0, 1.5, 1.0, 1.0);

    const auto result = IntersectBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_TRUE(result.bodies.empty());
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    const auto bounds = result.body.Bounds();
    EXPECT_NEAR(bounds.MinPoint().x, 0.5, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().y, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().z, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().x, 1.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().y, 1.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().z, 1.0, 1e-12);
}

TEST(BodyBooleanSdkCapabilityTest, OverlappingPolyhedronUnionReturnsSingleAxisAlignedBox)
{
    const PolyhedronBody first = geometry::test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildAxisAlignedBoxBody(0.5, 0.0, 0.0, 1.5, 1.0, 1.0);

    const auto result = UnionBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_TRUE(result.bodies.empty());
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    const auto bounds = result.body.Bounds();
    EXPECT_NEAR(bounds.MinPoint().x, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().y, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().z, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().x, 1.5, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().y, 1.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().z, 1.0, 1e-12);
}

TEST(BodyBooleanSdkCapabilityTest, OverlappingPolyhedronDifferenceReturnsRemainingSingleBox)
{
    const PolyhedronBody first = geometry::test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildAxisAlignedBoxBody(0.5, 0.0, 0.0, 1.0, 1.0, 1.0);

    const auto result = DifferenceBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_TRUE(result.bodies.empty());
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    const auto bounds = result.body.Bounds();
    EXPECT_NEAR(bounds.MinPoint().x, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().y, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().z, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().x, 0.5, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().y, 1.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().z, 1.0, 1e-12);
}

TEST(BodyBooleanSdkCapabilityTest, NonBoxOverlapUnionAndDifferenceRemainUnsupported)
{
    const PolyhedronBody first = geometry::test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildAxisAlignedBoxBody(0.5, 0.5, 0.0, 1.5, 1.5, 1.0);

    const auto united = UnionBodies(first, second);
    const auto difference = DifferenceBodies(first, second);

    EXPECT_EQ(united.issue, BodyBooleanIssue3d::UnsupportedOperation);
    EXPECT_EQ(difference.issue, BodyBooleanIssue3d::UnsupportedOperation);
    EXPECT_FALSE(united.IsSuccess());
    EXPECT_FALSE(difference.IsSuccess());
}

TEST(BodyBooleanSdkCapabilityTest, IdenticalBrepIntersectionReturnsClosedBody)
{
    const auto converted = geometry::sdk::ConvertToBrepBody(geometry::test::BuildUnitCubeBody());
    ASSERT_TRUE(converted.success);

    const auto result = IntersectBodies(converted.body, converted.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_EQ(result.body.ShellCount(), 1U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
}

TEST(BodyBooleanSdkCapabilityTest, OverlappingBrepUnionReturnsSingleAxisAlignedBox)
{
    const auto first = geometry::sdk::ConvertToBrepBody(geometry::test::BuildUnitCubeBody());
    const auto second = geometry::sdk::ConvertToBrepBody(BuildAxisAlignedBoxBody(0.5, 0.0, 0.0, 1.5, 1.0, 1.0));
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = UnionBodies(first.body, second.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_TRUE(result.bodies.empty());
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    const auto bounds = result.body.Bounds();
    EXPECT_NEAR(bounds.MinPoint().x, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().x, 1.5, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().y, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().y, 1.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().z, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().z, 1.0, 1e-12);
}
