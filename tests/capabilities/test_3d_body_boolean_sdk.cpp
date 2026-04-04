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
[[nodiscard]] PolyhedronBody BuildTranslatedUnitCubeBody(double dx, double dy, double dz)
{
    return PolyhedronBody(
        {
            PolyhedronFace3d(
                Plane::FromPointAndNormal(Point3d{dx + 0.0, dy + 0.0, dz + 0.0}, Vector3d{0.0, 0.0, -1.0}),
                PolyhedronLoop3d(
                    {
                        Point3d{dx + 0.0, dy + 0.0, dz + 0.0},
                        Point3d{dx + 0.0, dy + 1.0, dz + 0.0},
                        Point3d{dx + 1.0, dy + 1.0, dz + 0.0},
                        Point3d{dx + 1.0, dy + 0.0, dz + 0.0},
                    })),
            PolyhedronFace3d(
                Plane::FromPointAndNormal(Point3d{dx + 0.0, dy + 0.0, dz + 1.0}, Vector3d{0.0, 0.0, 1.0}),
                PolyhedronLoop3d(
                    {
                        Point3d{dx + 0.0, dy + 0.0, dz + 1.0},
                        Point3d{dx + 1.0, dy + 0.0, dz + 1.0},
                        Point3d{dx + 1.0, dy + 1.0, dz + 1.0},
                        Point3d{dx + 0.0, dy + 1.0, dz + 1.0},
                    })),
            PolyhedronFace3d(
                Plane::FromPointAndNormal(Point3d{dx + 0.0, dy + 0.0, dz + 0.0}, Vector3d{0.0, -1.0, 0.0}),
                PolyhedronLoop3d(
                    {
                        Point3d{dx + 0.0, dy + 0.0, dz + 0.0},
                        Point3d{dx + 1.0, dy + 0.0, dz + 0.0},
                        Point3d{dx + 1.0, dy + 0.0, dz + 1.0},
                        Point3d{dx + 0.0, dy + 0.0, dz + 1.0},
                    })),
            PolyhedronFace3d(
                Plane::FromPointAndNormal(Point3d{dx + 1.0, dy + 0.0, dz + 0.0}, Vector3d{1.0, 0.0, 0.0}),
                PolyhedronLoop3d(
                    {
                        Point3d{dx + 1.0, dy + 0.0, dz + 0.0},
                        Point3d{dx + 1.0, dy + 1.0, dz + 0.0},
                        Point3d{dx + 1.0, dy + 1.0, dz + 1.0},
                        Point3d{dx + 1.0, dy + 0.0, dz + 1.0},
                    })),
            PolyhedronFace3d(
                Plane::FromPointAndNormal(Point3d{dx + 0.0, dy + 1.0, dz + 0.0}, Vector3d{0.0, 1.0, 0.0}),
                PolyhedronLoop3d(
                    {
                        Point3d{dx + 0.0, dy + 1.0, dz + 0.0},
                        Point3d{dx + 0.0, dy + 1.0, dz + 1.0},
                        Point3d{dx + 1.0, dy + 1.0, dz + 1.0},
                        Point3d{dx + 1.0, dy + 1.0, dz + 0.0},
                    })),
            PolyhedronFace3d(
                Plane::FromPointAndNormal(Point3d{dx + 0.0, dy + 0.0, dz + 0.0}, Vector3d{-1.0, 0.0, 0.0}),
                PolyhedronLoop3d(
                    {
                        Point3d{dx + 0.0, dy + 0.0, dz + 0.0},
                        Point3d{dx + 0.0, dy + 0.0, dz + 1.0},
                        Point3d{dx + 0.0, dy + 1.0, dz + 1.0},
                        Point3d{dx + 0.0, dy + 1.0, dz + 0.0},
                    })),
        });
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
