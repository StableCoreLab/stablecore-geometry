#include "Support/BodyBooleanFixtures3d.h"

TEST(BodyBoolean3dCapabilityTest, IdenticalBrepIntersectionReturnsClosedBody)
{
    const auto converted = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    ASSERT_TRUE(converted.success);

    const auto result = IntersectBodies(converted.body, converted.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_EQ(result.body.ShellCount(), 1U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
}

TEST(BodyBoolean3dCapabilityTest, IdenticalBrepDifferenceReturnsDeterministicEmptyResult)
{
    const auto converted = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    ASSERT_TRUE(converted.success);

    const auto result = DifferenceBodies(converted.body, converted.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_TRUE(result.producedEmptyResult);
    EXPECT_EQ(result.body.FaceCount(), 0U);
    EXPECT_TRUE(result.bodies.empty());
}

TEST(BodyBoolean3dCapabilityTest, ContainedBrepIntersectionReturnsInnerBody)
{
    const auto first = ConvertToBrepBody(BuildAxisAlignedBoxBody(-0.5, -0.5, -0.5, 1.5, 1.5, 1.5));
    const auto second = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = IntersectBodies(first.body, second.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_TRUE(result.bodies.empty());
    EXPECT_FALSE(result.producedEmptyResult);
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_EQ(result.body.ShellCount(), 1U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    const auto bounds = result.body.Bounds();
    EXPECT_NEAR(bounds.MinPoint().x, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().y, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().z, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().x, 1.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().y, 1.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().z, 1.0, 1e-12);
}

TEST(BodyBoolean3dCapabilityTest, ContainedBrepUnionReturnsOuterBody)
{
    const auto first = ConvertToBrepBody(BuildAxisAlignedBoxBody(-0.5, -0.5, -0.5, 1.5, 1.5, 1.5));
    const auto second = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = UnionBodies(first.body, second.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_TRUE(result.bodies.empty());
    EXPECT_FALSE(result.producedEmptyResult);
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_EQ(result.body.ShellCount(), 1U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    const auto bounds = result.body.Bounds();
    EXPECT_NEAR(bounds.MinPoint().x, -0.5, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().y, -0.5, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().z, -0.5, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().x, 1.5, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().y, 1.5, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().z, 1.5, 1e-12);
}

TEST(BodyBoolean3dCapabilityTest, DisjointBrepIntersectionReturnsDeterministicEmptyResult)
{
    const auto first = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    const auto second = ConvertToBrepBody(BuildAxisAlignedBoxBody(3.0, 0.0, 0.0, 4.0, 1.0, 1.0));
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = IntersectBodies(first.body, second.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_TRUE(result.producedEmptyResult);
    EXPECT_EQ(result.body.FaceCount(), 0U);
    EXPECT_TRUE(result.bodies.empty());
}

TEST(BodyBoolean3dCapabilityTest, DisjointBrepUnionNormalizesBodyOrder)
{
    const auto first = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    const auto second = ConvertToBrepBody(BuildTranslatedUnitCubeBody(3.0, 0.0, 0.0));
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = UnionBodies(second.body, first.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_EQ(result.bodies.size(), 2U);
    EXPECT_EQ(result.bodies[0].FaceCount(), 6U);
    EXPECT_EQ(result.bodies[1].FaceCount(), 6U);
    const auto firstBounds = result.bodies[0].Bounds();
    const auto secondBounds = result.bodies[1].Bounds();
    EXPECT_NEAR(firstBounds.MinPoint().x, 0.0, 1e-12);
    EXPECT_NEAR(firstBounds.MaxPoint().x, 1.0, 1e-12);
    EXPECT_NEAR(secondBounds.MinPoint().x, 3.0, 1e-12);
    EXPECT_NEAR(secondBounds.MaxPoint().x, 4.0, 1e-12);
}

TEST(BodyBoolean3dCapabilityTest, OverlappingBrepUnionReturnsSingleAxisAlignedBox)
{
    const auto first = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    const auto second = ConvertToBrepBody(BuildAxisAlignedBoxBody(0.5, 0.0, 0.0, 1.5, 1.0, 1.0));
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

TEST(BodyBoolean3dCapabilityTest, TouchingBrepUnionReturnsSingleAxisAlignedBox)
{
    const auto first = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    const auto second = ConvertToBrepBody(BuildAxisAlignedBoxBody(1.0, 0.0, 0.0, 2.0, 1.0, 1.0));
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = UnionBodies(first.body, second.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_TRUE(result.bodies.empty());
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_EQ(result.body.ShellCount(), 1U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    const auto bounds = result.body.Bounds();
    EXPECT_NEAR(bounds.MinPoint().x, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().x, 2.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().y, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().y, 1.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().z, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().z, 1.0, 1e-12);
}

TEST(BodyBoolean3dCapabilityTest, TouchingBrepDifferenceReturnsOriginalBody)
{
    const auto first = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    const auto second = ConvertToBrepBody(BuildAxisAlignedBoxBody(1.0, 0.0, 0.0, 2.0, 1.0, 1.0));
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = DifferenceBodies(first.body, second.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_TRUE(result.bodies.empty());
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_EQ(result.body.ShellCount(), 1U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    const auto bounds = result.body.Bounds();
    EXPECT_NEAR(bounds.MinPoint().x, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().x, 1.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().y, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().y, 1.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().z, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().z, 1.0, 1e-12);
}

TEST(BodyBoolean3dCapabilityTest, TouchingBrepIntersectionReturnsDeterministicEmptyResult)
{
    const auto first = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    const auto second = ConvertToBrepBody(BuildAxisAlignedBoxBody(1.0, 0.0, 0.0, 2.0, 1.0, 1.0));
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = IntersectBodies(first.body, second.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_TRUE(result.producedEmptyResult);
    EXPECT_EQ(result.body.FaceCount(), 0U);
    EXPECT_TRUE(result.bodies.empty());
}

TEST(BodyBoolean3dCapabilityTest, EdgeTouchingBrepIntersectionReturnsDeterministicEmptyResult)
{
    const auto first = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    const auto second = ConvertToBrepBody(BuildAxisAlignedBoxBody(1.0, 1.0, 0.0, 2.0, 2.0, 1.0));
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = IntersectBodies(first.body, second.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_TRUE(result.producedEmptyResult);
    EXPECT_EQ(result.body.FaceCount(), 0U);
    EXPECT_TRUE(result.bodies.empty());
}

TEST(BodyBoolean3dCapabilityTest, VertexTouchingBrepIntersectionReturnsDeterministicEmptyResult)
{
    const auto first = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    const auto second = ConvertToBrepBody(BuildAxisAlignedBoxBody(1.0, 1.0, 1.0, 2.0, 2.0, 2.0));
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = IntersectBodies(first.body, second.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_TRUE(result.producedEmptyResult);
    EXPECT_EQ(result.body.FaceCount(), 0U);
    EXPECT_TRUE(result.bodies.empty());
}

TEST(BodyBoolean3dCapabilityTest, EdgeTouchingBrepUnionReturnsDeterministicOrderedMultiBodyResult)
{
    const auto first = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    const auto second = ConvertToBrepBody(BuildAxisAlignedBoxBody(1.0, 1.0, 0.0, 2.0, 2.0, 1.0));
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = UnionBodies(second.body, first.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_EQ(result.bodies.size(), 2U);
    EXPECT_EQ(result.body.FaceCount(), 0U);
    EXPECT_FALSE(result.producedEmptyResult);
    const auto firstBounds = result.bodies[0].Bounds();
    const auto secondBounds = result.bodies[1].Bounds();
    EXPECT_NEAR(firstBounds.MinPoint().x, 0.0, 1e-12);
    EXPECT_NEAR(firstBounds.MaxPoint().x, 1.0, 1e-12);
    EXPECT_NEAR(secondBounds.MinPoint().x, 1.0, 1e-12);
    EXPECT_NEAR(secondBounds.MaxPoint().x, 2.0, 1e-12);
}

TEST(BodyBoolean3dCapabilityTest, VertexTouchingBrepDifferenceReturnsOriginalBody)
{
    const auto first = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    const auto second = ConvertToBrepBody(BuildAxisAlignedBoxBody(1.0, 1.0, 1.0, 2.0, 2.0, 2.0));
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = DifferenceBodies(first.body, second.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_TRUE(result.bodies.empty());
    EXPECT_FALSE(result.producedEmptyResult);
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_EQ(result.body.ShellCount(), 1U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    const auto bounds = result.body.Bounds();
    EXPECT_NEAR(bounds.MinPoint().x, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().x, 1.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().y, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().y, 1.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().z, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().z, 1.0, 1e-12);
}

TEST(BodyBoolean3dCapabilityTest, ContainedBrepDifferenceReturnsDeterministicEmptyResult)
{
    const auto first = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    const auto second = ConvertToBrepBody(BuildAxisAlignedBoxBody(-0.5, -0.5, -0.5, 1.5, 1.5, 1.5));
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = DifferenceBodies(first.body, second.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_TRUE(result.producedEmptyResult);
    EXPECT_EQ(result.body.FaceCount(), 0U);
    EXPECT_TRUE(result.bodies.empty());
}

TEST(BodyBoolean3dCapabilityTest, SupportedResultsPreserveClosedValidTopology)
{
    const PolyhedronBody first = BuildAxisAlignedBoxBody(-0.5, -0.5, -0.5, 1.5, 1.5, 1.5);
    const PolyhedronBody second = Geometry::Test::BuildUnitCubeBody();

    const auto intersection = IntersectBodies(first, second);
    ASSERT_EQ(intersection.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(intersection.IsSuccess());
    ASSERT_TRUE(intersection.body.IsValid());
    ASSERT_EQ(intersection.body.ShellCount(), 1U);
    EXPECT_TRUE(intersection.body.ShellAt(0).IsClosed());

    const auto united = UnionBodies(first, second);
    ASSERT_EQ(united.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(united.IsSuccess());
    ASSERT_TRUE(united.body.IsValid());
    ASSERT_EQ(united.body.ShellCount(), 1U);
    EXPECT_TRUE(united.body.ShellAt(0).IsClosed());
}
