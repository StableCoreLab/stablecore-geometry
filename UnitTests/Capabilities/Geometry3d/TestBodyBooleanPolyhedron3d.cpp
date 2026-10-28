#include "Support/BodyBooleanFixtures3d.h"

TEST(BodyBoolean3dCapabilityTest, IdenticalPolyhedronIntersectionReturnsClosedBody)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = Geometry::Test::BuildUnitCubeBody();

    const auto result = IntersectBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_EQ(result.body.VertexCount(), 8U);
    EXPECT_EQ(result.body.EdgeCount(), 12U);
    EXPECT_EQ(result.body.ShellCount(), 1U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
}

TEST(BodyBoolean3dCapabilityTest, IdenticalPolyhedronUnionReturnsSingleClosedBody)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = Geometry::Test::BuildUnitCubeBody();

    const auto result = UnionBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    EXPECT_TRUE(result.bodies.empty());
}

TEST(BodyBoolean3dCapabilityTest, IdenticalPolyhedronDifferenceReturnsDeterministicEmptyResult)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = Geometry::Test::BuildUnitCubeBody();

    const auto result = DifferenceBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_TRUE(result.producedEmptyResult);
    EXPECT_EQ(result.body.FaceCount(), 0U);
    EXPECT_TRUE(result.bodies.empty());
}

TEST(BodyBoolean3dCapabilityTest, DisjointPolyhedronUnionReturnsTwoBodies)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildTranslatedUnitCubeBody(3.0, 0.0, 0.0);

    const auto result = UnionBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_EQ(result.bodies.size(), 2U);
    EXPECT_EQ(result.body.FaceCount(), 0U);
    EXPECT_EQ(result.bodies[0].FaceCount(), 6U);
    EXPECT_EQ(result.bodies[1].FaceCount(), 6U);
}

TEST(BodyBoolean3dCapabilityTest, DisjointPolyhedronUnionNormalizesBodyOrder)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildTranslatedUnitCubeBody(3.0, 0.0, 0.0);

    const auto result = UnionBodies(second, first);

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

TEST(BodyBoolean3dCapabilityTest, DisjointPolyhedronDifferenceReturnsOriginalBody)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildTranslatedUnitCubeBody(3.0, 0.0, 0.0);

    const auto result = DifferenceBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_EQ(result.body.VertexCount(), 8U);
    EXPECT_EQ(result.body.EdgeCount(), 12U);
}

TEST(BodyBoolean3dCapabilityTest, OverlappingPolyhedronIntersectionReturnsSingleOverlapBox)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
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

TEST(BodyBoolean3dCapabilityTest, OverlappingPolyhedronUnionReturnsSingleAxisAlignedBox)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
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

TEST(BodyBoolean3dCapabilityTest, TouchingPolyhedronUnionReturnsSingleAxisAlignedBox)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildAxisAlignedBoxBody(1.0, 0.0, 0.0, 2.0, 1.0, 1.0);

    const auto result = UnionBodies(first, second);

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

TEST(BodyBoolean3dCapabilityTest, TouchingPolyhedronIntersectionReturnsDeterministicEmptyResult)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildAxisAlignedBoxBody(1.0, 0.0, 0.0, 2.0, 1.0, 1.0);

    const auto intersection = IntersectBodies(first, second);

    EXPECT_EQ(intersection.issue, BodyBooleanIssue3d::None);
    EXPECT_TRUE(intersection.IsSuccess());
    EXPECT_TRUE(intersection.producedEmptyResult);
    EXPECT_EQ(intersection.body.FaceCount(), 0U);
    EXPECT_TRUE(intersection.bodies.empty());
}

TEST(BodyBoolean3dCapabilityTest, ContainedPolyhedronIntersectionReturnsInnerBody)
{
    const PolyhedronBody first = BuildAxisAlignedBoxBody(-0.5, -0.5, -0.5, 1.5, 1.5, 1.5);
    const PolyhedronBody second = Geometry::Test::BuildUnitCubeBody();

    const auto result = IntersectBodies(first, second);

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

TEST(BodyBoolean3dCapabilityTest, ContainedPolyhedronUnionReturnsOuterBody)
{
    const PolyhedronBody first = BuildAxisAlignedBoxBody(-0.5, -0.5, -0.5, 1.5, 1.5, 1.5);
    const PolyhedronBody second = Geometry::Test::BuildUnitCubeBody();

    const auto result = UnionBodies(first, second);

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

TEST(BodyBoolean3dCapabilityTest, DisjointPolyhedronIntersectionReturnsDeterministicEmptyResult)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildAxisAlignedBoxBody(3.0, 0.0, 0.0, 4.0, 1.0, 1.0);

    const auto result = IntersectBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_TRUE(result.producedEmptyResult);
    EXPECT_EQ(result.body.FaceCount(), 0U);
    EXPECT_TRUE(result.bodies.empty());
}

TEST(BodyBoolean3dCapabilityTest, EdgeTouchingPolyhedronIntersectionReturnsDeterministicEmptyResult)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildAxisAlignedBoxBody(1.0, 1.0, 0.0, 2.0, 2.0, 1.0);

    const auto intersection = IntersectBodies(first, second);

    EXPECT_EQ(intersection.issue, BodyBooleanIssue3d::None);
    EXPECT_TRUE(intersection.IsSuccess());
    EXPECT_TRUE(intersection.producedEmptyResult);
    EXPECT_EQ(intersection.body.FaceCount(), 0U);
    EXPECT_TRUE(intersection.bodies.empty());
}

TEST(BodyBoolean3dCapabilityTest, VertexTouchingPolyhedronIntersectionReturnsDeterministicEmptyResult)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildAxisAlignedBoxBody(1.0, 1.0, 1.0, 2.0, 2.0, 2.0);

    const auto intersection = IntersectBodies(first, second);

    EXPECT_EQ(intersection.issue, BodyBooleanIssue3d::None);
    EXPECT_TRUE(intersection.IsSuccess());
    EXPECT_TRUE(intersection.producedEmptyResult);
    EXPECT_EQ(intersection.body.FaceCount(), 0U);
    EXPECT_TRUE(intersection.bodies.empty());
}

TEST(BodyBoolean3dCapabilityTest, EdgeTouchingPolyhedronUnionReturnsDeterministicOrderedMultiBodyResult)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildAxisAlignedBoxBody(1.0, 1.0, 0.0, 2.0, 2.0, 1.0);

    const auto result = UnionBodies(second, first);

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

TEST(BodyBoolean3dCapabilityTest, VertexTouchingPolyhedronDifferenceReturnsOriginalBody)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildAxisAlignedBoxBody(1.0, 1.0, 1.0, 2.0, 2.0, 2.0);

    const auto result = DifferenceBodies(first, second);

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

TEST(BodyBoolean3dCapabilityTest, TouchingPolyhedronDifferenceReturnsOriginalBody)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildAxisAlignedBoxBody(1.0, 0.0, 0.0, 2.0, 1.0, 1.0);

    const auto result = DifferenceBodies(first, second);

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

TEST(BodyBoolean3dCapabilityTest, OverlappingPolyhedronDifferenceReturnsRemainingSingleBox)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
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

TEST(BodyBoolean3dCapabilityTest, ContainedPolyhedronDifferenceReturnsDeterministicEmptyResult)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildAxisAlignedBoxBody(-0.5, -0.5, -0.5, 1.5, 1.5, 1.5);

    const auto result = DifferenceBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_TRUE(result.producedEmptyResult);
    EXPECT_EQ(result.body.FaceCount(), 0U);
    EXPECT_TRUE(result.bodies.empty());
}

TEST(BodyBoolean3dCapabilityTest, OverlappingAxisAlignedBoxUnionReturnsClosedOrthogonalSolid)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody contained = BuildAxisAlignedBoxBody(0.5, 0.5, 0.5, 1.5, 1.5, 1.5);
    const auto united = UnionBodies(first, contained);

    ASSERT_EQ(united.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(united.IsSuccess());
    EXPECT_TRUE(united.bodies.empty());
    ASSERT_TRUE(united.body.IsValid());
    ASSERT_EQ(united.body.ShellCount(), 1U);
    EXPECT_TRUE(united.body.ShellAt(0).IsClosed());
    EXPECT_NEAR(Geometry::Volume(united.body), 1.875, 1e-9);
}

TEST(BodyBoolean3dCapabilityTest, PartialAxisAlignedBoxDifferenceRemainsUnsupported)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildAxisAlignedBoxBody(0.5, 0.5, 0.0, 1.5, 1.5, 1.0);

    const auto difference = DifferenceBodies(first, second);

    ExpectUnsupportedOperation(difference);
}

TEST(BodyBoolean3dCapabilityTest, FaceTouchingLShapePolyhedronUnionReturnsClosedOrthogonalSolid)
{
    const PolyhedronBody first = BuildAxisAlignedBoxBody(0.0, 0.0, 0.0, 2.0, 1.0, 1.0);
    const PolyhedronBody second = BuildAxisAlignedBoxBody(0.0, 1.0, 0.0, 1.0, 2.0, 1.0);

    const auto result = UnionBodies(first, second);
    const auto reversed = UnionBodies(second, first);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_TRUE(result.bodies.empty());
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.ShellCount(), 1U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    EXPECT_NEAR(Geometry::Volume(result.body), 3.0, 1e-9);
    const auto bounds = result.body.Bounds();
    EXPECT_TRUE(bounds.MinPoint().AlmostEquals(SCPoint3d{0.0, 0.0, 0.0}, 1e-12));
    EXPECT_TRUE(bounds.MaxPoint().AlmostEquals(SCPoint3d{2.0, 2.0, 1.0}, 1e-12));
    ASSERT_EQ(reversed.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(reversed.IsSuccess());
    EXPECT_EQ(reversed.body.FaceCount(), result.body.FaceCount());
    EXPECT_NEAR(Geometry::Volume(reversed.body), Geometry::Volume(result.body), 1e-9);
}

TEST(BodyBoolean3dCapabilityTest, FaceTouchingLShapeBrepUnionPreservesTopologyAndVolume)
{
    const auto first = ConvertToBrepBody(BuildAxisAlignedBoxBody(0.0, 0.0, 0.0, 2.0, 1.0, 1.0));
    const auto second = ConvertToBrepBody(BuildAxisAlignedBoxBody(0.0, 1.0, 0.0, 1.0, 2.0, 1.0));
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = UnionBodies(second.body, first.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_TRUE(result.bodies.empty());
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.ShellCount(), 1U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    EXPECT_NEAR(Geometry::Volume(result.body), 3.0, 1e-9);
}

TEST(BodyBoolean3dCapabilityTest, NonConvexBrepInputReturnsUnsupportedOperation)
{
    const auto first = ConvertToBrepBody(BuildAxisAlignedBoxBody(0.0, 0.0, 0.0, 2.0, 1.0, 1.0));
    const auto second = ConvertToBrepBody(BuildAxisAlignedBoxBody(0.0, 1.0, 0.0, 1.0, 2.0, 1.0));
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto lShape = UnionBodies(first.body, second.body);
    ASSERT_EQ(lShape.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(lShape.IsSuccess());
    ExpectUnsupportedOperation(IntersectBodies(lShape.body, first.body));
    ExpectUnsupportedOperation(UnionBodies(lShape.body, first.body));
    ExpectUnsupportedOperation(DifferenceBodies(lShape.body, first.body));
}

TEST(BodyBoolean3dCapabilityTest, RotatedBoxIntersectionReturnsExplicitUnsupportedOperation)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildRotatedOverlapBoxBody();

    const auto result = IntersectBodies(first, second);

    ExpectUnsupportedOperation(result);
}

TEST(BodyBoolean3dCapabilityTest, RotatedBoxUnionAndDifferenceRemainExplicitlyUnsupported)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildRotatedOverlapBoxBody();

    const auto united = UnionBodies(first, second);
    const auto difference = DifferenceBodies(first, second);

    ExpectUnsupportedOperation(united);
    ExpectUnsupportedOperation(difference);
}

TEST(BodyBoolean3dCapabilityTest, EqualBoundsButDifferentRotatedBoxesDoNotUseIdenticalBodyFastPath)
{
    const PolyhedronBody first = BuildRotatedPrismaticBox(0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.23);
    const PolyhedronBody second = BuildRotatedPrismaticBox(0.0, 0.0, 0.0, 1.0, 1.0, 1.0, -0.23);

    ASSERT_TRUE(first.Bounds().MinPoint().AlmostEquals(second.Bounds().MinPoint(), 1e-12));
    ASSERT_TRUE(first.Bounds().MaxPoint().AlmostEquals(second.Bounds().MaxPoint(), 1e-12));
    ExpectUnsupportedOperation(IntersectBodies(first, second));
    ExpectUnsupportedOperation(UnionBodies(first, second));
    ExpectUnsupportedOperation(DifferenceBodies(first, second));
}

TEST(BodyBoolean3dCapabilityTest, NonCoplanarRotatedConvexBoxesIntersectAsClosedSingleShell)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildNonCoplanarRotatedOverlapBoxBody();

    const auto result = IntersectBodies(first, second);
    const auto reversed = IntersectBodies(second, first);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.ShellCount(), 1U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    EXPECT_GT(Geometry::Volume(result.body), 0.0);
    ASSERT_EQ(reversed.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(reversed.IsSuccess());
    EXPECT_NEAR(Geometry::Volume(reversed.body), Geometry::Volume(result.body), 1e-9);
    EXPECT_TRUE(reversed.body.Bounds().MinPoint().AlmostEquals(result.body.Bounds().MinPoint(), 1e-9));
    EXPECT_TRUE(reversed.body.Bounds().MaxPoint().AlmostEquals(result.body.Bounds().MaxPoint(), 1e-9));
}

TEST(BodyBoolean3dCapabilityTest, NonCoplanarRotatedConvexBrepBoxesIntersectAsClosedSingleShell)
{
    const auto first = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    const auto second = ConvertToBrepBody(BuildNonCoplanarRotatedOverlapBoxBody());
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = IntersectBodies(first.body, second.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.ShellCount(), 1U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    EXPECT_GT(Geometry::Volume(result.body), 0.0);
}

TEST(BodyBoolean3dCapabilityTest, RotatedPrismaticBoxIntersectionMatchesIndependentHalfSpaceClippingOracle)
{
    std::mt19937 generator(20260902U);
    std::uniform_real_distribution<double> centerOffset(-0.25, 0.25);
    std::uniform_real_distribution<double> angleDistribution(0.12, 0.68);
    std::uniform_real_distribution<double> zDistribution(0.08, 0.32);

    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    for (int sample = 0; sample < 24; ++sample)
    {
        const double centerX = 0.5 + centerOffset(generator);
        const double centerY = 0.5 + centerOffset(generator);
        const double minZ = zDistribution(generator);
        const double angleRadians = angleDistribution(generator);
        const double minX = centerX - 0.5;
        const double minY = centerY - 0.5;
        const double maxX = centerX + 0.5;
        const double maxY = centerY + 0.5;
        const double maxZ = minZ + 1.0;
        const PolyhedronBody second =
            BuildRotatedPrismaticBox(minX, minY, minZ, maxX, maxY, maxZ, angleRadians);
        const double expectedVolume =
            RotatedPrismaticBoxIntersectionVolumeOracle(minX, minY, minZ, maxX, maxY, maxZ, angleRadians);

        ASSERT_GT(expectedVolume, 1e-5);
        const auto result = IntersectBodies(first, second);
        const auto reversed = IntersectBodies(second, first);
        ASSERT_EQ(result.issue, BodyBooleanIssue3d::None) << "sample=" << sample;
        ASSERT_EQ(reversed.issue, BodyBooleanIssue3d::None) << "sample=" << sample;
        ASSERT_TRUE(result.body.IsValid());
        ASSERT_EQ(result.body.ShellCount(), 1U);
        EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
        EXPECT_NEAR(Geometry::Volume(result.body), expectedVolume, 1e-8) << "sample=" << sample;
        EXPECT_NEAR(Geometry::Volume(reversed.body), expectedVolume, 1e-8) << "sample=" << sample;
    }
}

TEST(BodyBoolean3dCapabilityTest, BrepRotatedBoxOperationsReturnExplicitUnsupportedOperation)
{
    const auto first = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    const auto second = ConvertToBrepBody(BuildRotatedOverlapBoxBody());
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    ExpectUnsupportedOperation(IntersectBodies(first.body, second.body));
    ExpectUnsupportedOperation(UnionBodies(first.body, second.body));
    ExpectUnsupportedOperation(DifferenceBodies(first.body, second.body));
}
