#include "Support/BodyBooleanFixtures3d.h"

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
