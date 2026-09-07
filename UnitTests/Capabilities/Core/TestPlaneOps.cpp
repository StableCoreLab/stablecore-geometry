#include <cmath>

#include <gtest/gtest.h>

#include "Core/PlaneOps.h"

using namespace Geometry;

TEST(PlaneOpsTest, ConstructsAndEvaluatesPlaneThroughThreePoints)
{
    const auto result = CreatePlaneThroughThreePoints({0.0, 0.0, 10.0}, {1.0, 0.0, 12.0}, {0.0, 1.0, 13.0});
    ASSERT_TRUE(result.success);
    const auto elevation = EvaluatePlaneElevationAtXY(result.plane, {2.0, 3.0});
    ASSERT_TRUE(elevation.success);
    EXPECT_NEAR(elevation.elevation, 23.0, 1e-9);
}

TEST(PlaneOpsTest, RejectsDegenerateAndInvalidInput)
{
    EXPECT_EQ(CreatePlaneThroughThreePoints({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}).failure,
              SCPlaneConstructionFailure::CoincidentPoints);
    EXPECT_EQ(CreatePlaneThroughThreePoints({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}).failure,
              SCPlaneConstructionFailure::CollinearPoints);
    EXPECT_EQ(CreatePlaneThroughThreePoints({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, 0.0).failure,
              SCPlaneConstructionFailure::InvalidTolerance);
}

TEST(PlaneOpsTest, ConstructsFromGradientAndRejectsVerticalElevation)
{
    const auto result = CreatePlaneFromPointAndXYGradient({0.0, 0.0, 5.0}, {1.0, 0.0}, 0.25);
    ASSERT_TRUE(result.success);
    const auto elevation = EvaluatePlaneElevationAtXY(result.plane, {4.0, 0.0});
    ASSERT_TRUE(elevation.success);
    EXPECT_NEAR(elevation.elevation, 6.0, 1e-9);

    const SCPlane vertical{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
    EXPECT_EQ(EvaluatePlaneElevationAtXY(vertical, {0.0, 0.0}).failure,
              SCPlaneElevationFailure::VerticalOrNearVerticalPlane);
}

TEST(PlaneOpsTest, UsesRelativeAngularToleranceAndEquivalentOverloads)
{
    const SCPlaneTolerance tolerance{1e-6, 1e-3};
    const auto nearlyCollinear =
        CreatePlaneThroughThreePoints({0.0, 0.0, 0.0}, {1000.0, 0.0, 0.0}, {2000.0, 0.1, 0.0}, tolerance);
    EXPECT_EQ(nearlyCollinear.failure, SCPlaneConstructionFailure::CollinearPoints);

    const auto scalar = CreatePlaneFromPointAndXYGradient({1.0, 2.0, 3.0}, {4.0, 5.0}, 0.25, 1e-6);
    const auto structured = CreatePlaneFromPointAndXYGradient(
        {1.0, 2.0, 3.0}, {4.0, 5.0}, 0.25, SCPlaneTolerance{1e-6, 1e-6});
    ASSERT_TRUE(scalar.success);
    ASSERT_TRUE(structured.success);
    EXPECT_EQ(scalar.plane.origin, structured.plane.origin);
    EXPECT_EQ(scalar.plane.normal, structured.plane.normal);
}
