#include <gtest/gtest.h>

#include "Support/Geometry2d/Normalize2.h"
#include "Types/Geometry2d/SCMatrix2d.h"
#include "Types/Geometry2d/SCTransform2d.h"
#include "Types/Geometry3d/SCMatrix3d.h"
#include "Types/Geometry3d/SCTransform3d.h"

TEST(FailureSentinelContractTest, MatrixInverseUsesZeroMatrixForSingularInputs)
{
    const Geometry::SCMatrix2d singular2d{1.0, 2.0, 2.0, 4.0};
    const Geometry::SCMatrix2d inverse2d = singular2d.Inverse();
    EXPECT_FALSE(singular2d.IsInvertible());
    EXPECT_DOUBLE_EQ(inverse2d.m00, 0.0);
    EXPECT_DOUBLE_EQ(inverse2d.m11, 0.0);

    const Geometry::SCMatrix3d singular3d{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0};
    const Geometry::SCMatrix3d inverse3d = singular3d.Inverse();
    EXPECT_FALSE(singular3d.IsInvertible());
    EXPECT_DOUBLE_EQ(inverse3d.m00, 0.0);
    EXPECT_DOUBLE_EQ(inverse3d.m11, 0.0);
    EXPECT_DOUBLE_EQ(inverse3d.m22, 0.0);
}

TEST(FailureSentinelContractTest, TransformInverseDiffersBetweenTwoAndThreeDimensions)
{
    const Geometry::SCTransform2d singular2d = Geometry::SCTransform2d::Scale(Geometry::SCPoint2d{}, 0.0);
    EXPECT_FALSE(singular2d.Inverse().has_value());

    const Geometry::SCTransform3d singular3d = Geometry::SCTransform3d::Scale(Geometry::SCPoint3d{}, 0.0);
    const Geometry::SCTransform3d inverse3d = singular3d.Inverse();
    EXPECT_TRUE(inverse3d.linear.IsInvertible());
    EXPECT_DOUBLE_EQ(inverse3d.translation.x, 0.0);
    EXPECT_DOUBLE_EQ(inverse3d.translation.y, 0.0);
    EXPECT_DOUBLE_EQ(inverse3d.translation.z, 0.0);
}

TEST(FailureSentinelContractTest, TryNormalizeFailureLeavesCallerOutputUnchanged)
{
    Geometry::SCVector2d normalized{9.0, 9.0};
    EXPECT_FALSE(Geometry::TryNormalize(Geometry::SCVector2d{}, normalized));
    EXPECT_DOUBLE_EQ(normalized.x, 9.0);
    EXPECT_DOUBLE_EQ(normalized.y, 9.0);
}
