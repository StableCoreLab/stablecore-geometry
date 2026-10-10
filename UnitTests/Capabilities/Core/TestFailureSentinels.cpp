#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "Support/Geometry2d/Normalize2.h"
#include "Types/Geometry2d/SCMatrix2d.h"
#include "Types/Geometry2d/SCTransform2d.h"
#include "Types/Geometry3d/SCMatrix3d.h"
#include "Types/Geometry3d/SCTransform3d.h"

TEST(InverseContractTest, MatrixInverseReturnsNulloptForSingularInputs)
{
    const Geometry::SCMatrix2d singular2d{1.0, 2.0, 2.0, 4.0};
    EXPECT_FALSE(singular2d.IsInvertible());
    EXPECT_FALSE(singular2d.Inverse().has_value());

    const Geometry::SCMatrix3d singular3d{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0};
    EXPECT_FALSE(singular3d.IsInvertible());
    EXPECT_FALSE(singular3d.Inverse().has_value());
}

TEST(InverseContractTest, TransformInverseReturnsNulloptForSingularInputs)
{
    const Geometry::SCTransform2d singular2d = Geometry::SCTransform2d::Scale(Geometry::SCPoint2d{}, 0.0);
    EXPECT_FALSE(singular2d.Inverse().has_value());

    const Geometry::SCTransform3d singular3d = Geometry::SCTransform3d::Scale(Geometry::SCPoint3d{}, 0.0);
    EXPECT_FALSE(singular3d.Inverse().has_value());
}

TEST(InverseContractTest, InverseRejectsInvalidToleranceAndNonFiniteInput)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Geometry::SCMatrix2d identity2d = Geometry::SCMatrix2d::Identity();
    const Geometry::SCMatrix3d identity3d = Geometry::SCMatrix3d::Identity();
    EXPECT_FALSE(identity2d.IsInvertible(-1.0));
    EXPECT_FALSE(identity2d.Inverse(-1.0).has_value());
    EXPECT_FALSE(identity3d.IsInvertible(nan));
    EXPECT_FALSE(identity3d.Inverse(nan).has_value());
    EXPECT_FALSE((Geometry::SCMatrix2d{nan, 0.0, 0.0, 1.0}.Inverse().has_value()));
    EXPECT_FALSE((Geometry::SCMatrix3d{nan, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}.Inverse().has_value()));
    EXPECT_FALSE((Geometry::SCTransform2d{identity2d, Geometry::SCVector2d{nan, 0.0}}.Inverse().has_value()));
    EXPECT_FALSE((Geometry::SCTransform3d{identity3d, Geometry::SCVector3d{nan, 0.0, 0.0}}.Inverse().has_value()));
}

TEST(InverseContractTest, MatrixInverseUsesInclusiveDeterminantThreshold)
{
    const Geometry::SCMatrix2d matrix2d{2.0, 0.0, 0.0, 1.0};
    const Geometry::SCMatrix3d matrix3d{2.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    EXPECT_FALSE(matrix2d.Inverse(2.0).has_value());
    EXPECT_FALSE(matrix3d.Inverse(2.0).has_value());
    EXPECT_TRUE(matrix2d.Inverse(1.0).has_value());
    EXPECT_TRUE(matrix3d.Inverse(1.0).has_value());

    const double max = std::numeric_limits<double>::max();
    EXPECT_FALSE((Geometry::SCMatrix2d{max, 0.0, 0.0, max}.Inverse().has_value()));
    const auto largeInverse = Geometry::SCMatrix3d{max, 0.0, 0.0, 0.0, max, 0.0, 0.0, 0.0, max}.Inverse();
    ASSERT_TRUE(largeInverse.has_value());
    EXPECT_DOUBLE_EQ(largeInverse->m00, 1.0 / max);
    EXPECT_DOUBLE_EQ(largeInverse->m11, 1.0 / max);
    EXPECT_DOUBLE_EQ(largeInverse->m22, 1.0 / max);
}

TEST(InverseContractTest, TwoDimensionalInvertibilityRequiresFiniteInverse)
{
    const double large = std::ldexp(1.0, 1023);
    const double small = std::ldexp(1.0, -1040);
    const Geometry::SCMatrix2d matrix{large, 0.0, 0.0, small};
    EXPECT_GT(matrix.Determinant(), Geometry::kDefaultEpsilon);
    EXPECT_FALSE(matrix.Inverse().has_value());
    EXPECT_FALSE(matrix.IsInvertible());
}

TEST(InverseContractTest, TwoDimensionalInverseHandlesSubnormalDeterminant)
{
    const Geometry::SCMatrix2d matrix{std::ldexp(1.0, -500), 0.0,
                                      0.0, std::ldexp(1.0, -530)};
    const auto inverse = matrix.Inverse(0.0);
    ASSERT_TRUE(inverse.has_value());
    EXPECT_TRUE(matrix.IsInvertible(0.0));
    EXPECT_DOUBLE_EQ(inverse->m00, std::ldexp(1.0, 500));
    EXPECT_DOUBLE_EQ(inverse->m01, 0.0);
    EXPECT_DOUBLE_EQ(inverse->m10, 0.0);
    EXPECT_DOUBLE_EQ(inverse->m11, std::ldexp(1.0, 530));
}

TEST(InverseContractTest, ThreeDimensionalInverseHandlesMixedScales)
{
    const double large = std::ldexp(1.0, 600);
    const double small = std::ldexp(1.0, -600);
    const Geometry::SCMatrix3d diagonal{large, 0.0, 0.0, 0.0, small, 0.0, 0.0, 0.0, small};
    const auto diagonalInverse = diagonal.Inverse(0.0);
    ASSERT_TRUE(diagonalInverse.has_value());
    EXPECT_DOUBLE_EQ(diagonalInverse->m00, small);
    EXPECT_DOUBLE_EQ(diagonalInverse->m11, large);
    EXPECT_DOUBLE_EQ(diagonalInverse->m22, large);

    const Geometry::SCMatrix3d mixed{large, 1.0, 0.0, 0.5, small, 0.0, 0.0, 0.0, small};
    const auto mixedInverse = mixed.Inverse(0.0);
    ASSERT_TRUE(mixedInverse.has_value());
    EXPECT_DOUBLE_EQ(mixedInverse->m00, 2.0 * small);
    EXPECT_DOUBLE_EQ(mixedInverse->m22, large);

    const Geometry::SCMatrix3d singular{large, 1.0, 0.0, 1.0, small, 0.0, 0.0, 0.0, small};
    EXPECT_FALSE(singular.IsInvertible(0.0));
    EXPECT_FALSE(singular.Inverse(0.0).has_value());
}

TEST(InverseContractTest, ThreeDimensionalInverseAvoidsIntermediateOverflow)
{
    const double large = std::ldexp(1.0, 600);
    const double small = std::ldexp(1.0, -600);
    const Geometry::SCMatrix3d matrix{small, large, 0.0, small, -large, 0.0, 0.0, 0.0, 1.0};
    const auto inverse = matrix.Inverse();
    ASSERT_TRUE(inverse.has_value());
    EXPECT_TRUE(matrix.IsInvertible());
    EXPECT_DOUBLE_EQ(inverse->m00, large * 0.5);
    EXPECT_DOUBLE_EQ(inverse->m01, large * 0.5);
    EXPECT_DOUBLE_EQ(inverse->m10, small * 0.5);
    EXPECT_DOUBLE_EQ(inverse->m11, -small * 0.5);
}

TEST(InverseContractTest, ThreeDimensionalInversePreservesSmallElementsInLargeColumns)
{
    const double large = std::ldexp(1.0, 1000);
    const double small = std::ldexp(1.0, -1000);
    const Geometry::SCMatrix3d matrix{large, large, 0.0, small, 0.0, 0.0, 0.0, 0.0, 1.0};
    const auto inverse = matrix.Inverse();
    ASSERT_TRUE(inverse.has_value());
    EXPECT_TRUE(matrix.IsInvertible());
    EXPECT_DOUBLE_EQ(inverse->m00, 0.0);
    EXPECT_DOUBLE_EQ(inverse->m01, large);
    EXPECT_DOUBLE_EQ(inverse->m10, small);
    EXPECT_DOUBLE_EQ(inverse->m11, -large);
    EXPECT_DOUBLE_EQ(inverse->m22, 1.0);
}

TEST(InverseContractTest, ThreeDimensionalInversePreservesSmallDeterminantResidual)
{
    const double large = std::ldexp(1.0, 600);
    const Geometry::SCMatrix3d matrix{large, 1.0, large, 1.0, 0.0, 1.0, 0.0, 1.0, 1.0};
    const auto inverse = matrix.Inverse();
    ASSERT_TRUE(inverse.has_value());
    EXPECT_TRUE(matrix.IsInvertible());
    EXPECT_DOUBLE_EQ(inverse->m00, 1.0);
    EXPECT_DOUBLE_EQ(inverse->m10, 1.0);
    EXPECT_DOUBLE_EQ(inverse->m20, -1.0);
}

TEST(InverseContractTest, ThreeDimensionalInversePreservesRoundedProductResidual)
{
    const double n = std::ldexp(1.0, 27);
    const Geometry::SCMatrix3d matrix{n, n - 1.0, 0.0, n + 1.0, n, 0.0, 0.0, 0.0, 1.0};
    const auto inverse = matrix.Inverse();
    ASSERT_TRUE(inverse.has_value());
    EXPECT_TRUE(matrix.IsInvertible());
    EXPECT_DOUBLE_EQ(inverse->m00, n);
    EXPECT_DOUBLE_EQ(inverse->m01, 1.0 - n);
    EXPECT_DOUBLE_EQ(inverse->m10, -n - 1.0);
    EXPECT_DOUBLE_EQ(inverse->m11, n);
    EXPECT_DOUBLE_EQ(inverse->m22, 1.0);
    EXPECT_FALSE(matrix.Inverse(1.0).has_value());
}

TEST(InverseContractTest, ThreeDimensionalInverseRoundTrip)
{
    const Geometry::SCTransform3d transform{
        Geometry::SCMatrix3d{2.0, 1.0, 0.0, 0.0, 3.0, 1.0, 0.0, 0.0, 4.0},
        Geometry::SCVector3d{5.0, -2.0, 7.0}};
    const auto inverse = transform.Inverse();
    ASSERT_TRUE(inverse.has_value());

    const Geometry::SCPoint3d point{1.0, -3.0, 2.0};
    const Geometry::SCPoint3d roundTrip = inverse->Apply(transform.Apply(point));
    EXPECT_NEAR(roundTrip.x, point.x, 1e-12);
    EXPECT_NEAR(roundTrip.y, point.y, 1e-12);
    EXPECT_NEAR(roundTrip.z, point.z, 1e-12);
}

TEST(FailureSentinelContractTest, TryNormalizeFailureLeavesCallerOutputUnchanged)
{
    Geometry::SCVector2d normalized{9.0, 9.0};
    EXPECT_FALSE(Geometry::TryNormalize(Geometry::SCVector2d{}, normalized));
    EXPECT_DOUBLE_EQ(normalized.x, 9.0);
    EXPECT_DOUBLE_EQ(normalized.y, 9.0);
}
