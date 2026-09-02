#include <gtest/gtest.h>

#include "Core/Metrics.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Geometry2d/SCPolyline2d.h"
#include "Support/Epsilon.h"
#include "Types/Geometry2d/SCMatrix2d.h"
#include "Types/Geometry2d/SCTransform2d.h"

using Geometry::Distance;
using Geometry::DistanceSquared;
using Geometry::SCMatrix2d;
using Geometry::SCPoint2d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;
using Geometry::SCTransform2d;
using Geometry::SCVector2d;

TEST(Matrix2dTest, IdentityPreservesPointAndVector)
{
    const SCMatrix2d identity = SCMatrix2d::Identity();
    const SCPoint2d point = identity * SCPoint2d{3.0, -2.0};
    EXPECT_NEAR(point.x, 3.0, 1e-12);
    EXPECT_NEAR(point.y, -2.0, 1e-12);
    const SCVector2d vector = identity * SCVector2d{3.0, -2.0};
    EXPECT_NEAR(vector.x, 3.0, 1e-12);
    EXPECT_NEAR(vector.y, -2.0, 1e-12);
}

TEST(Matrix2dTest, Rotation90Degrees)
{
    const SCMatrix2d rotation = SCMatrix2d::Rotation(Geometry::kPi * 0.5);
    const SCPoint2d point = rotation * SCPoint2d{1.0, 0.0};
    EXPECT_NEAR(point.x, 0.0, 1e-12);
    EXPECT_NEAR(point.y, 1.0, 1e-12);
}

TEST(Matrix2dTest, DeterminantInverseTranspose)
{
    const SCMatrix2d matrix{2.0, 1.0, 0.0, 3.0};
    EXPECT_NEAR(matrix.Determinant(), 6.0, 1e-12);
    ASSERT_TRUE(matrix.IsInvertible());

    const SCMatrix2d product = matrix * matrix.Inverse();
    EXPECT_NEAR(product.m00, 1.0, 1e-12);
    EXPECT_NEAR(product.m01, 0.0, 1e-12);
    EXPECT_NEAR(product.m10, 0.0, 1e-12);
    EXPECT_NEAR(product.m11, 1.0, 1e-12);

    const SCMatrix2d transposed = matrix.Transpose();
    EXPECT_NEAR(transposed.m00, 2.0, 1e-12);
    EXPECT_NEAR(transposed.m01, 0.0, 1e-12);
    EXPECT_NEAR(transposed.m10, 1.0, 1e-12);
    EXPECT_NEAR(transposed.m11, 3.0, 1e-12);
}

TEST(Matrix2dTest, SingularInverseReturnsEmptyMatrix)
{
    const SCMatrix2d singular{1.0, 2.0, 2.0, 4.0};
    ASSERT_FALSE(singular.IsInvertible());
    const SCMatrix2d inverse = singular.Inverse();
    EXPECT_DOUBLE_EQ(inverse.m00, 0.0);
    EXPECT_DOUBLE_EQ(inverse.m01, 0.0);
    EXPECT_DOUBLE_EQ(inverse.m10, 0.0);
    EXPECT_DOUBLE_EQ(inverse.m11, 0.0);
}

TEST(Matrix2dTest, MultiplicationIsAssociative)
{
    const SCMatrix2d a = SCMatrix2d::Rotation(0.3);
    const SCMatrix2d b{2.0, 0.0, 0.0, 3.0};
    const SCMatrix2d c = SCMatrix2d::Rotation(1.1);

    const SCMatrix2d left = (a * b) * c;
    const SCMatrix2d right = a * (b * c);
    EXPECT_NEAR(left.m00, right.m00, 1e-12);
    EXPECT_NEAR(left.m01, right.m01, 1e-12);
    EXPECT_NEAR(left.m10, right.m10, 1e-12);
    EXPECT_NEAR(left.m11, right.m11, 1e-12);
}

TEST(Transform2dTest, TranslationMovesPointButNotVector)
{
    const SCTransform2d transform = SCTransform2d::Translation(SCVector2d{1.0, 2.0});
    const SCPoint2d point = transform.Apply(SCPoint2d{3.0, 4.0});
    EXPECT_NEAR(point.x, 4.0, 1e-12);
    EXPECT_NEAR(point.y, 6.0, 1e-12);
    const SCVector2d vector = transform.Apply(SCVector2d{3.0, 4.0});
    EXPECT_NEAR(vector.x, 3.0, 1e-12);
    EXPECT_NEAR(vector.y, 4.0, 1e-12);
}

TEST(Transform2dTest, RotationAroundCenter)
{
    const SCPoint2d center{2.0, 1.0};
    const SCTransform2d transform = SCTransform2d::Rotation(center, Geometry::kPi * 0.5);

    const SCPoint2d transformedCenter = transform.Apply(center);
    EXPECT_NEAR(transformedCenter.x, center.x, 1e-12);
    EXPECT_NEAR(transformedCenter.y, center.y, 1e-12);
    const SCPoint2d transformedNeighbor = transform.Apply(center + SCVector2d{1.0, 0.0});
    EXPECT_NEAR(transformedNeighbor.x, 2.0, 1e-12);
    EXPECT_NEAR(transformedNeighbor.y, 2.0, 1e-12);
}

TEST(Transform2dTest, ScaleFixesOrigin)
{
    const SCPoint2d origin{1.0, 1.0};
    const SCTransform2d transform = SCTransform2d::Scale(origin, 2.0);

    const SCPoint2d transformedOrigin = transform.Apply(origin);
    EXPECT_NEAR(transformedOrigin.x, origin.x, 1e-12);
    EXPECT_NEAR(transformedOrigin.y, origin.y, 1e-12);
    const SCPoint2d transformedNeighbor = transform.Apply(origin + SCVector2d{1.0, 0.0});
    EXPECT_NEAR(transformedNeighbor.x, 3.0, 1e-12);
    EXPECT_NEAR(transformedNeighbor.y, 1.0, 1e-12);
}

TEST(Transform2dTest, InverseRoundTrip)
{
    const SCTransform2d transform = SCTransform2d::Translation(SCVector2d{2.0, 3.0}) *
                                    SCTransform2d::Rotation(SCPoint2d{1.0, 1.0}, 0.5);

    const auto inverse = transform.Inverse();
    ASSERT_TRUE(inverse.has_value());

    const SCTransform2d composed = *inverse * transform;
    EXPECT_NEAR(composed.linear.m00, 1.0, 1e-12);
    EXPECT_NEAR(composed.linear.m01, 0.0, 1e-12);
    EXPECT_NEAR(composed.linear.m10, 0.0, 1e-12);
    EXPECT_NEAR(composed.linear.m11, 1.0, 1e-12);
    EXPECT_NEAR(composed.translation.x, 0.0, 1e-12);
    EXPECT_NEAR(composed.translation.y, 0.0, 1e-12);

    const SCPoint2d point{7.0, -3.0};
    const SCPoint2d roundTrip = inverse->Apply(transform.Apply(point));
    EXPECT_NEAR(roundTrip.x, point.x, 1e-12);
    EXPECT_NEAR(roundTrip.y, point.y, 1e-12);
}

TEST(Transform2dTest, SingularInverseReturnsNullopt)
{
    ASSERT_FALSE(SCTransform2d::Scale(SCPoint2d{0.0, 0.0}, 0.0).Inverse().has_value());
    ASSERT_FALSE((SCTransform2d{SCMatrix2d{1.0, 2.0, 2.0, 4.0}, SCVector2d{}}.Inverse().has_value()));
}

TEST(Transform2dTest, AxisGridPlacementEquivalence)
{
    // Axis-grid placement: Apply(local) = origin + R * (local - anchor),
    // i.e. linear = R and translation = origin - R * anchor.
    const SCPoint2d anchor{2.0, 3.0};
    const SCPoint2d origin{5.0, -1.0};
    const SCMatrix2d rotation = SCMatrix2d::Rotation(Geometry::kPi * 0.5);
    const SCTransform2d transform{rotation, origin - rotation * anchor};

    const SCPoint2d transformedAnchor = transform.Apply(anchor);
    EXPECT_NEAR(transformedAnchor.x, origin.x, 1e-12);
    EXPECT_NEAR(transformedAnchor.y, origin.y, 1e-12);

    // Compare pointwise against the hand-written expression, and prove the
    // wrong composition Translation(origin) * Rotation(anchor, angle) fails.
    const SCPoint2d local{-1.0, 4.0};
    const SCPoint2d transformedLocal = transform.Apply(local);
    const SCPoint2d expectedLocal = origin + rotation * (local - anchor);
    EXPECT_NEAR(transformedLocal.x, expectedLocal.x, 1e-12);
    EXPECT_NEAR(transformedLocal.y, expectedLocal.y, 1e-12);

    const SCTransform2d wrongComposition = SCTransform2d::Translation(SCVector2d{origin.x, origin.y}) *
                                           SCTransform2d::Rotation(anchor, Geometry::kPi * 0.5);
    // The wrong composition maps anchor to origin + anchor, not origin.
    ASSERT_FALSE(wrongComposition.Apply(anchor).AlmostEquals(origin, 1e-12));
}

TEST(Measure2dTest, PointSegmentDistance)
{
    const Geometry::SCLineSegment2d segment(SCPoint2d{0.0, 0.0}, SCPoint2d{2.0, 0.0});
    EXPECT_NEAR(DistanceSquared(SCPoint2d{1.0, 2.0}, segment), 4.0, 1e-12);
    EXPECT_NEAR(Distance(SCPoint2d{1.0, 2.0}, segment), 2.0, 1e-12);
}

TEST(Measure2dTest, PolylineBounds)
{
    const SCPolyline2d polyline({SCPoint2d{0.0, 0.0}, SCPoint2d{2.0, 0.0}, SCPoint2d{2.0, 2.0}, SCPoint2d{0.0, 2.0}},
                                SCPolylineClosure::Closed);
    const Geometry::SCBox2d bounds = Geometry::Bounds(polyline);
    EXPECT_NEAR(bounds.MinPoint().x, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().y, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().x, 2.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().y, 2.0, 1e-12);
}

TEST(Vector2Test, Normalized)
{
    const SCVector2d normalized = SCVector2d{3.0, 4.0}.Normalized();
    EXPECT_NEAR(normalized.x, 0.6, 1e-12);
    EXPECT_NEAR(normalized.y, 0.8, 1e-12);
    const SCVector2d zeroNormalized = SCVector2d{0.0, 0.0}.Normalized();
    EXPECT_DOUBLE_EQ(zeroNormalized.x, 0.0);
    EXPECT_DOUBLE_EQ(zeroNormalized.y, 0.0);
}
