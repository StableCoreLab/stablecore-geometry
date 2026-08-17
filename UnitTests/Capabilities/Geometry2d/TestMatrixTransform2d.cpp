#include <gtest/gtest.h>
#include <cmath>

#include "Core/Metrics.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Geometry2d/SCPolyline2d.h"
#include "Support/GeometryTestSupport.h"
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

namespace
{
    constexpr double kPi = 3.14159265358979323846;

    void AssertMatrixNear(const SCMatrix2d& actual, const SCMatrix2d& expected, double eps)
    {
        GEOMETRY_TEST_ASSERT_NEAR(actual.m00, expected.m00, eps);
        GEOMETRY_TEST_ASSERT_NEAR(actual.m01, expected.m01, eps);
        GEOMETRY_TEST_ASSERT_NEAR(actual.m10, expected.m10, eps);
        GEOMETRY_TEST_ASSERT_NEAR(actual.m11, expected.m11, eps);
    }
}  // namespace

TEST(Matrix2dTest, IdentityPreservesPointAndVector)
{
    const SCMatrix2d identity = SCMatrix2d::Identity();
    GEOMETRY_TEST_ASSERT_POINT_NEAR((identity * SCPoint2d{3.0, -2.0}), (SCPoint2d{3.0, -2.0}), 1e-12);
    GEOMETRY_TEST_ASSERT_VECTOR_NEAR((identity * SCVector2d{3.0, -2.0}), (SCVector2d{3.0, -2.0}), 1e-12);
}

TEST(Matrix2dTest, Rotation90Degrees)
{
    const SCMatrix2d rotation = SCMatrix2d::Rotation(kPi * 0.5);
    GEOMETRY_TEST_ASSERT_POINT_NEAR((rotation * SCPoint2d{1.0, 0.0}), (SCPoint2d{0.0, 1.0}), 1e-12);
}

TEST(Matrix2dTest, DeterminantInverseTranspose)
{
    const SCMatrix2d matrix{2.0, 1.0, 0.0, 3.0};
    GEOMETRY_TEST_ASSERT_NEAR(matrix.Determinant(), 6.0, 1e-12);
    ASSERT_TRUE(matrix.IsInvertible());

    const SCMatrix2d product = matrix * matrix.Inverse();
    AssertMatrixNear(product, SCMatrix2d::Identity(), 1e-12);

    const SCMatrix2d transposed = matrix.Transpose();
    GEOMETRY_TEST_ASSERT_NEAR(transposed.m00, 2.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(transposed.m01, 0.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(transposed.m10, 1.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(transposed.m11, 3.0, 1e-12);
}

TEST(Matrix2dTest, SingularInverseReturnsEmptyMatrix)
{
    const SCMatrix2d singular{1.0, 2.0, 2.0, 4.0};
    ASSERT_FALSE(singular.IsInvertible());
    const SCMatrix2d inverse = singular.Inverse();
    AssertMatrixNear(inverse, SCMatrix2d{}, 0.0);
}

TEST(Matrix2dTest, MultiplicationIsAssociative)
{
    const SCMatrix2d a = SCMatrix2d::Rotation(0.3);
    const SCMatrix2d b{2.0, 0.0, 0.0, 3.0};
    const SCMatrix2d c = SCMatrix2d::Rotation(1.1);

    AssertMatrixNear((a * b) * c, a * (b * c), 1e-12);
}

TEST(Transform2dTest, TranslationMovesPointButNotVector)
{
    const SCTransform2d transform = SCTransform2d::Translation(SCVector2d{1.0, 2.0});
    GEOMETRY_TEST_ASSERT_POINT_NEAR(transform.Apply(SCPoint2d{3.0, 4.0}), (SCPoint2d{4.0, 6.0}), 1e-12);
    GEOMETRY_TEST_ASSERT_VECTOR_NEAR(transform.Apply(SCVector2d{3.0, 4.0}), (SCVector2d{3.0, 4.0}), 1e-12);
}

TEST(Transform2dTest, RotationAroundCenter)
{
    const SCPoint2d center{2.0, 1.0};
    const SCTransform2d transform = SCTransform2d::Rotation(center, kPi * 0.5);

    GEOMETRY_TEST_ASSERT_POINT_NEAR(transform.Apply(center), center, 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(transform.Apply(center + SCVector2d{1.0, 0.0}), (center + SCVector2d{0.0, 1.0}), 1e-12);
}

TEST(Transform2dTest, ScaleFixesOrigin)
{
    const SCPoint2d origin{1.0, 1.0};
    const SCTransform2d transform = SCTransform2d::Scale(origin, 2.0);

    GEOMETRY_TEST_ASSERT_POINT_NEAR(transform.Apply(origin), origin, 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(transform.Apply(origin + SCVector2d{1.0, 0.0}), (origin + SCVector2d{2.0, 0.0}), 1e-12);
}

TEST(Transform2dTest, InverseRoundTrip)
{
    const SCTransform2d transform = SCTransform2d::Translation(SCVector2d{2.0, 3.0}) *
                                    SCTransform2d::Rotation(SCPoint2d{1.0, 1.0}, 0.5);

    const auto inverse = transform.Inverse();
    ASSERT_TRUE(inverse.has_value());

    const SCTransform2d composed = *inverse * transform;
    AssertMatrixNear(composed.linear, SCMatrix2d::Identity(), 1e-12);
    GEOMETRY_TEST_ASSERT_VECTOR_NEAR(composed.translation, (SCVector2d{0.0, 0.0}), 1e-12);

    const SCPoint2d point{7.0, -3.0};
    GEOMETRY_TEST_ASSERT_POINT_NEAR(inverse->Apply(transform.Apply(point)), point, 1e-12);
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
    const SCMatrix2d rotation = SCMatrix2d::Rotation(kPi * 0.5);
    const SCTransform2d transform{rotation, origin - rotation * anchor};

    GEOMETRY_TEST_ASSERT_POINT_NEAR(transform.Apply(anchor), origin, 1e-12);

    // Compare pointwise against the hand-written expression, and prove the
    // wrong composition Translation(origin) * Rotation(anchor, angle) fails.
    const SCPoint2d local{-1.0, 4.0};
    GEOMETRY_TEST_ASSERT_POINT_NEAR(transform.Apply(local), origin + rotation * (local - anchor), 1e-12);

    const SCTransform2d wrongComposition = SCTransform2d::Translation(SCVector2d{origin.x, origin.y}) *
                                           SCTransform2d::Rotation(anchor, kPi * 0.5);
    // The wrong composition maps anchor to origin + anchor, not origin.
    ASSERT_FALSE(wrongComposition.Apply(anchor).AlmostEquals(origin, 1e-12));
}

TEST(Measure2dTest, PointSegmentDistance)
{
    const Geometry::SCLineSegment2d segment(SCPoint2d{0.0, 0.0}, SCPoint2d{2.0, 0.0});
    GEOMETRY_TEST_ASSERT_NEAR(DistanceSquared(SCPoint2d{1.0, 2.0}, segment), 4.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(Distance(SCPoint2d{1.0, 2.0}, segment), 2.0, 1e-12);
}

TEST(Measure2dTest, PolylineBounds)
{
    const SCPolyline2d polyline({SCPoint2d{0.0, 0.0}, SCPoint2d{2.0, 0.0}, SCPoint2d{2.0, 2.0}, SCPoint2d{0.0, 2.0}},
                                SCPolylineClosure::Closed);
    GEOMETRY_TEST_ASSERT_BOX_NEAR(Geometry::Bounds(polyline),
                                  Geometry::SCBox2d::FromMinMax(SCPoint2d{0.0, 0.0}, SCPoint2d{2.0, 2.0}),
                                  1e-12);
}

TEST(Vector2Test, Normalized)
{
    GEOMETRY_TEST_ASSERT_VECTOR_NEAR((SCVector2d{3.0, 4.0}.Normalized()), (SCVector2d{0.6, 0.8}), 1e-12);
    GEOMETRY_TEST_ASSERT_VECTOR_NEAR((SCVector2d{0.0, 0.0}.Normalized()), (SCVector2d{0.0, 0.0}), 0.0);
}
