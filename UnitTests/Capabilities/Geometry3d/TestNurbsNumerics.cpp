#include <gtest/gtest.h>

#include "Geometry.h"
#include "Support/Fixtures3d.h"

using Geometry::Intersect;
using Geometry::ProjectPointToCurve;
using Geometry::ProjectPointToSurface;
using Geometry::SCLine3d;
using Geometry::SCNurbsCurve3d;
using Geometry::SCNurbsSurface;
using Geometry::SCPoint3d;
using Geometry::SCVector3d;

namespace
{
    TEST(NurbsNumericsTest, LinearCurvePreservesEndpointAndDerivativeBaseline)
    {
        const SCNurbsCurve3d curve(1, {SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{2.0, 4.0, -2.0}}, {0.0, 0.0, 1.0, 1.0});

        ASSERT_TRUE(curve.IsValid());
        EXPECT_TRUE(curve.PointAt(-1.0).AlmostEquals(SCPoint3d{0.0, 0.0, 0.0}, 1e-12));
        EXPECT_TRUE(curve.PointAt(0.5).AlmostEquals(SCPoint3d{1.0, 2.0, -1.0}, 1e-12));
        EXPECT_TRUE(curve.PointAt(2.0).AlmostEquals(SCPoint3d{2.0, 4.0, -2.0}, 1e-12));

        const auto evaluation = curve.Evaluate(0.5, 2);
        ASSERT_TRUE(evaluation.IsValid());
        EXPECT_EQ(evaluation.derivativeOrder, 2);
        EXPECT_TRUE(evaluation.firstDerivative.AlmostEquals(SCVector3d{2.0, 4.0, -2.0}, 1e-12));
        EXPECT_TRUE(evaluation.secondDerivative.AlmostEquals(SCVector3d{}, 1e-10));

        const auto projection = ProjectPointToCurve(SCPoint3d{3.0, 1.0, -1.0}, curve);
        ASSERT_TRUE(projection.success);
        EXPECT_NEAR(projection.parameter, 0.5, 1e-8);
        EXPECT_TRUE(projection.point.AlmostEquals(SCPoint3d{1.0, 2.0, -1.0}, 1e-8));

        const auto intersection =
            Intersect(SCLine3d::FromOriginAndDirection(SCPoint3d{1.0, 2.0, 5.0}, SCVector3d{0.0, 0.0, -1.0}), curve);
        ASSERT_TRUE(intersection.intersects);
        EXPECT_NEAR(intersection.curveParameter, 0.5, 1e-4);
        EXPECT_TRUE(intersection.point.AlmostEquals(SCPoint3d{1.0, 2.0, -1.0}, 1e-4));
    }

    TEST(NurbsNumericsTest, BilinearSurfacePreservesTensorProductAndDerivativeBaseline)
    {
        const SCNurbsSurface surface = Geometry::Test::BuildUnitNurbsSurface();

        ASSERT_TRUE(surface.IsValid());
        EXPECT_TRUE(surface.PointAt(-1.0, 2.0).AlmostEquals(SCPoint3d{0.0, 2.0, 0.0}, 1e-12));
        EXPECT_TRUE(surface.PointAt(0.25, 0.75).AlmostEquals(SCPoint3d{0.5, 1.5, 0.0}, 1e-12));
        EXPECT_TRUE(surface.PointAt(2.0, -1.0).AlmostEquals(SCPoint3d{2.0, 0.0, 0.0}, 1e-12));

        const auto evaluation = surface.Evaluate(0.5, 0.5, 1);
        ASSERT_TRUE(evaluation.IsValid());
        EXPECT_EQ(evaluation.derivativeOrder, 1);
        EXPECT_TRUE(evaluation.derivativeU.AlmostEquals(SCVector3d{2.0, 0.0, 0.0}, 1e-12));
        EXPECT_TRUE(evaluation.derivativeV.AlmostEquals(SCVector3d{0.0, 2.0, 0.0}, 1e-12));
        EXPECT_TRUE(evaluation.normal.AlmostEquals(SCVector3d{0.0, 0.0, 4.0}, 1e-12));
    }

    TEST(NurbsNumericsTest, PlanarSurfaceProjectionAndIntersectionMeetNumericalBaseline)
    {
        const SCNurbsSurface surface = Geometry::Test::BuildUnitNurbsSurface();
        const SCPoint3d source{0.6, 1.4, 1.0};

        const auto projection = ProjectPointToSurface(source, surface);
        ASSERT_TRUE(projection.success);
        ASSERT_TRUE(projection.IsValid());
        EXPECT_NEAR(projection.u, 0.3, 1e-4);
        EXPECT_NEAR(projection.v, 0.7, 1e-4);
        EXPECT_TRUE(projection.point.AlmostEquals(SCPoint3d{0.6, 1.4, 0.0}, 1e-4));
        EXPECT_NEAR(projection.distanceSquared, 1.0, 1e-7);

        const auto intersection =
            Intersect(SCLine3d::FromOriginAndDirection(source, SCVector3d{0.0, 0.0, -1.0}), surface);
        ASSERT_TRUE(intersection.intersects);
        ASSERT_TRUE(intersection.IsValid());
        EXPECT_NEAR(intersection.lineParameter, 1.0, 1e-8);
        EXPECT_NEAR(intersection.u, projection.u, 1e-4);
        EXPECT_NEAR(intersection.v, projection.v, 1e-4);
        EXPECT_TRUE(intersection.point.AlmostEquals(projection.point, 1e-4));
    }

    TEST(NurbsNumericsTest, InvalidNurbsInputsFailProjectionAndIntersectionWithoutPartialResults)
    {
        const SCNurbsCurve3d invalidCurve(1, {SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{1.0, 0.0, 0.0}}, {0.0, 0.0, 1.0});
        const SCNurbsSurface invalidSurface(
            1,
            1,
            2,
            2,
            {SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{2.0, 0.0, 0.0}, SCPoint3d{0.0, 2.0, 0.0}, SCPoint3d{2.0, 2.0, 0.0}},
            {0.0, 1.0, 0.0, 1.0},
            {0.0, 0.0, 1.0, 1.0});

        ASSERT_FALSE(invalidCurve.IsValid());
        ASSERT_FALSE(invalidSurface.IsValid());
        EXPECT_FALSE(ProjectPointToCurve(SCPoint3d{0.5, 1.0, 0.0}, invalidCurve).success);
        EXPECT_FALSE(ProjectPointToSurface(SCPoint3d{0.5, 1.0, 0.0}, invalidSurface).success);

        const auto intersection = Intersect(
            SCLine3d::FromOriginAndDirection(SCPoint3d{0.5, 1.0, 1.0}, SCVector3d{0.0, 0.0, -1.0}), invalidSurface);
        EXPECT_FALSE(intersection.intersects);
        EXPECT_FALSE(intersection.isParallel);
        EXPECT_FALSE(intersection.liesOnSurface);
    }
}  // namespace
