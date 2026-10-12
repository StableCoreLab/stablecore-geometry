#include <gtest/gtest.h>
#include <cmath>
#include <vector>

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

    TEST(NurbsNumericsTest, RationalQuarterCircleHasAnalyticEndpointAndMidpointDerivatives)
    {
        const double weight = std::sqrt(0.5);
        const SCNurbsCurve3d curve(2, {{1.0, 0.0, 0.0}, {1.0, 1.0, 0.0}, {0.0, 1.0, 0.0}},
                                   {0.0, 0.0, 0.0, 1.0, 1.0, 1.0}, {1.0, weight, 1.0});
        ASSERT_TRUE(curve.IsValid());
        EXPECT_TRUE(curve.PointAt(0.5).AlmostEquals(SCPoint3d{weight, weight, 0.0}, 1e-12));
        const auto middle = curve.Evaluate(0.5, 2);
        EXPECT_NEAR(middle.firstDerivative.x, -1.0 / (0.5 * (1.0 + weight)), 1e-12);
        EXPECT_NEAR(middle.firstDerivative.y, 1.0 / (0.5 * (1.0 + weight)), 1e-12);
        EXPECT_NEAR(middle.secondDerivative.x, middle.secondDerivative.y, 1e-12);
        EXPECT_LT(middle.secondDerivative.x, -1.0);
        EXPECT_TRUE(curve.Evaluate(0.0, 1).firstDerivative.AlmostEquals(
            SCVector3d{0.0, 2.0 * weight, 0.0}, 1e-12));

        const SCNurbsCurve3d invalid(2, curve.ControlPoints(), curve.Knots(), {1.0, 0.0, 1.0});
        EXPECT_FALSE(invalid.IsValid());
    }

    TEST(NurbsNumericsTest, RationalSurfaceUsesWeightedAnalyticPartials)
    {
        const SCNurbsSurface surface(1, 1, 2, 2,
            {{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {0.0, 2.0, 0.0}, {2.0, 2.0, 0.0}},
            {0.0, 0.0, 1.0, 1.0}, {0.0, 0.0, 1.0, 1.0}, {1.0, 1.0, 1.0, 2.0});
        ASSERT_TRUE(surface.IsValid());
        const auto evaluation = surface.Evaluate(0.5, 0.5);
        EXPECT_TRUE(evaluation.point.AlmostEquals(SCPoint3d{1.2, 1.2, 0.0}, 1e-12));
        EXPECT_TRUE(evaluation.derivativeU.AlmostEquals(SCVector3d{1.92, 0.32, 0.0}, 1e-12));
        EXPECT_TRUE(evaluation.derivativeV.AlmostEquals(SCVector3d{0.32, 1.92, 0.0}, 1e-12));
    }

    TEST(NurbsNumericsTest, LargeCommonWeightsKeepFiniteCurveAndSurfaceResults)
    {
        const std::vector<double> knots{0.0, 0.0, 1.0, 1.0};
        for (const double weight : {1e308, 1e-308})
        {
            const SCNurbsCurve3d curve(1, {{2.0, 0.0, 0.0}, {2.0, 0.0, 0.0}},
                                       knots, {weight, weight});
            ASSERT_TRUE(curve.IsValid());
            EXPECT_TRUE(curve.PointAt(0.5).AlmostEquals(SCPoint3d{2.0, 0.0, 0.0}, 1e-12));
            const auto curveEval = curve.Evaluate(0.5, 2);
            EXPECT_TRUE(curveEval.IsValid());
            EXPECT_TRUE(curveEval.point.AlmostEquals(SCPoint3d{2.0, 0.0, 0.0}, 1e-12));
            EXPECT_TRUE(curveEval.firstDerivative.AlmostEquals(SCVector3d{}, 1e-12));
            EXPECT_TRUE(curveEval.secondDerivative.AlmostEquals(SCVector3d{}, 1e-12));

            const SCNurbsSurface surface(1, 1, 2, 2,
                {{2.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {2.0, 0.0, 0.0}},
                knots, knots, {weight, weight, weight, weight});
            ASSERT_TRUE(surface.IsValid());
            EXPECT_TRUE(surface.PointAt(0.5, 0.5).AlmostEquals(SCPoint3d{2.0, 0.0, 0.0}, 1e-12));
            const auto surfaceEval = surface.Evaluate(0.5, 0.5);
            EXPECT_TRUE(surfaceEval.IsValid());
            EXPECT_TRUE(surfaceEval.point.AlmostEquals(SCPoint3d{2.0, 0.0, 0.0}, 1e-12));
            EXPECT_TRUE(surfaceEval.derivativeU.AlmostEquals(SCVector3d{}, 1e-12));
            EXPECT_TRUE(surfaceEval.derivativeV.AlmostEquals(SCVector3d{}, 1e-12));
        }
        const SCNurbsCurve3d nonConstant(1, {{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}},
                                        knots, {1e308, 1e308});
        EXPECT_TRUE(nonConstant.PointAt(0.5).AlmostEquals(SCPoint3d{1.0, 0.0, 0.0}, 1e-12));
        EXPECT_TRUE(nonConstant.Evaluate(0.5, 2).firstDerivative.AlmostEquals(
            SCVector3d{2.0, 0.0, 0.0}, 1e-12));
    }

    TEST(NurbsNumericsTest, EndpointIgnoresInactiveExtremeWeightsAndCoordinates)
    {
        const std::vector<double> knots{0.0, 0.0, 1.0, 1.0};
        const SCNurbsCurve3d weightedCurve(1, {{2.0, 0.0, 0.0}, {3.0, 0.0, 0.0}},
                                           knots, {1e-308, 1e308});
        EXPECT_EQ(weightedCurve.PointAt(0.0).x, 2.0);
        EXPECT_EQ(weightedCurve.Evaluate(0.0, 0).point.x, 2.0);

        const SCNurbsCurve3d constantCurve(1, {{2.0, 0.0, 0.0}, {2.0, 0.0, 0.0}},
                                           knots, {1e-308, 1e308});
        const auto constantCurveEval = constantCurve.Evaluate(0.0, 2);
        EXPECT_TRUE(constantCurveEval.IsValid());
        EXPECT_EQ(constantCurveEval.point.x, 2.0);
        EXPECT_EQ(constantCurveEval.firstDerivative.x, 0.0);
        EXPECT_EQ(constantCurveEval.secondDerivative.x, 0.0);

        const SCNurbsCurve3d scaledCurve(1, {{1e-308, 0.0, 0.0}, {1e308, 0.0, 0.0}},
                                         knots, {1.0, 1.0});
        EXPECT_EQ(scaledCurve.PointAt(0.0).x, 1e-308);
        EXPECT_EQ(scaledCurve.Evaluate(0.0, 0).point.x, 1e-308);

        const SCNurbsSurface weightedSurface(1, 1, 2, 2,
            {{2.0, 0.0, 0.0}, {3.0, 0.0, 0.0}, {3.0, 0.0, 0.0}, {3.0, 0.0, 0.0}},
            knots, knots, {1e-308, 1e308, 1e308, 1e308});
        EXPECT_EQ(weightedSurface.PointAt(0.0, 0.0).x, 2.0);
        EXPECT_EQ(weightedSurface.Evaluate(0.0, 0.0, 0).point.x, 2.0);

        const SCNurbsSurface constantSurface(1, 1, 2, 2,
            {{2.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {2.0, 0.0, 0.0}},
            knots, knots, {1e-308, 1e308, 1e308, 1e308});
        const auto constantSurfaceEval = constantSurface.Evaluate(0.0, 0.0);
        EXPECT_TRUE(constantSurfaceEval.IsValid());
        EXPECT_EQ(constantSurfaceEval.point.x, 2.0);
        EXPECT_EQ(constantSurfaceEval.derivativeU.x, 0.0);
        EXPECT_EQ(constantSurfaceEval.derivativeV.x, 0.0);

        const SCNurbsSurface scaledSurface(1, 1, 2, 2,
            {{1e-308, 0.0, 0.0}, {1e308, 0.0, 0.0},
             {1e308, 0.0, 0.0}, {1e308, 0.0, 0.0}},
            knots, knots, {1.0, 1.0, 1.0, 1.0});
        EXPECT_EQ(scaledSurface.PointAt(0.0, 0.0).x, 1e-308);
        EXPECT_EQ(scaledSurface.Evaluate(0.0, 0.0, 0).point.x, 1e-308);
    }

    TEST(NurbsNumericsTest, FiniteEndpointDerivativeSurvivesExtremeWeightRatio)
    {
        const SCNurbsCurve3d curve(1, {{0.0, 0.0, 0.0}, {1e-308, 0.0, 0.0}},
                                   {0.0, 0.0, 1.0, 1.0}, {1e-308, 1e308});
        const auto evaluation = curve.Evaluate(0.0, 1);
        ASSERT_TRUE(evaluation.IsValid());
        EXPECT_EQ(evaluation.point.x, 0.0);
        EXPECT_NEAR(evaluation.firstDerivative.x, 1e308, 1e294);
        EXPECT_EQ(evaluation.firstDerivative.y, 0.0);
        EXPECT_EQ(evaluation.firstDerivative.z, 0.0);
    }

    TEST(NurbsNumericsTest, RepeatedDomainEndUsesLastNonzeroKnotSpan)
    {
        const std::vector<double> knotsU{0.0, 0.0, 1.0, 1.0, 2.0};
        const std::vector<double> knotsV{0.0, 0.0, 1.0, 1.0};
        const SCNurbsCurve3d curve(1, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}}, knotsU);
        ASSERT_TRUE(curve.IsValid());
        EXPECT_DOUBLE_EQ(curve.PointAt(1.0).x, 1.0);
        EXPECT_DOUBLE_EQ(curve.Evaluate(1.0, 1).firstDerivative.x, 1.0);

        const SCNurbsSurface surface(1, 1, 3, 2,
            {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0},
             {0.0, 1.0, 0.0}, {1.0, 1.0, 0.0}, {2.0, 1.0, 0.0}}, knotsU, knotsV);
        ASSERT_TRUE(surface.IsValid());
        EXPECT_TRUE(surface.PointAt(1.0, 0.5).AlmostEquals(SCPoint3d{1.0, 0.5, 0.0}, 1e-12));
        EXPECT_DOUBLE_EQ(surface.Evaluate(1.0, 0.5).derivativeU.x, 1.0);
    }

    TEST(NurbsNumericsTest, FiniteEndpointDerivativeSurvivesScaledWeightUnderflow)
    {
        const SCNurbsCurve3d curve(1, {{1e308, 0.0, 0.0}, {0.0, 0.0, 0.0}},
                                   {0.0, 0.0, 1.0, 1.0}, {1e308, 1e-308});
        ASSERT_TRUE(curve.IsValid());
        const auto evaluation = curve.Evaluate(0.0, 1);
        ASSERT_TRUE(evaluation.IsValid());
        EXPECT_DOUBLE_EQ(evaluation.point.x, 1e308);
        EXPECT_NEAR(evaluation.firstDerivative.x, -1e-308, 1e-320);
    }

    TEST(NurbsNumericsTest, FiniteDerivativeSurvivesWeightedSumCancellation)
    {
        const std::vector<double> knots{0.0, 0.0, 1.0, 1.0};
        const SCNurbsCurve3d curve(1, {{1e308, 0.0, 0.0}, {0.0, 0.0, 0.0}},
                                   knots, {1e308, 1.0});
        ASSERT_TRUE(curve.IsValid());
        const auto curveEval = curve.Evaluate(0.0, 1);
        ASSERT_TRUE(curveEval.IsValid());
        EXPECT_NEAR(curveEval.firstDerivative.x, -1.0, 1e-12);

        const SCNurbsSurface surface(1, 1, 2, 2,
            {{1e308, 0.0, 0.0}, {0.0, 0.0, 0.0},
             {1e308, 1.0, 0.0}, {0.0, 1.0, 0.0}},
            knots, knots, {1e308, 1.0, 1e308, 1.0});
        ASSERT_TRUE(surface.IsValid());
        const auto surfaceEval = surface.Evaluate(0.0, 0.5);
        ASSERT_TRUE(surfaceEval.IsValid());
        EXPECT_NEAR(surfaceEval.derivativeU.x, -1.0, 1e-12);
    }

    TEST(NurbsNumericsTest, FiniteSecondDerivativeSurvivesExtremeEndpointWeights)
    {
        const SCNurbsCurve3d curve(2,
            {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {1e-309, 0.0, 0.0}},
            {0.0, 0.0, 0.0, 1.0, 1.0, 1.0}, {1e-308, 1e308, 1e308});
        ASSERT_TRUE(curve.IsValid());
        const auto evaluation = curve.Evaluate(0.0, 2);
        ASSERT_TRUE(evaluation.IsValid());
        EXPECT_DOUBLE_EQ(evaluation.firstDerivative.x, 0.0);
        EXPECT_NEAR(evaluation.secondDerivative.x, 2e307, 1e294);
    }

    TEST(NurbsNumericsTest, FiniteSecondDerivativeSurvivesWeightedSumCancellation)
    {
        const SCNurbsCurve3d curve(2,
            {{1e308, 0.0, 0.0}, {0.0, 0.0, 0.0}, {1e308, 0.0, 0.0}},
            {0.0, 0.0, 0.0, 1.0, 1.0, 1.0}, {1e308, 1.0, 1e308});
        ASSERT_TRUE(curve.IsValid());
        const auto evaluation = curve.Evaluate(0.0, 2);
        ASSERT_TRUE(evaluation.IsValid());
        EXPECT_NEAR(evaluation.firstDerivative.x, -2.0, 1e-12);
        EXPECT_NEAR(evaluation.secondDerivative.x, -4.0, 1e-12);
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
