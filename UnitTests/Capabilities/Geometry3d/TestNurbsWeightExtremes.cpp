#include <gtest/gtest.h>
#include <vector>

#include "Geometry.h"

using Geometry::SCNurbsCurve3d;
using Geometry::SCNurbsSurface;
using Geometry::SCPoint3d;
using Geometry::SCVector3d;

namespace
{
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
            EXPECT_FALSE(surfaceEval.IsValid());
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
        EXPECT_FALSE(constantSurfaceEval.IsValid());
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

}  // namespace
