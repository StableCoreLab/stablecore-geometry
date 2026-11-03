#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include "Geometry.h"
#include "Support/Fixtures3d.h"
#include "../../../Source/Detail/NurbsBasis.h"

using Geometry::Intersect;
using Geometry::ProjectPointToCurve;
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

    TEST(NurbsNumericsTest, LocalSurfaceEvaluationMatchesFullBasisAtRepeatedKnots)
    {
        const std::vector<double> uKnots{0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 2.0};
        const std::vector<double> vKnots{0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0};
        std::vector<SCPoint3d> controls;
        std::vector<double> weights;
        for (int v = 0; v < 4; ++v)
        {
            for (int u = 0; u < 5; ++u)
            {
                controls.emplace_back(static_cast<double>(u), static_cast<double>(v),
                                      0.25 * u * v);
                weights.push_back(1.0 + 0.25 * ((u + v) % 3));
            }
        }
        const SCNurbsSurface surface(2, 2, 5, 4, controls, uKnots, vKnots, weights);
        ASSERT_TRUE(surface.IsValid());
        for (double u : {0.0, 0.4, 1.0, 1.6, 2.0})
        {
            for (double v : {0.0, 0.3, 1.0, 1.7, 2.0})
            {
                const auto bu = Geometry::Detail::EvaluateNurbsBasis(uKnots, 5, 2, u);
                const auto bv = Geometry::Detail::EvaluateNurbsBasis(vKnots, 4, 2, v);
                SCVector3d weightedPoint{}, weightedU{}, weightedV{};
                double weight = 0.0, weightU = 0.0, weightV = 0.0;
                for (std::size_t iv = 0; iv < 4; ++iv)
                {
                    for (std::size_t iu = 0; iu < 5; ++iu)
                    {
                        const std::size_t index = iv * 5 + iu;
                        const SCVector3d control{controls[index].x, controls[index].y, controls[index].z};
                        const double basis = weights[index] * bu.value[iu] * bv.value[iv];
                        const double basisU = weights[index] * bu.first[iu] * bv.value[iv];
                        const double basisV = weights[index] * bu.value[iu] * bv.first[iv];
                        weightedPoint = weightedPoint + control * basis;
                        weightedU = weightedU + control * basisU;
                        weightedV = weightedV + control * basisV;
                        weight += basis;
                        weightU += basisU;
                        weightV += basisV;
                    }
                }
                const SCVector3d expectedPoint = weightedPoint / weight;
                const SCVector3d expectedU = (weightedU - expectedPoint * weightU) / weight;
                const SCVector3d expectedV = (weightedV - expectedPoint * weightV) / weight;
                const auto eval = surface.Evaluate(u, v, 1);
                EXPECT_TRUE(eval.point.AlmostEquals(
                    SCPoint3d{expectedPoint.x, expectedPoint.y, expectedPoint.z}, 1e-11));
                EXPECT_TRUE(eval.derivativeU.AlmostEquals(expectedU, 1e-10));
                EXPECT_TRUE(eval.derivativeV.AlmostEquals(expectedV, 1e-10));
            }
        }
    }

    TEST(NurbsNumericsTest, SurfaceEvaluationValidityUsesRobustNormalLength)
    {
        Geometry::SCSurfaceEval3d evaluation;
        evaluation.derivativeU = SCVector3d{1.0, 0.0, 0.0};
        evaluation.derivativeV = SCVector3d{0.0, 1.0, 0.0};
        evaluation.normal = SCVector3d{1e200, 0.0, 0.0};
        evaluation.derivativeOrder = 1;

        EXPECT_FALSE(evaluation.IsValid(1e300));
        EXPECT_TRUE(evaluation.IsValid(1e199));

        evaluation.normal = SCVector3d{1e-200, 0.0, 0.0};
        EXPECT_TRUE(evaluation.IsValid(0.0));
        EXPECT_FALSE(evaluation.IsValid(1e-199));
    }

    TEST(NurbsNumericsTest, CurvePlaneIntersectionKeepsEndpointWithManyKnotSpans)
    {
        constexpr int spanCount = 200;
        std::vector<SCPoint3d> controlPoints;
        std::vector<double> knots{0.0, 0.0};
        for (int i = 0; i <= spanCount; ++i)
        {
            controlPoints.emplace_back(static_cast<double>(i), 0.0, i == 0 ? 0.0 : 1.0);
            if (i > 0 && i < spanCount)
            {
                knots.push_back(static_cast<double>(i));
            }
        }
        knots.push_back(static_cast<double>(spanCount));
        knots.push_back(static_cast<double>(spanCount));

        const SCNurbsCurve3d curve(1, controlPoints, knots);
        const Geometry::SCPlane plane = Geometry::SCPlane::FromPointAndNormal(
            SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0});
        ASSERT_TRUE(curve.IsValid());
        const auto intersection = Intersect(plane, curve);
        ASSERT_TRUE(intersection.intersects);
        EXPECT_DOUBLE_EQ(intersection.curveParameter, 0.0);
        EXPECT_TRUE(intersection.point.AlmostEquals(curve.PointAt(0.0), 0.0));
    }

    TEST(NurbsNumericsTest, CurvePlaneIntersectionCoversInteriorKnotWithManySpans)
    {
        constexpr int spanCount = 1000;
        std::vector<SCPoint3d> controlPoints;
        std::vector<double> knots{0.0, 0.0};
        for (int i = 0; i <= spanCount; ++i)
        {
            controlPoints.emplace_back(static_cast<double>(i), 0.0, i == 500 ? 0.0 : 1.0);
            if (i > 0 && i < spanCount)
            {
                knots.push_back(static_cast<double>(i));
            }
        }
        knots.push_back(static_cast<double>(spanCount));
        knots.push_back(static_cast<double>(spanCount));

        const SCNurbsCurve3d curve(1, controlPoints, knots);
        const Geometry::SCPlane plane = Geometry::SCPlane::FromPointAndNormal(
            SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0});
        ASSERT_TRUE(curve.IsValid());
        const auto intersection = Intersect(plane, curve);
        ASSERT_TRUE(intersection.intersects);
        EXPECT_DOUBLE_EQ(intersection.curveParameter, 500.0);
        EXPECT_TRUE(intersection.point.AlmostEquals(curve.PointAt(500.0), 0.0));
    }

    TEST(NurbsNumericsTest, LineCurveIntersectionCoversInteriorKnotWithManySpans)
    {
        constexpr int spanCount = 1000;
        std::vector<SCPoint3d> controlPoints;
        std::vector<double> knots{0.0, 0.0};
        for (int i = 0; i <= spanCount; ++i)
        {
            controlPoints.emplace_back(0.0, i == 500 ? 0.0 : 1.0, 0.0);
            if (i > 0 && i < spanCount)
            {
                knots.push_back(static_cast<double>(i));
            }
        }
        knots.push_back(static_cast<double>(spanCount));
        knots.push_back(static_cast<double>(spanCount));

        const SCNurbsCurve3d curve(1, controlPoints, knots);
        const SCLine3d line = SCLine3d::FromOriginAndDirection(
            SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, 0.0});
        ASSERT_TRUE(curve.IsValid());
        const auto intersection = Intersect(line, curve);
        ASSERT_TRUE(intersection.intersects);
        EXPECT_DOUBLE_EQ(intersection.curveParameter, 500.0);
    }

    TEST(NurbsNumericsTest, CurveProjectionFindsInteriorKnotWithManySpans)
    {
        constexpr int spanCount = 1000;
        std::vector<SCPoint3d> controlPoints;
        std::vector<double> knots{0.0, 0.0};
        for (int i = 0; i <= spanCount; ++i)
        {
            controlPoints.emplace_back(0.0, i == 501 ? 0.0 : 1.0, 0.0);
            if (i > 0 && i < spanCount)
            {
                knots.push_back(static_cast<double>(i));
            }
        }
        knots.push_back(static_cast<double>(spanCount));
        knots.push_back(static_cast<double>(spanCount));

        const SCNurbsCurve3d curve(1, controlPoints, knots);
        ASSERT_TRUE(curve.IsValid());
        const auto projection = ProjectPointToCurve(SCPoint3d{0.0, 0.0, 0.0}, curve);
        ASSERT_TRUE(projection.success);
        EXPECT_NEAR(projection.parameter, 501.0, 1e-8);
        EXPECT_NEAR(projection.distanceSquared, 0.0, 1e-12);
    }

    TEST(NurbsNumericsTest, CurveProjectionHandlesOverlappingSpanBounds)
    {
        constexpr int spanCount = 1000;
        const SCPoint3d corners[] = {{-1.0, -1.0, 0.0}, {-1.0, 1.0, 0.0},
                                     {1.0, 1.0, 0.0}, {1.0, -1.0, 0.0}};
        std::vector<SCPoint3d> controlPoints;
        for (int i = 0; i < spanCount + 2; ++i)
        {
            controlPoints.push_back(corners[i % 4]);
        }
        std::vector<double> knots{0.0, 0.0, 0.0};
        for (int i = 1; i < spanCount; ++i)
        {
            knots.push_back(static_cast<double>(i));
        }
        knots.insert(knots.end(), {static_cast<double>(spanCount), static_cast<double>(spanCount),
                                   static_cast<double>(spanCount)});

        const SCNurbsCurve3d curve(2, controlPoints, knots);
        ASSERT_TRUE(curve.IsValid());
        const auto projection = ProjectPointToCurve(SCPoint3d{0.0, 0.0, 0.0}, curve);
        ASSERT_TRUE(projection.success);
        EXPECT_LT(projection.distanceSquared, 0.8);
        EXPECT_TRUE(projection.point.AlmostEquals(curve.PointAt(projection.parameter), 1e-10));
    }

    TEST(NurbsNumericsTest, LocalBasisMatchesFullBasisAtRepeatedKnotsAndEndpoints)
    {
        const auto compare = [](const std::vector<double>& knots, int degree,
                                const std::vector<double>& parameters) {
            const std::size_t count = knots.size() - static_cast<std::size_t>(degree) - 1;
            for (double parameter : parameters)
            {
                const auto full = Geometry::Detail::EvaluateNurbsBasis(knots, count, degree, parameter);
                const auto local = Geometry::Detail::EvaluateLocalNurbsBasis(knots, count, degree, parameter);
                for (std::size_t index = 0; index < count; ++index)
                {
                    const bool active = index >= local.firstControlIndex &&
                                        index < local.firstControlIndex + local.value.size();
                    const std::size_t localIndex = active ? index - local.firstControlIndex : 0;
                    EXPECT_NEAR(full.value[index], active ? local.value[localIndex] : 0.0, 1e-12);
                    EXPECT_NEAR(full.first[index], active ? local.first[localIndex] : 0.0, 1e-12);
                    EXPECT_NEAR(full.second[index], active ? local.second[localIndex] : 0.0, 1e-12);
                }
            }
        };
        compare({0.0, 0.0, 1.0, 2.0, 3.0, 3.0}, 1, {-1.0, 0.0, 0.4, 1.0, 2.4, 3.0, 4.0});
        compare({0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 3.0, 3.0}, 2,
                {-1.0, 0.0, 0.4, 1.0, 2.4, 3.0, 4.0});
        compare({0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0, 1.0}, 3,
                {-1.0, 0.0, 0.25, 0.5, std::nextafter(0.5, 1.0), 0.75, 1.0, 2.0});
        compare({0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 3.0, 3.0, 3.0, 3.0}, 4,
                {-1.0, 0.0, 0.4, 1.0, 2.4, 3.0, 4.0});
    }

    TEST(NurbsNumericsTest, RationalManySpanCurveProjectsItsOwnPoints)
    {
        constexpr int spanCount = 200;
        std::vector<SCPoint3d> controlPoints;
        std::vector<double> weights;
        std::vector<double> knots{0.0, 0.0, 0.0};
        for (int i = 0; i < spanCount + 2; ++i)
        {
            controlPoints.emplace_back(static_cast<double>(i), std::sin(0.1 * i), 0.0);
            weights.push_back(1.0 + 0.25 * (i % 3));
            if (i > 0 && i < spanCount)
            {
                knots.push_back(static_cast<double>(i));
            }
        }
        knots.insert(knots.end(), {static_cast<double>(spanCount), static_cast<double>(spanCount),
                                   static_cast<double>(spanCount)});
        const SCNurbsCurve3d curve(2, controlPoints, knots, weights);
        ASSERT_TRUE(curve.IsValid());
        for (double parameter : {0.0, 0.37, 33.37, 97.13, 150.71, 199.63, 200.0})
        {
            const SCPoint3d point = curve.PointAt(parameter);
            const auto projection = ProjectPointToCurve(point, curve);
            ASSERT_TRUE(projection.success);
            EXPECT_LE(projection.distanceSquared, 1e-12) << parameter;
        }
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

}  // namespace

TEST(NurbsNumericsTest, PeriodicDeclarationIsIndependentOfGeometricClosure)
{
    for (const bool declaredPeriodic : {false, true})
    {
        const SCNurbsCurve3d open(1, {{0, 0, 0}, {1, 0, 0}}, {0, 0, 1, 1}, declaredPeriodic);
        const SCNurbsCurve3d closed(1, {{0, 0, 0}, {1, 0, 0}, {0, 0, 0}},
            {0, 0, 0.5, 1, 1}, declaredPeriodic);
        ASSERT_TRUE(open.IsValid());
        ASSERT_TRUE(closed.IsValid());
        EXPECT_FALSE(open.IsClosed());
        EXPECT_TRUE(closed.IsClosed());
        EXPECT_EQ(open.IsPeriodic(), declaredPeriodic);
        EXPECT_EQ(closed.IsPeriodic(), declaredPeriodic);
        const auto clone = closed.Clone();
        ASSERT_NE(clone, nullptr);
        EXPECT_EQ(clone->IsPeriodic(), declaredPeriodic);
        EXPECT_TRUE(clone->IsClosed());
    }
}

TEST(NurbsNumericsTest, NonClampedClosureUsesEvaluatedEndpoints)
{
    const SCNurbsCurve3d curve(2, {{-1, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, -1, 0}},
        {0, 1, 2, 3, 4, 5, 6});
    ASSERT_TRUE(curve.IsValid());
    EXPECT_FALSE(curve.ControlPoints().front().AlmostEquals(curve.ControlPoints().back()));
    EXPECT_TRUE(curve.PointAt(2.0).AlmostEquals({0, 0, 0}, 1e-12));
    EXPECT_TRUE(curve.PointAt(4.0).AlmostEquals({0, 0, 0}, 1e-12));
    EXPECT_TRUE(curve.IsClosed());
    EXPECT_FALSE(curve.IsPeriodic());
}

TEST(NurbsNumericsTest, PeriodicDeclarationKeepsFiniteOutOfDomainEvaluationClamped)
{
    for (const bool declaredPeriodic : {false, true})
    {
        const SCNurbsCurve3d curve(1, {{0, 0, 0}, {1, 0, 0}}, {0, 0, 1, 1}, declaredPeriodic);
        EXPECT_TRUE(curve.PointAt(-0.25).AlmostEquals({0, 0, 0}, 1e-12));
        EXPECT_TRUE(curve.PointAt(1.25).AlmostEquals({1, 0, 0}, 1e-12));
    }
}
