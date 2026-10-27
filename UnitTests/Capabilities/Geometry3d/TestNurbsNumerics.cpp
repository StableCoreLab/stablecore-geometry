#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <numbers>
#include <vector>

#include "Geometry.h"
#include "Support/Fixtures3d.h"
#include "../../../Source/Detail/NurbsBasis.h"
#include "../../../Source/Core/Detail/SurfaceProjectionSolver.h"

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
    TEST(NurbsNumericsTest, ExtremeKnotSpanKeepsActualPositionsAndCacheKeysConsistent)
    {
        const double maximum=std::numeric_limits<double>::max();
        const std::vector<double> knots{-maximum,-maximum,maximum,maximum};
        const SCNurbsSurface surface(1,1,2,2,{{0,0,1},{2,0,1},{0,2,1},{2,2,1}},knots,{0,0,1,1});
        ASSERT_TRUE(surface.IsValid());
        Geometry::Detail::SurfaceSearchBudget cache;
        cache.PreparePositionCache(surface);
        for (const auto [u,x]:{std::pair{-maximum,0.0},std::pair{0.0,1.0},std::pair{maximum,2.0}}) {
            const auto actual=surface.PointAt(u,0.5);
            ASSERT_TRUE(actual.IsValid());
            EXPECT_DOUBLE_EQ(actual.x,x);
            EXPECT_DOUBLE_EQ(actual.y,1);
            EXPECT_DOUBLE_EQ(actual.z,1);
            EXPECT_EQ(cache.Position(surface,u,0.5),actual);
        }
        EXPECT_EQ(cache.statistics.positionEvaluations,3u);
        EXPECT_EQ(cache.Position(surface,0,0.5),surface.PointAt(0,0.5));
        EXPECT_EQ(cache.statistics.positionEvaluations,3u);
        const SCNurbsCurve3d curve(1,{{0,0,0},{2,0,0}},knots);
        ASSERT_TRUE(curve.IsValid());
        const auto eval=curve.Evaluate(0,1);
        ASSERT_TRUE(eval.point.IsValid());
        EXPECT_DOUBLE_EQ(eval.point.x,1);
        EXPECT_GT(eval.firstDerivative.x,0);
        EXPECT_TRUE(std::isfinite(eval.firstDerivative.x));
        EXPECT_FALSE(surface.PointAt(std::numeric_limits<double>::infinity(),0.5).IsValid());
        EXPECT_FALSE(curve.PointAt(std::numeric_limits<double>::infinity()).IsValid());
        EXPECT_FALSE(SCNurbsSurface{}.PointAt(0,0).IsValid());
        EXPECT_FALSE(SCNurbsCurve3d{}.PointAt(0).IsValid());
    }

    class PointOnlyPlaneTouchCurve final : public Geometry::ISCCurve3d
    {
    public:
        explicit PointOnlyPlaneTouchCurve(bool invalidFirstOrder, double oscillations = 1.0,
                                          bool mismatchedZeroOrder = false)
            : invalidFirstOrder_(invalidFirstOrder), oscillations_(oscillations),
              mismatchedZeroOrder_(mismatchedZeroOrder)
        {
        }

        [[nodiscard]] bool IsValid(const Geometry::SCGeometryTolerance3d& = {}) const override { return true; }
        [[nodiscard]] Geometry::SCIntervald ParameterRange() const override { return {0.0, 1.0}; }
        [[nodiscard]] bool IsClosed(const Geometry::SCGeometryTolerance3d& = {}) const override { return false; }
        [[nodiscard]] bool IsPeriodic(const Geometry::SCGeometryTolerance3d& = {}) const override { return false; }
        [[nodiscard]] SCPoint3d PointAt(double parameter) const override
        {
            const double height = std::cos(2.0 * std::numbers::pi * oscillations_ * parameter);
            return {parameter, 0.0, height * height};
        }
        [[nodiscard]] Geometry::SCCurveEval3d Evaluate(double parameter, int derivativeOrder = 1) const override
        {
            Geometry::SCCurveEval3d eval{};
            eval.point = PointAt(parameter);
            if (mismatchedZeroOrder_ && derivativeOrder == 0)
            {
                eval.point.z += 1.0;
            }
            if (invalidFirstOrder_ && derivativeOrder > 0)
            {
                eval.derivativeOrder = -1;
            }
            return eval;
        }
        [[nodiscard]] Geometry::SCBox3d Bounds() const override
        {
            return Geometry::SCBox3d::FromMinMax({0.0, 0.0, 0.0}, {1.0, 0.0, 1.0});
        }
        [[nodiscard]] std::unique_ptr<Geometry::ISCCurve3d> Clone() const override
        {
            return std::make_unique<PointOnlyPlaneTouchCurve>(*this);
        }

    private:
        bool invalidFirstOrder_;
        double oscillations_;
        bool mismatchedZeroOrder_;
    };

    class PointOnlyPatchSurface final : public Geometry::ISCSurface
    {
    public:
        [[nodiscard]] bool IsValid(const Geometry::SCGeometryTolerance3d& = {}) const override { return true; }
        [[nodiscard]] Geometry::SCIntervald URange() const override { return {0.0, 1.0}; }
        [[nodiscard]] Geometry::SCIntervald VRange() const override { return {0.0, 1.0}; }
        [[nodiscard]] SCPoint3d PointAt(double u, double v) const override { return {u, v, 0.0}; }
        [[nodiscard]] Geometry::SCSurfaceEval3d Evaluate(double u, double v, int = 1) const override
        {
            Geometry::SCSurfaceEval3d eval{};
            eval.point = {u, v, 1.0};
            eval.derivativeOrder = -1;
            return eval;
        }
        [[nodiscard]] Geometry::SCBox3d Bounds() const override
        {
            return Geometry::SCBox3d::FromMinMax({0.0, 0.0, 0.0}, {1.0, 1.0, 0.0});
        }
        [[nodiscard]] std::unique_ptr<Geometry::ISCSurface> Clone() const override
        {
            return std::make_unique<PointOnlyPatchSurface>(*this);
        }
    };

    class MisleadingCurveEval final : public Geometry::ISCCurve3d
    {
    public:
        [[nodiscard]] bool IsValid(const Geometry::SCGeometryTolerance3d& = {}) const override { return true; }
        [[nodiscard]] Geometry::SCIntervald ParameterRange() const override { return {0.0, 1.0}; }
        [[nodiscard]] bool IsClosed(const Geometry::SCGeometryTolerance3d& = {}) const override { return false; }
        [[nodiscard]] bool IsPeriodic(const Geometry::SCGeometryTolerance3d& = {}) const override { return false; }
        [[nodiscard]] SCPoint3d PointAt(double parameter) const override { return {parameter, 0.0, 1.0}; }
        [[nodiscard]] Geometry::SCCurveEval3d Evaluate(double parameter, int = 1) const override
        {
            Geometry::SCCurveEval3d eval{};
            eval.point = {parameter, 0.0, 0.0};
            eval.firstDerivative = {1.0, 0.0, 0.0};
            eval.derivativeOrder = 1;
            return eval;
        }
        [[nodiscard]] Geometry::SCBox3d Bounds() const override
        {
            return Geometry::SCBox3d::FromMinMax({0.0, 0.0, 1.0}, {1.0, 0.0, 1.0});
        }
        [[nodiscard]] std::unique_ptr<Geometry::ISCCurve3d> Clone() const override
        {
            return std::make_unique<MisleadingCurveEval>(*this);
        }
    };

    TEST(NurbsNumericsTest, PointOnlyCurvePlaneIntersectionFindsQuarterTouch)
    {
        const Geometry::SCPlane plane = Geometry::SCPlane::FromPointAndNormal(
            SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0});
        for (const bool invalidFirstOrder : {false, true})
        {
            SCOPED_TRACE(invalidFirstOrder);
            const PointOnlyPlaneTouchCurve curve(invalidFirstOrder);
            const auto intersection = Intersect(plane, curve);
            ASSERT_TRUE(intersection.intersects);
            EXPECT_NEAR(intersection.curveParameter, 0.25, 1e-12);
            EXPECT_TRUE(intersection.point.AlmostEquals(curve.PointAt(0.25), 1e-12));
        }
    }

    TEST(NurbsNumericsTest, PointOnlyCurvePlaneIntersectionKeepsFullDomainSamples)
    {
        const Geometry::SCPlane plane = Geometry::SCPlane::FromPointAndNormal(
            SCPoint3d{}, SCVector3d{0.0, 0.0, 1.0});
        for (const bool invalidFirstOrder : {false, true})
        {
            const PointOnlyPlaneTouchCurve curve(invalidFirstOrder, 4.0, true);
            const auto hit = Intersect(plane, curve);
            ASSERT_TRUE(hit.intersects);
            EXPECT_NEAR(hit.curveParameter, 1.0 / 16.0, 1e-12);
            EXPECT_TRUE(hit.point.AlmostEquals(curve.PointAt(hit.curveParameter), 1e-12));
        }
    }

    TEST(NurbsNumericsTest, PointOnlyCurveProjectionUsesPointAt)
    {
        const PointOnlyPlaneTouchCurve curve(true, 1.0, true);
        const SCPoint3d point = curve.PointAt(0.25);
        const auto projection = ProjectPointToCurve(point, curve);
        ASSERT_TRUE(projection.success);
        EXPECT_TRUE(projection.point.AlmostEquals(curve.PointAt(projection.parameter), 1e-12));
        EXPECT_NEAR(projection.distanceSquared, 0.0, 1e-12);
    }

    TEST(NurbsNumericsTest, CurvePlaneIntersectionRejectsMismatchedEvaluationPoint)
    {
        const Geometry::SCPlane plane = Geometry::SCPlane::FromPointAndNormal(
            SCPoint3d{}, SCVector3d{0.0, 0.0, 1.0});
        EXPECT_FALSE(Intersect(plane, MisleadingCurveEval{}).intersects);
    }

    TEST(NurbsNumericsTest, PointOnlySurfaceProjectionUsesPointAt)
    {
        const PointOnlyPatchSurface surface;
        const auto projection = ProjectPointToSurface(SCPoint3d{0.25, 0.75, 0.1}, surface);
        ASSERT_TRUE(projection.success);
        EXPECT_TRUE(projection.point.AlmostEquals(surface.PointAt(projection.u, projection.v), 1e-12));
        EXPECT_NEAR(projection.u, 0.25, 1e-8);
        EXPECT_NEAR(projection.v, 0.75, 1e-8);
        EXPECT_NEAR(projection.distanceSquared, 0.01, 1e-10);
    }

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
