#include <gtest/gtest.h>
#include <cmath>
#include <chrono>
#include <iostream>
#include <memory>
#include <numbers>
#include <vector>

#include "Geometry.h"
#include "Geometry3d/SCOffsetSurface.h"
#include "Geometry3d/SCPlaneSurface.h"
#include "Support/Fixtures3d.h"
#include "../../../Source/Detail/SurfaceProjectionSolver.h"

using Geometry::Intersect;
using Geometry::ProjectPointToCurve;
using Geometry::ProjectPointToCurveOnSurface;
using Geometry::ProjectPointToSurface;
using Geometry::SCCurveOnSurface;
using Geometry::SCLine3d;
using Geometry::SCNurbsCurve3d;
using Geometry::SCNurbsSurface;
using Geometry::SCPoint2d;
using Geometry::SCPoint3d;
using Geometry::SCPolyline2d;
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

    TEST(NurbsNumericsTest, NearlyPlanarNurbsDoesNotReportParallelLineAsCoincident)
    {
        constexpr double bend = 2e-10;
        const SCNurbsSurface surface(1, 1, 2, 2,
            {{0.0, 0.0, bend}, {1.0, 0.0, -bend},
             {0.0, 1.0, -bend}, {1.0, 1.0, bend}},
            {0.0, 0.0, 1.0, 1.0}, {0.0, 0.0, 1.0, 1.0});
        const SCLine3d line = SCLine3d::FromOriginAndDirection(
            SCPoint3d{0.0, 0.75, bend * 0.25}, SCVector3d{1.0, 0.0, 0.0});
        ASSERT_TRUE(surface.IsValid());
        ASSERT_TRUE(line.IsValid());

        const auto intersection = Intersect(line, surface);
        EXPECT_FALSE(intersection.liesOnSurface);
        if (intersection.intersects)
        {
            EXPECT_TRUE(intersection.point.IsValid());
            EXPECT_TRUE(surface.PointAt(intersection.u, intersection.v).AlmostEquals(
                intersection.point, 1e-12));
        }
    }

    TEST(NurbsNumericsTest, CurveOnSurfaceProjectionUsesActualUvSegmentImage)
    {
        auto surface = std::make_shared<SCNurbsSurface>(
            1, 1, 2, 2,
            std::vector<SCPoint3d>{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
                                   {0.0, 1.0, 0.0}, {1.0, 1.0, 4.0}},
            std::vector<double>{0.0, 0.0, 1.0, 1.0},
            std::vector<double>{0.0, 0.0, 1.0, 1.0});
        const SCCurveOnSurface curve(
            surface, SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{1.0, 1.0}}));
        ASSERT_TRUE(curve.IsValid());
        const SCPoint3d expected = surface->PointAt(0.5, 0.5);
        const double actualLength=Geometry::Length(curve);
        const double expectedLength=0.5*std::sqrt(66.0)+0.125*std::asinh(4*std::sqrt(2.0));
        EXPECT_NEAR(actualLength,expectedLength,1e-6);
        EXPECT_GT(actualLength,Geometry::Distance(curve.PointAt(0),curve.PointAt(1)));

        const auto projection = ProjectPointToCurveOnSurface(expected, curve);
        ASSERT_TRUE(projection.success);
        EXPECT_NEAR(projection.segmentParameter, 0.5, 1e-12);
        EXPECT_TRUE(projection.point.AlmostEquals(expected, 1e-12));
        EXPECT_TRUE(projection.point.AlmostEquals(
            surface->PointAt(projection.uv.x, projection.uv.y), 1e-12));
        EXPECT_DOUBLE_EQ(projection.distanceSquared, 0.0);

        const Geometry::SCPlane plane = Geometry::SCPlane::FromPointAndNormal(
            SCPoint3d{0.0, 0.0, 1.0}, SCVector3d{0.0, 0.0, 1.0});
        const auto intersection = Intersect(plane, curve);
        ASSERT_TRUE(intersection.intersects);
        EXPECT_NEAR(intersection.segmentParameter, 0.5, 1e-12);
        EXPECT_TRUE(intersection.point.AlmostEquals(expected, 1e-12));
        EXPECT_TRUE(intersection.point.AlmostEquals(
            surface->PointAt(intersection.uv.x, intersection.uv.y), 1e-12));

        const SCLine3d line = SCLine3d::FromOriginAndDirection(
            SCPoint3d{0.5, 0.5, 3.0}, SCVector3d{0.0, 0.0, -1.0});
        const auto lineIntersection = Intersect(line, curve);
        ASSERT_TRUE(lineIntersection.intersects);
        EXPECT_NEAR(lineIntersection.segmentParameter, 0.5, 1e-12);
        EXPECT_TRUE(lineIntersection.point.AlmostEquals(expected, 1e-12));
        EXPECT_TRUE(lineIntersection.point.AlmostEquals(
            surface->PointAt(lineIntersection.uv.x, lineIntersection.uv.y), 1e-12));

        constexpr double nearStartParameter = 1.0 / 128.0;
        const SCPoint3d nearStart = surface->PointAt(nearStartParameter, nearStartParameter);
        const auto nearStartProjection = ProjectPointToCurveOnSurface(nearStart, curve);
        ASSERT_TRUE(nearStartProjection.success);
        EXPECT_NEAR(nearStartProjection.segmentParameter, nearStartParameter, 1e-8);
        EXPECT_TRUE(nearStartProjection.point.AlmostEquals(nearStart, 1e-9));

        const SCLine3d nearStartLine = SCLine3d::FromOriginAndDirection(
            SCPoint3d{nearStart.x, nearStart.y, 1.0}, SCVector3d{0.0, 0.0, -1.0});
        const auto nearStartIntersection = Intersect(nearStartLine, curve);
        ASSERT_TRUE(nearStartIntersection.intersects);
        EXPECT_NEAR(nearStartIntersection.segmentParameter, nearStartParameter, 1e-8);
        EXPECT_TRUE(nearStartIntersection.point.AlmostEquals(nearStart, 1e-9));
    }

    TEST(NurbsNumericsTest, FaceProjectionRequiresCompatibleTrimRegion)
    {
        const auto support = std::make_shared<Geometry::SCPlaneSurface>(
            Geometry::SCPlane::FromPointAndNormal({0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}),
            SCVector3d{1.0, 0.0, 0.0}, SCVector3d{0.0, 1.0, 0.0},
            Geometry::SCIntervald{-1.0, 1.0}, Geometry::SCIntervald{-1.0, 1.0});
        const auto displaced = std::make_shared<Geometry::SCPlaneSurface>(
            Geometry::SCPlane::FromPointAndNormal({0.0, 0.0, 1.0}, {0.0, 0.0, 1.0}),
            SCVector3d{1.0, 0.0, 0.0}, SCVector3d{0.0, 1.0, 0.0},
            Geometry::SCIntervald{-1.0, 1.0}, Geometry::SCIntervald{-1.0, 1.0});
        const Geometry::SCBrepLoop loop({Geometry::SCBrepCoedge{0}});
        const SCPolyline2d ring(
            std::vector<SCPoint2d>{{-0.5, -0.5}, {0.5, -0.5}, {0.5, 0.5}, {-0.5, 0.5}},
            Geometry::SCPolylineClosure::Closed);
        const SCCurveOnSurface compatibleTrim(std::make_shared<Geometry::SCPlaneSurface>(*support), ring);
        const SCCurveOnSurface incompatibleTrim(displaced, ring);
        const Geometry::SCBrepFace missingTrim(support, loop);
        const Geometry::SCBrepFace compatibleFace(support, loop, {}, compatibleTrim);
        const Geometry::SCBrepFace incompatibleFace(support, loop, {}, incompatibleTrim);
        ASSERT_TRUE(missingTrim.IsValid());
        ASSERT_TRUE(compatibleFace.IsValid());
        ASSERT_TRUE(incompatibleFace.IsValid());

        const SCPoint3d query = support->PointAt(0.0, 0.0);
        EXPECT_FALSE(Geometry::ProjectPointToBrepFace(query, missingTrim).success);
        const SCLine3d faceLine = SCLine3d::FromOriginAndDirection(
            SCPoint3d{0.0, 0.0, 1.0}, SCVector3d{0.0, 0.0, -1.0});
        EXPECT_FALSE(Intersect(faceLine, missingTrim).intersects);
        const auto validProjection = Geometry::ProjectPointToBrepFace(query, compatibleFace);
        ASSERT_TRUE(validProjection.success);
        EXPECT_TRUE(validProjection.point.AlmostEquals(query, 1e-12));
        EXPECT_FALSE(Geometry::ProjectPointToBrepFace(query, incompatibleFace).success);
        const auto faceIntersection = Intersect(faceLine, compatibleFace);
        ASSERT_TRUE(faceIntersection.intersects);
        EXPECT_TRUE(faceIntersection.point.AlmostEquals(query, 1e-12));
        EXPECT_FALSE(Intersect(faceLine, incompatibleFace).intersects);

        const auto narrowSupport = std::make_shared<Geometry::SCPlaneSurface>(
            support->SupportPlane(), support->UAxis(), support->VAxis(),
            Geometry::SCIntervald{-0.1, 0.1}, Geometry::SCIntervald{-0.1, 0.1});
        const Geometry::SCBrepFace extendedTrimFace(
            narrowSupport, loop, {},
            SCCurveOnSurface(std::make_shared<Geometry::SCPlaneSurface>(*narrowSupport), ring));
        const auto extendedProjection = Geometry::ProjectPointToBrepFace({0.4, 0.0, 1.0}, extendedTrimFace);
        ASSERT_TRUE(extendedProjection.success);
        EXPECT_NEAR(extendedProjection.u, 0.4, 1e-12);
        EXPECT_NEAR(extendedProjection.v, 0.0, 1e-12);
        EXPECT_TRUE(extendedProjection.point.AlmostEquals(SCPoint3d{0.4, 0.0, 0.0}, 1e-12));

        const std::vector<SCPoint3d> corners{{-0.5, -0.5, 0.0}, {0.5, -0.5, 0.0},
                                             {0.5, 0.5, 0.0}, {-0.5, 0.5, 0.0}};
        std::vector<Geometry::SCBrepVertex> vertices;
        std::vector<Geometry::SCBrepEdge> edges;
        std::vector<Geometry::SCBrepCoedge> coedges;
        for (std::size_t index = 0; index < corners.size(); ++index)
        {
            const std::size_t next = (index + 1) % corners.size();
            vertices.emplace_back(corners[index]);
            edges.emplace_back(std::make_shared<Geometry::SCLineCurve3d>(
                                   SCLine3d::FromOriginAndDirection(
                                       corners[index], corners[next] - corners[index]),
                                   Geometry::SCIntervald{0.0, 1.0}), index, next);
            coedges.emplace_back(index);
        }
        const Geometry::SCBrepFace topologyFace(support, Geometry::SCBrepLoop(std::move(coedges)));
        const Geometry::SCBrepBody body(std::move(vertices), std::move(edges),
                                        {Geometry::SCBrepShell({topologyFace})});
        ASSERT_TRUE(body.IsValid());
        const auto bodyProjection = Geometry::ProjectPointToBrepBody(query, body);
        ASSERT_TRUE(bodyProjection.success);
        EXPECT_TRUE(bodyProjection.projection.point.AlmostEquals(query, 1e-12));
        const auto bodyIntersection = Intersect(faceLine, body);
        ASSERT_TRUE(bodyIntersection.intersects);
        ASSERT_EQ(bodyIntersection.hits.size(), 1u);
        EXPECT_TRUE(bodyIntersection.hits.front().point.AlmostEquals(query, 1e-12));

        std::vector<Geometry::SCBrepEdge> reversedEdges = body.Edges();
        reversedEdges[0] = Geometry::SCBrepEdge(
            std::make_shared<Geometry::SCLineCurve3d>(
                SCLine3d::FromOriginAndDirection(corners[1], corners[0] - corners[1]),
                Geometry::SCIntervald{0.0, 1.0}), 0, 1);
        const Geometry::SCBrepBody mismatchedBody(body.Vertices(), std::move(reversedEdges), body.Shells());
        ASSERT_TRUE(mismatchedBody.IsValid());
        EXPECT_FALSE(Geometry::ProjectPointToBrepBody(query, mismatchedBody).success);
        EXPECT_FALSE(Intersect(faceLine, mismatchedBody).intersects);
    }

    TEST(NurbsNumericsTest, SmallScaleCurveProjectionAndIntersectionRefineBetweenSeeds)
    {
        constexpr double length = 1e-5;
        const SCNurbsCurve3d curve(1, {{0.0, 0.0, 0.0}, {length, 0.0, 0.0}},
                                   {0.0, 0.0, 1.0, 1.0});
        const SCPoint3d source{length * 0.25, length, 0.0};
        const SCPoint3d expected{length * 0.25, 0.0, 0.0};
        ASSERT_TRUE(curve.IsValid());

        const auto projection = ProjectPointToCurve(source, curve);
        ASSERT_TRUE(projection.success);
        EXPECT_NEAR(projection.parameter, 0.25, 1e-10);
        EXPECT_TRUE(projection.point.AlmostEquals(expected, 1e-12));

        const auto intersection = Intersect(
            SCLine3d::FromOriginAndDirection(source, SCVector3d{0.0, -1.0, 0.0}), curve);
        ASSERT_TRUE(intersection.intersects);
        EXPECT_NEAR(intersection.curveParameter, 0.25, 1e-10);
        EXPECT_TRUE(intersection.point.AlmostEquals(expected, 1e-12));
    }

    TEST(NurbsNumericsTest, LineCurveIntersectionAcceptsValidSmallDirection)
    {
        constexpr double directionScale = 1e-7;
        const SCNurbsCurve3d curve(1, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}},
                                   {0.0, 0.0, 1.0, 1.0});
        const SCLine3d line = SCLine3d::FromOriginAndDirection(
            SCPoint3d{0.25, -directionScale, 0.0}, SCVector3d{0.0, directionScale, 0.0});
        ASSERT_TRUE(curve.IsValid());
        ASSERT_TRUE(line.IsValid());

        const auto intersection = Intersect(line, curve);
        ASSERT_TRUE(intersection.intersects);
        EXPECT_NEAR(intersection.lineParameter, 1.0, 1e-10);
        EXPECT_NEAR(intersection.curveParameter, 0.25, 1e-10);
        EXPECT_TRUE(intersection.point.AlmostEquals(SCPoint3d{0.25, 0.0, 0.0}, 1e-12));
    }

    TEST(NurbsNumericsTest, ShallowTangentCurvePlaneIntersectionRefinesBetweenSeeds)
    {
        constexpr double height = 1e-10;
        const SCNurbsCurve3d curve(2,
            {{0.0, 0.0, height * 0.0625},
             {0.5, 0.0, -height * 0.1875},
             {1.0, 0.0, height * 0.5625}},
            {0.0, 0.0, 0.0, 1.0, 1.0, 1.0});
        const Geometry::SCPlane plane = Geometry::SCPlane::FromPointAndNormal(
            SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0});
        Geometry::SCGeometryTolerance3d tolerance;
        tolerance.distanceEpsilon = 1e-18;
        ASSERT_TRUE(curve.IsValid(tolerance));

        const auto intersection = Intersect(plane, curve, tolerance);
        ASSERT_TRUE(intersection.intersects);
        EXPECT_NEAR(intersection.curveParameter, 0.25, 1e-4);
        EXPECT_LE(std::abs(intersection.point.z), tolerance.distanceEpsilon);
    }

    TEST(NurbsNumericsTest, CurvePlaneBisectionDoesNotReportPointOutsideTolerance)
    {
        constexpr double parameterSpan = 1e8;
        const SCNurbsCurve3d curve(1,
            {{0.0, 0.0, -parameterSpan / 7.0},
             {parameterSpan, 0.0, parameterSpan * 6.0 / 7.0}},
            {0.0, 0.0, parameterSpan, parameterSpan});
        const Geometry::SCPlane plane = Geometry::SCPlane::FromPointAndNormal(
            SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0});
        Geometry::SCGeometryTolerance3d tolerance;
        tolerance.distanceEpsilon = 1e-30;
        ASSERT_TRUE(curve.IsValid(tolerance));

        const auto intersection = Intersect(plane, curve, tolerance);
        if (intersection.intersects)
        {
            EXPECT_LE(std::abs(plane.SignedDistanceTo(intersection.point, tolerance.distanceEpsilon)),
                      tolerance.distanceEpsilon);
        }
    }

    TEST(NurbsNumericsTest, SmallScaleSurfaceProjectionRefinesWithinPlanarPatch)
    {
        for (const double width : {1e-3, 1e-5})
        {
            const SCNurbsSurface surface(1, 1, 2, 2,
                {{0.0, 0.0, 0.0}, {width, 0.0, 0.0},
                 {0.0, width, 0.0}, {width, width, 0.0}},
                {0.0, 0.0, 1.0, 1.0}, {0.0, 0.0, 1.0, 1.0});
            const SCPoint3d expected{width * 0.25, width * 0.75, 0.0};
            ASSERT_TRUE(surface.IsValid());

            const auto projection = ProjectPointToSurface(
                SCPoint3d{expected.x, expected.y, width}, surface);
            ASSERT_TRUE(projection.success);
            EXPECT_NEAR(projection.u, 0.25, 1e-10);
            EXPECT_NEAR(projection.v, 0.75, 1e-10);
            EXPECT_TRUE(projection.point.AlmostEquals(expected, 1e-12));
        }
    }

    TEST(NurbsNumericsTest, SmallPlaneSurfaceAxesRemainProjectable)
    {
        constexpr double width = 1e-10;
        const Geometry::SCPlane plane = Geometry::SCPlane::FromPointAndNormal(
            SCPoint3d{}, SCVector3d{0.0, 0.0, 1.0});
        const Geometry::SCPlaneSurface surface(plane, {width, 0.0, 0.0}, {0.0, width, 0.0},
                                               {0.0, 1.0}, {0.0, 1.0});
        ASSERT_TRUE(surface.IsValid());
        const auto projection = ProjectPointToSurface({0.25 * width, 0.75 * width, width}, surface);
        ASSERT_TRUE(projection.success);
        EXPECT_NEAR(projection.u, 0.25, 1e-10);
        EXPECT_NEAR(projection.v, 0.75, 1e-10);
        EXPECT_TRUE(projection.point.AlmostEquals(surface.PointAt(projection.u, projection.v), 1e-15));
    }

    TEST(NurbsNumericsTest, SkewPlaneSurfaceUsesCoupledParameters)
    {
        const Geometry::SCPlane plane = Geometry::SCPlane::FromPointAndNormal(
            SCPoint3d{}, SCVector3d{0.0, 0.0, 1.0});
        const Geometry::SCPlaneSurface surface(plane, {1.0, 0.0, 0.0}, {5e-10, 1.0, 0.0},
                                               {0.0, 1e9}, {0.0, 1e9});
        ASSERT_TRUE(surface.IsValid());
        const SCPoint3d onSurface = surface.PointAt(1e8, 1e9);
        const SCPoint3d query{onSurface.x, onSurface.y, 1.0};

        const auto projection = ProjectPointToSurface(query, surface);
        ASSERT_TRUE(projection.success);
        EXPECT_NEAR(projection.u, 1e8, 1e-7);
        EXPECT_NEAR(projection.v, 1e9, 1e-7);
        EXPECT_TRUE(projection.point.AlmostEquals(onSurface, 1e-7));
        EXPECT_NEAR(projection.distanceSquared, 1.0, 1e-12);

        const SCLine3d line = SCLine3d::FromOriginAndDirection(query, {0.0, 0.0, -1.0});
        const auto hit = Intersect(line, surface);
        ASSERT_TRUE(hit.intersects);
        EXPECT_NEAR(hit.u, 1e8, 1e-7);
        EXPECT_NEAR(hit.v, 1e9, 1e-7);
        EXPECT_TRUE(hit.point.AlmostEquals(onSurface, 1e-7));
    }

    TEST(NurbsNumericsTest, PlaneSurfaceIntersectionUsesActualTiltedPlane)
    {
        const Geometry::SCPlane storedPlane = Geometry::SCPlane::FromPointAndNormal(
            SCPoint3d{}, SCVector3d{0.0, 0.0, 1.0});
        const Geometry::SCPlaneSurface surface(storedPlane, {1.0, 0.0, 5e-10}, {0.0, 1.0, 0.0},
                                               {0.0, 1e9}, {0.0, 1.0});
        ASSERT_TRUE(surface.IsValid());
        const SCPoint3d onSurface = surface.PointAt(1e9, 0.5);
        const SCLine3d line = SCLine3d::FromOriginAndDirection(
            SCPoint3d{onSurface.x, onSurface.y, 1.0}, {0.0, 0.0, -1.0});
        const auto hit = Intersect(line, surface);
        ASSERT_TRUE(hit.intersects);
        EXPECT_NEAR(hit.u, 1e9, 1e-6);
        EXPECT_NEAR(hit.v, 0.5, 1e-9);
        EXPECT_NEAR(hit.point.z, 0.5, 1e-9);
        EXPECT_TRUE(hit.point.AlmostEquals(onSurface, 1e-6));
    }

    TEST(NurbsNumericsTest, SmallOffsetSurfaceKeepsItsDisplacement)
    {
        constexpr double width = 1e-5;
        constexpr double offset = 1e-3;
        const SCNurbsSurface base(1, 1, 2, 2,
            {{0.0, 0.0, 0.0}, {width, 0.0, 0.0},
             {0.0, width, 0.0}, {width, width, 0.0}},
            {0.0, 0.0, 1.0, 1.0}, {0.0, 0.0, 1.0, 1.0});
        const auto surface = Geometry::SCOffsetSurface::FromSurface(base, offset);
        ASSERT_TRUE(surface.IsValid());
        EXPECT_NEAR(surface.PointAt(0.25, 0.75).z, offset, 1e-12);
        const auto projection = ProjectPointToSurface({0.25 * width, 0.75 * width, 2.0 * offset}, surface);
        ASSERT_TRUE(projection.success);
        EXPECT_NEAR(projection.point.z, offset, 1e-12);
    }

    TEST(NurbsNumericsTest, BilinearSaddleProjectionDoesNotStopAtStationaryCenter)
    {
        const SCNurbsSurface surface(1, 1, 2, 2,
            {{0.0, 0.0, 2.0}, {1.0, 0.0, -2.0},
             {0.0, 1.0, -2.0}, {1.0, 1.0, 2.0}},
            {0.0, 0.0, 1.0, 1.0}, {0.0, 0.0, 1.0, 1.0});
        ASSERT_TRUE(surface.IsValid());

        const auto projection = ProjectPointToSurface(SCPoint3d{0.5, 0.5, 2.0}, surface);
        ASSERT_TRUE(projection.success);
        EXPECT_LT(projection.distanceSquared, 0.6);
        EXPECT_TRUE(projection.point.AlmostEquals(surface.PointAt(projection.u, projection.v), 1e-12));
    }

    TEST(NurbsNumericsTest, DegenerateSurfaceProjectionUsesPointOnlyRefinement)
    {
        const SCNurbsSurface surface(1, 1, 2, 2,
            {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
             {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}},
            {0.0, 0.0, 1.0, 1.0}, {0.0, 0.0, 1.0, 1.0});
        ASSERT_TRUE(surface.IsValid());

        const auto projection = ProjectPointToSurface(SCPoint3d{0.75, 1.0, 0.0}, surface);
        ASSERT_TRUE(projection.success);
        EXPECT_NEAR(projection.u, 0.75, 1e-10);
        EXPECT_TRUE(projection.point.AlmostEquals(SCPoint3d{0.75, 0.0, 0.0}, 1e-12));
    }

    TEST(NurbsNumericsTest, SurfaceProjectionFindsIsolatedKnotBeyondSeedBudget)
    {
        constexpr int uSpans = 200;
        constexpr int vSpans = 10;
        std::vector<SCPoint3d> controlPoints;
        for (int v = 0; v <= vSpans; ++v)
        {
            for (int u = 0; u <= uSpans; ++u)
            {
                controlPoints.emplace_back(0.0, 0.0, u == 99 && v == 3 ? 0.0 : 1.0);
            }
        }

        std::vector<double> uKnots{0.0, 0.0};
        std::vector<double> vKnots{0.0, 0.0};
        for (int u = 1; u < uSpans; ++u)
        {
            uKnots.push_back(static_cast<double>(u));
        }
        for (int v = 1; v < vSpans; ++v)
        {
            vKnots.push_back(static_cast<double>(v));
        }
        uKnots.insert(uKnots.end(), {static_cast<double>(uSpans), static_cast<double>(uSpans)});
        vKnots.insert(vKnots.end(), {static_cast<double>(vSpans), static_cast<double>(vSpans)});

        const SCNurbsSurface surface(1, 1, uSpans + 1, vSpans + 1,
                                     std::move(controlPoints), std::move(uKnots), std::move(vKnots));
        ASSERT_TRUE(surface.IsValid());
        EXPECT_TRUE(surface.PointAt(99.0, 3.0).AlmostEquals(SCPoint3d{}, 0.0));

        const auto projection = ProjectPointToSurface(SCPoint3d{}, surface);
        ASSERT_TRUE(projection.success);
        EXPECT_LE(projection.distanceSquared, 1e-12);
    }

    TEST(NurbsNumericsTest, ManySpanRationalSurfaceProjectsItsInteriorPoint)
    {
        constexpr int spanCount = 18;
        std::vector<SCPoint3d> controlPoints;
        std::vector<double> weights;
        for (int v = 0; v <= spanCount; ++v)
        {
            for (int u = 0; u <= spanCount; ++u)
            {
                controlPoints.emplace_back(static_cast<double>(u), static_cast<double>(v),
                                           std::sin(0.2 * u) * std::cos(0.2 * v));
                weights.push_back(1.0 + 0.2 * ((u + v) % 3));
            }
        }
        std::vector<double> knots{0.0, 0.0};
        for (int i = 1; i < spanCount; ++i)
        {
            knots.push_back(static_cast<double>(i));
        }
        knots.insert(knots.end(), {static_cast<double>(spanCount), static_cast<double>(spanCount)});
        const SCNurbsSurface surface(1, 1, spanCount + 1, spanCount + 1,
                                     std::move(controlPoints), knots, knots, std::move(weights));
        ASSERT_TRUE(surface.IsValid());
        const SCPoint3d point = surface.PointAt(8.37, 12.68);
        const auto projection = ProjectPointToSurface(point, surface);
        ASSERT_TRUE(projection.success);
        EXPECT_LE(projection.distanceSquared, 1e-12);
    }

    TEST(NurbsNumericsTest, SmallScaleNonPlanarSurfaceIntersectionRefinesBetweenSeeds)
    {
        constexpr double width = 1e-5;
        const SCNurbsSurface surface(1, 1, 2, 2,
            {{0.0, 0.0, 0.0}, {width, 0.0, 0.0},
             {0.0, width, 0.0}, {width, width, width * 0.2}},
            {0.0, 0.0, 1.0, 1.0}, {0.0, 0.0, 1.0, 1.0});
        constexpr double expectedU = 0.37;
        constexpr double expectedV = 0.63;
        const SCPoint3d expected = surface.PointAt(expectedU, expectedV);
        Geometry::SCGeometryTolerance3d tolerance;
        tolerance.distanceEpsilon = 1e-15;
        ASSERT_TRUE(surface.IsValid(tolerance));

        const auto intersection = Intersect(
            SCLine3d::FromOriginAndDirection(
                SCPoint3d{expected.x, expected.y, width}, SCVector3d{0.0, 0.0, -1.0}),
            surface, tolerance);
        ASSERT_TRUE(intersection.intersects);
        EXPECT_NEAR(intersection.u, expectedU, 1e-10);
        EXPECT_NEAR(intersection.v, expectedV, 1e-10);
        EXPECT_TRUE(intersection.point.AlmostEquals(expected, 1e-14));
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

    TEST(NurbsNumericsTest, SurfaceProjectionPrunesOverlappingControlBounds)
    {
        constexpr int spans = 18;
        for (const double scale : {1.0, 1e80})
        {
            SCOPED_TRACE(scale);
            std::vector<SCPoint3d> points;
            for (int v = 0; v <= spans; ++v)
            {
                for (int u = 0; u <= spans; ++u)
                {
                    const int choice = (u + 2 * v) % 3;
                    points.emplace_back(choice == 0 ? 0.0 : 2.0 * scale,
                                        choice == 1 ? 0.0 : 2.0 * scale, 0.0);
                }
            }
            std::vector<double> knots{0.0, 0.0};
            for (int i = 1; i < spans; ++i)
            {
                knots.push_back(static_cast<double>(i));
            }
            knots.insert(knots.end(), {static_cast<double>(spans), static_cast<double>(spans)});
            const SCNurbsSurface surface(1, 1, spans + 1, spans + 1, std::move(points), knots, knots);
            ASSERT_TRUE(surface.IsValid());
            const auto projection = ProjectPointToSurface(SCPoint3d{}, surface);
            ASSERT_TRUE(projection.success);
            EXPECT_NEAR(projection.distanceSquared / (scale * scale), 2.0, 1e-12);
        }
    }

    TEST(NurbsNumericsTest, MultiCellProjectionDoesNotTreatUnderflowedSquareAsExactHit)
    {
        constexpr int spans = 257;
        std::vector<SCPoint3d> points;
        points.reserve(2 * (spans + 1));
        for (int v = 0; v < 2; ++v)
        {
            for (int u = 0; u <= spans; ++u)
            {
                points.emplace_back(0.0, 0.0, u == spans ? 1e-200 : 2e-200);
            }
        }
        std::vector<double> uKnots{0.0, 0.0};
        for (int u = 1; u < spans; ++u)
        {
            uKnots.push_back(static_cast<double>(u));
        }
        uKnots.insert(uKnots.end(), {static_cast<double>(spans), static_cast<double>(spans)});
        const SCNurbsSurface surface(1, 1, spans + 1, 2, std::move(points), std::move(uKnots),
                                     {0.0, 0.0, 1.0, 1.0});
        ASSERT_TRUE(surface.IsValid());

        Geometry::SCGeometryTolerance3d tolerance{};
        tolerance.distanceEpsilon = 1e-250;
        tolerance.parameterEpsilon = 1.0;
        const auto projection = ProjectPointToSurface(SCPoint3d{}, surface, tolerance);
        ASSERT_TRUE(projection.success);
        EXPECT_EQ(projection.distanceSquared, 0.0);
        EXPECT_GT(projection.point.z, 0.0);
        EXPECT_LT(projection.point.z, 1.5e-200);
        EXPECT_EQ(projection.u, static_cast<double>(spans));
        EXPECT_EQ(projection.point, surface.PointAt(projection.u, projection.v));
    }
    TEST(NurbsNumericsTest, FixedLargeCoordinateWorkloadPerformance)
    {
        constexpr int spans=18;
        std::vector<SCPoint3d> points;
        for (int v=0; v<=spans; ++v) for (int u=0; u<=spans; ++u) {
            const int choice=(u+2*v)%3;
            points.emplace_back(choice==0?0:2e80,choice==1?0:2e80,0);
        }
        std::vector<double> knots{0,0};
        for (int i=1; i<spans; ++i) knots.push_back(i);
        knots.insert(knots.end(),{18,18});
        const SCNurbsSurface surface(1,1,19,19,std::move(points),knots,knots);
        std::vector<double> timings;
        for (int i=0; i<12; ++i) {
            const auto start=std::chrono::steady_clock::now();
            const auto projection=ProjectPointToSurface({},surface);
            const auto end=std::chrono::steady_clock::now();
            ASSERT_TRUE(projection.success);
            EXPECT_NEAR(projection.distanceSquared/1e160,2,1e-12);
            EXPECT_TRUE(surface.URange().Contains(projection.u));
            EXPECT_TRUE(surface.VRange().Contains(projection.v));
            const auto actual=surface.PointAt(projection.u,projection.v);
            EXPECT_LE(std::hypot(actual.x-projection.point.x,actual.y-projection.point.y,actual.z-projection.point.z),1e65);
            if (i>=2) timings.push_back(std::chrono::duration<double,std::milli>(end-start).count());
        }
        std::cout << "fixed workload samples_ms=";
        for (const auto timing:timings) std::cout << ' ' << timing;
        std::cout << std::endl;
        std::sort(timings.begin(),timings.end());
        const double median=(timings[4]+timings[5])/2;
        std::cout << "fixed workload median_ms=" << median << " p95_ms=" << timings[9] << std::endl;
        const auto status=Geometry::ProjectPointToSurfaceWithStatus({},surface);
        std::cout << "positions=" << status.statistics.positionEvaluations << " derivatives=" << status.statistics.derivativeEvaluations
                  << " seeds=" << status.statistics.seedCount << std::endl;
        EXPECT_LE(median,250);
        EXPECT_LE(timings[9],500);
    }

    TEST(NurbsNumericsTest, MultiCellWorkloadPerformanceMatrix)
    {
        const auto knots=[](int spans) {
            std::vector<double> result{0,0};
            for (int i=1;i<spans;++i) result.push_back(i);
            result.insert(result.end(),{static_cast<double>(spans),static_cast<double>(spans)});
            return result;
        };
        // 同一无跳片求解器的固定位置求值基线；只约束成本，不充当最优性证明。
        struct Workload { const char* name; int u,v,kind; std::size_t baseline; };
        for (const auto workload : {
            Workload{"255",255,1,0,79305},Workload{"256",256,1,0,79616},Workload{"257",257,1,0,95973},
            Workload{"constant",200,10,1,10561},Workload{"overlap",18,18,2,18901},
            Workload{"sparse",18,18,3,40140},Workload{"affine",18,18,0,38179},Workload{"general",18,18,4,51852}}) {
            SCOPED_TRACE(workload.name);
            std::vector<SCPoint3d> controls;
            for (int v=0;v<=workload.v;++v) for (int u=0;u<=workload.u;++u) {
                if (workload.kind==1) controls.push_back({0,0,1});
                else if (workload.kind==2) controls.push_back({double(u%2),double(v%2),1});
                else if (workload.kind==3) controls.push_back({double(u*u),double(v*v),1});
                else controls.push_back({double(u),double(v),workload.kind==4?1+0.1*std::sin(double(u+v)):1});
            }
            const SCNurbsSurface surface(1,1,workload.u+1,workload.v+1,std::move(controls),knots(workload.u),knots(workload.v));
            const SCPoint3d query{0.37,0.43,2};
            std::vector<double> elapsed;
            Geometry::SCGeometrySearchStatistics statistics{};
            for (int i=0;i<12;++i) {
                const auto start=std::chrono::steady_clock::now();
                const auto candidate=ProjectPointToSurface(query,surface);
                const auto finish=std::chrono::steady_clock::now();
                ASSERT_TRUE(candidate.success);
                EXPECT_EQ(candidate.point,surface.PointAt(candidate.u,candidate.v));
                if (i>=2) elapsed.push_back(std::chrono::duration<double,std::milli>(finish-start).count());
            }
            const auto status=Geometry::ProjectPointToSurfaceWithStatus(query,surface);
            ASSERT_TRUE(status.payloadValid);
            EXPECT_FALSE(status.statistics.searchIncomplete);
            statistics=status.statistics;
            EXPECT_LE(statistics.positionEvaluations,1.25*workload.baseline);
            const auto legacy=ProjectPointToSurface(query,surface);
            EXPECT_EQ(legacy.point,status.payload.point);
            EXPECT_DOUBLE_EQ(legacy.u,status.payload.u);
            EXPECT_DOUBLE_EQ(legacy.v,status.payload.v);
            std::cout << "matrix " << workload.name << " samples_ms=";
            for (const auto timing:elapsed) std::cout << ' ' << timing;
            std::cout << std::endl;
            std::sort(elapsed.begin(),elapsed.end());
            std::cout << "matrix " << workload.name << " median_ms=" << (elapsed[4]+elapsed[5])/2
                << " p95_ms=" << elapsed[9] << " positions=" << statistics.positionEvaluations
                << " derivatives=" << statistics.derivativeEvaluations << " seeds=" << statistics.seedCount << std::endl;
        }
    }

    TEST(NurbsNumericsTest, SurfaceBudgetMarksOnlyTheNextRequiredEvaluationIncomplete)
    {
        Geometry::Detail::SurfaceSearchBudget positions{};
        positions.bounded=true;
        for (std::size_t i=0;i<2000000;++i) ASSERT_TRUE(positions.Consume(0));
        EXPECT_EQ(positions.statistics.positionEvaluations,2000000u);
        EXPECT_FALSE(positions.statistics.searchIncomplete);
        EXPECT_FALSE(positions.Consume(0));
        EXPECT_TRUE(positions.statistics.searchIncomplete);
        EXPECT_EQ(positions.statistics.positionEvaluations,2000000u);
        Geometry::Detail::SurfaceSearchBudget derivatives{};
        derivatives.bounded=true;
        for (std::size_t i=0;i<1000000;++i) ASSERT_TRUE(derivatives.Consume(1));
        EXPECT_FALSE(derivatives.statistics.searchIncomplete);
        EXPECT_TRUE(derivatives.Consume(0));
        EXPECT_FALSE(derivatives.Consume(1));
        EXPECT_EQ(derivatives.statistics.derivativeEvaluations,1000000u);
        EXPECT_EQ(derivatives.statistics.positionEvaluations,1000001u);
        Geometry::Detail::SurfaceSearchBudget legacy{};
        for (std::size_t i=0;i<2000001;++i) ASSERT_TRUE(legacy.Consume(0));
        EXPECT_EQ(legacy.statistics.positionEvaluations,2000001u);
        EXPECT_FALSE(legacy.statistics.searchIncomplete);
    }

    TEST(NurbsNumericsTest, ActualSurfaceBudgetRetainsCandidateWhileLegacyContinues)
    {
        constexpr int spans=10000;
        std::vector<SCPoint3d> controls;
        for (int v=0;v<2;++v) for (int u=0;u<=spans;++u)
            controls.push_back({0,0,2.0-double(u)/spans});
        std::vector<double> knots{0,0};
        for (int i=1;i<spans;++i) knots.push_back(i);
        knots.insert(knots.end(),{spans,spans});
        const SCNurbsSurface surface(1,1,spans+1,2,std::move(controls),std::move(knots),{0,0,1,1});
        const auto status=Geometry::ProjectPointToSurfaceWithStatus({},surface);
        std::cout << "actual budget positions=" << status.statistics.positionEvaluations
            << " derivatives=" << status.statistics.derivativeEvaluations
            << " incomplete=" << status.statistics.searchIncomplete << std::endl;
        ASSERT_TRUE(status.payloadValid);
        EXPECT_EQ(status.state,Geometry::SCGeometryOperationState::HasResult);
        ASSERT_TRUE(status.statistics.searchIncomplete);
        EXPECT_TRUE(Geometry::HasGeometryFailureReason(status.evidence.reasons,Geometry::SCGeometryFailureReason::SearchIncomplete));
        EXPECT_EQ(status.evidence.optimality,Geometry::SCGeometryEvidenceState::Unknown);
        EXPECT_EQ(status.payload.point,surface.PointAt(status.payload.u,status.payload.v));
        const auto legacy=ProjectPointToSurface({},surface);
        ASSERT_TRUE(legacy.success);
        EXPECT_LT(legacy.point.z,status.payload.point.z);
        EXPECT_DOUBLE_EQ(legacy.point.z,1);
        EXPECT_DOUBLE_EQ(legacy.u,spans);
        EXPECT_DOUBLE_EQ(Geometry::Distance({},surface),1);
        EXPECT_DOUBLE_EQ(Geometry::DistanceSquared({},surface),1);
        const auto distance=Geometry::DistanceWithStatus({},surface);
        ASSERT_TRUE(distance.payloadValid);
        EXPECT_DOUBLE_EQ(distance.payload.distance,status.payload.point.z);
        EXPECT_TRUE(Geometry::HasGeometryFailureReason(distance.evidence.reasons,Geometry::SCGeometryFailureReason::SearchIncomplete));
    }

    TEST(NurbsNumericsTest, CurvedOffsetProjectionUsesActualOffsetPositions)
    {
        const SCNurbsSurface base(2,1,3,2,
            {{0,0,0},{0.5,0,0},{1,0,1},{0,1,0},{0.5,1,0},{1,1,1}},
            {0,0,0,1,1,1},{0,0,1,1});
        const auto surface=Geometry::SCOffsetSurface::FromSurface(base,0.2);
        const auto target=surface.PointAt(0.37,0.43);
        const auto projection=ProjectPointToSurface(target,surface);
        ASSERT_TRUE(projection.success);
        EXPECT_EQ(projection.point,surface.PointAt(projection.u,projection.v));
        EXPECT_LE(std::hypot(target.x-projection.point.x,target.y-projection.point.y,target.z-projection.point.z),1e-7);
        const auto zero=Geometry::SCOffsetSurface::FromSurface(base,0.0);
        EXPECT_EQ(zero.PointAt(0.37,0.43),base.PointAt(0.37,0.43));
    }

    TEST(NurbsNumericsTest, BodySurfaceBudgetsRemainIndependentAfterActualTruncation)
    {
        using namespace Geometry;
        constexpr int spans=10000;
        std::vector<SCPoint3d> controls;
        for (int v=0;v<2;++v) for (int u=0;u<=spans;++u)
            controls.push_back({0,double(v)*1e-200,2.0-double(u)/spans});
        std::vector<double> knots{0,0};
        for (int i=1;i<spans;++i) knots.push_back(i);
        knots.insert(knots.end(),{spans,spans});
        auto surface=std::make_shared<SCNurbsSurface>(1,1,spans+1,2,std::move(controls),std::move(knots),
            std::vector<double>{0,0,1,1});
        const SCPolyline2d ring({{0,0},{spans,0},{spans,1},{0,1}},SCPolylineClosure::Closed);
        std::vector<SCBrepVertex> vertices;
        std::vector<SCBrepEdge> edges;
        for (std::size_t i=0;i<4;++i) vertices.emplace_back(surface->PointAt(ring.PointAt(i).x,ring.PointAt(i).y));
        for (std::size_t i=0;i<4;++i) {
            const auto a=vertices[i].Point(), b=vertices[(i+1)%4].Point();
            const auto delta=b-a;
            const double length=std::hypot(delta.x,delta.y,delta.z);
            edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                SCLine3d::FromOriginAndDirection(a,delta/length),{0,length})),i,(i+1)%4);
        }
        const SCBrepFace face(surface,Geometry::Test::BuildPlanarBrepOuterLoop(),{},SCCurveOnSurface(surface,ring),{});
        const SCBrepBody body(std::move(vertices),std::move(edges),{SCBrepShell({face}),SCBrepShell({face})});
        ASSERT_TRUE(body.IsValid());
        const SCPoint3d query{0,0.5e-200,0};
        const auto result=ProjectPointToBrepBodyWithStatus(query,body);
        ASSERT_TRUE(result.payloadValid);
        ASSERT_EQ(result.faces.size(),2u);
        for (const auto& record:result.faces) {
            std::cout << "body budget face=" << record.faceIndex << " positions=" << record.result.statistics.positionEvaluations
                << " derivatives=" << record.result.statistics.derivativeEvaluations
                << " incomplete=" << record.result.statistics.searchIncomplete << std::endl;
            EXPECT_TRUE(record.result.payloadValid);
            EXPECT_TRUE(record.result.statistics.searchIncomplete);
            EXPECT_TRUE(HasGeometryFailureReason(record.result.evidence.reasons,SCGeometryFailureReason::SearchIncomplete));
            EXPECT_TRUE(record.result.statistics.positionEvaluations==2000000 ||
                        record.result.statistics.derivativeEvaluations==1000000);
        }
        EXPECT_EQ(result.faces[0].result.statistics.positionEvaluations,result.faces[1].result.statistics.positionEvaluations);
        EXPECT_EQ(result.faces[0].result.statistics.derivativeEvaluations,result.faces[1].result.statistics.derivativeEvaluations);
        EXPECT_TRUE(HasGeometryFailureReason(result.evidence.reasons,SCGeometryFailureReason::SearchIncomplete));
        const auto legacy=ProjectPointToBrepBody(query,body);
        ASSERT_TRUE(legacy.success);
        EXPECT_NEAR(legacy.projection.point.z,1,1e-12);
    }

}  // namespace
