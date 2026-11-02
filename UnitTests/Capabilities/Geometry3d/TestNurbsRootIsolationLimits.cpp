#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "Core/Intersection.h"
#include "Geometry3d/SCNurbsCurve3d.h"

using Geometry::Intersect;
using Geometry::SCLine3d;
using Geometry::SCNurbsCurve3d;
using Geometry::SCPoint3d;

namespace
{
    TEST(NurbsNumericsTest, LineCoincidenceRequiresOriginalControlHullProof)
    {
        const double tiny = std::numeric_limits<double>::denorm_min();
        const SCNurbsCurve3d curve(2,
            {{0, 1, 0}, {1, 1, 0}, {2, 1, 0}, {3, 0, 0}, {4, tiny, tiny},
             {5, 0, 0}, {6, 0, 0}, {7, 0, 1}, {8, 0, 1}},
            {0, 0, 0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 1, 1, 1});
        Geometry::SCGeometryTolerance3d tolerance;
        tolerance.distanceEpsilon = tiny;
        ASSERT_TRUE(curve.IsValid(tolerance));
        const auto result = Geometry::IntersectAllLineCurveWithStatus(
            SCLine3d::FromOriginAndDirection({0, 0, 0}, {1, 0, 0}), curve, tolerance);
        EXPECT_NE(result.state, Geometry::SCGeometryOperationState::NonUnique);
        EXPECT_EQ(result.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Unknown);
    }

    TEST(NurbsNumericsTest, KnotInsertionUnderflowDoesNotClaimCoincidentSpan)
    {
        const double tiny = std::numeric_limits<double>::denorm_min();
        const SCNurbsCurve3d curve(2, {{0, 0, 1}, {0.3, 0, tiny}, {0.7, 0, 0}, {1, 0, 0}},
                                    {0, 0, 0, 0.5, 1, 1, 1});
        Geometry::SCGeometryTolerance3d tolerance;
        tolerance.distanceEpsilon = tiny;
        tolerance.parameterEpsilon = 1e-12;
        ASSERT_TRUE(curve.IsValid(tolerance));
        const auto result = Geometry::IntersectAllPlaneCurveWithStatus(
            Geometry::SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1}), curve, tolerance);
        EXPECT_NE(result.state, Geometry::SCGeometryOperationState::NonUnique);
        EXPECT_EQ(result.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Unknown);
    }

    TEST(NurbsNumericsTest, SubdivisionUnderflowDoesNotClaimCoincidentCurve)
    {
        const double offset = std::ldexp(1.0, -1060);
        const SCNurbsCurve3d curve(1, {{0, 0, -offset}, {1, 0, offset}}, {0, 0, 1, 1});
        Geometry::SCGeometryTolerance3d tolerance;
        tolerance.distanceEpsilon = std::numeric_limits<double>::denorm_min();
        tolerance.parameterEpsilon = 1e-12;
        ASSERT_TRUE(curve.IsValid(tolerance));
        const auto result = Geometry::IntersectAllPlaneCurveWithStatus(
            Geometry::SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1}), curve, tolerance);
        EXPECT_NE(result.state, Geometry::SCGeometryOperationState::NonUnique);
        EXPECT_EQ(result.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Unknown);
        ASSERT_FALSE(result.hits.empty());
        EXPECT_NEAR(result.hits.front().curveParameter, 0.5, 1e-3);
    }

    constexpr std::size_t kCellBudgetSpanCount = 32769;

    SCNurbsCurve3d MakeCellBudgetCurve(bool rootAtEnd)
    {
        std::vector<SCPoint3d> controls;
        controls.reserve(kCellBudgetSpanCount + 1);
        for (std::size_t i = 0; i <= kCellBudgetSpanCount; ++i)
            controls.push_back({static_cast<double>(i) / kCellBudgetSpanCount, 0.0,
                                (rootAtEnd ? i == kCellBudgetSpanCount : i == 0) ? -1.0 : 1.0});
        std::vector<double> knots{0.0, 0.0};
        knots.reserve(kCellBudgetSpanCount + 3);
        for (std::size_t i = 1; i < kCellBudgetSpanCount; ++i)
            knots.push_back(static_cast<double>(i) / kCellBudgetSpanCount);
        knots.insert(knots.end(), {1.0, 1.0});
        return SCNurbsCurve3d(1, std::move(controls), std::move(knots));
    }

    TEST(NurbsNumericsTest, PlaneCurveCellBudgetDoesNotProveUnvisitedRootAbsent)
    {
        const auto curve = MakeCellBudgetCurve(false);
        ASSERT_TRUE(curve.IsValid());
        EXPECT_LT(curve.PointAt(0.0).z, 0.0);
        EXPECT_GT(curve.PointAt(1.0 / kCellBudgetSpanCount).z, 0.0);
        const auto result = Geometry::IntersectAllPlaneCurveWithStatus(
            Geometry::SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1}), curve);
        EXPECT_NE(result.state, Geometry::SCGeometryOperationState::NoResult);
        EXPECT_EQ(result.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Unknown);
        EXPECT_TRUE(Geometry::HasGeometryFailureReason(result.evidence.reasons,
                                                       Geometry::SCGeometryFailureReason::SearchIncomplete));
        if (result.hits.empty())
        {
            EXPECT_EQ(result.state, Geometry::SCGeometryOperationState::Unknown);
            EXPECT_EQ(result.evidence.hit, Geometry::SCGeometryEvidenceState::Unknown);
        }
    }

    TEST(NurbsNumericsTest, CurveCellBudgetRetainsVerifiedPlaneAndLineHits)
    {
        const auto curve = MakeCellBudgetCurve(true);
        ASSERT_TRUE(curve.IsValid());
        const auto planeResult = Geometry::IntersectAllPlaneCurveWithStatus(
            Geometry::SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1}), curve);
        const auto lineResult = Geometry::IntersectAllLineCurveWithStatus(
            SCLine3d::FromOriginAndDirection({0, 0, 0}, {1, 0, 0}), curve);
        ASSERT_FALSE(planeResult.hits.empty());
        ASSERT_FALSE(lineResult.hits.empty());
        EXPECT_EQ(planeResult.state, Geometry::SCGeometryOperationState::HasResult);
        EXPECT_EQ(lineResult.state, Geometry::SCGeometryOperationState::HasResult);
        for (const auto* evidence : {&planeResult.evidence, &lineResult.evidence})
        {
            EXPECT_EQ(evidence->hit, Geometry::SCGeometryEvidenceState::Verified);
            EXPECT_EQ(evidence->negativeCompleteness, Geometry::SCGeometryEvidenceState::Unknown);
            EXPECT_TRUE(Geometry::HasGeometryFailureReason(evidence->reasons,
                                                           Geometry::SCGeometryFailureReason::SearchIncomplete));
        }
        const double expected = 1.0 - 0.5 / kCellBudgetSpanCount;
        EXPECT_NEAR(planeResult.hits.front().curveParameter, expected, 1e-10);
        EXPECT_NEAR(lineResult.hits.front().curveParameter, expected, 1e-10);
    }

    TEST(NurbsNumericsTest, RootIsolationPreservesUniformWeightScaling)
    {
        const auto plane = Geometry::SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});
        const auto line = SCLine3d::FromOriginAndDirection({0, 0, 0}, {1, 0, 0});
        for (const double scale : {1.0, 1e308, 1e-308})
        {
            const SCNurbsCurve3d curve(1, {{0, 0, -2}, {1, 0, 2}},
                                       {0, 0, 1, 1}, {0.5 * scale, scale});
            ASSERT_TRUE(curve.IsValid());
            const auto planeResult = Geometry::IntersectAllPlaneCurveWithStatus(plane, curve);
            const auto lineResult = Geometry::IntersectAllLineCurveWithStatus(line, curve);
            EXPECT_EQ(planeResult.hits.size(), 1U);
            EXPECT_EQ(lineResult.hits.size(), 1U);
            if (!planeResult.hits.empty())
                EXPECT_NEAR(planeResult.hits.front().curveParameter, 1.0 / 3.0, 1e-6);
            if (!lineResult.hits.empty())
                EXPECT_NEAR(lineResult.hits.front().curveParameter, 1.0 / 3.0, 1e-6);
        }
        const SCNurbsCurve3d separated(1, {{0, 0, 2}, {1, 0, 3}},
                                        {0, 0, 1, 1}, {1e308, 1e308});
        const auto result = Geometry::IntersectAllPlaneCurveWithStatus(plane, separated);
        EXPECT_EQ(result.state, Geometry::SCGeometryOperationState::NoResult);
        EXPECT_EQ(result.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Verified);
        const SCNurbsCurve3d coplanar(1, {{0, 0, 0}, {1, 0, 0}},
                                       {0, 0, 1, 1}, {1e-308, 1e308});
        EXPECT_EQ(Geometry::IntersectAllPlaneCurveWithStatus(plane, coplanar).state,
                  Geometry::SCGeometryOperationState::NonUnique);
    }

    TEST(NurbsNumericsTest, DegreeFortyCurveCanProvePlaneSeparation)
    {
        constexpr int degree = 40;
        std::vector<SCPoint3d> controls;
        for (int i = 0; i <= degree; ++i)
            controls.push_back({static_cast<double>(i) / degree, 0.0, 1.0});
        std::vector<double> knots(2 * (degree + 1), 1.0);
        std::fill(knots.begin(), knots.begin() + degree + 1, 0.0);
        const SCNurbsCurve3d curve(degree, std::move(controls), std::move(knots));
        ASSERT_TRUE(curve.IsValid());
        const auto result = Geometry::IntersectAllPlaneCurveWithStatus(
            Geometry::SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1}), curve);
        EXPECT_TRUE(result.hits.empty());
        EXPECT_EQ(result.state, Geometry::SCGeometryOperationState::NoResult);
        EXPECT_EQ(result.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Verified);
    }

    TEST(NurbsNumericsTest, DegreeFortyCurveReturnsBothPlaneAndLineRoots)
    {
        constexpr int degree = 40;
        std::vector<SCPoint3d> controls;
        std::vector<double> weights;
        for (int i = 0; i <= degree; ++i)
        {
            const double x = static_cast<double>(i) / degree;
            const double z = static_cast<double>(i * (i - 1)) / (degree * (degree - 1)) - x + 0.1875;
            const double weight = 1.0 + 0.1 * (i % 3);
            controls.push_back({x, 0.0, z / weight});
            weights.push_back(weight);
        }
        std::vector<double> knots(2 * (degree + 1), 1.0);
        std::fill(knots.begin(), knots.begin() + degree + 1, 0.0);
        const SCNurbsCurve3d curve(degree, std::move(controls), std::move(knots), std::move(weights));
        ASSERT_TRUE(curve.IsValid());
        const auto plane = Geometry::SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});
        const auto line = SCLine3d::FromOriginAndDirection({0, 0, 0}, {1, 0, 0});
        const auto planeResult = Geometry::IntersectAllPlaneCurveWithStatus(plane, curve);
        const auto lineResult = Geometry::IntersectAllLineCurveWithStatus(line, curve);
        ASSERT_EQ(planeResult.hits.size(), 2U);
        ASSERT_EQ(lineResult.hits.size(), 2U);
        for (std::size_t i = 0; i < 2; ++i)
        {
            const double expected = i == 0 ? 0.25 : 0.75;
            EXPECT_NEAR(planeResult.hits[i].curveParameter, expected, 1e-6);
            EXPECT_NEAR(lineResult.hits[i].curveParameter, expected, 1e-6);
        }
        const auto firstPlaneHit = Intersect(plane, curve);
        const auto firstLineHit = Intersect(line, curve);
        ASSERT_TRUE(firstPlaneHit.intersects);
        ASSERT_TRUE(firstLineHit.intersects);
        EXPECT_NEAR(firstPlaneHit.curveParameter, 0.25, 1e-6);
        EXPECT_NEAR(firstLineHit.curveParameter, 0.25, 1e-6);
    }

}
