#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "Geometry.h"
#include "Geometry3d/SCLineCurve3d.h"
#include "Geometry3d/SCPlaneSurface.h"
#include "Support/Fixtures3d.h"

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
    TEST(NurbsNumerics3dTest, LineCurveIntersectionAcceptsValidSmallDirection)
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

    TEST(NurbsNumerics3dTest, ShallowTangentCurvePlaneIntersectionRefinesBetweenSeeds)
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

    TEST(NurbsNumerics3dTest, PlaneCurveIsolationReturnsTwoNarrowRoots)
    {
        constexpr double firstRoot = 0.5001;
        constexpr double secondRoot = 0.5002;
        constexpr double constant = firstRoot * secondRoot;
        constexpr double linear = -(firstRoot + secondRoot);
        const SCNurbsCurve3d curve(2,
            {{0.0, 0.0, constant}, {0.5, 0.0, constant + linear * 0.5},
             {1.0, 0.0, constant + linear + 1.0}},
            {0.0, 0.0, 0.0, 1.0, 1.0, 1.0});
        const Geometry::SCPlane plane = Geometry::SCPlane::FromPointAndNormal(
            SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0});
        Geometry::SCGeometryTolerance3d tolerance;
        tolerance.distanceEpsilon = 1e-14;
        tolerance.parameterEpsilon = 1e-12;
        ASSERT_TRUE(curve.IsValid(tolerance));

        const auto result = Geometry::IntersectAllPlaneCurveWithStatus(plane, curve, tolerance);
        ASSERT_EQ(result.hits.size(), 2U);
        EXPECT_NEAR(result.hits[0].curveParameter, firstRoot, 1e-7);
        EXPECT_NEAR(result.hits[1].curveParameter, secondRoot, 1e-7);
        EXPECT_EQ(result.state, Geometry::SCGeometryOperationState::HasResult);
        EXPECT_EQ(result.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Unknown);
        for (const auto& hit : result.hits)
            EXPECT_LE(std::abs(plane.SignedDistanceTo(hit.point, tolerance.distanceEpsilon)),
                      tolerance.distanceEpsilon);
    }

    TEST(NurbsNumerics3dTest, UnprovenEmptyCurveIntersectionsKeepHitEvidenceUnknown)
    {
        const SCNurbsCurve3d curve(2,
            {{0.0, 0.0, 1e-4}, {0.5, 0.0, 5e-7}, {1.0, 0.0, 1e-4}},
            {0.0, 0.0, 0.0, 1.0, 1.0, 1.0});
        const Geometry::SCPlane plane = Geometry::SCPlane::FromPointAndNormal(
            {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0});
        const SCLine3d line = SCLine3d::FromOriginAndDirection(
            {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
        Geometry::SCGeometryTolerance3d tolerance{};
        tolerance.distanceEpsilon = 1e-6;
        ASSERT_TRUE(curve.IsValid(tolerance));

        const auto planeResult = Geometry::IntersectAllPlaneCurveWithStatus(plane, curve, tolerance);
        const auto lineResult = Geometry::IntersectAllLineCurveWithStatus(line, curve, tolerance);
        for (const auto* result : {&planeResult.evidence, &lineResult.evidence})
        {
            EXPECT_EQ(result->hit, Geometry::SCGeometryEvidenceState::Unknown);
            EXPECT_EQ(result->negativeCompleteness, Geometry::SCGeometryEvidenceState::Unknown);
        }
        EXPECT_TRUE(planeResult.hits.empty());
        EXPECT_TRUE(lineResult.hits.empty());
        EXPECT_EQ(planeResult.state, Geometry::SCGeometryOperationState::Unknown);
        EXPECT_EQ(lineResult.state, Geometry::SCGeometryOperationState::Unknown);

        const SCNurbsCurve3d separated(1,
            {{0.0, 0.0, 1.0}, {1.0, 0.0, 2.0}},
            {0.0, 0.0, 1.0, 1.0});
        const auto provenPlaneMiss = Geometry::IntersectAllPlaneCurveWithStatus(
            plane, separated, tolerance);
        const auto provenLineMiss = Geometry::IntersectAllLineCurveWithStatus(
            line, separated, tolerance);
        for (const auto* result : {&provenPlaneMiss.evidence, &provenLineMiss.evidence})
        {
            EXPECT_EQ(result->hit, Geometry::SCGeometryEvidenceState::NotApplicable);
            EXPECT_EQ(result->negativeCompleteness, Geometry::SCGeometryEvidenceState::Verified);
        }
        EXPECT_EQ(provenPlaneMiss.state, Geometry::SCGeometryOperationState::NoResult);
        EXPECT_EQ(provenLineMiss.state, Geometry::SCGeometryOperationState::NoResult);
    }

    TEST(NurbsNumerics3dTest, LineCurveUnrepresentableProjectionKeepsCompletenessUnknown)
    {
        const SCNurbsCurve3d curve(1,
            {{-1e308, 0.0, -1.0}, {-1e308, 0.0, 1.0}},
            {0.0, 0.0, 1.0, 1.0});
        const SCLine3d line = SCLine3d::FromOriginAndDirection(
            {1e308, 0.0, 0.0}, {1.0, 0.0, 0.0});
        const Geometry::SCPlane plane = Geometry::SCPlane::FromPointAndNormal(
            {1e308, 0.0, 0.0}, {0.0, 0.0, 1.0});
        const Geometry::SCGeometryTolerance3d tolerance{};
        ASSERT_TRUE(curve.IsValid(tolerance));
        ASSERT_TRUE(line.IsValid(tolerance.distanceEpsilon));

        const auto planeResult = Geometry::IntersectAllPlaneCurveWithStatus(plane, curve, tolerance);
        ASSERT_FALSE(planeResult.hits.empty());
        ASSERT_EQ(planeResult.evidence.negativeCompleteness,
                  Geometry::SCGeometryEvidenceState::Verified);

        const auto lineResult = Geometry::IntersectAllLineCurveWithStatus(line, curve, tolerance);
        EXPECT_TRUE(lineResult.hits.empty());
        EXPECT_EQ(lineResult.state, Geometry::SCGeometryOperationState::Unknown);
        EXPECT_EQ(lineResult.evidence.hit, Geometry::SCGeometryEvidenceState::Unknown);
        EXPECT_EQ(lineResult.evidence.negativeCompleteness,
                  Geometry::SCGeometryEvidenceState::Unknown);

        const SCNurbsCurve3d separated(1,
            {{0.0, 2.0, -1.0}, {0.0, 2.0, 1.0}},
            {0.0, 0.0, 1.0, 1.0});
        const SCLine3d ordinaryLine = SCLine3d::FromOriginAndDirection(
            {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
        const auto provenMiss = Geometry::IntersectAllLineCurveWithStatus(
            ordinaryLine, separated, tolerance);
        EXPECT_TRUE(provenMiss.hits.empty());
        EXPECT_EQ(provenMiss.state, Geometry::SCGeometryOperationState::NoResult);
        EXPECT_EQ(provenMiss.evidence.negativeCompleteness,
                  Geometry::SCGeometryEvidenceState::Verified);
    }

    TEST(NurbsNumerics3dTest, PlaneCurveIsolationKeepsRefinementInsideCandidateCells)
    {
        const SCNurbsCurve3d curve(2,
            {{0.0, 0.0, 0.16}, {0.5, 0.0, -0.34}, {1.0, 0.0, 0.16}},
            {0.0, 0.0, 0.0, 1.0, 1.0, 1.0});
        const Geometry::SCPlane plane = Geometry::SCPlane::FromPointAndNormal(
            SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0});
        Geometry::SCGeometryTolerance3d tolerance;
        tolerance.distanceEpsilon = 1e-12;
        tolerance.parameterEpsilon = 0.05;
        ASSERT_TRUE(curve.IsValid(tolerance));

        const auto result = Geometry::IntersectAllPlaneCurveWithStatus(plane, curve, tolerance);
        ASSERT_EQ(result.hits.size(), 2U);
        EXPECT_NEAR(result.hits[0].curveParameter, 0.2, 1e-5);
        EXPECT_NEAR(result.hits[1].curveParameter, 0.8, 1e-5);
        for (const auto& hit : result.hits)
            EXPECT_LE(std::abs(plane.SignedDistanceTo(hit.point, tolerance.distanceEpsilon)),
                      tolerance.distanceEpsilon);
    }

    TEST(NurbsNumerics3dTest, PlaneCurveIsolationDistinguishesTangentAndCoincidentSpan)
    {
        const Geometry::SCPlane plane = Geometry::SCPlane::FromPointAndNormal(
            SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0});
        const SCNurbsCurve3d tangent(2,
            {{0.0, 0.0, 0.0625}, {0.5, 0.0, -0.1875}, {1.0, 0.0, 0.5625}},
            {0.0, 0.0, 0.0, 1.0, 1.0, 1.0});
        ASSERT_TRUE(tangent.IsValid());
        const auto tangentResult = Geometry::IntersectAllPlaneCurveWithStatus(plane, tangent);
        ASSERT_FALSE(tangentResult.hits.empty());
        EXPECT_NEAR(tangentResult.hits.front().curveParameter, 0.25, 1e-5);
        EXPECT_EQ(tangentResult.state, Geometry::SCGeometryOperationState::HasResult);

        const SCNurbsCurve3d coincident(1,
            {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}, {0.0, 0.0, 1.0, 1.0});
        const auto coincidentResult = Geometry::IntersectAllPlaneCurveWithStatus(plane, coincident);
        EXPECT_EQ(coincidentResult.state, Geometry::SCGeometryOperationState::NonUnique);
        EXPECT_TRUE(coincidentResult.hits.empty());
    }

    TEST(NurbsNumerics3dTest, ShortLineCurveCrossingPlaneHasOneIntersection)
    {
        const Geometry::SCPlane plane = Geometry::SCPlane::FromPointAndNormal(
            SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0});
        const Geometry::SCLineCurve3d crossing = Geometry::SCLineCurve3d::FromLine(
            SCLine3d::FromOriginAndDirection(
                SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}),
            {-5e-10, 5e-10});
        ASSERT_TRUE(crossing.IsValid());
        const auto result = Geometry::IntersectAllPlaneCurveWithStatus(plane, crossing);
        ASSERT_EQ(result.hits.size(), 1U);
        EXPECT_EQ(result.state, Geometry::SCGeometryOperationState::HasResult);
        EXPECT_EQ(result.evidence.negativeCompleteness,
                  Geometry::SCGeometryEvidenceState::Verified);
        EXPECT_NEAR(result.hits.front().curveParameter, 0.0, 1e-12);

        const Geometry::SCLineCurve3d pointCurve = Geometry::SCLineCurve3d::FromLine(
            SCLine3d::FromOriginAndDirection(
                SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
            {2.0, 2.0});
        const auto pointResult = Geometry::IntersectAllPlaneCurveWithStatus(plane, pointCurve);
        ASSERT_EQ(pointResult.hits.size(), 1U);
        EXPECT_EQ(pointResult.state, Geometry::SCGeometryOperationState::HasResult);
        EXPECT_NEAR(pointResult.hits.front().point.x, 2.0, 1e-12);
    }

    TEST(NurbsNumerics3dTest, PlaneCurveIsolationHandlesSingularCrossing)
    {
        const SCNurbsCurve3d curve(3,
            {{0.0, 0.0, -0.125}, {1.0 / 3.0, 0.0, 0.125},
             {2.0 / 3.0, 0.0, -0.125}, {1.0, 0.0, 0.125}},
            {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0});
        const Geometry::SCPlane plane = Geometry::SCPlane::FromPointAndNormal(
            SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0});
        Geometry::SCGeometryTolerance3d tolerance;
        tolerance.distanceEpsilon = 1e-14;
        tolerance.parameterEpsilon = 1e-12;
        ASSERT_TRUE(curve.IsValid(tolerance));
        const auto result = Geometry::IntersectAllPlaneCurveWithStatus(plane, curve, tolerance);
        ASSERT_EQ(result.hits.size(), 1U);
        EXPECT_NEAR(result.hits.front().curveParameter, 0.5, 1e-6);
        EXPECT_EQ(result.state, Geometry::SCGeometryOperationState::HasResult);
    }

    TEST(NurbsNumerics3dTest, PlaneCurveIsolationReportsUnresolvedRootWithoutFalseNegative)
    {
        const Geometry::SCPlane plane = Geometry::SCPlane::FromPointAndNormal(
            SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0});
        Geometry::SCGeometryTolerance3d tolerance;
        tolerance.distanceEpsilon = 1e-30;
        const SCNurbsCurve3d crossing(2,
            {{0.0, 0.0, -0.3}, {0.5, 0.0, -0.3}, {1.0, 0.0, 0.7}},
            {0.0, 0.0, 0.0, 1.0, 1.0, 1.0});
        ASSERT_TRUE(crossing.IsValid(tolerance));
        const auto result = Geometry::IntersectAllPlaneCurveWithStatus(plane, crossing, tolerance);
        for (const auto& hit : result.hits)
            EXPECT_LE(std::abs(plane.SignedDistanceTo(hit.point, tolerance.distanceEpsilon)),
                      tolerance.distanceEpsilon);
        if (result.hits.empty())
        {
            EXPECT_EQ(result.state, Geometry::SCGeometryOperationState::Unknown);
            EXPECT_EQ(result.evidence.negativeCompleteness,
                      Geometry::SCGeometryEvidenceState::Unknown);
        }

        const SCNurbsCurve3d separated(1,
            {{0.0, 0.0, 1.0}, {1.0, 0.0, 2.0}}, {0.0, 0.0, 1.0, 1.0});
        const auto noHit = Geometry::IntersectAllPlaneCurveWithStatus(plane, separated);
        EXPECT_TRUE(noHit.hits.empty());
        EXPECT_EQ(noHit.state, Geometry::SCGeometryOperationState::NoResult);
        EXPECT_EQ(noHit.evidence.negativeCompleteness,
                  Geometry::SCGeometryEvidenceState::Verified);
    }

    TEST(NurbsNumerics3dTest, PlaneCurveIsolationHandlesRationalInteriorKnot)
    {
        const SCNurbsCurve3d curve(2,
            {{0.0, 0.0, -1.0}, {0.3, 0.0, -0.5},
             {0.7, 0.0, 0.5}, {1.0, 0.0, 1.0}},
            {0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0},
            {1.0, 2.0, 2.0, 1.0});
        const Geometry::SCPlane plane = Geometry::SCPlane::FromPointAndNormal(
            SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0});
        ASSERT_TRUE(curve.IsValid());
        const auto result = Geometry::IntersectAllPlaneCurveWithStatus(plane, curve);
        ASSERT_EQ(result.hits.size(), 1U);
        EXPECT_NEAR(result.hits[0].curveParameter, 0.5, 1e-9);
        EXPECT_LE(std::abs(result.hits[0].point.z), 1e-9);
    }

    TEST(NurbsNumerics3dTest, LineCurveIsolationReturnsMultipleIntersections)
    {
        constexpr double firstRoot = 0.25;
        constexpr double secondRoot = 0.75;
        const SCNurbsCurve3d curve(2,
            {{0.0, 0.0, firstRoot * secondRoot},
             {0.5, 0.0, firstRoot * secondRoot - 0.5},
             {1.0, 0.0, firstRoot * secondRoot}},
            {0.0, 0.0, 0.0, 1.0, 1.0, 1.0});
        const SCLine3d line = SCLine3d::FromOriginAndDirection(
            SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, 0.0});
        ASSERT_TRUE(curve.IsValid());
        const auto result = Geometry::IntersectAllLineCurveWithStatus(line, curve);
        ASSERT_EQ(result.hits.size(), 2U);
        EXPECT_NEAR(result.hits[0].curveParameter, firstRoot, 1e-7);
        EXPECT_NEAR(result.hits[1].curveParameter, secondRoot, 1e-7);
        EXPECT_EQ(result.state, Geometry::SCGeometryOperationState::HasResult);
        EXPECT_EQ(result.evidence.negativeCompleteness,
                  Geometry::SCGeometryEvidenceState::Unknown);
    }

    TEST(NurbsNumerics3dTest, LineCurveIsolationHandlesPlanarAndCoincidentCurves)
    {
        const SCLine3d line = SCLine3d::FromOriginAndDirection(
            SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, 0.0});
        const SCNurbsCurve3d planar(1,
            {{0.0, -1.0, 0.0}, {1.0, 1.0, 0.0}}, {0.0, 0.0, 1.0, 1.0});
        const auto pointHit = Geometry::IntersectAllLineCurveWithStatus(line, planar);
        ASSERT_EQ(pointHit.hits.size(), 1U);
        EXPECT_NEAR(pointHit.hits.front().curveParameter, 0.5, 1e-9);

        const SCNurbsCurve3d coincident(1,
            {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}, {0.0, 0.0, 1.0, 1.0});
        const auto overlap = Geometry::IntersectAllLineCurveWithStatus(line, coincident);
        EXPECT_EQ(overlap.state, Geometry::SCGeometryOperationState::NonUnique);
        EXPECT_TRUE(overlap.hits.empty());
    }

    TEST(NurbsNumerics3dTest, LineCurveIsolationClassifiesPartialOverlapBySharedSpan)
    {
        const SCLine3d line = SCLine3d::FromOriginAndDirection(
            SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, 0.0});
        const SCNurbsCurve3d partialOverlap(1,
            {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 1.0, 0.0}},
            {0.0, 0.0, 0.5, 1.0, 1.0});
        ASSERT_TRUE(partialOverlap.IsValid());
        const auto overlap = Geometry::IntersectAllLineCurveWithStatus(line, partialOverlap);
        EXPECT_EQ(overlap.state, Geometry::SCGeometryOperationState::NonUnique);
        EXPECT_EQ(overlap.evidence.hit, Geometry::SCGeometryEvidenceState::Verified);

        const SCNurbsCurve3d separatePlanarSpans(1,
            {{0.0, 1.0, 0.0}, {1.0, 2.0, 0.0},
             {2.0, 0.0, 1.0}, {3.0, 0.0, 2.0}},
            {0.0, 0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0, 1.0});
        ASSERT_TRUE(separatePlanarSpans.IsValid());
        const auto separate = Geometry::IntersectAllLineCurveWithStatus(line, separatePlanarSpans);
        EXPECT_NE(separate.state, Geometry::SCGeometryOperationState::NonUnique);

        const SCNurbsCurve3d sharedKnot(1,
            {{0.0, 1.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 1.0}},
            {0.0, 0.0, 0.5, 1.0, 1.0});
        ASSERT_TRUE(sharedKnot.IsValid());
        const auto pointHit = Geometry::IntersectAllLineCurveWithStatus(line, sharedKnot);
        ASSERT_EQ(pointHit.hits.size(), 1U);
        EXPECT_NEAR(pointHit.hits.front().curveParameter, 0.5, 1e-12);
        EXPECT_EQ(pointHit.state, Geometry::SCGeometryOperationState::HasResult);
        EXPECT_EQ(pointHit.evidence.negativeCompleteness,
                  Geometry::SCGeometryEvidenceState::Unknown);
    }

    TEST(NurbsNumerics3dTest, LineSurfaceCollectionReturnsTwoVerifiedHits)
    {
        const SCNurbsSurface surface(2, 1, 3, 2,
            {{0.1875, 0.0, 0.0}, {-0.3125, 0.0, 0.5}, {0.1875, 0.0, 1.0},
             {0.1875, 1.0, 0.0}, {-0.3125, 1.0, 0.5}, {0.1875, 1.0, 1.0}},
            {0.0, 0.0, 0.0, 1.0, 1.0, 1.0}, {0.0, 0.0, 1.0, 1.0});
        const SCLine3d line = SCLine3d::FromOriginAndDirection(
            SCPoint3d{0.0, 0.5, 0.0}, SCVector3d{0.0, 0.0, 1.0});
        ASSERT_TRUE(surface.IsValid());
        const auto result = Geometry::IntersectAllLineSurfaceWithStatus(line, surface);
        ASSERT_EQ(result.hits.size(), 2U);
        EXPECT_NEAR(result.hits[0].u, 0.25, 1e-6);
        EXPECT_NEAR(result.hits[1].u, 0.75, 1e-6);
        EXPECT_EQ(result.evidence.negativeCompleteness,
                  Geometry::SCGeometryEvidenceState::Unknown);
        for (const auto& hit : result.hits)
            EXPECT_TRUE(hit.point.AlmostEquals(line.PointAt(hit.lineParameter), 1e-9));
    }

    TEST(NurbsNumerics3dTest, CurvePlaneBisectionDoesNotReportPointOutsideTolerance)
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

}  // namespace
