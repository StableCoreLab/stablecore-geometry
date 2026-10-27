#include <gtest/gtest.h>
#include <cmath>
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

    TEST(NurbsNumericsTest, PlaneCurveIsolationReturnsTwoNarrowRoots)
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

    TEST(NurbsNumericsTest, PlaneCurveIsolationKeepsRefinementInsideCandidateCells)
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

    TEST(NurbsNumericsTest, PlaneCurveIsolationDistinguishesTangentAndCoincidentSpan)
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

    TEST(NurbsNumericsTest, ShortLineCurveCrossingPlaneHasOneIntersection)
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

    TEST(NurbsNumericsTest, PlaneCurveIsolationHandlesSingularCrossing)
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

    TEST(NurbsNumericsTest, PlaneCurveIsolationReportsUnresolvedRootWithoutFalseNegative)
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

    TEST(NurbsNumericsTest, PlaneCurveIsolationHandlesRationalInteriorKnot)
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

    TEST(NurbsNumericsTest, LineCurveIsolationReturnsMultipleIntersections)
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

    TEST(NurbsNumericsTest, LineCurveIsolationHandlesPlanarAndCoincidentCurves)
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

    TEST(NurbsNumericsTest, LineCurveIsolationClassifiesPartialOverlapBySharedSpan)
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

    TEST(NurbsNumericsTest, LineSurfaceCollectionReturnsTwoVerifiedHits)
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

}  // namespace
