#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <utility>

#include "Core/GeometryEvidence.h"
#include "Geometry.h"
#include "Support/Fixtures3d.h"

namespace
{
    using Geometry::SCGeometryEvidenceState;
    using Geometry::SCGeometryFailureReason;
    using Geometry::SCGeometryOperationState;
    using Geometry::SCPoint3d;

    TEST(GeometryEvidenceTest, OpenObjectsKeepIntersectionAndBoundaryButCannotClassifyVolume)
    {
        using namespace Geometry;
        const auto cube=Geometry::Test::BuildUnitCubeBody();
        const PolyhedronBody open({cube.FaceAtRef(0)});
        ASSERT_TRUE(open.IsValid());
        const auto line=SCLine3d::FromOriginAndDirection({0.5,0.5,1},{0,0,-1});
        const auto hit=IntersectLinePolyhedronBodyWithStatus(line,open);
        ASSERT_EQ(hit.state,SCGeometryOperationState::HasResult);
        ASSERT_TRUE(hit.payloadValid);
        ASSERT_TRUE(hit.payload.intersects);
        ASSERT_EQ(hit.payload.hits.size(),1u);
        EXPECT_TRUE(hit.payload.hits[0].intersects);
        EXPECT_EQ(hit.evidence.negativeCompleteness,SCGeometryEvidenceState::Verified);
        const auto boundary=LocatePointWithStatus(SCPoint3d{0.5,0.5,0},open);
        ASSERT_TRUE(boundary.payloadValid);
        EXPECT_EQ(boundary.payload,SCPointContainment3d::OnBoundary);
        const auto away=LocatePointWithStatus(SCPoint3d{0.5,0.5,1},open);
        EXPECT_EQ(away.state,SCGeometryOperationState::Unknown);
        EXPECT_FALSE(away.payloadValid);
        EXPECT_TRUE(HasGeometryFailureReason(away.evidence.reasons,SCGeometryFailureReason::NotClosed));
        const TriangleMesh mesh({{0,0,0},{1,0,0},{0,1,0}},{{0,1,2}});
        const auto meshHit=IntersectLineTriangleMeshWithStatus(
            SCLine3d::FromOriginAndDirection({0.2,0.2,1},{0,0,-1}),mesh);
        ASSERT_TRUE(meshHit.payloadValid);
        EXPECT_EQ(meshHit.payload.points.size(),meshHit.hitEvidence.size());
        EXPECT_EQ(LocatePointWithStatus(SCPoint3d{0.2,0.2,1},mesh).state,SCGeometryOperationState::Unknown);
        EXPECT_EQ(LocatePointWithStatus(SCPoint3d{0.2,0.2,0},mesh).payload,SCPointContainment3d::OnBoundary);
    }

    TEST(GeometryEvidenceTest, ClosedCubeAndPlaneEnumsAreValidResults)
    {
        using namespace Geometry;
        const auto cube=Geometry::Test::BuildUnitCubeBody();
        for (const auto& item : {std::pair{SCPoint3d{0.5,0.5,0.5},SCPointContainment3d::Inside},
                                std::pair{SCPoint3d{2,2,2},SCPointContainment3d::Outside}}) {
            const auto result=LocatePointWithStatus(item.first,cube);
            ASSERT_EQ(result.state,SCGeometryOperationState::HasResult);
            ASSERT_TRUE(result.payloadValid);
            EXPECT_EQ(result.payload,item.second);
            EXPECT_EQ(result.evidence.negativeCompleteness,SCGeometryEvidenceState::Verified);
        }
        const auto plane=SCPlane::FromPointAndNormal({}, {0,0,1e200});
        for (const auto& item : {std::pair{1.0,SCPointPlaneSide3d::Above},
             std::pair{-1.0,SCPointPlaneSide3d::Below},std::pair{0.0,SCPointPlaneSide3d::OnPlane}}) {
            const auto result=LocatePointWithStatus(SCPoint3d{0,0,item.first},plane);
            ASSERT_TRUE(result.payloadValid);
            EXPECT_EQ(result.state,SCGeometryOperationState::HasResult);
            EXPECT_EQ(result.payload,item.second);
        }
    }

    TEST(GeometryEvidenceTest, NearOriginRayParameterIsNotSilentlyDiscarded)
    {
        using namespace Geometry;
        const auto cube=Geometry::Test::BuildUnitCubeBody();
        SCGeometryTolerance3d tolerance{};
        tolerance.parameterEpsilon=0.01;
        const auto result=LocatePointWithStatus(SCPoint3d{0.999,0.5,0.5},cube,tolerance);
        // 空间距离远超边界容差，但线参数的侧别位于参数误差范围内。
        EXPECT_EQ(result.state,SCGeometryOperationState::Unknown);
        EXPECT_FALSE(result.payloadValid);
        EXPECT_TRUE(HasGeometryFailureReason(result.evidence.reasons,SCGeometryFailureReason::NoProof));
    }

    TEST(GeometryEvidenceTest, InvalidCompositeInputHasPriorityOverPartialCandidate)
    {
        using namespace Geometry;
        const auto cube=Geometry::Test::BuildUnitCubeBody();
        const PolyhedronBody invalid({cube.FaceAtRef(0),PolyhedronFace3d{}});
        EXPECT_EQ(ProjectPointToPolyhedronBodyWithStatus({},invalid).state,SCGeometryOperationState::InvalidInput);
        EXPECT_EQ(IntersectLinePolyhedronBodyWithStatus(SCLine3d::FromOriginAndDirection({}, {0,0,1}),invalid).state,
                  SCGeometryOperationState::InvalidInput);
        const TriangleMesh mesh({{0,0,0},{1,0,0},{0,1,0}},{{0,1,2},{0,1,7}});
        EXPECT_EQ(ProjectPointToTriangleMeshWithStatus({},mesh).state,SCGeometryOperationState::InvalidInput);
        EXPECT_EQ(LocatePointWithStatus(SCPoint3d{},mesh).state,SCGeometryOperationState::InvalidInput);
    }

    TEST(GeometryEvidenceTest, UnrepresentablePlaneOffsetKeepsHitUnknown)
    {
        const auto first = Geometry::SCPlane::FromPointAndNormal(
            {-1e308, 0.0, 0.0}, {1.0, 0.0, 0.0});
        const auto second = Geometry::SCPlane::FromPointAndNormal(
            {1e308, 0.0, 0.0}, {1.0, 0.0, 0.0});
        const auto line = Geometry::SCLine3d::FromOriginAndDirection(
            {1e308, 0.0, 0.0}, {0.0, 1.0, 0.0});
        ASSERT_TRUE(first.IsValid());
        ASSERT_TRUE(second.IsValid());
        ASSERT_TRUE(line.IsValid());

        const auto lineResult = Geometry::IntersectLinePlaneWithStatus(line, first);
        EXPECT_EQ(lineResult.state, SCGeometryOperationState::Unknown);
        EXPECT_EQ(lineResult.evidence.hit, SCGeometryEvidenceState::Unknown);
        const auto planeResult = Geometry::IntersectPlanePlaneWithStatus(first, second);
        EXPECT_EQ(planeResult.state, SCGeometryOperationState::Unknown);
        EXPECT_EQ(planeResult.evidence.hit, SCGeometryEvidenceState::Unknown);
    }

    TEST(GeometryEvidenceTest, UnrepresentableVertexDistanceKeepsHitUnknown)
    {
        const Geometry::SCBrepVertex vertex({1e308, 0.0, 0.0});
        const auto line = Geometry::SCLine3d::FromOriginAndDirection(
            {-1e308, 0.0, 0.0}, {0.0, 1.0, 0.0});
        const auto plane = Geometry::SCPlane::FromPointAndNormal(
            {-1e308, 0.0, 0.0}, {1.0, 0.0, 0.0});
        ASSERT_TRUE(vertex.IsValid());
        ASSERT_TRUE(line.IsValid());
        ASSERT_TRUE(plane.IsValid());

        const auto lineResult = Geometry::IntersectLineBrepVertexWithStatus(line, vertex);
        EXPECT_EQ(lineResult.state, SCGeometryOperationState::Unknown);
        EXPECT_EQ(lineResult.evidence.hit, SCGeometryEvidenceState::Unknown);
        const auto planeResult = Geometry::IntersectPlaneBrepVertexWithStatus(plane, vertex);
        EXPECT_EQ(planeResult.state, SCGeometryOperationState::Unknown);
        EXPECT_EQ(planeResult.evidence.hit, SCGeometryEvidenceState::Unknown);
    }

    TEST(GeometryEvidenceTest, UnprovenLineCurveMissKeepsHitUnknown)
    {
        const auto line = Geometry::SCLine3d::FromOriginAndDirection(
            {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
        const auto offset = Geometry::SCLine3d::FromOriginAndDirection(
            {0.0, 1.0, 0.0}, {1.0, 0.0, 0.0});
        const auto curve = Geometry::SCLineCurve3d::FromLine(offset, {0.0, 1.0});
        ASSERT_TRUE(curve.IsValid());

        const auto result = Geometry::IntersectLineCurveWithStatus(line, curve);
        EXPECT_EQ(result.state, SCGeometryOperationState::Unknown);
        EXPECT_FALSE(result.payloadValid);
        EXPECT_EQ(result.evidence.hit, SCGeometryEvidenceState::Unknown);
        EXPECT_EQ(result.evidence.negativeCompleteness, SCGeometryEvidenceState::Unknown);
    }

    TEST(GeometryEvidenceTest, UnavailablePlanarSurfaceOffsetKeepsHitUnknown)
    {
        const auto plane = Geometry::SCPlane::FromPointAndNormal(
            {-1e308, 0.0, 0.0}, {1.0, 0.0, 0.0});
        const Geometry::SCPlaneSurface surface(
            plane, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0},
            {-1.0, 1.0}, {-1.0, 1.0});
        const auto line = Geometry::SCLine3d::FromOriginAndDirection(
            {1e308, 0.0, 0.0}, {0.0, 1.0, 0.0});
        ASSERT_TRUE(surface.IsValid());
        ASSERT_TRUE(line.IsValid());

        const auto result = Geometry::IntersectLineSurfaceWithStatus(line, surface);
        EXPECT_EQ(result.state, SCGeometryOperationState::Unknown);
        EXPECT_FALSE(result.payloadValid);
        EXPECT_EQ(result.evidence.hit, SCGeometryEvidenceState::Unknown);
        EXPECT_EQ(result.evidence.negativeCompleteness, SCGeometryEvidenceState::Unknown);
    }

    TEST(GeometryEvidenceTest, UnavailableTriangleEvaluationKeepsHitUnknown)
    {
        Geometry::SCGeometryTolerance3d tolerance{};
        tolerance.distanceEpsilon = 1e-25;
        const Geometry::TriangleMesh mesh(
            {{0.0, 0.0, 0.0}, {1e308, 0.0, 0.0}, {1e308, 1e-20, 0.0}},
            {{0, 1, 2}});
        ASSERT_TRUE(mesh.IsValid(tolerance.distanceEpsilon));

        const auto line = Geometry::SCLine3d::FromOriginAndDirection(
            {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});
        const auto result = Geometry::IntersectLineTriangleMeshWithStatus(
            line, mesh, tolerance);
        EXPECT_EQ(result.state, SCGeometryOperationState::Unknown);
        EXPECT_FALSE(result.payloadValid);
        EXPECT_EQ(result.evidence.hit, SCGeometryEvidenceState::Unknown);
        EXPECT_EQ(result.evidence.negativeCompleteness, SCGeometryEvidenceState::Unknown);
        ASSERT_EQ(result.triangles.size(), 1U);
        EXPECT_EQ(result.triangles.front().hit, SCGeometryEvidenceState::Unknown);
        EXPECT_TRUE(Geometry::HasGeometryFailureReason(
            result.evidence.reasons, SCGeometryFailureReason::EvaluationUnavailable));
    }

    TEST(GeometryEvidenceTest, UnavailablePolyhedronProjectionKeepsHitUnknown)
    {
        Geometry::SCGeometryTolerance3d tolerance{};
        tolerance.distanceEpsilon = 1e-12;
        const Geometry::PolyhedronFace3d face(
            Geometry::SCPlane::FromPointAndNormal({}, {0.0, 0.0, 1.0}),
            Geometry::PolyhedronLoop3d(
                {{-1e308, 0.0, 0.0}, {1e308, 0.0, 0.0}, {0.0, 1.0, 0.0}}));
        ASSERT_TRUE(face.IsValid(tolerance));
        const auto line = Geometry::SCLine3d::FromOriginAndDirection(
            {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});
        const auto faceResult = Geometry::IntersectLinePolyhedronFaceWithStatus(
            line, face, tolerance);
        EXPECT_EQ(faceResult.state, SCGeometryOperationState::Unknown);
        EXPECT_EQ(faceResult.evidence.hit, SCGeometryEvidenceState::Unknown);
        const Geometry::PolyhedronBody body({face});
        ASSERT_TRUE(body.IsValid(tolerance));
        const auto bodyResult = Geometry::IntersectLinePolyhedronBodyWithStatus(
            line, body, tolerance);
        EXPECT_EQ(bodyResult.state, SCGeometryOperationState::Unknown);
        EXPECT_EQ(bodyResult.evidence.hit, SCGeometryEvidenceState::Unknown);
    }

    TEST(GeometryEvidenceTest, UnavailableBrepFacePreparationKeepsHitUnknown)
    {
        auto edges = Geometry::Test::BuildPlanarBrepEdgesAtZ5();
        edges[0] = Geometry::SCBrepEdge(
            std::make_shared<Geometry::SCLineCurve3d>(
                Geometry::SCLineCurve3d::FromLine(
                    Geometry::SCLine3d::FromOriginAndDirection(
                        {0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}), {0.0, 1.0})),
            0, 1);
        const Geometry::SCBrepFace face(
            std::make_shared<Geometry::SCPlaneSurface>(
                Geometry::Test::BuildFinitePlaneSurfaceAtZ5()),
            Geometry::Test::BuildPlanarBrepOuterLoop());
        const Geometry::SCBrepBody body(
            Geometry::Test::BuildPlanarBrepVerticesAtZ5(), std::move(edges),
            {Geometry::SCBrepShell({face}, false)});
        ASSERT_TRUE(body.IsValid());
        const auto line = Geometry::SCLine3d::FromOriginAndDirection(
            {1.0, 1.0, 6.0}, {0.0, 0.0, -1.0});
        const auto result = Geometry::IntersectLineBrepBodyWithStatus(line, body);
        EXPECT_EQ(result.state, SCGeometryOperationState::Unknown);
        EXPECT_EQ(result.evidence.hit, SCGeometryEvidenceState::Unknown);
        ASSERT_EQ(result.faces.size(), 1U);
        EXPECT_EQ(result.faces.front().result.evidence.hit,
                  SCGeometryEvidenceState::Unknown);

        const auto faceResult = Geometry::IntersectLineBrepFaceWithStatus(line, face);
        EXPECT_EQ(faceResult.state, SCGeometryOperationState::Unknown);
        EXPECT_EQ(faceResult.evidence.hit, SCGeometryEvidenceState::Unknown);
    }

    TEST(GeometryEvidenceTest, PlanarFaceListAndCoplanarContinuumHaveDifferentPayloadRules)
    {
        using namespace Geometry;
        auto support=std::make_shared<SCPlaneSurface>(SCPlaneSurface::FromPlane(
            SCPlane::FromPointAndNormal({}, {0,0,1})));
        const SCPolyline2d ring({{-2,-2},{2,-2},{2,2},{-2,2}},SCPolylineClosure::Closed);
        const SCBrepFace face(support,Geometry::Test::BuildPlanarBrepOuterLoop(),{},SCCurveOnSurface(support,ring),{});
        const auto finite=IntersectLineBrepFaceWithStatus(SCLine3d::FromOriginAndDirection({0,0,1},{0,0,-1}),face);
        ASSERT_TRUE(finite.payloadValid);
        ASSERT_EQ(finite.hits.size(),1u);
        EXPECT_EQ(finite.hitEvidence.size(),finite.hits.size());
        EXPECT_EQ(finite.evidence.negativeCompleteness,SCGeometryEvidenceState::Verified);
        const auto continuum=IntersectLineBrepFaceWithStatus(SCLine3d::FromOriginAndDirection({}, {1,0,0}),face);
        EXPECT_EQ(continuum.state,SCGeometryOperationState::NonUnique);
        EXPECT_FALSE(continuum.payloadValid);
        EXPECT_EQ(continuum.evidence.hit,SCGeometryEvidenceState::Verified);

        SCGeometryTolerance3d wideAngleTolerance{};
        wideAngleTolerance.angleEpsilon = 0.9;
        const auto slanted = IntersectLineBrepFaceWithStatus(
            SCLine3d::FromOriginAndDirection({0,0,1}, {std::cos(1.0),0,-std::sin(1.0)}),
            face, wideAngleTolerance);
        ASSERT_EQ(slanted.state, SCGeometryOperationState::HasResult);
        ASSERT_EQ(slanted.hits.size(), 1U);
        EXPECT_NEAR(slanted.hits.front().point.z, 0.0, wideAngleTolerance.distanceEpsilon);
    }

    TEST(GeometryEvidenceTest, PlanarTrimProjectionKeepsAnalyticEvidenceAcrossSquaredLengthOverflow)
    {
        using namespace Geometry;
        auto support=std::make_shared<SCPlaneSurface>(
            SCPlane::FromPointAndNormal({}, {0,0,1}),
            SCVector3d{1e200,0,0},SCVector3d{0,1,0},SCIntervald{0,1},SCIntervald{0,1});
        const SCPolyline2d ring({{0,0},{1,0},{1,1},{0,1}},SCPolylineClosure::Closed);
        const SCBrepFace face(support,Geometry::Test::BuildPlanarBrepOuterLoop(),{},
                              SCCurveOnSurface(support,ring),{});
        const SCPoint3d query{5e199,-1,0};
        const auto projection=ProjectPointToBrepFaceWithStatus(query,face);
        ASSERT_EQ(projection.state,SCGeometryOperationState::HasResult);
        ASSERT_TRUE(projection.payloadValid);
        EXPECT_EQ(projection.evidence.optimality,SCGeometryEvidenceState::Verified);
        EXPECT_DOUBLE_EQ(projection.payload.u,0.5);
        EXPECT_DOUBLE_EQ(projection.payload.v,0.0);
        EXPECT_EQ(projection.payload.point,support->PointAt(projection.payload.u,projection.payload.v));
        const auto distance=DistanceWithStatus(query,face);
        ASSERT_TRUE(distance.payloadValid);
        EXPECT_DOUBLE_EQ(distance.payload.distance,1.0);
        EXPECT_EQ(distance.evidence.optimality,SCGeometryEvidenceState::Verified);
    }

    TEST(GeometryEvidenceTest, PlanarTrimFallbackPreservesCandidateWithoutOptimalityProof)
    {
        using namespace Geometry;
        auto support=std::make_shared<SCPlaneSurface>(
            SCPlane::FromPointAndNormal({}, {0,0,1}),
            SCVector3d{1e308,0,0},SCVector3d{0,1,0},SCIntervald{-1,1},SCIntervald{0,1});
        const SCPolyline2d ring({{-1,0},{1,0},{1,1},{-1,1}},SCPolylineClosure::Closed);
        const SCBrepFace face(support,Geometry::Test::BuildPlanarBrepOuterLoop(),{},
                              SCCurveOnSurface(support,ring),{});
        const SCPoint3d query{0,-1,0};
        const auto projection=ProjectPointToBrepFaceWithStatus(query,face);
        ASSERT_EQ(projection.state,SCGeometryOperationState::HasResult);
        ASSERT_TRUE(projection.payloadValid);
        EXPECT_EQ(projection.evidence.optimality,SCGeometryEvidenceState::Unknown);
        EXPECT_TRUE(HasGeometryFailureReason(projection.evidence.reasons,SCGeometryFailureReason::NoProof));
        EXPECT_EQ(projection.payload.point,support->PointAt(projection.payload.u,projection.payload.v));
        const auto distance=DistanceWithStatus(query,face);
        ASSERT_TRUE(distance.payloadValid);
        EXPECT_EQ(distance.evidence.optimality,SCGeometryEvidenceState::Unknown);
        const SCCurveOnSurface boundary(support,SCPolyline2d({{-1,0},{1,0}},SCPolylineClosure::Open));
        const auto curveProjection=ProjectPointToCurveOnSurfaceWithStatus(query,boundary);
        ASSERT_TRUE(curveProjection.payloadValid);
        EXPECT_EQ(curveProjection.evidence.optimality,SCGeometryEvidenceState::Unknown);
        const auto curveDistance=DistanceWithStatus(query,boundary);
        ASSERT_TRUE(curveDistance.payloadValid);
        EXPECT_EQ(curveDistance.evidence.optimality,SCGeometryEvidenceState::Unknown);
    }

    TEST(GeometryEvidenceTest, BoundedPlaneDistinguishesCoplanarMissFromContinuum)
    {
        using namespace Geometry;
        const auto surface=SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal({}, {0,0,1}));
        const auto miss=IntersectLineSurfaceWithStatus(SCLine3d::FromOriginAndDirection({0,3,0},{1,0,0}),surface);
        EXPECT_EQ(miss.state,SCGeometryOperationState::NoResult);
        EXPECT_FALSE(miss.payloadValid);
        EXPECT_EQ(miss.evidence.negativeCompleteness,SCGeometryEvidenceState::Verified);
        const auto continuum=IntersectLineSurfaceWithStatus(SCLine3d::FromOriginAndDirection({}, {1,0,0}),surface);
        EXPECT_EQ(continuum.state,SCGeometryOperationState::NonUnique);
        EXPECT_TRUE(continuum.payload.liesOnSurface);
        EXPECT_FALSE(continuum.payloadValid);
        const auto finite=IntersectLineSurfaceWithStatus(SCLine3d::FromOriginAndDirection({0,0,2},{0,0,-1}),surface);
        ASSERT_TRUE(finite.payloadValid);
        EXPECT_TRUE(finite.payload.intersects);
        EXPECT_EQ(finite.payload.point,surface.PointAt(finite.payload.u,finite.payload.v));
    }

    TEST(GeometryEvidenceTest, AnalyticIntersectionStatesKeepCoincidenceSeparateFromUniquePayload)
    {
        using namespace Geometry;
        const auto plane=SCPlane::FromPointAndNormal({}, {0,0,1e200});
        const auto line=SCLine3d::FromOriginAndDirection({0,0,2},{0,0,-1e200});
        const auto finite=IntersectLinePlaneWithStatus(line,plane);
        ASSERT_TRUE(finite.payloadValid);
        EXPECT_TRUE(finite.payload.intersects);
        EXPECT_EQ(finite.payload.point,line.PointAt(finite.payload.parameter));
        const auto continuous=IntersectLinePlaneWithStatus(SCLine3d::FromOriginAndDirection({}, {1,0,0}),plane);
        EXPECT_EQ(continuous.state,SCGeometryOperationState::NonUnique);
        EXPECT_TRUE(continuous.payload.liesOnPlane);
        EXPECT_FALSE(continuous.payloadValid);
        const auto parallel=IntersectLinePlaneWithStatus(SCLine3d::FromOriginAndDirection({0,0,1},{1,0,0}),plane);
        EXPECT_EQ(parallel.state,SCGeometryOperationState::NoResult);
        EXPECT_EQ(parallel.evidence.negativeCompleteness,SCGeometryEvidenceState::Verified);
        const auto planes=IntersectPlanePlaneWithStatus(plane,SCPlane::FromPointAndNormal({2,0,0},{1e200,0,0}));
        ASSERT_TRUE(planes.payloadValid);
        EXPECT_TRUE(planes.payload.intersects);
        EXPECT_DOUBLE_EQ(planes.payload.line.origin.x,2);
        EXPECT_DOUBLE_EQ(planes.payload.line.origin.z,0);
        const auto coincident=IntersectPlanePlaneWithStatus(plane,plane);
        EXPECT_EQ(coincident.state,SCGeometryOperationState::NonUnique);
        EXPECT_TRUE(coincident.payload.isCoincident);
        EXPECT_FALSE(coincident.payloadValid);
        const SCBrepVertex vertex({0,0,0});
        EXPECT_EQ(IntersectLineBrepVertexWithStatus(line,vertex).state,SCGeometryOperationState::HasResult);
        EXPECT_EQ(IntersectPlaneBrepVertexWithStatus(plane,vertex).state,SCGeometryOperationState::HasResult);
        EXPECT_EQ(IntersectPlaneBrepVertexWithStatus(plane,SCBrepVertex({0,0,1})).state,SCGeometryOperationState::NoResult);
    }

    TEST(GeometryEvidenceTest, LinePlaneIntersectionUsesAngularToleranceInRadians)
    {
        using namespace Geometry;
        const SCPlane plane = SCPlane::FromPointAndNormal({}, {0.0, 0.0, 1.0});
        const SCLine3d line = SCLine3d::FromOriginAndDirection(
            {0.0, 0.0, 1.0}, {std::cos(1.0), 0.0, -std::sin(1.0)});
        SCGeometryTolerance3d tolerance{};
        tolerance.angleEpsilon = 0.9;

        const auto hit = Intersect(line, plane, tolerance);
        EXPECT_TRUE(hit.intersects);
        EXPECT_FALSE(hit.isParallel);
        EXPECT_NEAR(hit.point.z, 0.0, tolerance.distanceEpsilon);
        EXPECT_EQ(IntersectLinePlaneWithStatus(line, plane, tolerance).state,
                  SCGeometryOperationState::HasResult);
    }

    TEST(GeometryEvidenceTest, PlanePlaneIntersectionUsesAngularToleranceInRadians)
    {
        using namespace Geometry;
        const SCPlane first = SCPlane::FromPointAndNormal({}, {0.0, 0.0, 1.0});
        const SCPlane second = SCPlane::FromPointAndNormal(
            {}, {std::sin(1.0), 0.0, std::cos(1.0)});
        SCGeometryTolerance3d tolerance{};
        tolerance.angleEpsilon = 0.9;

        const auto hit = IntersectPlanePlaneWithStatus(first, second, tolerance);
        ASSERT_EQ(hit.state, SCGeometryOperationState::HasResult);
        ASSERT_TRUE(hit.payloadValid);
        EXPECT_TRUE(hit.payload.intersects);
    }

    TEST(GeometryEvidenceTest, LineCurveIntersectionUsesAngularToleranceInRadians)
    {
        using namespace Geometry;
        const SCLine3d line = SCLine3d::FromOriginAndDirection({}, {1.0, 0.0, 0.0});
        const SCVector3d direction{std::cos(1.0), std::sin(1.0), 0.0};
        const SCLineCurve3d curve = SCLineCurve3d::FromLine(
            SCLine3d::FromOriginAndDirection(
                {0.5 - 0.5 * direction.x, -0.5 * direction.y, 0.0}, direction),
            {0.0, 1.0});
        SCGeometryTolerance3d tolerance{};
        tolerance.angleEpsilon = 0.9;

        const auto hit = Intersect(line, curve, tolerance);
        ASSERT_TRUE(hit.intersects);
        EXPECT_NEAR(hit.curveParameter, 0.5, 1e-12);
        EXPECT_NEAR(hit.point.x, 0.5, 1e-12);
    }
}
