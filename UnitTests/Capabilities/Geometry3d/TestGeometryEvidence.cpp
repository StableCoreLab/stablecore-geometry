#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <map>

#include "Core/GeometryEvidence.h"
#include "Core/Measure.h"
#include "Geometry.h"
#include "Support/Fixtures3d.h"
#include "Geometry3d/Detail/SurfaceNumerics.h"
#include "Geometry3d/SCRuledSurface.h"
#include "Support/StableNumerics.h"
#include "Support/Geometry2d/Predicate2.h"
#include "Brep/Tessellation.h"

namespace
{
    using Geometry::SCGeometryEvidenceState;
    using Geometry::SCGeometryFailureReason;
    using Geometry::SCGeometryOperationState;
    using Geometry::SCPoint3d;

    TEST(GeometryEvidenceTest, StableParametersAndDimensionalThresholdsKeepExtremeScales)
    {
        using namespace Geometry::Detail;
        const double maximum=std::numeric_limits<double>::max();
        EXPECT_FALSE(StableSpan(-maximum,maximum));
        ASSERT_TRUE(StableMidpoint(maximum,maximum));
        EXPECT_DOUBLE_EQ(*StableMidpoint(maximum,maximum),maximum);
        EXPECT_DOUBLE_EQ(*StableLerp(-maximum,maximum,0.5),0);
        EXPECT_DOUBLE_EQ(*StableNormalizedParameter(-maximum,maximum,0),0.5);
        EXPECT_DOUBLE_EQ(*StableNormalizedParameter(2,2,2),0);
        EXPECT_FALSE(StableAdvance(maximum,maximum));
        EXPECT_FALSE(StableContains(0,1,std::numeric_limits<double>::infinity(),maximum));
        EXPECT_FALSE(*MagnitudeWithinPower(1e-300,1e-200,2));
        EXPECT_TRUE(*MagnitudeWithinPower(1e300,1e200,2));
        EXPECT_FALSE(*MagnitudeWithinPower(1e-300,0,3));
        EXPECT_TRUE(*MagnitudeWithinPower(0,0,3));
        EXPECT_FALSE(Geometry::IsZero(Geometry::SCVector2d{1e-200,0},0));
        EXPECT_FALSE(Geometry::IsZero(Geometry::SCVector2d{1e200,1e200},1e200));
        EXPECT_TRUE(Geometry::IsZero(Geometry::SCVector2d{1e-200,0},1e-200));
        EXPECT_TRUE(Geometry::IsParallel({1e200,0,0},{-1e200,0,0}));
        EXPECT_FALSE(Geometry::IsParallel({1e200,0,0},{0,1e200,0}));
        EXPECT_TRUE(Geometry::IsPerpendicular({1e-200,0,0},{0,1e-200,0}));
        EXPECT_FALSE(Geometry::IsPerpendicular({1e200,0,0},{1e200,0,0}));
    }

    TEST(GeometryEvidenceTest, RuledSurfaceUsesStableNormalizationAndParameterDerivative)
    {
        using namespace Geometry;
        const auto first=SCLineCurve3d::FromLine(SCLine3d::FromOriginAndDirection({0,0,0},{1,0,0}),{0,1});
        const auto second=SCLineCurve3d::FromLine(SCLine3d::FromOriginAndDirection({0,4,0},{1,0,0}),{0,1});
        const auto ruled=SCRuledSurface::FromCurves(first,second,{2,4});
        const auto eval=ruled.Evaluate(0.5,3,1);
        ASSERT_EQ(eval.derivativeOrder,1);
        EXPECT_DOUBLE_EQ(eval.point.y,2);
        EXPECT_DOUBLE_EQ(eval.derivativeV.y,2);
        const double maximum=std::numeric_limits<double>::max();
        const auto wide=SCRuledSurface::FromCurves(first,second,{-maximum,maximum});
        EXPECT_DOUBLE_EQ(wide.PointAt(0.5,0).y,2);
        EXPECT_DOUBLE_EQ(wide.PointAt(0.5,maximum).y,4);
        const double adjacent=std::nextafter(2.0,3.0);
        const auto narrow=SCRuledSurface::FromCurves(first,second,{2,adjacent});
        EXPECT_DOUBLE_EQ(narrow.PointAt(0.5,adjacent).y,4);
    }

    TEST(GeometryEvidenceTest, TessellationDoesNotOverflowFiniteParameterEndpoints)
    {
        using namespace Geometry;
        const double maximum=std::numeric_limits<double>::max();
        const SCPlaneSurface surface(SCPlane::FromPointAndNormal({0,0,0},{0,0,1}),
            {0.5,0,0},{0,1,0},{-maximum,maximum},{0,1});
        ASSERT_TRUE(surface.IsValid());
        const auto mesh=Tessellate(surface,2,1);
        ASSERT_EQ(mesh.VertexCount(),6u);
        for (const auto& point:mesh.Vertices()) EXPECT_TRUE(point.IsValid());
        EXPECT_DOUBLE_EQ(mesh.VertexAt(0).x,-maximum*0.5);
        EXPECT_DOUBLE_EQ(mesh.VertexAt(1).x,0);
        EXPECT_DOUBLE_EQ(mesh.VertexAt(2).x,maximum*0.5);
    }

    TEST(GeometryEvidenceTest, PointDistanceKeepsFiniteFirstOrderWhenSquareOverflows)
    {
        const SCPoint3d origin{};
        const SCPoint3d farPoint{1e200, 0.0, 0.0};
        const auto distance = Geometry::DistanceWithStatus(origin, farPoint);
        ASSERT_EQ(distance.state, SCGeometryOperationState::HasResult);
        ASSERT_TRUE(distance.payloadValid);
        ASSERT_TRUE(distance.payload.distanceValid);
        EXPECT_DOUBLE_EQ(distance.payload.distance, 1e200);
        EXPECT_FALSE(distance.payload.distanceSquaredValid);
        EXPECT_EQ(distance.evidence.optimality, SCGeometryEvidenceState::Verified);
        EXPECT_DOUBLE_EQ(Geometry::Distance(origin, farPoint), 1e200);

        const auto squared = Geometry::DistanceSquaredWithStatus(origin, farPoint);
        EXPECT_EQ(squared.state, SCGeometryOperationState::Unknown);
        EXPECT_FALSE(squared.payloadValid);
        EXPECT_TRUE(Geometry::HasGeometryFailureReason(
            squared.evidence.reasons, SCGeometryFailureReason::Unrepresentable));
        EXPECT_TRUE(std::isinf(Geometry::DistanceSquared(origin, farPoint)));
    }

    TEST(GeometryEvidenceTest, PointDistanceDistinguishesRoundedSquareZeroFromCoincidence)
    {
        const SCPoint3d origin{};
        const SCPoint3d nearby{1e-200, 0.0, 0.0};
        const auto nearbySquared = Geometry::DistanceSquaredWithStatus(origin, nearby);
        ASSERT_EQ(nearbySquared.state, SCGeometryOperationState::HasResult);
        ASSERT_TRUE(nearbySquared.payloadValid);
        ASSERT_TRUE(nearbySquared.payload.distanceValid);
        ASSERT_TRUE(nearbySquared.payload.distanceSquaredValid);
        EXPECT_DOUBLE_EQ(nearbySquared.payload.distance, 1e-200);
        EXPECT_DOUBLE_EQ(nearbySquared.payload.distanceSquared, 0.0);
        EXPECT_DOUBLE_EQ(Geometry::DistanceSquared(origin, nearby), 0.0);

        const auto coincident = Geometry::DistanceSquaredWithStatus(origin, origin);
        ASSERT_EQ(coincident.state, SCGeometryOperationState::HasResult);
        ASSERT_TRUE(coincident.payloadValid);
        EXPECT_DOUBLE_EQ(coincident.payload.distance, 0.0);
        EXPECT_DOUBLE_EQ(coincident.payload.distanceSquared, 0.0);
    }

    TEST(GeometryEvidenceTest, PointDistanceRejectsInvalidAndUnrepresentableInputs)
    {
        const SCPoint3d origin{};
        const SCPoint3d invalid{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0};
        const auto invalidResult = Geometry::DistanceWithStatus(origin, invalid);
        EXPECT_EQ(invalidResult.state, SCGeometryOperationState::InvalidInput);
        EXPECT_FALSE(invalidResult.payloadValid);
        EXPECT_TRUE(Geometry::HasGeometryFailureReason(
            invalidResult.evidence.reasons, SCGeometryFailureReason::InvalidInput));
        EXPECT_TRUE(std::isinf(Geometry::Distance(origin, invalid)));

        const double largest = std::numeric_limits<double>::max();
        const auto unrepresentable = Geometry::DistanceWithStatus(
            SCPoint3d{-largest, 0.0, 0.0}, SCPoint3d{largest, 0.0, 0.0});
        EXPECT_EQ(unrepresentable.state, SCGeometryOperationState::Unknown);
        EXPECT_FALSE(unrepresentable.payloadValid);
        EXPECT_TRUE(Geometry::HasGeometryFailureReason(
            unrepresentable.evidence.reasons, SCGeometryFailureReason::Unrepresentable));
    }

    TEST(GeometryEvidenceTest, LineAndPlaneDistanceKeepInputAndSquareValiditySeparate)
    {
        const SCPoint3d origin{};
        EXPECT_TRUE(std::isinf(Geometry::Distance(origin, Geometry::SCLine3d{})));
        EXPECT_TRUE(std::isinf(Geometry::DistanceSquared(origin, Geometry::SCLine3d{})));
        EXPECT_EQ(Geometry::DistanceWithStatus(origin, Geometry::SCLine3d{}).state,
                  SCGeometryOperationState::InvalidInput);
        EXPECT_TRUE(std::isinf(Geometry::Distance(origin, Geometry::SCPlane{})));
        EXPECT_TRUE(std::isinf(Geometry::DistanceSquared(origin, Geometry::SCPlane{})));
        EXPECT_EQ(Geometry::DistanceWithStatus(origin, Geometry::SCPlane{}).state,
                  SCGeometryOperationState::InvalidInput);

        const Geometry::SCLine3d line = Geometry::SCLine3d::FromOriginAndDirection(
            origin, {1.0, 0.0, 0.0});
        const auto small = Geometry::DistanceWithStatus({0.0, 1e-200, 0.0}, line);
        ASSERT_EQ(small.state, SCGeometryOperationState::HasResult);
        EXPECT_DOUBLE_EQ(small.payload.distance, 1e-200);
        EXPECT_TRUE(small.payload.distanceSquaredValid);
        EXPECT_DOUBLE_EQ(small.payload.distanceSquared, 0.0);

        const Geometry::SCLine3d largeDirection = Geometry::SCLine3d::FromOriginAndDirection(
            origin, {1e200, 0.0, 0.0});
        const auto largeLineDistance = Geometry::DistanceWithStatus({0.0, 1e200, 0.0}, largeDirection);
        ASSERT_EQ(largeLineDistance.state, SCGeometryOperationState::HasResult);
        EXPECT_DOUBLE_EQ(largeLineDistance.payload.distance, 1e200);
        EXPECT_FALSE(largeLineDistance.payload.distanceSquaredValid);

        const Geometry::SCPlane plane = Geometry::SCPlane::FromPointAndNormal(
            origin, {0.0, 0.0, 1e200});
        const auto large = Geometry::DistanceWithStatus({0.0, 0.0, 1e200}, plane);
        ASSERT_EQ(large.state, SCGeometryOperationState::HasResult);
        EXPECT_DOUBLE_EQ(large.payload.distance, 1e200);
        EXPECT_FALSE(large.payload.distanceSquaredValid);
        EXPECT_TRUE(std::isinf(Geometry::DistanceSquared({0.0, 0.0, 1e200}, plane)));

        const Geometry::SCBrepVertex farVertex({1e200, 0.0, 0.0});
        const auto vertexDistance = Geometry::DistanceWithStatus(origin, farVertex);
        ASSERT_EQ(vertexDistance.state, SCGeometryOperationState::HasResult);
        EXPECT_DOUBLE_EQ(vertexDistance.payload.distance, 1e200);
        EXPECT_FALSE(vertexDistance.payload.distanceSquaredValid);
        EXPECT_DOUBLE_EQ(Geometry::Distance(origin, farVertex), 1e200);
        EXPECT_TRUE(std::isinf(Geometry::DistanceSquared(origin, farVertex)));
        EXPECT_FALSE(Geometry::ProjectPointToBrepVertex(origin, farVertex).success);
        const auto farProjection = Geometry::ProjectPointToBrepVertexWithStatus(origin, farVertex);
        EXPECT_EQ(farProjection.state, SCGeometryOperationState::Unknown);
        EXPECT_FALSE(farProjection.payloadValid);
        EXPECT_TRUE(Geometry::HasGeometryFailureReason(
            farProjection.evidence.reasons, SCGeometryFailureReason::Unrepresentable));
    }

    TEST(GeometryEvidenceTest, VertexProjectionStatusChecksInputAndRealCandidate)
    {
        const SCPoint3d point{1.0, 2.0, 3.0};
        const Geometry::SCBrepVertex vertex({1.0, 2.0, 4.0});
        const auto result = Geometry::ProjectPointToBrepVertexWithStatus(point, vertex);
        ASSERT_EQ(result.state, SCGeometryOperationState::HasResult);
        ASSERT_TRUE(result.payloadValid);
        ASSERT_TRUE(result.payload.success);
        EXPECT_EQ(result.payload.point, vertex.Point());
        EXPECT_DOUBLE_EQ(result.payload.distanceSquared, 1.0);
        EXPECT_EQ(result.evidence.hit, SCGeometryEvidenceState::Verified);
        EXPECT_EQ(result.evidence.optimality, SCGeometryEvidenceState::Verified);
        EXPECT_EQ(result.evidence.reasons, SCGeometryFailureReason::None);

        const Geometry::SCBrepVertex invalidVertex(
            SCPoint3d{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0});
        const auto invalid = Geometry::ProjectPointToBrepVertexWithStatus(point, invalidVertex);
        EXPECT_EQ(invalid.state, SCGeometryOperationState::InvalidInput);
        EXPECT_FALSE(invalid.payloadValid);
        EXPECT_TRUE(Geometry::HasGeometryFailureReason(
            invalid.evidence.reasons, SCGeometryFailureReason::InvalidInput));
    }

    TEST(GeometryEvidenceTest, FailedProjectionDistancesDoNotReadDefaultZero)
    {
        const SCPoint3d point{};
        const Geometry::SCNurbsCurve3d invalidCurve;
        const Geometry::SCNurbsSurface invalidSurface;
        const Geometry::SCBrepFace invalidFace;
        const Geometry::SCBrepBody invalidBody;
        const Geometry::PolyhedronFace3d invalidPolyhedronFace;
        const Geometry::PolyhedronBody invalidPolyhedronBody;
        const Geometry::TriangleMesh emptyMesh;
        EXPECT_TRUE(std::isinf(Geometry::DistanceSquared(point, invalidCurve)));
        EXPECT_TRUE(std::isinf(Geometry::Distance(point, invalidCurve)));
        EXPECT_TRUE(std::isinf(Geometry::DistanceSquared(point, invalidSurface)));
        EXPECT_TRUE(std::isinf(Geometry::Distance(point, invalidSurface)));
        EXPECT_TRUE(std::isinf(Geometry::DistanceSquared(point, invalidFace)));
        EXPECT_TRUE(std::isinf(Geometry::Distance(point, invalidFace)));
        EXPECT_TRUE(std::isinf(Geometry::DistanceSquared(point, invalidBody)));
        EXPECT_TRUE(std::isinf(Geometry::Distance(point, invalidBody)));
        EXPECT_TRUE(std::isinf(Geometry::DistanceSquared(point, invalidPolyhedronFace)));
        EXPECT_TRUE(std::isinf(Geometry::DistanceSquared(point, invalidPolyhedronBody)));
        EXPECT_TRUE(std::isinf(Geometry::DistanceSquared(point, emptyMesh)));
    }
    TEST(GeometryEvidenceTest, CoordinateConsistencyUsesOnlyRepresentableRounding)
    {
        using Geometry::Detail::SameRoundedCoordinate;
        EXPECT_TRUE(SameRoundedCoordinate(1e-200,std::nextafter(1e-200,1.0)));
        EXPECT_FALSE(SameRoundedCoordinate(1e-200,2e-200));
        EXPECT_FALSE(SameRoundedCoordinate(0.0,1e-200));
        EXPECT_TRUE(SameRoundedCoordinate(0.0,std::numeric_limits<double>::denorm_min()));
        EXPECT_FALSE(SameRoundedCoordinate(0.0,std::numeric_limits<double>::infinity()));
    }

    TEST(GeometryEvidenceTest, ProjectionAndSurfaceDistanceSeparateUnrepresentableSquare)
    {
        const auto plane=Geometry::SCPlaneSurface::FromPlane(
            Geometry::SCPlane::FromPointAndNormal({}, {0,0,1}));
        const SCPoint3d point{0,0,1e200};
        const auto projection=Geometry::ProjectPointToSurfaceWithStatus(point,plane);
        EXPECT_EQ(projection.state,SCGeometryOperationState::Unknown);
        EXPECT_FALSE(projection.payloadValid);
        EXPECT_FALSE(Geometry::ProjectPointToSurface(point,plane).success);
        const auto distance=Geometry::DistanceWithStatus(point,plane);
        ASSERT_TRUE(distance.payloadValid);
        EXPECT_EQ(distance.state,SCGeometryOperationState::HasResult);
        EXPECT_DOUBLE_EQ(distance.payload.distance,1e200);
        EXPECT_FALSE(distance.payload.distanceSquaredValid);
        EXPECT_DOUBLE_EQ(Geometry::Distance(point,plane),1e200);
        EXPECT_FALSE(Geometry::DistanceSquaredWithStatus(point,plane).payloadValid);
        EXPECT_TRUE(std::isinf(Geometry::DistanceSquared(point,plane)));
    }

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

    TEST(GeometryEvidenceTest, ClosedCubeRayUsesHalfOpenSharedEdgeAndVertexOwnership)
    {
        using namespace Geometry;
        const auto cube=Geometry::Test::BuildUnitCubeBody();
        const SCVector3d ray{1,0.3713906763541037,0.52999894000318};
        for (const auto& item : {
            std::pair{SCPoint3d{0,0,0}-ray,SCPointContainment3d::Outside},
            std::pair{SCPoint3d{1,1,1}-ray*0.5,SCPointContainment3d::Inside},
            std::pair{SCPoint3d{1,1,0.5}-ray*0.25,SCPointContainment3d::Inside},
            std::pair{SCPoint3d{1,0,0.5}-ray,SCPointContainment3d::Outside}}) {
            const auto result=LocatePointWithStatus(item.first,cube);
            ASSERT_EQ(result.state,SCGeometryOperationState::HasResult);
            ASSERT_TRUE(result.payloadValid);
            EXPECT_EQ(result.payload,item.second);
            EXPECT_EQ(result.evidence.negativeCompleteness,SCGeometryEvidenceState::Verified);
        }
    }

    TEST(GeometryEvidenceTest, MeshAndBrepShareCanonicalEdgeAndVertexCrossingRules)
    {
        using namespace Geometry;
        const auto cube=Geometry::Test::BuildUnitCubeBody();
        const auto brep=ConvertToBrepBody(cube);
        ASSERT_TRUE(brep.success);
        std::map<std::array<double,3>,std::size_t> ids;
        std::vector<SCPoint3d> vertices;
        std::vector<TriangleMesh::TriangleIndices> triangles;
        for (const auto& face:cube.Faces()) {
            std::vector<std::size_t> ring;
            for (const auto& p:face.OuterLoopRef().Vertices()) {
                const auto [it,inserted]=ids.emplace(std::array<double,3>{p.x,p.y,p.z},ids.size());
                if (inserted) vertices.push_back(p);
                ring.push_back(it->second);
            }
            for (std::size_t i=1;i+1<ring.size();++i) triangles.push_back({ring[0],ring[i],ring[i+1]});
        }
        const TriangleMesh mesh(std::move(vertices),std::move(triangles));
        ASSERT_TRUE(mesh.IsValid());
        const SCVector3d ray{1,0.3713906763541037,0.52999894000318};
        for (const auto& item:{
            std::pair{SCPoint3d{0,0,0}-ray,SCPointContainment3d::Outside},
            std::pair{SCPoint3d{1,1,1}-ray*0.5,SCPointContainment3d::Inside},
            std::pair{SCPoint3d{1,1,0.5}-ray*0.25,SCPointContainment3d::Inside},
            std::pair{SCPoint3d{1,0,0.5}-ray,SCPointContainment3d::Outside}}) {
            for (const auto result:{LocatePointWithStatus(item.first,brep.body),LocatePointWithStatus(item.first,mesh)}) {
                ASSERT_EQ(result.state,SCGeometryOperationState::HasResult);
                ASSERT_TRUE(result.payloadValid);
                EXPECT_EQ(result.payload,item.second);
                EXPECT_EQ(result.evidence.negativeCompleteness,SCGeometryEvidenceState::Verified);
            }
        }
    }

    TEST(GeometryEvidenceTest, CoincidentEndpointPairsDoNotReplaceSharedBrepEdgeIdentity)
    {
        using namespace Geometry;
        const auto converted=ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
        ASSERT_TRUE(converted.success);
        std::vector<SCBrepEdge> edges;
        std::vector<SCBrepFace> faces;
        for (const auto& face:converted.body.ShellAtRef(0).Faces()) {
            std::vector<SCBrepCoedge> coedges;
            for (const auto& coedge:face.OuterLoopRef().Coedges()) {
                coedges.emplace_back(edges.size(),coedge.Reversed());
                edges.push_back(converted.body.Edges()[coedge.EdgeIndex()]);
            }
            faces.emplace_back(std::shared_ptr<ISCSurface>(face.SupportSurface()->Clone().release()),
                SCBrepLoop(std::move(coedges)),std::vector<SCBrepLoop>{},face.OuterTrim());
        }
        const SCBrepBody disconnectedEdges(converted.body.Vertices(),std::move(edges),{SCBrepShell(std::move(faces),true)});
        ASSERT_TRUE(disconnectedEdges.IsValid());
        const auto result=LocatePointWithStatus(SCPoint3d{0.5,0.5,0.5},disconnectedEdges);
        EXPECT_EQ(result.state,SCGeometryOperationState::Unknown);
        EXPECT_FALSE(result.payloadValid);
        EXPECT_TRUE(HasGeometryFailureReason(result.evidence.reasons,SCGeometryFailureReason::TopologyInvalid));
        const auto boundary=LocatePointWithStatus(SCPoint3d{0,0.5,0.5},disconnectedEdges);
        ASSERT_TRUE(boundary.payloadValid);
        EXPECT_EQ(boundary.payload,SCPointContainment3d::OnBoundary);
    }

    TEST(GeometryEvidenceTest, CrossShellRecordsRetainUnknownFaceAndStableFlatIndices)
    {
        using namespace Geometry;
        auto plane=std::make_shared<SCPlaneSurface>(Geometry::Test::BuildFinitePlaneSurfaceAtZ5());
        auto nurbs=std::make_shared<SCNurbsSurface>(Geometry::Test::BuildUnitNurbsSurface());
        const auto loop=Geometry::Test::BuildPlanarBrepOuterLoop();
        const SCBrepBody body(Geometry::Test::BuildPlanarBrepVerticesAtZ5(),
            Geometry::Test::BuildPlanarBrepEdgesAtZ5(),
            {SCBrepShell({SCBrepFace(plane,loop)}),SCBrepShell({SCBrepFace(nurbs,loop)})});
        ASSERT_TRUE(body.IsValid());
        const auto hit=IntersectLineBrepBodyWithStatus(SCLine3d::FromOriginAndDirection({1,1,6},{0,0,-1}),body);
        ASSERT_TRUE(hit.payloadValid);
        EXPECT_TRUE(hit.payload.intersects);
        ASSERT_EQ(hit.faces.size(),2u);
        for (std::size_t i=0;i<2;++i) {
            EXPECT_EQ(hit.faces[i].shellIndex,i);
            EXPECT_EQ(hit.faces[i].localFaceIndex,0u);
            EXPECT_EQ(hit.faces[i].faceIndex,i);
        }
        EXPECT_EQ(hit.faces[1].result.state,SCGeometryOperationState::Unknown);
        EXPECT_TRUE(HasGeometryFailureReason(hit.faces[1].result.evidence.reasons,SCGeometryFailureReason::RegionUnavailable));
        EXPECT_EQ(hit.payload.hits.size(),hit.payload.faceIndices.size());
        EXPECT_EQ(hit.payload.hits.size(),hit.hitEvidence.size());
        for (auto index:hit.payload.faceIndices) EXPECT_EQ(index,0u);
        EXPECT_EQ(hit.evidence.negativeCompleteness,SCGeometryEvidenceState::Unknown);
        EXPECT_FALSE(Intersect(SCLine3d::FromOriginAndDirection({1,1,6},{0,0,-1}),body).intersects);
        const auto projection=ProjectPointToBrepBodyWithStatus({1,1,6},body);
        ASSERT_TRUE(projection.payloadValid);
        EXPECT_TRUE(projection.payload.success);
        EXPECT_TRUE(projection.payload.projection.success);
        EXPECT_EQ(projection.payload.faceIndex,0u);
        EXPECT_EQ(projection.faces.size(),2u);
        EXPECT_EQ(projection.evidence.optimality,SCGeometryEvidenceState::Unknown);
        EXPECT_FALSE(ProjectPointToBrepBody({1,1,6},body).success);
        const auto classification=LocatePointWithStatus(SCPoint3d{1,1,6},body);
        EXPECT_EQ(classification.state,SCGeometryOperationState::Unknown);
        EXPECT_FALSE(classification.payloadValid);
        EXPECT_EQ(classification.faces.size(),2u);
    }

    TEST(GeometryEvidenceTest, ReversedCoedgesDoNotReverseTheCurvesIntrinsicEndpoints)
    {
        using namespace Geometry;
        auto surface=std::make_shared<SCPlaneSurface>(Geometry::Test::BuildFinitePlaneSurfaceAtZ5());
        const SCBrepLoop reversed({SCBrepCoedge(3,true),SCBrepCoedge(2,true),SCBrepCoedge(1,true),SCBrepCoedge(0,true)});
        const SCBrepBody body(Geometry::Test::BuildPlanarBrepVerticesAtZ5(),Geometry::Test::BuildPlanarBrepEdgesAtZ5(),
            {SCBrepShell({SCBrepFace(surface,reversed)})});
        ASSERT_TRUE(body.IsValid());
        const auto good=ProjectPointToBrepBodyWithStatus({1,1,6},body);
        ASSERT_TRUE(good.payloadValid);
        EXPECT_DOUBLE_EQ(good.payload.projection.point.z,5);
        auto edges=Geometry::Test::BuildPlanarBrepEdgesAtZ5();
        edges[0]=SCBrepEdge(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
            SCLine3d::FromOriginAndDirection({0,0,5},{2,0,0}),{0,1})),1,0);
        const SCBrepBody mismatched(Geometry::Test::BuildPlanarBrepVerticesAtZ5(),std::move(edges),
            {SCBrepShell({SCBrepFace(surface,reversed)})});
        ASSERT_TRUE(mismatched.IsValid());
        const auto unknown=ProjectPointToBrepBodyWithStatus({1,1,6},mismatched);
        EXPECT_EQ(unknown.state,SCGeometryOperationState::Unknown);
        EXPECT_FALSE(unknown.payloadValid);
        EXPECT_TRUE(HasGeometryFailureReason(unknown.evidence.reasons,SCGeometryFailureReason::RegionUnavailable));
    }

    TEST(GeometryEvidenceTest, LargeMeshKeepsFirstOrderDistanceWhenOnlySquareOverflows)
    {
        using namespace Geometry;
        const TriangleMesh mesh({{0,0,0},{1e150,0,0},{0,1e150,0}},{{0,1,2}});
        ASSERT_TRUE(mesh.IsValid());
        const SCPoint3d query{1e149,1e149,1e200};
        const auto distance=DistanceWithStatus(query,mesh);
        ASSERT_EQ(distance.state,SCGeometryOperationState::HasResult);
        ASSERT_TRUE(distance.payloadValid);
        EXPECT_DOUBLE_EQ(distance.payload.distance,1e200);
        EXPECT_FALSE(distance.payload.distanceSquaredValid);
        EXPECT_DOUBLE_EQ(Distance(query,mesh),1e200);
        const auto projection=ProjectPointToTriangleMeshWithStatus(query,mesh);
        EXPECT_EQ(projection.state,SCGeometryOperationState::Unknown);
        EXPECT_FALSE(projection.payloadValid);
        EXPECT_TRUE(HasGeometryFailureReason(projection.evidence.reasons,SCGeometryFailureReason::Unrepresentable));
        EXPECT_FALSE(DistanceSquaredWithStatus(query,mesh).payloadValid);
    }

    TEST(GeometryEvidenceTest, PlaneDistanceRecoversFiniteCancellationAfterDotProductOverflow)
    {
        using namespace Geometry;
        const auto plane=SCPlane::FromPointAndNormal({}, {1,1,-1});
        const SCPoint3d point{1.7e308,1.7e308,1.7e308};
        const auto result=DistanceWithStatus(point,plane);
        ASSERT_TRUE(result.payloadValid);
        EXPECT_NEAR(result.payload.distance/1.7e308,1/std::sqrt(3.0),1e-15);
        EXPECT_FALSE(result.payload.distanceSquaredValid);
        const auto side=LocatePointWithStatus(point,plane);
        ASSERT_TRUE(side.payloadValid);
        EXPECT_EQ(side.payload,SCPointPlaneSide3d::Above);
    }

    TEST(GeometryEvidenceTest, OneFaceKeepsMultipleIsolatedRootsWithoutClaimingCompleteness)
    {
        using namespace Geometry;
        auto surface=std::make_shared<SCNurbsSurface>(1,1,3,2,
            std::vector<SCPoint3d>{{0,0,1},{1,0,-1},{2,0,1},{0,1,1},{1,1,-1},{2,1,1}},
            std::vector<double>{0,0,1,2,2},std::vector<double>{0,0,1,1});
        const SCPolyline2d ring({{0,0},{2,0},{2,1},{0,1}},SCPolylineClosure::Closed);
        const SCBrepFace face(surface,Geometry::Test::BuildPlanarBrepOuterLoop(),{},SCCurveOnSurface(surface,ring),{});
        const auto line=SCLine3d::FromOriginAndDirection({0,0.5,0},{1,0,0});
        const auto result=IntersectLineBrepFaceWithStatus(line,face);
        ASSERT_TRUE(result.payloadValid);
        ASSERT_EQ(result.hits.size(),2u);
        EXPECT_EQ(result.hits.size(),result.hitEvidence.size());
        EXPECT_NEAR(result.hits[0].u,0.5,1e-8);
        EXPECT_NEAR(result.hits[1].u,1.5,1e-8);
        EXPECT_EQ(result.evidence.negativeCompleteness,SCGeometryEvidenceState::Unknown);
        for (const auto& hit:result.hits) {
            EXPECT_TRUE(hit.intersects);
            EXPECT_EQ(hit.point,surface->PointAt(hit.u,hit.v));
            EXPECT_LE(Distance(hit.point,line),1e-8);
        }
    }

    TEST(GeometryEvidenceTest, BodyRetainsFiniteHitsAndContinuousFaceEvidenceSeparately)
    {
        using namespace Geometry;
        const auto cube=Geometry::Test::BuildUnitCubeBody();
        const auto line=SCLine3d::FromOriginAndDirection({0,0.5,0},{1,0,0});
        const PolyhedronBody continuous({cube.FaceAtRef(0)});
        const auto only=IntersectLinePolyhedronBodyWithStatus(line,continuous);
        EXPECT_EQ(only.state,SCGeometryOperationState::NonUnique);
        EXPECT_FALSE(only.payloadValid);
        const auto mixed=IntersectLinePolyhedronBodyWithStatus(line,cube);
        ASSERT_EQ(mixed.state,SCGeometryOperationState::HasResult);
        ASSERT_TRUE(mixed.payloadValid);
        EXPECT_EQ(mixed.payload.hits.size(),mixed.hitEvidence.size());
        EXPECT_TRUE(HasGeometryFailureReason(mixed.evidence.reasons,SCGeometryFailureReason::NonUnique));
        EXPECT_EQ(mixed.evidence.negativeCompleteness,SCGeometryEvidenceState::Unknown);
        EXPECT_FALSE(Intersect(line,cube).intersects);
    }

    TEST(GeometryEvidenceTest, BodyKeepsIndependentSurfaceSubsearchStatisticsPerFace)
    {
        using namespace Geometry;
        auto surface=std::make_shared<SCNurbsSurface>(Geometry::Test::BuildUnitNurbsSurface());
        const SCPolyline2d ring({{0,0},{1,0},{1,1},{0,1}},SCPolylineClosure::Closed);
        const auto loop=Geometry::Test::BuildPlanarBrepOuterLoop();
        const SCBrepFace face(surface,loop,{},SCCurveOnSurface(surface,ring),{});
        std::vector<SCBrepVertex> vertices;
        std::vector<SCBrepEdge> edges;
        for (std::size_t i=0;i<4;++i) vertices.emplace_back(surface->PointAt(ring.PointAt(i).x,ring.PointAt(i).y));
        for (std::size_t i=0;i<4;++i) {
            const auto a=vertices[i].Point(), b=vertices[(i+1)%4].Point();
            edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                SCLine3d::FromOriginAndDirection(a,b-a),{0,1})),i,(i+1)%4);
        }
        const SCBrepBody body(std::move(vertices),std::move(edges),{SCBrepShell({face}),SCBrepShell({face})});
        ASSERT_TRUE(body.IsValid());
        const SCPoint3d query{0.7,0.8,1};
        const auto standalone=ProjectPointToBrepFaceWithStatus(query,face);
        ASSERT_TRUE(standalone.payloadValid);
        const auto projection=ProjectPointToBrepBodyWithStatus(query,body);
        ASSERT_TRUE(projection.payloadValid);
        ASSERT_EQ(projection.faces.size(),2u);
        for (const auto& record:projection.faces) {
            ASSERT_TRUE(record.result.payloadValid);
            EXPECT_EQ(record.result.statistics.positionEvaluations,standalone.statistics.positionEvaluations);
            EXPECT_EQ(record.result.statistics.derivativeEvaluations,standalone.statistics.derivativeEvaluations);
            EXPECT_GT(record.result.statistics.positionEvaluations,0u);
            EXPECT_FALSE(record.result.statistics.searchIncomplete);
        }
    }

    TEST(GeometryEvidenceTest, CoplanarPolygonKeepsIsolatedTouchesApartFromIntervals)
    {
        using namespace Geometry;
        const auto plane=SCPlane::FromPointAndNormal({}, {0,0,1});
        const auto line=SCLine3d::FromOriginAndDirection({}, {1,0,0});
        const PolyhedronFace3d isolated(plane,PolyhedronLoop3d(
            {{-3,2,0},{-2,0,0},{-1,1,0},{1,1,0},{2,0,0},{3,2,0}}));
        const auto points=IntersectLinePolyhedronFaceWithStatus(line,isolated);
        ASSERT_TRUE(points.payloadValid);
        ASSERT_EQ(points.hits.size(),2u);
        EXPECT_EQ(points.hits.size(),points.hitEvidence.size());
        EXPECT_EQ(points.evidence.negativeCompleteness,SCGeometryEvidenceState::Verified);
        const auto body=IntersectLinePolyhedronBodyWithStatus(line,PolyhedronBody({isolated}));
        ASSERT_TRUE(body.payloadValid);
        ASSERT_EQ(body.payload.hits.size(),2u);
        EXPECT_EQ(body.payload.faceIndices,std::vector<std::size_t>({0,0}));
        const PolyhedronFace3d mixed(plane,PolyhedronLoop3d(
            {{-4,2,0},{-3,0,0},{-2,1,0},{0,1,0},{0,0,0},{2,0,0},{2,2,0}}));
        const auto continuum=IntersectLinePolyhedronFaceWithStatus(line,mixed);
        ASSERT_TRUE(continuum.payloadValid);
        ASSERT_EQ(continuum.hits.size(),1u);
        EXPECT_DOUBLE_EQ(continuum.hits[0].point.x,-3);
        EXPECT_EQ(continuum.state,SCGeometryOperationState::HasResult);
        EXPECT_TRUE(HasGeometryFailureReason(continuum.evidence.reasons,SCGeometryFailureReason::NonUnique));
        EXPECT_EQ(continuum.evidence.negativeCompleteness,SCGeometryEvidenceState::Unknown);
    }

}
