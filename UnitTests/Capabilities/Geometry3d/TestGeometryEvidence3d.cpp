#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "Core/GeometryEvidence.h"
#include "Core/Measure.h"
#include "Geometry.h"
#include "Geometry3d/Detail/SurfaceNumerics.h"
#include "Geometry3d/SCRuledSurface.h"
#include "Brep/Tessellation.h"

namespace
{
    using Geometry::SCGeometryEvidenceState;
    using Geometry::SCGeometryFailureReason;
    using Geometry::SCGeometryOperationState;
    using Geometry::SCPoint3d;


    TEST(GeometryEvidence3dTest, VectorRelationsUseAngularToleranceInRadians)
    {
        Geometry::SCGeometryTolerance3d tolerance{};
        tolerance.angleEpsilon = 0.9;
        const Geometry::SCVector3d reference{1.0, 0.0, 0.0};
        const Geometry::SCVector3d rotated{std::cos(1.0), std::sin(1.0), 0.0};
        const Geometry::SCVector3d skew{std::sin(1.0), std::cos(1.0), 0.0};
        EXPECT_FALSE(Geometry::IsParallel(reference, rotated, tolerance));
        EXPECT_FALSE(Geometry::IsPerpendicular(reference, skew, tolerance));

        tolerance.angleEpsilon = 1.1;
        EXPECT_TRUE(Geometry::IsParallel(reference, rotated, tolerance));
        EXPECT_TRUE(Geometry::IsPerpendicular(reference, skew, tolerance));
    }

    TEST(GeometryEvidence3dTest, RuledSurfaceUsesStableNormalizationAndParameterDerivative)
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

    TEST(GeometryEvidence3dTest, TessellationDoesNotOverflowFiniteParameterEndpoints)
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

    TEST(GeometryEvidence3dTest, PointDistanceKeepsFiniteFirstOrderWhenSquareOverflows)
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

    TEST(GeometryEvidence3dTest, PointDistanceDistinguishesRoundedSquareZeroFromCoincidence)
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

    TEST(GeometryEvidence3dTest, PointDistanceRejectsInvalidAndUnrepresentableInputs)
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

    TEST(GeometryEvidence3dTest, LineAndPlaneDistanceKeepInputAndSquareValiditySeparate)
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

    TEST(GeometryEvidence3dTest, VertexProjectionStatusChecksInputAndRealCandidate)
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

    TEST(GeometryEvidence3dTest, FailedProjectionDistancesDoNotReadDefaultZero)
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
    TEST(GeometryEvidence3dTest, CoordinateConsistencyUsesOnlyRepresentableRounding)
    {
        using Geometry::Detail::SameRoundedCoordinate;
        EXPECT_TRUE(SameRoundedCoordinate(1e-200,std::nextafter(1e-200,1.0)));
        EXPECT_FALSE(SameRoundedCoordinate(1e-200,2e-200));
        EXPECT_FALSE(SameRoundedCoordinate(0.0,1e-200));
        EXPECT_TRUE(SameRoundedCoordinate(0.0,std::numeric_limits<double>::denorm_min()));
        EXPECT_FALSE(SameRoundedCoordinate(0.0,std::numeric_limits<double>::infinity()));
    }

    TEST(GeometryEvidence3dTest, ProjectionAndSurfaceDistanceSeparateUnrepresentableSquare)
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

    TEST(GeometryEvidence3dTest, LineCurveProjectionKeepsUnrepresentableDistanceEvidence)
    {
        const Geometry::SCLineCurve3d curve = Geometry::SCLineCurve3d::FromLine(
            Geometry::SCLine3d::FromOriginAndDirection({}, {1.0, 0.0, 0.0}), {0.0, 1.0});
        const SCPoint3d query{0.0, 1e200, 0.0};
        EXPECT_FALSE(Geometry::ProjectPointToCurve(query, curve).success);
        const auto result = Geometry::ProjectPointToCurveWithStatus(query, curve);
        EXPECT_EQ(result.state, SCGeometryOperationState::Unknown);
        EXPECT_FALSE(result.payloadValid);
        EXPECT_TRUE(Geometry::HasGeometryFailureReason(
            result.evidence.reasons, SCGeometryFailureReason::Unrepresentable));
    }

    TEST(GeometryEvidence3dTest, ObliquePlaneProjectionUsesAngularToleranceInRadians)
    {
        using namespace Geometry;
        const SCPlaneSurface surface(
            SCPlane::FromPointAndNormal({}, {0.0, 0.0, 1.0}),
            {1.0, 0.0, 0.0}, {std::cos(1.0), std::sin(1.0), 0.0},
            {0.0, 1.0}, {0.0, 1.0});
        SCGeometryTolerance3d tolerance{};
        tolerance.angleEpsilon = 0.9;
        ASSERT_TRUE(surface.IsValid(tolerance));

        const SCPoint3d onSurface = surface.PointAt(0.4, 0.6);
        const SCPoint3d query{onSurface.x, onSurface.y, 1.0};
        const auto projection = ProjectPointToSurfaceWithStatus(query, surface, tolerance);
        ASSERT_EQ(projection.state, SCGeometryOperationState::HasResult);
        ASSERT_TRUE(projection.payloadValid);
        EXPECT_NEAR(projection.payload.u, 0.4, 1e-12);
        EXPECT_NEAR(projection.payload.v, 0.6, 1e-12);
    }

    TEST(GeometryEvidence3dTest, SurfaceTangentIndependenceUsesAngularToleranceInRadians)
    {
        Geometry::SCSurfaceEval3d evaluation{};
        evaluation.derivativeOrder = 1;
        evaluation.normal = {0.0, 0.0, 1.0};
        evaluation.derivativeU = {1.0, 0.0, 0.0};
        evaluation.derivativeV = {std::cos(1.0), std::sin(1.0), 0.0};
        EXPECT_TRUE(Geometry::Detail::HasIndependentSurfaceTangents(evaluation, 0.9));
        EXPECT_FALSE(Geometry::Detail::HasIndependentSurfaceTangents(evaluation, 1.1));
    }
}
