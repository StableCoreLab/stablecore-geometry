#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "Core/GeometryEvidence.h"
#include "Core/Measure.h"
#include "Geometry.h"
#include "Support/Fixtures3d.h"

namespace
{
    using Geometry::SCGeometryEvidenceState;
    using Geometry::SCGeometryFailureReason;
    using Geometry::SCGeometryOperationState;
    using Geometry::SCPoint3d;

    constexpr Geometry::SCVector3d kContainmentProbeRay{1.0, 0.3713906763541037, 0.52999894000318};

    TEST(GeometryEvidence3dTest, ClosedCubeRayUsesHalfOpenSharedEdgeAndVertexOwnership)
    {
        using namespace Geometry;
        const auto cube=Geometry::Test::BuildUnitCubeBody();
        const SCVector3d ray=kContainmentProbeRay;
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

    TEST(GeometryEvidence3dTest, MeshAndBrepShareCanonicalEdgeAndVertexCrossingRules)
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
        const SCVector3d ray=kContainmentProbeRay;
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

    TEST(GeometryEvidence3dTest, CoincidentEndpointPairsDoNotReplaceSharedBrepEdgeIdentity)
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

    TEST(GeometryEvidence3dTest, CrossShellRecordsRetainUnknownFaceAndStableFlatIndices)
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

    TEST(GeometryEvidence3dTest, ReversedCoedgesDoNotReverseTheCurvesIntrinsicEndpoints)
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

    TEST(GeometryEvidence3dTest, LargeMeshKeepsFirstOrderDistanceWhenOnlySquareOverflows)
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

    TEST(GeometryEvidence3dTest, PlaneDistanceRecoversFiniteCancellationAfterDotProductOverflow)
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

    TEST(GeometryEvidence3dTest, OneFaceKeepsMultipleIsolatedRootsWithoutClaimingCompleteness)
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

    TEST(GeometryEvidence3dTest, BodyRetainsFiniteHitsAndContinuousFaceEvidenceSeparately)
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

    TEST(GeometryEvidence3dTest, BodyKeepsIndependentSurfaceSubsearchStatisticsPerFace)
    {
        using namespace Geometry;
        auto surface=std::make_shared<SCNurbsSurface>(Geometry::Test::BuildUnitNurbsSurface());
        const SCPolyline2d ring({{0,0},{1,0},{1,1},{0,1}},SCPolylineClosure::Closed);
        const auto loop=Geometry::Test::BuildPlanarBrepOuterLoop();
        const SCBrepFace face(surface,loop,{},SCCurveOnSurface(surface,ring),{});
        std::vector<SCBrepVertex> vertices;
        std::vector<SCBrepEdge> edges;
        for (std::size_t i=0;i<4;++i) vertices.emplace_back(surface->PointAt(ring.VertexAt(i).x,ring.VertexAt(i).y));
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

    TEST(GeometryEvidence3dTest, CoplanarPolygonKeepsIsolatedTouchesApartFromIntervals)
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
