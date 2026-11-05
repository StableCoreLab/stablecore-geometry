#include <gtest/gtest.h>
#include <cmath>
#include <chrono>
#include <iostream>
#include <vector>

#include "Geometry.h"
#include "Geometry3d/SCLineCurve3d.h"
#include "Geometry3d/SCOffsetSurface.h"
#include "Support/Fixtures3d.h"
#include "../../../Source/Core/Detail/SurfaceProjectionSolver.h"

using Geometry::Intersect;
using Geometry::ProjectPointToCurve;
using Geometry::ProjectPointToSurface;
using Geometry::SCCurveOnSurface;
using Geometry::SCLine3d;
using Geometry::SCNurbsSurface;
using Geometry::SCPoint3d;
using Geometry::SCPolyline2d;
using Geometry::SCVector3d;

namespace
{
    TEST(NurbsNumerics3dTest, FixedLargeCoordinateWorkloadPerformance)
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

    TEST(NurbsNumerics3dTest, MultiCellWorkloadPerformanceMatrix)
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

    TEST(NurbsNumerics3dTest, SurfaceBudgetMarksOnlyTheNextRequiredEvaluationIncomplete)
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

    TEST(NurbsNumerics3dTest, ActualSurfaceBudgetRetainsCandidateWhileLegacyContinues)
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

    TEST(NurbsNumerics3dTest, CurvedOffsetProjectionUsesActualOffsetPositions)
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

    TEST(NurbsNumerics3dTest, BodySurfaceBudgetsRemainIndependentAfterActualTruncation)
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
        for (std::size_t i=0;i<4;++i) vertices.emplace_back(surface->PointAt(ring.VertexAt(i).x,ring.VertexAt(i).y));
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
