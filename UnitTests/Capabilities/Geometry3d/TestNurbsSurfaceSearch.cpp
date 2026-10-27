#include <gtest/gtest.h>
#include <cmath>
#include <chrono>
#include <iostream>
#include <vector>

#include "Geometry.h"
#include "Geometry3d/SCLineCurve3d.h"
#include "Geometry3d/SCOffsetSurface.h"
#include "Geometry3d/SCPlaneSurface.h"
#include "Support/Fixtures3d.h"
#include "../../../Source/Core/Detail/SurfaceProjectionSolver.h"

using Geometry::Intersect;
using Geometry::ProjectPointToCurve;
using Geometry::ProjectPointToSurface;
using Geometry::SCCurveOnSurface;
using Geometry::SCLine3d;
using Geometry::SCNurbsCurve3d;
using Geometry::SCNurbsSurface;
using Geometry::SCPoint3d;
using Geometry::SCPolyline2d;
using Geometry::SCVector3d;

namespace
{
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
