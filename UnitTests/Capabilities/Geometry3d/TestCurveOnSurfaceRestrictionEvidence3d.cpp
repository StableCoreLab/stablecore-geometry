#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <vector>

#include "Core/Intersection.h"
#include "Geometry3d/SCCurveOnSurface.h"
#include "Geometry3d/SCNurbsCurve3d.h"
#include "Geometry3d/SCNurbsSurface.h"
#include "../../../Source/Core/Detail/CurveOnSurfaceNumerics.h"

TEST(CurveOnSurfaceInterface3dTest, AxisRestrictionIgnoresOverlapOutsideUvSegment)
{
    auto surface = std::make_shared<Geometry::SCNurbsSurface>(1, 1, 3, 2,
        std::vector<Geometry::SCPoint3d>{{0,0,0}, {0.5,0,0}, {1,0,1},
                                        {0,1,0}, {0.5,1,0}, {1,1,1}},
        std::vector<double>{0,0,0.5,1,1}, std::vector<double>{0,0,1,1},
        std::vector<double>(6, 1.0));
    for (const bool reverse : {false, true})
    {
        const Geometry::SCPoint2d first{reverse ? 1.0 : 0.75, 0.5};
        const Geometry::SCPoint2d second{reverse ? 0.75 : 1.0, 0.5};
        const Geometry::SCCurveOnSurface curve(surface, Geometry::SCPolyline2d({first, second}));
        ASSERT_TRUE(curve.IsValid());
        const auto plane = Geometry::SCPlane::FromPointAndNormal({0,0,0}, {0,0,1});
        const auto line = Geometry::SCLine3d::FromOriginAndDirection({0,0.5,0}, {1,0,0});
        const auto planeHits = Geometry::IntersectAllPlaneCurveWithStatus(plane, curve);
        const auto lineHits = Geometry::IntersectAllLineCurveWithStatus(line, curve);
        EXPECT_EQ(planeHits.state, Geometry::SCGeometryOperationState::Unknown);
        EXPECT_EQ(lineHits.state, Geometry::SCGeometryOperationState::Unknown);
        EXPECT_TRUE(planeHits.hits.empty());
        EXPECT_TRUE(lineHits.hits.empty());
        const Geometry::SCCurveOnSurface overlapping(surface, Geometry::SCPolyline2d({
            {reverse ? 0.25 : 0.0, 0.5}, {reverse ? 0.0 : 0.25, 0.5}}));
        ASSERT_TRUE(overlapping.IsValid());
        EXPECT_EQ(Geometry::IntersectAllPlaneCurveWithStatus(plane, overlapping).state,
                  Geometry::SCGeometryOperationState::NonUnique);
        EXPECT_EQ(Geometry::IntersectAllLineCurveWithStatus(line, overlapping).state,
                  Geometry::SCGeometryOperationState::NonUnique);
    }
}

TEST(CurveOnSurfaceInterface3dTest, LostSurfaceWeightsDoNotProvePlaneOverlap)
{
    for (const bool lostWeight : {false, true})
    {
        const double height = lostWeight ? 1.0 : 1e-30;
        const double high = lostWeight ? 1e200 : 1.0;
        const double low = lostWeight ? 1e-200 : 1e-300;
        const auto surface = std::make_shared<Geometry::SCNurbsSurface>(1, 1, 2, 2,
            std::vector<Geometry::SCPoint3d>{{0,0,0}, {1,0,height}, {0,1,height}, {1,1,0}},
            std::vector<double>{0,0,1,1}, std::vector<double>{0,0,1,1},
            std::vector<double>{high,low,low,high});
        for (const auto& uv : {Geometry::SCPolyline2d({{0.25,0.25},{0.75,0.75}}),
                               Geometry::SCPolyline2d({{0,0.5},{1,0.5}}),
                               Geometry::SCPolyline2d({{0.5,0},{0.5,1}})})
        {
            const Geometry::SCCurveOnSurface curve(surface, uv);
            ASSERT_TRUE(curve.IsValid());
            const auto plane = Geometry::SCPlane::FromPointAndNormal({0,0,0}, {0,0,1});
            const auto result = Geometry::IntersectAllPlaneCurveWithStatus(plane, curve);
            EXPECT_NE(result.state, Geometry::SCGeometryOperationState::NonUnique);
            EXPECT_EQ(result.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Unknown);
        }
    }
}

TEST(CurveOnSurfaceInterface3dTest, BernsteinProductUnderflowDoesNotProvePlaneOverlap)
{
    const double height = std::numeric_limits<double>::denorm_min();
    const auto surface = std::make_shared<Geometry::SCNurbsSurface>(1, 1, 2, 2,
        std::vector<Geometry::SCPoint3d>{{0,0,0}, {1,0,height}, {0,1,height}, {1,1,0}},
        std::vector<double>{0,0,1,1}, std::vector<double>{0,0,1,1},
        std::vector<double>(4, 1.0));
    const Geometry::SCCurveOnSurface curve(surface, Geometry::SCPolyline2d({{0,0},{1,1}}));
    ASSERT_TRUE(curve.IsValid());
    const auto result = Geometry::IntersectAllPlaneCurveWithStatus(
        Geometry::SCPlane::FromPointAndNormal({0,0,0}, {0,0,1}), curve);
    EXPECT_NE(result.state, Geometry::SCGeometryOperationState::NonUnique);
    EXPECT_EQ(result.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Unknown);
}

TEST(CurveOnSurfaceInterface3dTest, DegreeReductionPreservesRepresentableTinyCurvature)
{
    using Control = Geometry::Detail::HomogeneousSurfaceControl;
    for (const double height : {1e-6, 1e-300, 4.0 * std::numeric_limits<double>::denorm_min()})
    {
        SCOPED_TRACE(height);
        std::vector<Control> curved{{0,0,0,1}, {0.5,0,height,1}, {1,0,0,1}};
        EXPECT_FALSE(Geometry::Detail::TryReduceBezierControlDegree(curved));
        EXPECT_EQ(curved.size(), 3U);
        std::vector<Control> linear{{0,0,0,1}, {0.5,0,height * 0.5,1}, {1,0,height,1}};
        EXPECT_TRUE(Geometry::Detail::TryReduceBezierControlDegree(linear));
        ASSERT_EQ(linear.size(), 2U);
        EXPECT_EQ(linear.front().z, 0.0);
        EXPECT_EQ(linear.back().z, height);
    }
}

TEST(CurveOnSurfaceInterface3dTest, RestrictionInterpolationLossDoesNotProvePlaneOverlap)
{
    const auto surface = std::make_shared<Geometry::SCNurbsSurface>(1, 1, 2, 2,
        std::vector<Geometry::SCPoint3d>{{0,0,0}, {1,0,0}, {0,1,1e-300}, {1,1,1e-300}},
        std::vector<double>{0,0,1,1}, std::vector<double>{0,0,1,1},
        std::vector<double>(4, 1.0));
    const Geometry::SCCurveOnSurface curve(surface,
        Geometry::SCPolyline2d({{0,0},{1,1e-100}}));
    ASSERT_TRUE(curve.IsValid());
    const auto result = Geometry::IntersectAllPlaneCurveWithStatus(
        Geometry::SCPlane::FromPointAndNormal({0,0,0}, {0,0,1}), curve);
    EXPECT_NE(result.state, Geometry::SCGeometryOperationState::NonUnique);
    EXPECT_EQ(result.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Unknown);
}

TEST(CurveOnSurfaceInterface3dTest, HomogeneousInterpolationPreservesRealZerosAndReportsLoss)
{
    using Control = Geometry::Detail::HomogeneousSurfaceControl;
    const Control zero{0,0,0,1};
    for (const double height : {1e-300, -1e-300})
    {
        const Control nonzero{0,0,height,1};
        const auto start = Geometry::Detail::LerpSurfaceControl(zero, nonzero, 0.0);
        const auto end = Geometry::Detail::LerpSurfaceControl(zero, nonzero, 1.0);
        ASSERT_TRUE(start.has_value());
        ASSERT_TRUE(end.has_value());
        EXPECT_EQ(start->z, 0.0);
        EXPECT_EQ(end->z, height);
        EXPECT_FALSE(Geometry::Detail::LerpSurfaceControl(zero, nonzero, 1e-100).has_value());
    }
    const auto cancelled = Geometry::Detail::LerpSurfaceControl(
        Control{0,0,1e-300,1}, Control{0,0,-1e-300,1}, 0.5);
    ASSERT_TRUE(cancelled.has_value());
    EXPECT_EQ(cancelled->z, 0.0);
    const auto inserted = Geometry::Detail::InsertNurbsControlKnot(
        {zero, Control{0.5,0,1e-300,1}, Control{1,0,0,1}}, {0,0,0,1,1,1}, 2, 1e-100);
    EXPECT_TRUE(inserted.empty());
}

TEST(CurveOnSurfaceInterface3dTest, DegreeReductionRetainsPositiveWeights)
{
    using Control = Geometry::Detail::HomogeneousSurfaceControl;
    const double epsilon = std::numeric_limits<double>::epsilon();
    for (const double middleWeight : {0.5, 0.5 - epsilon * 0.25})
    {
        std::vector<Control> controls{{0,0,0,1}, {0,0,0,middleWeight}, {0,0,0,epsilon * 0.25}};
        const auto original = controls;
        EXPECT_FALSE(Geometry::Detail::TryReduceBezierControlDegree(controls));
        EXPECT_EQ(controls.size(), 3U);
        if (controls.size() == original.size())
            for (std::size_t i = 0; i < controls.size(); ++i) EXPECT_EQ(controls[i].w, original[i].w);
    }
    std::vector<Control> reducible{{0,0,0,1}, {0,0,0,0.5 + epsilon * 0.5}, {0,0,0,epsilon}};
    EXPECT_TRUE(Geometry::Detail::TryReduceBezierControlDegree(reducible));
    ASSERT_EQ(reducible.size(), 2U);
    EXPECT_EQ(reducible.back().w, epsilon);
}

TEST(CurveOnSurfaceInterface3dTest, ParameterDeduplicationPreservesDistantNearbyRoots)
{
    // z(t)=t*(t-4e-9)，两根的参数接近但空间位置相距4。
    const std::vector<Geometry::SCPoint3d> profile{
        {0, 0.5, 0}, {5e8, 0.5, -2e-9}, {1e9, 0.5, 1 - 4e-9}};
    const std::vector<double> knots{0, 0, 0, 1, 1, 1};
    const Geometry::SCNurbsCurve3d direct(2, profile, knots, {1, 1, 1});
    std::vector<Geometry::SCPoint3d> controls;
    for (const double v : {0.0, 1.0})
        for (const auto& point : profile)
            controls.push_back({point.x, v, point.z});
    const auto surface = std::make_shared<Geometry::SCNurbsSurface>(2, 1, 3, 2, controls,
        knots, std::vector<double>{0, 0, 1, 1}, std::vector<double>(6, 1.0));
    Geometry::SCGeometryTolerance3d tolerance;
    tolerance.distanceEpsilon = 5e-18;
    tolerance.parameterEpsilon = 1e-12;
    ASSERT_TRUE(direct.IsValid(tolerance));
    const auto plane = Geometry::SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});
    const auto line = Geometry::SCLine3d::FromOriginAndDirection({0, 0.5, 0}, {1, 0, 0});
    const double positionTolerance = 1e9 * tolerance.parameterEpsilon;
    const auto directHits = Geometry::IntersectAllPlaneCurveWithStatus(plane, direct, tolerance);
    ASSERT_EQ(directHits.hits.size(), 2U);
    EXPECT_NEAR(directHits.hits.front().point.x, 0.0, positionTolerance);
    EXPECT_NEAR(directHits.hits.back().point.x, 4.0, positionTolerance);
    for (const bool reverse : {false, true})
    {
        SCOPED_TRACE(reverse);
        const Geometry::SCCurveOnSurface curve(surface, Geometry::SCPolyline2d({
            {reverse ? 1.0 : 0.0, 0.5}, {reverse ? 0.0 : 1.0, 0.5}}));
        ASSERT_TRUE(curve.IsValid(tolerance));
        const auto check = [&](const auto& result) {
            EXPECT_EQ(result.state, Geometry::SCGeometryOperationState::HasResult);
            ASSERT_EQ(result.hits.size(), 2U);
            EXPECT_NEAR(result.hits.front().point.x, reverse ? 4.0 : 0.0, positionTolerance);
            EXPECT_NEAR(result.hits.back().point.x, reverse ? 0.0 : 4.0, positionTolerance);
            EXPECT_GT(result.hits.back().curveParameter - result.hits.front().curveParameter,
                      tolerance.parameterEpsilon);
        };
        check(Geometry::IntersectAllPlaneCurveWithStatus(plane, curve, tolerance));
        check(Geometry::IntersectAllLineCurveWithStatus(line, curve, tolerance));
    }
}
