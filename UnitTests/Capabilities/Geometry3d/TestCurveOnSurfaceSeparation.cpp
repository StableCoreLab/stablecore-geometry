#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "Core/Intersection.h"
#include "Geometry3d/SCCurveOnSurface.h"
#include "Geometry3d/SCNurbsSurface.h"

TEST(CurveOnSurfaceInterfaceTest, SeparatedNurbsSurfaceControlHullProvesPlaneMiss)
{
    const Geometry::SCPlane plane = Geometry::SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});
    for (const double sign : {-1.0, 1.0})
    {
        for (const auto& weights : {std::vector<double>{1, 1, 1, 1}, std::vector<double>{1, 2, 3, 4},
                                    std::vector<double>{1e-300, 2e-300, 3e-300, 4e-300},
                                    std::vector<double>{1e300, 2e300, 3e300, 4e300}})
        {
            const auto surface = std::make_shared<Geometry::SCNurbsSurface>(
                1, 1, 2, 2,
                std::vector<Geometry::SCPoint3d>{{0, 0, sign}, {1, 0, 2 * sign},
                                                 {0, 1, 2 * sign}, {1, 1, 4 * sign}},
                std::vector<double>{0, 0, 1, 1}, std::vector<double>{0, 0, 1, 1}, weights);
            const Geometry::SCCurveOnSurface curve(surface, Geometry::SCPolyline2d({{0.1, 0.2}, {0.9, 0.8}}));
            ASSERT_TRUE(curve.IsValid());
            const auto result = Geometry::IntersectAllPlaneCurveWithStatus(plane, curve);
            EXPECT_TRUE(result.hits.empty());
            EXPECT_EQ(result.state, Geometry::SCGeometryOperationState::NoResult);
            EXPECT_EQ(result.evidence.hit, Geometry::SCGeometryEvidenceState::NotApplicable);
            EXPECT_EQ(result.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Verified);
        }
    }
}

TEST(CurveOnSurfaceInterfaceTest, MixedSurfaceControlHullDoesNotProvePlaneMiss)
{
    const auto surface = std::make_shared<Geometry::SCNurbsSurface>(
        1, 1, 2, 2, std::vector<Geometry::SCPoint3d>{{0, 0, 1}, {1, 0, 1}, {0, 1, -1}, {1, 1, -1}},
        std::vector<double>{0, 0, 1, 1}, std::vector<double>{0, 0, 1, 1});
    const Geometry::SCCurveOnSurface curve(surface, Geometry::SCPolyline2d({{0.1, 0}, {0.9, 0}}));
    ASSERT_TRUE(curve.IsValid());
    const auto plane = Geometry::SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});
    const auto result = Geometry::IntersectAllPlaneCurveWithStatus(plane, curve);
    EXPECT_TRUE(result.hits.empty());
    EXPECT_EQ(result.state, Geometry::SCGeometryOperationState::Unknown);
    EXPECT_EQ(result.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Unknown);
}

TEST(CurveOnSurfaceInterfaceTest, SeparatedSurfaceControlHullProvesObliqueLineMiss)
{
    for (const auto& controls : {
        std::vector<Geometry::SCPoint3d>{{0, 5, 1}, {1, 5, 2}, {0, 6, 2}, {1, 6, 4}},
        std::vector<Geometry::SCPoint3d>{{5, -1, 1}, {6, -1, 2}, {5, 3, 2}, {6, 3, 4}}})
    for (const double directionScale : {-2.0, 1.0, 1e200})
    {
        const auto surface = std::make_shared<Geometry::SCNurbsSurface>(
            1, 1, 2, 2, controls,
            std::vector<double>{0, 0, 1, 1}, std::vector<double>{0, 0, 1, 1},
            std::vector<double>{1, 2, 3, 4});
        const Geometry::SCCurveOnSurface curve(surface, Geometry::SCPolyline2d({{0.1, 0.2}, {0.9, 0.8}}));
        const auto line = Geometry::SCLine3d::FromOriginAndDirection(
            {0, 0, 0}, {directionScale, 2 * directionScale, 3 * directionScale});
        ASSERT_TRUE(curve.IsValid());
        ASSERT_TRUE(line.IsValid());
        const auto result = Geometry::IntersectAllLineCurveWithStatus(line, curve);
        EXPECT_TRUE(result.hits.empty());
        EXPECT_EQ(result.state, Geometry::SCGeometryOperationState::NoResult);
        EXPECT_EQ(result.evidence.hit, Geometry::SCGeometryEvidenceState::NotApplicable);
        EXPECT_EQ(result.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Verified);
    }
}

TEST(CurveOnSurfaceInterfaceTest, MixedSurfaceControlHullDoesNotProveLineMiss)
{
    const auto surface = std::make_shared<Geometry::SCNurbsSurface>(
        1, 1, 2, 2, std::vector<Geometry::SCPoint3d>{{-1, -1, 0}, {1, -1, 0}, {-1, 1, 0}, {1, 1, 0}},
        std::vector<double>{0, 0, 1, 1}, std::vector<double>{0, 0, 1, 1});
    const Geometry::SCCurveOnSurface curve(surface, Geometry::SCPolyline2d({{0.1, 1}, {0.9, 1}}));
    const auto line = Geometry::SCLine3d::FromOriginAndDirection({0, 0, -1}, {0, 0, 1});
    ASSERT_TRUE(curve.IsValid());
    const auto result = Geometry::IntersectAllLineCurveWithStatus(line, curve);
    EXPECT_TRUE(result.hits.empty());
    EXPECT_EQ(result.state, Geometry::SCGeometryOperationState::Unknown);
    EXPECT_EQ(result.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Unknown);
}

TEST(CurveOnSurfaceInterfaceTest, ThirdCoordinatePlaneProvesSurfaceLineMiss)
{
    const auto surface = std::make_shared<Geometry::SCNurbsSurface>(
        1, 1, 2, 2, std::vector<Geometry::SCPoint3d>{{1, 0, 4}, {1, 1, 0}, {2, 1, 7}, {2, 2, 0}},
        std::vector<double>{0, 0, 1, 1}, std::vector<double>{0, 0, 1, 1},
        std::vector<double>{1, 2, 3, 4});
    const Geometry::SCCurveOnSurface curve(surface, Geometry::SCPolyline2d({{0.1, 0.2}, {0.9, 0.8}}));
    ASSERT_TRUE(curve.IsValid());
    for (const double scale : {-2.0, 1.0, 1e200})
    {
        const auto line = Geometry::SCLine3d::FromOriginAndDirection({0, 0, 0}, {scale, 2 * scale, 3 * scale});
        ASSERT_TRUE(line.IsValid());
        const auto result = Geometry::IntersectAllLineCurveWithStatus(line, curve);
        EXPECT_TRUE(result.hits.empty());
        EXPECT_EQ(result.state, Geometry::SCGeometryOperationState::NoResult);
        EXPECT_EQ(result.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Verified);
    }
}

TEST(CurveOnSurfaceInterfaceTest, ControlHullAtToleranceBoundaryDoesNotProveSeparation)
{
    Geometry::SCGeometryTolerance3d tolerance;
    tolerance.distanceEpsilon = 0.125;
    const double height = tolerance.distanceEpsilon;
    const auto surface = std::make_shared<Geometry::SCNurbsSurface>(
        1, 1, 2, 2, std::vector<Geometry::SCPoint3d>{{0, 0, height}, {1, 0, height},
                                                {0, 1, height}, {1, 1, height}},
        std::vector<double>{0, 0, 1, 1}, std::vector<double>{0, 0, 1, 1});
    const Geometry::SCCurveOnSurface curve(surface, Geometry::SCPolyline2d({{0.1, 0.2}, {0.9, 0.8}}));
    ASSERT_TRUE(curve.IsValid(tolerance));
    const auto plane = Geometry::SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});
    const auto result = Geometry::IntersectAllPlaneCurveWithStatus(plane, curve, tolerance);
    EXPECT_TRUE(result.state == Geometry::SCGeometryOperationState::HasResult ||
                result.state == Geometry::SCGeometryOperationState::NonUnique);
    EXPECT_EQ(result.evidence.hit, Geometry::SCGeometryEvidenceState::Verified);
    EXPECT_NE(result.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Verified);
}

TEST(CurveOnSurfaceInterfaceTest, UnrepresentableControlPlaneDistanceKeepsCompletenessUnknown)
{
    const auto surface = std::make_shared<Geometry::SCNurbsSurface>(
        1, 1, 2, 2, std::vector<Geometry::SCPoint3d>{{-1e308, 0, 0}, {-1e308, 1, 0},
                                                {-1e308, 0, 1}, {-1e308, 1, 1}},
        std::vector<double>{0, 0, 1, 1}, std::vector<double>{0, 0, 1, 1});
    const Geometry::SCCurveOnSurface curve(surface, Geometry::SCPolyline2d({{0.1, 0.2}, {0.9, 0.8}}));
    const auto plane = Geometry::SCPlane::FromPointAndNormal({1e308, 0, 0}, {1, 0, 0});
    ASSERT_TRUE(curve.IsValid());
    ASSERT_TRUE(plane.IsValid());
    const auto result = Geometry::IntersectAllPlaneCurveWithStatus(plane, curve);
    EXPECT_TRUE(result.hits.empty());
    EXPECT_EQ(result.state, Geometry::SCGeometryOperationState::Unknown);
    EXPECT_EQ(result.evidence.hit, Geometry::SCGeometryEvidenceState::Unknown);
    EXPECT_EQ(result.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Unknown);
}
