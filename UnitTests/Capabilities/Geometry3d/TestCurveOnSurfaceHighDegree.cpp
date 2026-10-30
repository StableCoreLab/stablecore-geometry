#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <vector>

#include "Core/Intersection.h"
#include "Geometry3d/SCCurveOnSurface.h"
#include "Geometry3d/SCNurbsSurface.h"
#include "../../../Source/Core/Detail/CurveOnSurfaceNumerics.h"

namespace
{
    TEST(CurveOnSurfaceInterfaceTest, DegreeThirtyTwoRationalRestrictionUsesRootKernel)
    {
        std::vector<Geometry::SCPoint3d> controls;
        std::vector<double> weights;
        for (int v = 0; v <= 16; ++v)
        {
            for (int u = 0; u <= 16; ++u)
            {
                const double weight = 1.0 + 0.1 * ((u + v) % 3);
                controls.push_back({static_cast<double>(u) / 16.0,
                                    static_cast<double>(v) / 16.0,
                                    (static_cast<double>(u) / 16.0 - 0.5) / weight});
                weights.push_back(weight);
            }
        }
        std::vector<double> knots(34, 1.0);
        std::fill(knots.begin(), knots.begin() + 17, 0.0);
        const auto surface = std::make_shared<Geometry::SCNurbsSurface>(
            16, 16, 17, 17, std::move(controls), knots, knots, std::move(weights));
        const Geometry::SCCurveOnSurface diagonal(
            surface, Geometry::SCPolyline2d({{0, 0}, {1, 1}}));
        ASSERT_TRUE(diagonal.IsValid());
        const auto restricted = Geometry::Detail::RestrictBezierNurbsSurfaceSegment(diagonal, 0, {});
        ASSERT_TRUE(restricted.has_value());
        ASSERT_EQ(restricted->curve.Degree(), 32);
        const auto hits = Geometry::IntersectAllPlaneCurveWithStatus(
            Geometry::SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1}),
            static_cast<const Geometry::ISCCurve3d&>(diagonal));
        ASSERT_EQ(hits.hits.size(), 1U);
        EXPECT_NEAR(hits.hits.front().curveParameter, 0.5, 1e-6);
    }

    TEST(CurveOnSurfaceInterfaceTest, RationalAxisAlignedUvSegmentsPreserveReverseParameterMapping)
    {
        const auto surface = std::make_shared<Geometry::SCNurbsSurface>(
            1, 1, 2, 2,
            std::vector<Geometry::SCPoint3d>{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}},
            std::vector<double>{0, 0, 1, 1}, std::vector<double>{0, 0, 1, 1},
            std::vector<double>{1, 2, 3, 4});
        const Geometry::SCCurveOnSurface reverseU(
            surface, Geometry::SCPolyline2d({{1, 0.25}, {0, 0.25}}));
        const Geometry::SCCurveOnSurface reverseV(
            surface, Geometry::SCPolyline2d({{0.25, 1}, {0.25, 0}}));
        ASSERT_TRUE(reverseU.IsValid());
        ASSERT_TRUE(reverseV.IsValid());

        const auto planeX = Geometry::SCPlane::FromPointAndNormal({0.5, 0, 0}, {1, 0, 0});
        const auto planeY = Geometry::SCPlane::FromPointAndNormal({0, 0.5, 0}, {0, 1, 0});
        const auto uHits = Geometry::IntersectAllPlaneCurveWithStatus(planeX, reverseU);
        const auto vHits = Geometry::IntersectAllPlaneCurveWithStatus(planeY, reverseV);
        ASSERT_EQ(uHits.hits.size(), 1U);
        ASSERT_EQ(vHits.hits.size(), 1U);
        EXPECT_NEAR(uHits.hits.front().curveParameter, 0.625, 1e-7);
        EXPECT_NEAR(vHits.hits.front().curveParameter, 1.0 - 1.25 / 4.5, 1e-7);
        EXPECT_TRUE(uHits.hits.front().point.AlmostEquals(
            reverseU.PointAt(uHits.hits.front().curveParameter), 1e-9));
        EXPECT_TRUE(vHits.hits.front().point.AlmostEquals(
            reverseV.PointAt(vHits.hits.front().curveParameter), 1e-9));
    }

    TEST(CurveOnSurfaceInterfaceTest, AxisAlignedNurbsOverlapReportsNonUnique)
    {
        const auto surface = std::make_shared<Geometry::SCNurbsSurface>(
            1, 1, 2, 2,
            std::vector<Geometry::SCPoint3d>{{0, 0, 0}, {1, 0, 0},
                                             {0, 1, 1}, {1, 1, 1}},
            std::vector<double>{0, 0, 1, 1}, std::vector<double>{0, 0, 1, 1});
        const Geometry::SCCurveOnSurface curve(
            surface, Geometry::SCPolyline2d({{0, 0.5}, {1, 0.5}}));
        ASSERT_TRUE(curve.IsValid());
        const auto plane = Geometry::SCPlane::FromPointAndNormal({0, 0, 0.5}, {0, 0, 1});
        const auto line = Geometry::SCLine3d::FromOriginAndDirection({0, 0.5, 0.5}, {1, 0, 0});

        const auto planeHits = Geometry::IntersectAllPlaneCurveWithStatus(plane, curve);
        const auto lineHits = Geometry::IntersectAllLineCurveWithStatus(line, curve);
        EXPECT_EQ(planeHits.state, Geometry::SCGeometryOperationState::NonUnique);
        EXPECT_TRUE(planeHits.hits.empty());
        EXPECT_EQ(lineHits.state, Geometry::SCGeometryOperationState::NonUnique);
        EXPECT_TRUE(lineHits.hits.empty());
    }
}
