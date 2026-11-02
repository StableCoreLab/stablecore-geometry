#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>
#include <vector>

#include "Core/Intersection.h"
#include "Geometry3d/SCCurveOnSurface.h"
#include "Geometry3d/SCNurbsSurface.h"
#include "../../../Source/Core/Detail/CurveOnSurfaceNumerics.h"

namespace
{
    TEST(CurveOnSurfaceInterfaceTest, BernsteinProductFactorsRemainNormalizedAtHighDegree)
    {
        const auto lowDegree = Geometry::Detail::BernsteinProductFactors(2, 1, 1);
        ASSERT_EQ(lowDegree.size(), 2U);
        EXPECT_DOUBLE_EQ(lowDegree[0], 1.0 / 3.0);
        EXPECT_DOUBLE_EQ(lowDegree[1], 2.0 / 3.0);
        const auto factors = Geometry::Detail::BernsteinProductFactors(600, 600, 600);
        ASSERT_EQ(factors.size(), 601U);
        EXPECT_NEAR(std::accumulate(factors.begin(), factors.end(), 0.0), 1.0, 1e-14);
        double mean = 0.0;
        for (std::size_t i = 0; i < factors.size(); ++i)
        {
            EXPECT_TRUE(std::isfinite(factors[i]));
            EXPECT_GE(factors[i], 0.0);
            EXPECT_NEAR(factors[i], factors[factors.size() - i - 1], 1e-14);
            mean += static_cast<double>(i) * factors[i];
        }
        EXPECT_NEAR(mean, 300.0, 1e-11);
        EXPECT_GT(factors[300], factors[299]);
    }

    TEST(CurveOnSurfaceInterfaceTest, DegreeFortyRationalRestrictionPreservesSurfaceAndRoot)
    {
        constexpr int axisDegree = 20;
        std::vector<Geometry::SCPoint3d> controls;
        std::vector<double> weights;
        for (int v = 0; v <= axisDegree; ++v)
            for (int u = 0; u <= axisDegree; ++u)
            {
                const double weight = 1.0 + 0.1 * ((u + v) % 3);
                controls.push_back({static_cast<double>(u) / axisDegree,
                                    static_cast<double>(v) / axisDegree,
                                    (static_cast<double>(u) / axisDegree - 0.5) / weight});
                weights.push_back(weight);
            }
        std::vector<double> knots(2 * (axisDegree + 1), 1.0);
        std::fill(knots.begin(), knots.begin() + axisDegree + 1, 0.0);
        const auto surface = std::make_shared<Geometry::SCNurbsSurface>(
            axisDegree, axisDegree, axisDegree + 1, axisDegree + 1,
            std::move(controls), knots, knots, std::move(weights));
        const Geometry::SCCurveOnSurface diagonal(surface, Geometry::SCPolyline2d({{0, 0}, {1, 1}}));
        ASSERT_TRUE(diagonal.IsValid());
        const auto restricted = Geometry::Detail::RestrictBezierNurbsSurfaceSegment(diagonal, 0, {});
        ASSERT_TRUE(restricted.has_value());
        EXPECT_EQ(restricted->curve.Degree(), 40);
        for (const double parameter : {0.0, 0.1, 0.25, 0.5, 0.9, 1.0})
            EXPECT_TRUE(restricted->curve.PointAt(parameter).AlmostEquals(surface->PointAt(parameter, parameter), 1e-10));
        const auto hits = Geometry::IntersectAllPlaneCurveWithStatus(
            Geometry::SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1}),
            static_cast<const Geometry::ISCCurve3d&>(diagonal));
        ASSERT_EQ(hits.hits.size(), 1U);
        EXPECT_NEAR(hits.hits.front().curveParameter, 0.5, 1e-6);
    }

    TEST(CurveOnSurfaceInterfaceTest, AsymmetricHighDegreeRestrictionPreservesPartialReverseSegment)
    {
        constexpr int degreeU = 21;
        constexpr int degreeV = 17;
        std::vector<Geometry::SCPoint3d> controls;
        std::vector<double> weights;
        for (int v = 0; v <= degreeV; ++v)
            for (int u = 0; u <= degreeU; ++u)
            {
                const double weight = 1.0 + 0.2 * ((2 * u + v) % 4);
                controls.push_back({static_cast<double>(u) / degreeU,
                                    static_cast<double>(v) / degreeV,
                                    (static_cast<double>(u) / degreeU - 0.5) / weight});
                weights.push_back(weight);
            }
        std::vector<double> knotsU(2 * (degreeU + 1), 1.0);
        std::vector<double> knotsV(2 * (degreeV + 1), 1.0);
        std::fill(knotsU.begin(), knotsU.begin() + degreeU + 1, 0.0);
        std::fill(knotsV.begin(), knotsV.begin() + degreeV + 1, 0.0);
        const auto surface = std::make_shared<Geometry::SCNurbsSurface>(
            degreeU, degreeV, degreeU + 1, degreeV + 1,
            std::move(controls), std::move(knotsU), std::move(knotsV), std::move(weights));
        const Geometry::SCCurveOnSurface curve(surface, Geometry::SCPolyline2d({{0.9, 0.2}, {0.1, 0.8}}));
        ASSERT_TRUE(curve.IsValid());
        const auto restricted = Geometry::Detail::RestrictBezierNurbsSurfaceSegment(curve, 0, {});
        ASSERT_TRUE(restricted.has_value());
        for (const double parameter : {0.0, 0.1, 0.25, 0.5, 0.9, 1.0})
        {
            const auto expected = surface->PointAt(std::lerp(0.9, 0.1, parameter),
                                                   std::lerp(0.2, 0.8, parameter));
            EXPECT_TRUE(restricted->curve.PointAt(parameter).AlmostEquals(expected, 1e-10));
        }
        const auto hits = Geometry::IntersectAllPlaneCurveWithStatus(
            Geometry::SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1}),
            static_cast<const Geometry::ISCCurve3d&>(curve));
        ASSERT_EQ(hits.hits.size(), 1U);
        EXPECT_NEAR(hits.hits.front().curveParameter, 0.5, 1e-6);
    }

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
