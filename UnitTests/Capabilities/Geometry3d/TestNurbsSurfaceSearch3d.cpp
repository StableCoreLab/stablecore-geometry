#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include "Geometry.h"
#include "Geometry3d/SCOffsetSurface.h"
#include "Geometry3d/SCPlaneSurface.h"

using Geometry::Intersect;
using Geometry::ProjectPointToCurve;
using Geometry::ProjectPointToSurface;
using Geometry::SCLine3d;
using Geometry::SCNurbsCurve3d;
using Geometry::SCNurbsSurface;
using Geometry::SCPoint3d;
using Geometry::SCVector3d;

namespace
{
    TEST(NurbsNumerics3dTest, SmallScaleSurfaceProjectionRefinesWithinPlanarPatch)
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

    TEST(NurbsNumerics3dTest, SmallPlaneSurfaceAxesRemainProjectable)
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

    TEST(NurbsNumerics3dTest, SkewPlaneSurfaceUsesCoupledParameters)
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

    TEST(NurbsNumerics3dTest, PlaneSurfaceIntersectionUsesActualTiltedPlane)
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

    TEST(NurbsNumerics3dTest, SmallOffsetSurfaceKeepsItsDisplacement)
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

    TEST(NurbsNumerics3dTest, BilinearSaddleProjectionDoesNotStopAtStationaryCenter)
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

    TEST(NurbsNumerics3dTest, DegenerateSurfaceProjectionUsesPointOnlyRefinement)
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

    TEST(NurbsNumerics3dTest, SurfaceProjectionFindsIsolatedKnotBeyondSeedBudget)
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

    TEST(NurbsNumerics3dTest, ManySpanRationalSurfaceProjectsItsInteriorPoint)
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

    TEST(NurbsNumerics3dTest, SmallScaleNonPlanarSurfaceIntersectionRefinesBetweenSeeds)
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

    TEST(NurbsNumerics3dTest, InvalidNurbsInputsFailProjectionAndIntersectionWithoutPartialResults)
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

    TEST(NurbsNumerics3dTest, SurfaceProjectionPrunesOverlappingControlBounds)
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

    TEST(NurbsNumerics3dTest, MultiCellProjectionDoesNotTreatUnderflowedSquareAsExactHit)
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
}  // namespace
