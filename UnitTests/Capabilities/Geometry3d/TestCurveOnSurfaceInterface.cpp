#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <numeric>
#include <type_traits>
#include <vector>

#include "Core/Projection.h"
#include "Core/Intersection.h"
#include "Geometry3d/ISCCurve3d.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry3d/SCCurveOnSurface.h"
#include "Geometry3d/SCNurbsCurve3d.h"
#include "Geometry3d/SCNurbsSurface.h"
#include "Geometry3d/SCPlaneSurface.h"
#include "../../../Source/Core/Detail/CurveOnSurfaceNumerics.h"

static_assert(std::is_base_of_v<Geometry::ISCCurve3d, Geometry::SCCurveOnSurface>);

namespace
{
    Geometry::SCCurveOnSurface MakeQuadraticHeightCurve(double center, double offset)
    {
        // 沿 UV 线段的高度为 (u - center)^2 - offset。
        const double z0 = center * center - offset;
        const double z1 = z0 - center;
        const double z2 = (1.0 - center) * (1.0 - center) - offset;
        const auto surface = std::make_shared<Geometry::SCNurbsSurface>(
            2, 1, 3, 2,
            std::vector<Geometry::SCPoint3d>{{0, 0, z0}, {0.5, 0, z1}, {1, 0, z2},
                                             {0, 1, z0}, {0.5, 1, z1}, {1, 1, z2}},
            std::vector<double>{0, 0, 0, 1, 1, 1}, std::vector<double>{0, 0, 1, 1});
        return Geometry::SCCurveOnSurface(
            surface, Geometry::SCPolyline2d({{0, 0.5}, {1, 0.5}}));
    }

    Geometry::SCCurveOnSurface MakeCubicHeightCurve(const std::array<double, 3>& roots)
    {
        // 将三个给定根的三次多项式转换为 [0, 1] 上的 Bezier 控制值。
        const double c0 = -roots[0] * roots[1] * roots[2];
        const double c1 = roots[0] * roots[1] + roots[0] * roots[2] + roots[1] * roots[2];
        const double c2 = -(roots[0] + roots[1] + roots[2]);
        const std::array<double, 4> heights{
            c0, c0 + c1 / 3.0, c0 + 2.0 * c1 / 3.0 + c2 / 3.0, c0 + c1 + c2 + 1.0};
        std::vector<Geometry::SCPoint3d> controls;
        for (double v : {0.0, 1.0})
        {
            for (std::size_t i = 0; i < heights.size(); ++i)
            {
                controls.push_back({static_cast<double>(i) / 3.0, v, heights[i]});
            }
        }
        const auto surface = std::make_shared<Geometry::SCNurbsSurface>(
            3, 1, 4, 2, std::move(controls),
            std::vector<double>{0, 0, 0, 0, 1, 1, 1, 1}, std::vector<double>{0, 0, 1, 1});
        return Geometry::SCCurveOnSurface(
            surface, Geometry::SCPolyline2d({{0, 0.5}, {1, 0.5}}));
    }

    TEST(CurveOnSurfaceInterfaceTest, GenericCurveUsesUvSegmentParametersAndSurfaceDerivatives)
    {
        auto surface = std::make_shared<Geometry::SCNurbsSurface>(
            1, 1, 2, 2,
            std::vector<Geometry::SCPoint3d>{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 4}},
            std::vector<double>{0, 0, 1, 1}, std::vector<double>{0, 0, 1, 1});
        const Geometry::SCCurveOnSurface composed(
            surface, Geometry::SCPolyline2d({{0, 0}, {1, 0}, {1, 1}}));
        const Geometry::ISCCurve3d& curve = composed;

        ASSERT_TRUE(curve.IsValid());
        EXPECT_DOUBLE_EQ(curve.ParameterRange().min, 0.0);
        EXPECT_DOUBLE_EQ(curve.ParameterRange().max, 2.0);
        EXPECT_TRUE(curve.PointAt(0.5).AlmostEquals(surface->PointAt(0.5, 0.0), 1e-12));
        EXPECT_TRUE(curve.PointAt(1.5).AlmostEquals(surface->PointAt(1.0, 0.5), 1e-12));
        EXPECT_TRUE(composed.UvAt(1.5).AlmostEquals({1.0, 0.5}, 1e-12));
        EXPECT_TRUE(composed.PointAt(1).AlmostEquals(surface->PointAt(1.0, 0.0), 1e-12));
        EXPECT_TRUE(composed.PointAt(1u).AlmostEquals(surface->PointAt(1.0, 0.0), 1e-12));

        const auto first = curve.Evaluate(0.5, 1);
        const auto second = curve.Evaluate(1.5, 1);
        ASSERT_TRUE(first.IsValid());
        ASSERT_TRUE(second.IsValid());
        EXPECT_TRUE(first.firstDerivative.AlmostEquals({1, 0, 0}, 1e-12));
        EXPECT_TRUE(second.firstDerivative.AlmostEquals({0, 1, 4}, 1e-12));

        const auto projected = Geometry::ProjectPointToCurve(curve.PointAt(1.5), curve);
        ASSERT_TRUE(projected.success);
        EXPECT_TRUE(projected.point.AlmostEquals(curve.PointAt(1.5), 1e-7));
        const Geometry::SCPoint3d probe{1.2, 0.4, 2.4};
        const auto genericProjection = Geometry::ProjectPointToCurve(probe, curve);
        const auto surfaceProjection = Geometry::ProjectPointToCurveOnSurface(probe, composed);
        ASSERT_TRUE(genericProjection.success);
        ASSERT_TRUE(surfaceProjection.success);
        EXPECT_TRUE(genericProjection.point.AlmostEquals(surfaceProjection.point, 1e-12));
        EXPECT_NEAR(genericProjection.parameter,
                    static_cast<double>(surfaceProjection.segmentIndex) + surfaceProjection.segmentParameter,
                    1e-12);
        EXPECT_NEAR(genericProjection.distanceSquared, surfaceProjection.distanceSquared, 1e-12);
        const auto line = Geometry::SCLine3d::FromOriginAndDirection(
            {1.0, 0.5, 3.0}, {0.0, 0.0, -1.0});
        const auto lineHit = Geometry::Intersect(line, curve);
        const auto surfaceLineHit = Geometry::Intersect(line, composed);
        ASSERT_TRUE(lineHit.intersects);
        ASSERT_TRUE(surfaceLineHit.intersects);
        EXPECT_TRUE(lineHit.point.AlmostEquals(curve.PointAt(1.5), 1e-7));
        EXPECT_NEAR(lineHit.curveParameter,
                    static_cast<double>(surfaceLineHit.segmentIndex) + surfaceLineHit.segmentParameter, 1e-12);
        EXPECT_NEAR(lineHit.lineParameter, surfaceLineHit.lineParameter, 1e-12);
        const auto plane = Geometry::SCPlane::FromPointAndNormal(
            {0.0, 0.0, 2.0}, {0.0, 0.0, 1.0});
        const auto planeHit = Geometry::Intersect(plane, curve);
        const auto surfacePlaneHit = Geometry::Intersect(plane, composed);
        ASSERT_TRUE(planeHit.intersects);
        ASSERT_TRUE(surfacePlaneHit.intersects);
        EXPECT_TRUE(planeHit.point.AlmostEquals(curve.PointAt(1.5), 1e-7));
        EXPECT_NEAR(planeHit.curveParameter,
                    static_cast<double>(surfacePlaneHit.segmentIndex) + surfacePlaneHit.segmentParameter, 1e-12);
        ASSERT_TRUE(curve.Bounds().IsValid());
        EXPECT_GE(curve.Bounds().MaxPoint().z, 4.0);

        const auto clone = curve.Clone();
        ASSERT_NE(clone, nullptr);
        EXPECT_TRUE(clone->PointAt(1.5).AlmostEquals(curve.PointAt(1.5), 1e-12));
        const auto* clonedComposition = dynamic_cast<const Geometry::SCCurveOnSurface*>(clone.get());
        ASSERT_NE(clonedComposition, nullptr);
        EXPECT_NE(clonedComposition->SupportSurface(), composed.SupportSurface());
    }

    TEST(CurveOnSurfaceInterfaceTest, ClosedCurveAndInvalidUvDomainHaveExplicitContracts)
    {
        const auto plane = std::make_shared<Geometry::SCPlaneSurface>(
            Geometry::SCPlaneSurface::FromPlane(
                Geometry::SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1}), {-1, 1}, {-1, 1}));
        const Geometry::SCCurveOnSurface closed(
            plane, Geometry::SCPolyline2d({{0, 0}, {1, 0}, {0, 1}}, Geometry::SCPolylineClosure::Closed));
        ASSERT_TRUE(closed.IsValid());
        EXPECT_TRUE(closed.IsClosed());
        EXPECT_FALSE(closed.IsPeriodic());
        EXPECT_TRUE(closed.PointAt(3.0).AlmostEquals(closed.PointAt(0.0), 1e-12));
        EXPECT_FALSE(closed.UvAt(-0.1).IsValid());

        auto curvedUv = std::make_shared<Geometry::SCArcSegment2d>(
            Geometry::SCPoint2d{0, 0}, 1.0, 0.0, Geometry::kPi * 0.5);
        const Geometry::SCCurveOnSurface curved(
            plane, Geometry::SCPolyline2d(
                std::vector<std::shared_ptr<Geometry::ISCSegment2d>>{curvedUv},
                Geometry::SCPolylineClosure::Open));
        EXPECT_FALSE(curved.IsValid());

        const auto surface = std::make_shared<Geometry::SCNurbsSurface>(
            1, 1, 2, 2,
            std::vector<Geometry::SCPoint3d>{{0, 0, 0}, {1, 0, 4}, {0, 1, 4}, {1, 1, 0}},
            std::vector<double>{0, 0, 1, 1}, std::vector<double>{0, 0, 1, 1});
        const Geometry::SCCurveOnSurface outside(
            surface, Geometry::SCPolyline2d({{0, 0}, {2, 0}}));
        EXPECT_FALSE(outside.IsValid());

        const Geometry::SCCurveOnSurface diagonal(
            surface, Geometry::SCPolyline2d({{0, 0}, {1, 1}}));
        ASSERT_TRUE(diagonal.IsValid());
        ASSERT_TRUE(diagonal.Bounds().IsValid());
        EXPECT_GT(diagonal.PointAt(0.5).z, 0.0);
        EXPECT_GE(diagonal.Bounds().MaxPoint().z, diagonal.PointAt(0.5).z);
    }

    TEST(CurveOnSurfaceInterfaceTest, GenericProjectionEvidenceMatchesSurfaceSpecialization)
    {
        const auto plane = std::make_shared<Geometry::SCPlaneSurface>(
            Geometry::SCPlaneSurface::FromPlane(
                Geometry::SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1}), {-1, 1}, {-1, 1}));
        const Geometry::SCCurveOnSurface composed(
            plane, Geometry::SCPolyline2d({{0, 0}, {1, 0}}));
        ASSERT_TRUE(composed.IsValid());

        const Geometry::ISCCurve3d& genericCurve = composed;
        const Geometry::SCPoint3d query{0.25, 0.25, 0.5};
        const auto genericStatus = Geometry::ProjectPointToCurveWithStatus(query, genericCurve);
        const auto surfaceStatus = Geometry::ProjectPointToCurveOnSurfaceWithStatus(query, composed);
        ASSERT_TRUE(genericStatus.payloadValid);
        ASSERT_TRUE(surfaceStatus.payloadValid);
        EXPECT_TRUE(genericStatus.payload.point.AlmostEquals(surfaceStatus.payload.point, 1e-12));
        EXPECT_EQ(genericStatus.evidence.optimality, surfaceStatus.evidence.optimality);
        EXPECT_EQ(genericStatus.evidence.optimality, Geometry::SCGeometryEvidenceState::Verified);
    }

    TEST(CurveOnSurfaceInterfaceTest, GenericIntersectionEvidenceMatchesSurfaceSpecialization)
    {
        const auto surface = std::make_shared<Geometry::SCPlaneSurface>(
            Geometry::SCPlaneSurface::FromPlane(
                Geometry::SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1}), {-1, 1}, {-1, 1}));
        const Geometry::SCCurveOnSurface composed(
            surface, Geometry::SCPolyline2d({{0, 0}, {1, 0}}));
        ASSERT_TRUE(composed.IsValid());
        const Geometry::ISCCurve3d& genericCurve = composed;

        const auto line = Geometry::SCLine3d::FromOriginAndDirection(
            {0.5, 0.0, 1.0}, {0.0, 0.0, -1.0});
        const auto genericLine = Geometry::IntersectLineCurveWithStatus(line, genericCurve);
        const auto surfaceLine = Geometry::IntersectLineCurveOnSurfaceWithStatus(line, composed);
        ASSERT_TRUE(genericLine.payloadValid);
        ASSERT_TRUE(surfaceLine.payloadValid);
        EXPECT_EQ(genericLine.state, surfaceLine.state);
        EXPECT_EQ(genericLine.evidence.reasons, surfaceLine.evidence.reasons);
        EXPECT_NEAR(genericLine.payload.curveParameter,
                    static_cast<double>(surfaceLine.payload.segmentIndex) +
                        surfaceLine.payload.segmentParameter, 1e-12);

        const auto plane = Geometry::SCPlane::FromPointAndNormal(
            {0.5, 0.0, 0.0}, {1.0, 0.0, 0.0});
        const auto genericPlane = Geometry::IntersectPlaneCurveWithStatus(plane, genericCurve);
        const auto surfacePlane = Geometry::IntersectPlaneCurveOnSurfaceWithStatus(plane, composed);
        ASSERT_TRUE(genericPlane.payloadValid);
        ASSERT_TRUE(surfacePlane.payloadValid);
        EXPECT_EQ(genericPlane.state, surfacePlane.state);
        EXPECT_EQ(genericPlane.evidence.reasons, surfacePlane.evidence.reasons);
        EXPECT_EQ(genericPlane.evidence.negativeCompleteness,
                  surfacePlane.evidence.negativeCompleteness);
        EXPECT_NEAR(genericPlane.payload.curveParameter,
                    static_cast<double>(surfacePlane.payload.segmentIndex) +
                        surfacePlane.payload.segmentParameter, 1e-12);

        const auto missLine = Geometry::SCLine3d::FromOriginAndDirection(
            {0.5, 2.0, 1.0}, {0.0, 0.0, -1.0});
        const auto genericLineMiss = Geometry::IntersectLineCurveWithStatus(missLine, genericCurve);
        const auto surfaceLineMiss = Geometry::IntersectLineCurveOnSurfaceWithStatus(missLine, composed);
        EXPECT_FALSE(genericLineMiss.payloadValid);
        EXPECT_FALSE(surfaceLineMiss.payloadValid);
        EXPECT_EQ(genericLineMiss.state, surfaceLineMiss.state);
        EXPECT_EQ(genericLineMiss.evidence.reasons, surfaceLineMiss.evidence.reasons);

        const auto missPlane = Geometry::SCPlane::FromPointAndNormal(
            {0.0, 2.0, 0.0}, {0.0, 1.0, 0.0});
        const auto genericPlaneMiss = Geometry::IntersectPlaneCurveWithStatus(missPlane, genericCurve);
        const auto surfacePlaneMiss = Geometry::IntersectPlaneCurveOnSurfaceWithStatus(missPlane, composed);
        EXPECT_FALSE(genericPlaneMiss.payloadValid);
        EXPECT_FALSE(surfacePlaneMiss.payloadValid);
        EXPECT_EQ(genericPlaneMiss.state, surfacePlaneMiss.state);
        EXPECT_EQ(genericPlaneMiss.evidence.reasons, surfacePlaneMiss.evidence.reasons);
        EXPECT_EQ(genericPlaneMiss.evidence.negativeCompleteness,
                  Geometry::SCGeometryEvidenceState::Unknown);
    }

    TEST(CurveOnSurfaceInterfaceTest, PlanarSurfaceCurveCollectsMultipleVerifiedLineAndPlaneHits)
    {
        const auto surface = std::make_shared<Geometry::SCPlaneSurface>(
            Geometry::SCPlaneSurface::FromPlane(
                Geometry::SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1}), {-2, 2}, {-2, 2}));
        const Geometry::SCCurveOnSurface planeCrossings(
            surface, Geometry::SCPolyline2d({{0, 0}, {1, 0}, {0, 1}, {1, 1}}));
        ASSERT_TRUE(planeCrossings.IsValid());
        const Geometry::ISCCurve3d& genericPlaneCurve = planeCrossings;
        const auto cutPlane = Geometry::SCPlane::FromPointAndNormal(
            {0.5, 0.0, 0.0}, {1.0, 0.0, 0.0});
        const auto planeHits = Geometry::IntersectAllPlaneCurveWithStatus(cutPlane, genericPlaneCurve);
        ASSERT_EQ(planeHits.hits.size(), 3);
        EXPECT_EQ(planeHits.state, Geometry::SCGeometryOperationState::HasResult);
        EXPECT_EQ(planeHits.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Verified);
        for (std::size_t index = 0; index < planeHits.hits.size(); ++index)
        {
            EXPECT_NEAR(planeHits.hits[index].curveParameter, static_cast<double>(index) + 0.5, 1e-12);
        }

        const Geometry::SCCurveOnSurface lineCrossings(
            surface, Geometry::SCPolyline2d({{0, -1}, {0, 1}, {1, -1}, {1, 1}}));
        ASSERT_TRUE(lineCrossings.IsValid());
        const Geometry::ISCCurve3d& genericLineCurve = lineCrossings;
        const auto cutLine = Geometry::SCLine3d::FromOriginAndDirection(
            {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
        const auto lineHits = Geometry::IntersectAllLineCurveWithStatus(cutLine, genericLineCurve);
        ASSERT_EQ(lineHits.hits.size(), 3);
        EXPECT_EQ(lineHits.state, Geometry::SCGeometryOperationState::HasResult);
        EXPECT_EQ(lineHits.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Unknown);
        for (std::size_t index = 0; index < lineHits.hits.size(); ++index)
        {
            EXPECT_NEAR(lineHits.hits[index].curveParameter, static_cast<double>(index) + 0.5, 1e-12);
        }
    }

    TEST(CurveOnSurfaceInterfaceTest, CurvedSurfaceCurveRetainsMultipleHitsWithoutCompletenessClaim)
    {
        const auto surface = std::make_shared<Geometry::SCNurbsSurface>(
            1, 1, 2, 2,
            std::vector<Geometry::SCPoint3d>{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 4}},
            std::vector<double>{0, 0, 1, 1}, std::vector<double>{0, 0, 1, 1});
        const Geometry::SCCurveOnSurface composed(
            surface, Geometry::SCPolyline2d({{0, 0}, {1, 0}, {0, 1}, {1, 1}}));
        ASSERT_TRUE(composed.IsValid());
        const Geometry::ISCCurve3d& curve = composed;
        const auto plane = Geometry::SCPlane::FromPointAndNormal(
            {0.5, 0.0, 0.0}, {1.0, 0.0, 0.0});
        const auto hits = Geometry::IntersectAllPlaneCurveWithStatus(plane, curve);
        ASSERT_EQ(hits.hits.size(), 3);
        EXPECT_EQ(hits.state, Geometry::SCGeometryOperationState::HasResult);
        EXPECT_EQ(hits.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Unknown);
        for (std::size_t index = 0; index < hits.hits.size(); ++index)
        {
            EXPECT_NEAR(hits.hits[index].curveParameter, static_cast<double>(index) + 0.5, 1e-12);
            EXPECT_TRUE(hits.hits[index].point.AlmostEquals(curve.PointAt(hits.hits[index].curveParameter), 1e-12));
        }

        const Geometry::SCCurveOnSurface repeated(
            surface, Geometry::SCPolyline2d({{0, 0}, {1, 0}, {0, 0}, {1, 0}}));
        ASSERT_TRUE(repeated.IsValid());
        const Geometry::ISCCurve3d& repeatedCurve = repeated;
        const auto line = Geometry::SCLine3d::FromOriginAndDirection(
            {0.5, 0.0, 1.0}, {0.0, 0.0, -1.0});
        const auto lineHits = Geometry::IntersectAllLineCurveWithStatus(line, repeatedCurve);
        ASSERT_EQ(lineHits.hits.size(), 3);
        EXPECT_EQ(lineHits.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Unknown);
        for (std::size_t index = 0; index < lineHits.hits.size(); ++index)
        {
            EXPECT_NEAR(lineHits.hits[index].curveParameter, static_cast<double>(index) + 0.5, 1e-8);
        }

        const auto missPlane = Geometry::SCPlane::FromPointAndNormal(
            {2.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
        const auto noPlaneHits = Geometry::IntersectAllPlaneCurveWithStatus(missPlane, curve);
        EXPECT_TRUE(noPlaneHits.hits.empty());
        EXPECT_EQ(noPlaneHits.state, Geometry::SCGeometryOperationState::Unknown);
        EXPECT_EQ(noPlaneHits.evidence.hit, Geometry::SCGeometryEvidenceState::Unknown);
        EXPECT_EQ(noPlaneHits.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Unknown);
    }

    TEST(CurveOnSurfaceInterfaceTest, CurvedSingleUvSegmentRetainsTwoIsolatedRoots)
    {
        const auto surface = std::make_shared<Geometry::SCNurbsSurface>(
            2, 1, 3, 2,
            std::vector<Geometry::SCPoint3d>{{0, 0, 1}, {0.5, 0, -3}, {1, 0, 1},
                                             {0, 1, 1}, {0.5, 1, -3}, {1, 1, 1}},
            std::vector<double>{0, 0, 0, 1, 1, 1}, std::vector<double>{0, 0, 1, 1});
        const Geometry::SCCurveOnSurface composed(
            surface, Geometry::SCPolyline2d({{0, 0.5}, {1, 0.5}}));
        ASSERT_TRUE(composed.IsValid());
        const Geometry::ISCCurve3d& curve = composed;
        const auto plane = Geometry::SCPlane::FromPointAndNormal(
            {0, 0, 0}, {0, 0, 1});
        const auto line = Geometry::SCLine3d::FromOriginAndDirection(
            {0, 0.5, 0}, {1, 0, 0});
        const double first = (1.0 - std::sqrt(0.5)) * 0.5;
        const double second = (1.0 + std::sqrt(0.5)) * 0.5;

        const auto planeHits = Geometry::IntersectAllPlaneCurveWithStatus(plane, curve);
        ASSERT_EQ(planeHits.hits.size(), 2);
        EXPECT_NEAR(planeHits.hits[0].curveParameter, first, 1e-6);
        EXPECT_NEAR(planeHits.hits[1].curveParameter, second, 1e-6);
        EXPECT_EQ(planeHits.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Unknown);

        const auto lineHits = Geometry::IntersectAllLineCurveWithStatus(line, curve);
        ASSERT_EQ(lineHits.hits.size(), 2);
        EXPECT_NEAR(lineHits.hits[0].curveParameter, first, 1e-6);
        EXPECT_NEAR(lineHits.hits[1].curveParameter, second, 1e-6);
        EXPECT_EQ(lineHits.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Unknown);
    }

    TEST(CurveOnSurfaceInterfaceTest, CurvedSingleUvSegmentFindsOffGridTangency)
    {
        constexpr double root = 0.37;
        const auto composed = MakeQuadraticHeightCurve(root, 0.0);
        ASSERT_TRUE(composed.IsValid());
        const Geometry::ISCCurve3d& curve = composed;
        const auto plane = Geometry::SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});
        const auto line = Geometry::SCLine3d::FromOriginAndDirection({0, 0.5, 0}, {1, 0, 0});

        const auto planeHits = Geometry::IntersectAllPlaneCurveWithStatus(plane, curve);
        ASSERT_EQ(planeHits.hits.size(), 1);
        EXPECT_NEAR(planeHits.hits.front().curveParameter, root, 1e-6);
        EXPECT_EQ(planeHits.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Unknown);

        const auto lineHits = Geometry::IntersectAllLineCurveWithStatus(line, curve);
        ASSERT_EQ(lineHits.hits.size(), 1);
        EXPECT_NEAR(lineHits.hits.front().curveParameter, root, 1e-6);
        EXPECT_EQ(lineHits.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Unknown);
    }

    TEST(CurveOnSurfaceInterfaceTest, CurvedSingleUvSegmentFindsNarrowOffGridPair)
    {
        constexpr double center = 0.37;
        constexpr double radius = 0.003;
        const auto composed = MakeQuadraticHeightCurve(center, radius * radius);
        ASSERT_TRUE(composed.IsValid());
        const Geometry::ISCCurve3d& curve = composed;
        const auto plane = Geometry::SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});
        const auto line = Geometry::SCLine3d::FromOriginAndDirection({0, 0.5, 0}, {1, 0, 0});

        const auto planeHits = Geometry::IntersectAllPlaneCurveWithStatus(plane, curve);
        ASSERT_EQ(planeHits.hits.size(), 2);
        EXPECT_NEAR(planeHits.hits[0].curveParameter, center - radius, 1e-6);
        EXPECT_NEAR(planeHits.hits[1].curveParameter, center + radius, 1e-6);
        EXPECT_EQ(planeHits.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Unknown);

        const auto lineHits = Geometry::IntersectAllLineCurveWithStatus(line, curve);
        ASSERT_EQ(lineHits.hits.size(), 2);
        EXPECT_NEAR(lineHits.hits[0].curveParameter, center - radius, 1e-6);
        EXPECT_NEAR(lineHits.hits[1].curveParameter, center + radius, 1e-6);
        EXPECT_EQ(lineHits.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Unknown);
    }

    TEST(CurveOnSurfaceInterfaceTest, CurvedUvSegmentFindsNearEndpointTangency)
    {
        constexpr double root = 0.01;
        const auto composed = MakeQuadraticHeightCurve(root, 0.0);
        ASSERT_TRUE(composed.IsValid());
        const Geometry::ISCCurve3d& curve = composed;
        const auto plane = Geometry::SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});
        const auto line = Geometry::SCLine3d::FromOriginAndDirection({0, 0.5, 0}, {1, 0, 0});

        const auto planeHits = Geometry::IntersectAllPlaneCurveWithStatus(plane, curve);
        ASSERT_EQ(planeHits.hits.size(), 1);
        EXPECT_NEAR(planeHits.hits.front().curveParameter, root, 1e-6);
        const auto lineHits = Geometry::IntersectAllLineCurveWithStatus(line, curve);
        ASSERT_EQ(lineHits.hits.size(), 1);
        EXPECT_NEAR(lineHits.hits.front().curveParameter, root, 1e-6);
    }

    TEST(CurveOnSurfaceInterfaceTest, CurvedUvSegmentFindsThreeRootsInsideOneSampleCell)
    {
        constexpr std::array<double, 3> roots{0.35, 0.355, 0.36};
        const auto composed = MakeCubicHeightCurve(roots);
        ASSERT_TRUE(composed.IsValid());
        const Geometry::ISCCurve3d& curve = composed;
        const auto plane = Geometry::SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});
        const auto line = Geometry::SCLine3d::FromOriginAndDirection({0, 0.5, 0}, {1, 0, 0});

        const auto& surface = static_cast<const Geometry::SCNurbsSurface&>(*composed.SupportSurface());
        const auto& controls = surface.ControlPoints();
        const Geometry::SCNurbsCurve3d directCurve(
            3, std::vector<Geometry::SCPoint3d>(controls.begin(), controls.begin() + 4), surface.KnotsU());
        const auto directHits = Geometry::IntersectAllPlaneCurveWithStatus(plane, directCurve);
        ASSERT_EQ(directHits.hits.size(), roots.size());

        const auto planeHits = Geometry::IntersectAllPlaneCurveWithStatus(plane, curve);
        ASSERT_EQ(planeHits.hits.size(), roots.size());
        const auto lineHits = Geometry::IntersectAllLineCurveWithStatus(line, curve);
        ASSERT_EQ(lineHits.hits.size(), roots.size());
        for (std::size_t i = 0; i < roots.size(); ++i)
        {
            EXPECT_NEAR(planeHits.hits[i].curveParameter, roots[i], 1e-5);
            EXPECT_NEAR(lineHits.hits[i].curveParameter, roots[i], 1e-5);
        }
        EXPECT_EQ(planeHits.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Unknown);
        EXPECT_EQ(lineHits.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Unknown);
    }

    TEST(CurveOnSurfaceInterfaceTest, DiagonalUvSegmentFindsThreeRootsInsideOneSampleCell)
    {
        constexpr std::array<double, 3> roots{0.35, 0.355, 0.36};
        const auto axial = MakeCubicHeightCurve(roots);
        const auto* sourceSurface = static_cast<const Geometry::SCNurbsSurface*>(axial.SupportSurface());
        ASSERT_NE(sourceSurface, nullptr);
        const auto surface = std::make_shared<Geometry::SCNurbsSurface>(*sourceSurface);
        const Geometry::SCCurveOnSurface diagonal(
            surface, Geometry::SCPolyline2d({{0, 0}, {1, 1}}));
        ASSERT_TRUE(diagonal.IsValid());

        const auto plane = Geometry::SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});
        const auto line = Geometry::SCLine3d::FromOriginAndDirection({0, 0, 0}, {1, 1, 0});
        const Geometry::ISCCurve3d& curve = diagonal;
        const auto restricted = Geometry::Detail::RestrictBezierNurbsSurfaceSegment(diagonal, 0, {});
        ASSERT_TRUE(restricted.has_value());
        EXPECT_EQ(restricted->curve.Degree(), 3);
        for (const double parameter : {0.0, 0.2, 0.5, 0.8, 1.0})
        {
            EXPECT_TRUE(restricted->curve.PointAt(parameter).AlmostEquals(diagonal.PointAt(parameter), 1e-12));
        }
        const auto restrictedHits = Geometry::IntersectAllPlaneCurveWithStatus(plane, restricted->curve);
        EXPECT_EQ(restrictedHits.hits.size(), roots.size());
        const auto planeHits = Geometry::IntersectAllPlaneCurveWithStatus(plane, curve);
        const auto lineHits = Geometry::IntersectAllLineCurveWithStatus(line, curve);
        ASSERT_EQ(planeHits.hits.size(), roots.size());
        ASSERT_EQ(lineHits.hits.size(), roots.size());
        for (std::size_t i = 0; i < roots.size(); ++i)
        {
            EXPECT_NEAR(planeHits.hits[i].curveParameter, roots[i], 1e-5);
            EXPECT_NEAR(lineHits.hits[i].curveParameter, roots[i], 1e-5);
        }
    }

    TEST(CurveOnSurfaceInterfaceTest, RationalDiagonalUvSegmentPreservesReverseParameterMapping)
    {
        const auto surface = std::make_shared<Geometry::SCNurbsSurface>(
            1, 1, 2, 2,
            std::vector<Geometry::SCPoint3d>{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}},
            std::vector<double>{2, 2, 5, 5}, std::vector<double>{-1, -1, 3, 3},
            std::vector<double>{1, 2, 3, 4});
        const Geometry::SCCurveOnSurface diagonal(
            surface, Geometry::SCPolyline2d({{5, 3}, {2, -1}}));
        ASSERT_TRUE(diagonal.IsValid());
        const auto restricted = Geometry::Detail::RestrictBezierNurbsSurfaceSegment(diagonal, 0, {});
        ASSERT_TRUE(restricted.has_value());
        EXPECT_EQ(restricted->curve.Degree(), 2);
        for (const double parameter : {0.0, 0.2, 0.5, 0.8, 1.0})
        {
            EXPECT_TRUE(restricted->curve.PointAt(parameter).AlmostEquals(diagonal.PointAt(parameter), 1e-12));
        }

        const auto plane = Geometry::SCPlane::FromPointAndNormal({0.5, 0, 0}, {1, 0, 0});
        const auto line = Geometry::SCLine3d::FromOriginAndDirection({0.5, 0, 0}, {0, 1, 0});
        double lower = 0.0;
        double upper = 1.0;
        for (int iteration = 0; iteration < 60; ++iteration)
        {
            const double middle = std::midpoint(lower, upper);
            if (diagonal.PointAt(middle).x > 0.5)
            {
                lower = middle;
            }
            else
            {
                upper = middle;
            }
        }
        const double expected = std::midpoint(lower, upper);
        const Geometry::ISCCurve3d& curve = diagonal;
        const auto planeHits = Geometry::IntersectAllPlaneCurveWithStatus(plane, curve);
        const auto lineHits = Geometry::IntersectAllLineCurveWithStatus(line, curve);
        ASSERT_EQ(planeHits.hits.size(), 1U);
        ASSERT_EQ(lineHits.hits.size(), 1U);
        EXPECT_NEAR(planeHits.hits.front().curveParameter, expected, 1e-8);
        EXPECT_NEAR(lineHits.hits.front().curveParameter, expected, 1e-8);
        EXPECT_TRUE(planeHits.hits.front().point.AlmostEquals(diagonal.PointAt(expected), 1e-8));
        const auto overlap = Geometry::IntersectAllPlaneCurveWithStatus(
            Geometry::SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1}), curve);
        EXPECT_EQ(overlap.state, Geometry::SCGeometryOperationState::NonUnique);
        EXPECT_TRUE(overlap.hits.empty());
    }

    TEST(CurveOnSurfaceInterfaceTest, TensorBezierDiagonalRestrictionMatchesRationalSurface)
    {
        std::vector<Geometry::SCPoint3d> controls;
        std::vector<double> weights;
        for (int v = 0; v < 3; ++v)
        {
            for (int u = 0; u < 3; ++u)
            {
                controls.push_back({0.5 * u, 0.5 * v,
                                    0.1 * u * u + 0.2 * u * v - 0.15 * v});
                weights.push_back(1.0 + 0.2 * u + 0.3 * v + 0.1 * u * v);
            }
        }
        const auto surface = std::make_shared<Geometry::SCNurbsSurface>(
            2, 2, 3, 3, std::move(controls),
            std::vector<double>{0, 0, 0, 1, 1, 1},
            std::vector<double>{0, 0, 0, 1, 1, 1}, std::move(weights));
        const Geometry::SCCurveOnSurface diagonal(
            surface, Geometry::SCPolyline2d({{0.1, 0.2}, {0.9, 0.7}}));
        ASSERT_TRUE(diagonal.IsValid());
        const auto restricted = Geometry::Detail::RestrictBezierNurbsSurfaceSegment(diagonal, 0, {});
        ASSERT_TRUE(restricted.has_value());
        EXPECT_EQ(restricted->curve.Degree(), 4);
        for (int index = 0; index <= 10; ++index)
        {
            const double parameter = 0.1 * index;
            EXPECT_TRUE(restricted->curve.PointAt(parameter).AlmostEquals(
                diagonal.PointAt(parameter), 1e-12)) << "parameter=" << parameter;
        }
    }

    TEST(CurveOnSurfaceInterfaceTest, DiagonalUvSegmentAcrossKnotFindsAllCloseRoots)
    {
        constexpr std::array<double, 3> roots{0.35, 0.355, 0.36};
        const auto single = MakeCubicHeightCurve(roots);
        const auto* singleSurface = static_cast<const Geometry::SCNurbsSurface*>(single.SupportSurface());
        ASSERT_NE(singleSurface, nullptr);
        const auto& firstPatch = singleSurface->ControlPoints();
        const double trailingHeight = firstPatch[3].z;

        std::vector<Geometry::SCPoint3d> controls;
        for (double v : {0.0, 1.0})
        {
            for (int u = 0; u < 7; ++u)
            {
                controls.push_back({static_cast<double>(u) / 6.0, v,
                                    u < 4 ? firstPatch[u].z : trailingHeight});
            }
        }
        const auto surface = std::make_shared<Geometry::SCNurbsSurface>(
            3, 1, 7, 2, std::move(controls),
            std::vector<double>{0, 0, 0, 0, 0.5, 0.5, 0.5, 1, 1, 1, 1},
            std::vector<double>{0, 0, 1, 1});
        const Geometry::SCCurveOnSurface diagonal(
            surface, Geometry::SCPolyline2d({{0, 0}, {1, 1}}));
        ASSERT_TRUE(diagonal.IsValid());

        const auto plane = Geometry::SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});
        const auto line = Geometry::SCLine3d::FromOriginAndDirection({0, 0, 0}, {1, 1, 0});
        const Geometry::ISCCurve3d& curve = diagonal;
        const auto planeHits = Geometry::IntersectAllPlaneCurveWithStatus(plane, curve);
        const auto lineHits = Geometry::IntersectAllLineCurveWithStatus(line, curve);
        ASSERT_EQ(planeHits.hits.size(), roots.size());
        ASSERT_EQ(lineHits.hits.size(), roots.size());
        for (std::size_t index = 0; index < roots.size(); ++index)
        {
            const double expected = 0.5 * roots[index];
            EXPECT_NEAR(planeHits.hits[index].curveParameter, expected, 1e-5);
            EXPECT_NEAR(lineHits.hits[index].curveParameter, expected, 1e-5);
        }
    }

    TEST(CurveOnSurfaceInterfaceTest, CrossKnotRestrictionMatchesRationalSurfaceOnBothAxes)
    {
        std::vector<Geometry::SCPoint3d> controls;
        std::vector<double> weights;
        for (int v = 0; v < 4; ++v)
        {
            for (int u = 0; u < 4; ++u)
            {
                controls.push_back({0.3 * u, 0.2 * v, 0.1 * u * v + 0.05 * u * u});
                weights.push_back(1.0 + 0.1 * u + 0.15 * v);
            }
        }
        const auto surface = std::make_shared<Geometry::SCNurbsSurface>(
            2, 2, 4, 4, std::move(controls),
            std::vector<double>{0, 0, 0, 0.5, 1, 1, 1},
            std::vector<double>{0, 0, 0, 0.6, 1, 1, 1}, std::move(weights));
        const Geometry::SCCurveOnSurface diagonal(
            surface, Geometry::SCPolyline2d({{0.9, 0.95}, {0.1, 0.05}}));
        ASSERT_TRUE(diagonal.IsValid());
        const auto pieces = Geometry::Detail::RestrictNurbsSurfaceSegmentPieces(diagonal, 0, {});
        ASSERT_EQ(pieces.size(), 3U);
        for (const auto& piece : pieces)
        {
            for (const double local : {0.0, 0.25, 0.5, 0.75, 1.0})
            {
                const double parameter = std::lerp(piece.segmentStart, piece.segmentEnd, local);
                EXPECT_TRUE(piece.curve.PointAt(local).AlmostEquals(
                    diagonal.PointAt(parameter), 1e-11)) << "parameter=" << parameter;
            }
        }
    }

    TEST(CurveOnSurfaceInterfaceTest, NonClampedCrossKnotRestrictionMatchesRationalSurface)
    {
        std::vector<Geometry::SCPoint3d> controls;
        std::vector<double> weights;
        for (int v = 0; v < 4; ++v)
        {
            for (int u = 0; u < 4; ++u)
            {
                controls.push_back({0.3 * u, 0.2 * v, 0.1 * u * v + 0.05 * u * u});
                weights.push_back(1.0 + 0.1 * u + 0.15 * v);
            }
        }
        const auto surface = std::make_shared<Geometry::SCNurbsSurface>(
            2, 2, 4, 4, std::move(controls),
            std::vector<double>{0, 0, 0.2, 0.5, 0.8, 1, 1},
            std::vector<double>{-1, -1, 0.1, 0.6, 0.9, 2, 2}, std::move(weights));
        const Geometry::SCCurveOnSurface diagonal(
            surface, Geometry::SCPolyline2d({{0.8, 0.9}, {0.2, 0.1}}));
        ASSERT_TRUE(diagonal.IsValid());
        const auto pieces = Geometry::Detail::RestrictNurbsSurfaceSegmentPieces(diagonal, 0, {});
        ASSERT_EQ(pieces.size(), 3U);
        for (const auto& piece : pieces)
        {
            for (const double local : {0.0, 0.25, 0.5, 0.75, 1.0})
            {
                const double parameter = std::lerp(piece.segmentStart, piece.segmentEnd, local);
                EXPECT_TRUE(piece.curve.PointAt(local).AlmostEquals(
                    diagonal.PointAt(parameter), 1e-11)) << "parameter=" << parameter;
            }
        }
    }

    TEST(CurveOnSurfaceInterfaceTest, NonClampedAxisSegmentUsesClampedRootPieces)
    {
        std::vector<Geometry::SCPoint3d> controls;
        for (double v : {0.0, 1.0})
        {
            controls.insert(controls.end(), {{0.0, v, 0.2}, {0.3, v, -0.1},
                                             {0.6, v, 0.1}, {0.9, v, -0.2}});
        }
        const auto surface = std::make_shared<Geometry::SCNurbsSurface>(
            2, 1, 4, 2, std::move(controls),
            std::vector<double>{0, 0, 0.2, 0.5, 0.8, 1, 1},
            std::vector<double>{0, 0, 1, 1});
        const Geometry::SCCurveOnSurface axis(
            surface, Geometry::SCPolyline2d({{0.2, 0.5}, {0.8, 0.5}}));
        ASSERT_TRUE(axis.IsValid());
        const auto pieces = Geometry::Detail::RestrictNurbsSurfaceSegmentPieces(axis, 0, {});
        ASSERT_EQ(pieces.size(), 2U);
        for (const auto& piece : pieces)
        {
            const auto& knots = piece.curve.Knots();
            const int degree = piece.curve.Degree();
            ASSERT_EQ(std::count(knots.begin(), knots.end(), 0.0), degree + 1);
            ASSERT_EQ(std::count(knots.begin(), knots.end(), 1.0), degree + 1);
            for (double local : {0.0, 0.25, 0.5, 0.75, 1.0})
            {
                const double parameter = std::lerp(piece.segmentStart, piece.segmentEnd, local);
                EXPECT_TRUE(piece.curve.PointAt(local).AlmostEquals(
                    axis.PointAt(parameter), 1e-11));
            }
        }
    }

    TEST(CurveOnSurfaceInterfaceTest, DegreeEighteenRationalRestrictionFindsCloseRoots)
    {
        constexpr std::array<double, 3> roots{0.35, 0.355, 0.36};
        const auto cubic = MakeCubicHeightCurve(roots);
        const auto* cubicSurface = static_cast<const Geometry::SCNurbsSurface*>(cubic.SupportSurface());
        ASSERT_NE(cubicSurface, nullptr);
        std::vector<double> heights;
        for (int u = 0; u < 4; ++u)
        {
            heights.push_back(cubicSurface->ControlPoints()[u].z);
        }
        for (int degree = 3; degree < 9; ++degree)
        {
            std::vector<double> elevated(static_cast<std::size_t>(degree + 2));
            elevated.front() = heights.front();
            elevated.back() = heights.back();
            for (int u = 1; u <= degree; ++u)
            {
                elevated[u] = std::lerp(heights[u], heights[u - 1],
                                        static_cast<double>(u) / (degree + 1));
            }
            heights = std::move(elevated);
        }

        std::vector<Geometry::SCPoint3d> controls;
        std::vector<double> weights;
        for (int v = 0; v <= 9; ++v)
        {
            for (int u = 0; u <= 9; ++u)
            {
                const double weight = 1.0 + 0.1 * ((u + v) % 3) + 0.05 * u * v / 81.0;
                controls.push_back({static_cast<double>(u) / 9.0,
                                    static_cast<double>(v) / 9.0, heights[u] / weight});
                weights.push_back(weight);
            }
        }
        std::vector<double> knots(20, 1.0);
        std::fill(knots.begin(), knots.begin() + 10, 0.0);
        const auto surface = std::make_shared<Geometry::SCNurbsSurface>(
            9, 9, 10, 10, std::move(controls), knots, knots, std::move(weights));
        const Geometry::SCCurveOnSurface diagonal(
            surface, Geometry::SCPolyline2d({{0, 0}, {1, 1}}));
        ASSERT_TRUE(diagonal.IsValid());
        const auto restricted = Geometry::Detail::RestrictBezierNurbsSurfaceSegment(diagonal, 0, {});
        ASSERT_TRUE(restricted.has_value());
        ASSERT_EQ(restricted->curve.Degree(), 18);
        for (const double parameter : {0.0, 0.2, 0.5, 0.8, 1.0})
        {
            EXPECT_TRUE(restricted->curve.PointAt(parameter).AlmostEquals(
                diagonal.PointAt(parameter), 1e-10));
        }
        const auto plane = Geometry::SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});
        const auto line = Geometry::SCLine3d::FromOriginAndDirection({0, 0, 0}, {1, 1, 0});
        const Geometry::ISCCurve3d& curve = diagonal;
        const auto restrictedHits = Geometry::IntersectAllPlaneCurveWithStatus(plane, restricted->curve);
        ASSERT_EQ(restrictedHits.hits.size(), roots.size());
        const auto planeHits = Geometry::IntersectAllPlaneCurveWithStatus(plane, curve);
        const auto lineHits = Geometry::IntersectAllLineCurveWithStatus(line, curve);
        ASSERT_EQ(planeHits.hits.size(), roots.size());
        ASSERT_EQ(lineHits.hits.size(), roots.size());
        for (std::size_t index = 0; index < roots.size(); ++index)
        {
            EXPECT_NEAR(planeHits.hits[index].curveParameter, roots[index], 1e-5);
            EXPECT_NEAR(lineHits.hits[index].curveParameter, roots[index], 1e-5);
        }
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
