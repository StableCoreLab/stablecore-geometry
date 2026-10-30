#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <type_traits>
#include <vector>

#include "Core/Projection.h"
#include "Core/Intersection.h"
#include "Core/Measure.h"
#include "Geometry3d/ISCCurve3d.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry3d/SCCurveOnSurface.h"
#include "Geometry3d/SCNurbsSurface.h"
#include "Geometry3d/SCPlaneSurface.h"

static_assert(std::is_base_of_v<Geometry::ISCCurve3d, Geometry::SCCurveOnSurface>);

namespace
{
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

    TEST(CurveOnSurfaceInterfaceTest, SmallUvSegmentUsesCallerParameterToleranceAcrossEvaluation)
    {
        constexpr double uvWidth = 1e-10;
        Geometry::SCGeometryTolerance3d tolerance{};
        tolerance.distanceEpsilon = 1e-15;
        tolerance.parameterEpsilon = 1e-15;

        const auto plane = std::make_shared<Geometry::SCPlaneSurface>(
            Geometry::SCPlaneSurface::FromPlane(
                Geometry::SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1}), {-1, 1}, {-1, 1}));
        const Geometry::SCCurveOnSurface planar(
            plane, Geometry::SCPolyline2d({{0.0, 0.0}, {uvWidth, 0.0}}));
        ASSERT_FALSE(planar.IsValid());
        ASSERT_TRUE(planar.UvCurve().IsValid(tolerance.parameterEpsilon));
        EXPECT_TRUE(planar.IsValid(tolerance));
        auto exactParameters = tolerance;
        exactParameters.parameterEpsilon = 0.0;
        EXPECT_TRUE(planar.IsValid(exactParameters));
        EXPECT_NEAR(planar.UvAt(0.5).x, uvWidth * 0.5, tolerance.parameterEpsilon);
        EXPECT_TRUE(planar.PointAt(0.5).IsValid());
        EXPECT_TRUE(planar.Evaluate(0.5, 1).IsValid());
        EXPECT_TRUE(planar.Bounds().IsValid());
        EXPECT_NEAR(Geometry::Length(planar), uvWidth, tolerance.distanceEpsilon);
        EXPECT_NEAR(Geometry::Length(planar, tolerance), uvWidth, tolerance.distanceEpsilon);

        const auto surface = std::make_shared<Geometry::SCNurbsSurface>(
            1, 1, 2, 2,
            std::vector<Geometry::SCPoint3d>{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}},
            std::vector<double>{0, 0, 1, 1}, std::vector<double>{0, 0, 1, 1});
        const Geometry::SCCurveOnSurface curved(
            surface, Geometry::SCPolyline2d({{0.5, 0.5}, {0.5 + uvWidth, 0.5}}));
        ASSERT_FALSE(curved.IsValid());
        EXPECT_TRUE(curved.IsValid(tolerance));
        EXPECT_TRUE(curved.UvAt(0.5).IsValid());
        EXPECT_TRUE(curved.PointAt(0.5).IsValid());
        EXPECT_TRUE(curved.Evaluate(0.5, 1).IsValid());
        EXPECT_TRUE(curved.Bounds().IsValid());
        EXPECT_NEAR(Geometry::Length(curved), uvWidth, tolerance.distanceEpsilon);
        EXPECT_NEAR(Geometry::Length(curved, tolerance), uvWidth, tolerance.distanceEpsilon);
    }

    TEST(CurveOnSurfaceInterfaceTest, LengthAcceptsCallerSurfaceDistanceTolerance)
    {
        const auto surface = std::make_shared<Geometry::SCPlaneSurface>(
            Geometry::SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1e-10}),
            Geometry::SCVector3d{1, 0, 0}, Geometry::SCVector3d{0, 1, 0},
            Geometry::SCIntervald{-1, 1}, Geometry::SCIntervald{-1, 1});
        const Geometry::SCCurveOnSurface curve(surface, Geometry::SCPolyline2d({{0, 0}, {1, 0}}));
        Geometry::SCGeometryTolerance3d tolerance{};
        tolerance.distanceEpsilon = 1e-15;
        ASSERT_FALSE(curve.IsValid());
        ASSERT_TRUE(curve.IsValid(tolerance));
        EXPECT_DOUBLE_EQ(Geometry::Length(curve), 0.0);
        EXPECT_NEAR(Geometry::Length(curve, tolerance), 1.0, tolerance.distanceEpsilon);

        tolerance.distanceEpsilon = -1.0;
        EXPECT_DOUBLE_EQ(Geometry::Length(curve, tolerance), 0.0);
    }

    TEST(CurveOnSurfaceInterfaceTest, LengthRefinesSmallCurvedSurfaceAtCallerTolerance)
    {
        constexpr double scale = 1e-7;
        const auto surface = std::make_shared<Geometry::SCNurbsSurface>(
            1, 1, 2, 2,
            std::vector<Geometry::SCPoint3d>{{0, 0, 0}, {scale, 0, 0}, {0, scale, 0},
                                             {scale, scale, scale}},
            std::vector<double>{0, 0, 1, 1}, std::vector<double>{0, 0, 1, 1});
        const Geometry::SCCurveOnSurface curve(surface, Geometry::SCPolyline2d({{0, 0}, {1, 1}}));
        Geometry::SCGeometryTolerance3d tolerance{};
        tolerance.distanceEpsilon = 1e-15;
        ASSERT_TRUE(curve.IsValid(tolerance));

        const double exactLength = scale *
            (std::sqrt(6.0) / 2.0 + std::asinh(std::sqrt(2.0)) / 2.0);
        EXPECT_NEAR(Geometry::Length(curve, tolerance), exactLength, 1e-13);
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
        EXPECT_FALSE(outside.Bounds().IsValid());

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
}
