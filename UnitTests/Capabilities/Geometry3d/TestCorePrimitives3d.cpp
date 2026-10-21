#include <gtest/gtest.h>

#include "Geometry.h"
#include "Support/Fixtures3d.h"

using namespace Geometry;

TEST(GeometryTest, ThreeDimensionalIntegerPrimitivesFollowTheTemplatedFamily)
{
    const SCPoint3i minimum{1, 2, 3};
    const SCPoint3i maximum{4, 6, 8};
    SCBox3i box = SCBox3i::FromMinMax(minimum, maximum);
    EXPECT_TRUE(box.IsValid());
    EXPECT_EQ(box.Width(), 3.0);
    EXPECT_EQ(box.Height(), 4.0);
    EXPECT_EQ(box.Depth(), 5.0);
    EXPECT_EQ(box.Center(), SCPoint3d(2.5, 4.0, 5.5));
    EXPECT_TRUE(box.AlmostEquals(SCBox3i::FromMinMax(minimum, maximum)));
    EXPECT_TRUE((SCVector3i{1, 2, 3} + SCVector3i{2, 3, 4}) == SCVector3i(3, 5, 7));
}

TEST(GeometryTest, CoversThreeDimensionalCapabilities)
{
    const SCLine3d line3 = SCLine3d::FromOriginAndDirection(SCPoint3d{1.0, 2.0, 3.0}, SCVector3d{2.0, 0.0, 0.0});
    const SCLineCurve3d lineCurve = SCLineCurve3d::FromLine(line3, SCIntervald{-2.0, 3.0});
    ASSERT_TRUE(lineCurve.IsValid());
    ASSERT_FALSE(lineCurve.IsClosed());
    ASSERT_FALSE(lineCurve.IsPeriodic());
    EXPECT_NEAR(lineCurve.StartParameter(), -2.0, 1e-12);
    EXPECT_NEAR(lineCurve.EndParameter(), 3.0, 1e-12);
    ASSERT_TRUE(lineCurve.PointAt(0.5).AlmostEquals(SCPoint3d{2.0, 2.0, 3.0}, 1e-12));
    const auto lineEval = lineCurve.Evaluate(1.5, 2);
    ASSERT_TRUE(lineEval.IsValid());
    ASSERT_EQ(lineEval.derivativeOrder, 2);
    ASSERT_TRUE(lineEval.point.AlmostEquals(SCPoint3d{4.0, 2.0, 3.0}, 1e-12));
    ASSERT_TRUE(lineEval.firstDerivative.AlmostEquals(SCVector3d{2.0, 0.0, 0.0}, 1e-12));
    ASSERT_TRUE(lineEval.secondDerivative.AlmostEquals(SCVector3d{0.0, 0.0, 0.0}, 1e-12));
    const SCBox3d lineBounds = lineCurve.Bounds();
    ASSERT_TRUE(lineBounds.IsValid());
    ASSERT_TRUE(lineBounds.MinPoint().AlmostEquals(SCPoint3d{-3.0, 2.0, 3.0}, 1e-12));
    ASSERT_TRUE(lineBounds.MaxPoint().AlmostEquals(SCPoint3d{7.0, 2.0, 3.0}, 1e-12));
    std::unique_ptr<ISCCurve3d> lineClone = lineCurve.Clone();
    ASSERT_NE(lineClone, nullptr);
    ASSERT_TRUE(lineClone->IsValid());
    ASSERT_TRUE(lineClone->PointAt(-2.0).AlmostEquals(SCPoint3d{-3.0, 2.0, 3.0}, 1e-12));

    const SCPlane supportPlane = Geometry::Test::BuildSupportPlaneAtZ5();
    const SCPlaneSurface planeSurface = Geometry::Test::BuildFinitePlaneSurfaceAtZ5();
    ASSERT_TRUE(planeSurface.IsValid());
    EXPECT_NEAR(planeSurface.StartU(), -2.0, 1e-12);
    EXPECT_NEAR(planeSurface.EndU(), 2.0, 1e-12);
    EXPECT_NEAR(planeSurface.StartV(), -3.0, 1e-12);
    EXPECT_NEAR(planeSurface.EndV(), 1.0, 1e-12);
    const SCPoint3d planePoint = planeSurface.PointAt(0.5, -1.0);
    EXPECT_NEAR(supportPlane.SignedDistanceTo(planePoint), 0.0, 1e-12);
    const auto surfaceEval = planeSurface.Evaluate(0.25, 0.75, 1);
    ASSERT_TRUE(surfaceEval.IsValid());
    ASSERT_TRUE(surfaceEval.point.AlmostEquals(planeSurface.PointAt(0.25, 0.75), 1e-12));
    EXPECT_NEAR(Geometry::Dot(surfaceEval.derivativeU, surfaceEval.normal), 0.0, 1e-12);
    EXPECT_NEAR(Geometry::Dot(surfaceEval.derivativeV, surfaceEval.normal), 0.0, 1e-12);
    const SCBox3d surfaceBounds = planeSurface.Bounds();
    ASSERT_TRUE(surfaceBounds.IsValid());
    EXPECT_NEAR(surfaceBounds.MinPoint().z, 5.0, 1e-12);
    EXPECT_NEAR(surfaceBounds.MaxPoint().z, 5.0, 1e-12);
    std::unique_ptr<ISCSurface> surfaceClone = planeSurface.Clone();
    ASSERT_NE(surfaceClone, nullptr);
    ASSERT_TRUE(surfaceClone->IsValid());
    EXPECT_NEAR(supportPlane.SignedDistanceTo(surfaceClone->PointAt(-2.0, -3.0)), 0.0, 1e-12);

    const SCNurbsCurve3d nurbsCurve(1,
                                  {
                                      SCPoint3d{0.0, 0.0, 0.0},
                                      SCPoint3d{1.0, 1.0, 0.0},
                                      SCPoint3d{2.0, 0.0, 0.0},
                                  },
                                  {0.0, 0.0, 0.5, 1.0, 1.0});
    ASSERT_TRUE(nurbsCurve.IsValid());
    ASSERT_FALSE(nurbsCurve.IsPeriodic());
    EXPECT_NEAR(nurbsCurve.StartParameter(), 0.0, 1e-12);
    EXPECT_NEAR(nurbsCurve.EndParameter(), 1.0, 1e-12);
    ASSERT_TRUE(nurbsCurve.PointAt(0.5).AlmostEquals(SCPoint3d{1.0, 1.0, 0.0}, 1e-12));
    const auto nurbsCurveEval = nurbsCurve.Evaluate(0.25, 2);
    ASSERT_TRUE(nurbsCurveEval.IsValid());
    ASSERT_EQ(nurbsCurveEval.derivativeOrder, 2);
    const auto nurbsCurveProjection = ProjectPointToCurve(SCPoint3d{1.0, 0.0, 1.0}, nurbsCurve);
    ASSERT_TRUE(nurbsCurveProjection.success);
    ASSERT_TRUE(nurbsCurveProjection.IsValid());
    EXPECT_NEAR(nurbsCurveProjection.point.z, 0.0, 1e-12);
    EXPECT_NEAR(Length(nurbsCurve, 64), std::sqrt(2.0) * 2.0, 5e-2);
    const SCBox3d nurbsCurveBounds = nurbsCurve.Bounds();
    ASSERT_TRUE(nurbsCurveBounds.IsValid());
    ASSERT_TRUE(nurbsCurveBounds.MinPoint().AlmostEquals(SCPoint3d{0.0, 0.0, 0.0}, 1e-12));
    ASSERT_TRUE(nurbsCurveBounds.MaxPoint().AlmostEquals(SCPoint3d{2.0, 1.0, 0.0}, 1e-12));
    std::unique_ptr<ISCCurve3d> nurbsCurveClone = nurbsCurve.Clone();
    ASSERT_NE(nurbsCurveClone, nullptr);
    ASSERT_TRUE(nurbsCurveClone->IsValid());
    ASSERT_TRUE(nurbsCurveClone->PointAt(0.5).AlmostEquals(SCPoint3d{1.0, 1.0, 0.0}, 1e-12));

    const SCNurbsSurface nurbsSurface = Geometry::Test::BuildUnitNurbsSurface();
    ASSERT_TRUE(nurbsSurface.IsValid());
    EXPECT_NEAR(nurbsSurface.StartU(), 0.0, 1e-12);
    EXPECT_NEAR(nurbsSurface.EndU(), 1.0, 1e-12);
    EXPECT_NEAR(nurbsSurface.StartV(), 0.0, 1e-12);
    EXPECT_NEAR(nurbsSurface.EndV(), 1.0, 1e-12);
    ASSERT_TRUE(nurbsSurface.PointAt(0.5, 0.5).AlmostEquals(SCPoint3d{1.0, 1.0, 0.0}, 1e-12));
    const auto nurbsSurfaceEval = nurbsSurface.Evaluate(0.5, 0.5, 1);
    ASSERT_TRUE(nurbsSurfaceEval.IsValid());
    EXPECT_NEAR(nurbsSurfaceEval.normal.Normalized().z, 1.0, 1e-12);
    const SCBox3d nurbsSurfaceBounds = nurbsSurface.Bounds();
    ASSERT_TRUE(nurbsSurfaceBounds.IsValid());
    ASSERT_TRUE(nurbsSurfaceBounds.MinPoint().AlmostEquals(SCPoint3d{0.0, 0.0, 0.0}, 1e-12));
    ASSERT_TRUE(nurbsSurfaceBounds.MaxPoint().AlmostEquals(SCPoint3d{2.0, 2.0, 0.0}, 1e-12));
    std::unique_ptr<ISCSurface> nurbsSurfaceClone = nurbsSurface.Clone();
    ASSERT_NE(nurbsSurfaceClone, nullptr);
    ASSERT_TRUE(nurbsSurfaceClone->IsValid());
    ASSERT_TRUE(nurbsSurfaceClone->PointAt(0.25, 0.75).AlmostEquals(SCPoint3d{0.5, 1.5, 0.0}, 1e-12));

    const SCLineCurve3d lowerRail = SCLineCurve3d::FromLine(
        SCLine3d::FromOriginAndDirection(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{2.0, 0.0, 0.0}), SCIntervald{0.0, 1.0});
    const SCLineCurve3d upperRail = SCLineCurve3d::FromLine(
        SCLine3d::FromOriginAndDirection(SCPoint3d{0.0, 0.0, 2.0}, SCVector3d{2.0, 0.0, 0.0}), SCIntervald{0.0, 1.0});
    const auto lineCurveProjection = ProjectPointToCurve(SCPoint3d{0.5, 1.0, 0.0}, lowerRail);
    ASSERT_TRUE(lineCurveProjection.success);
    ASSERT_TRUE(lineCurveProjection.IsValid());
    EXPECT_NEAR(lineCurveProjection.parameter, 0.25, 1e-12);
    EXPECT_NEAR(Distance(SCPoint3d{0.5, 1.0, 0.0}, lowerRail), 1.0, 1e-12);
    const auto lineCurveIntersection =
        Intersect(SCLine3d::FromOriginAndDirection(SCPoint3d{0.5, 1.0, 0.0}, SCVector3d{0.0, -1.0, 0.0}), lowerRail);
    ASSERT_TRUE(lineCurveIntersection.intersects);
    ASSERT_TRUE(lineCurveIntersection.IsValid());
    EXPECT_NEAR(lineCurveIntersection.curveParameter, 0.25, 1e-12);
    const auto planeCurveIntersection =
        Intersect(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}), lowerRail);
    ASSERT_TRUE(planeCurveIntersection.intersects);
    ASSERT_TRUE(planeCurveIntersection.IsValid());
    EXPECT_NEAR(planeCurveIntersection.curveParameter, 0.0, 1e-12);
    ASSERT_TRUE(planeCurveIntersection.point.AlmostEquals(SCPoint3d{0.0, 0.0, 0.0}, 1e-12));
    ASSERT_EQ(LocatePoint(SCPoint3d{0.5, 0.0, 0.0}, lowerRail), SCPointContainment3d::OnBoundary);
    ASSERT_EQ(LocatePoint(SCPoint3d{0.5, 1.0, 0.0}, lowerRail), SCPointContainment3d::Outside);
    const SCRuledSurface ruledSurface = SCRuledSurface::FromCurves(lowerRail, upperRail);
    ASSERT_TRUE(ruledSurface.IsValid());
    ASSERT_NE(ruledSurface.FirstCurve(), nullptr);
    ASSERT_NE(ruledSurface.SecondCurve(), nullptr);
    ASSERT_TRUE(ruledSurface.PointAt(0.5, 0.5).AlmostEquals(SCPoint3d{1.0, 0.0, 1.0}, 1e-12));
    const auto ruledEval = ruledSurface.Evaluate(0.25, 0.25, 1);
    ASSERT_TRUE(ruledEval.IsValid());
    EXPECT_NEAR(ruledEval.normal.Length(), 4.0, 1e-12);

    const SCOffsetSurface offsetSurface = SCOffsetSurface::FromSurface(planeSurface, 2.0);
    ASSERT_TRUE(offsetSurface.IsValid());
    ASSERT_NE(offsetSurface.BaseSurface(), nullptr);
    EXPECT_NEAR(offsetSurface.OffsetDistance(), 2.0, 1e-12);
    EXPECT_NEAR(offsetSurface.PointAt(0.0, 0.0).z, 7.0, 1e-12);
    const SCBox3d offsetBounds = offsetSurface.Bounds();
    ASSERT_TRUE(offsetBounds.IsValid());
    ASSERT_LE(offsetBounds.MinPoint().z, 3.0 + 1e-12);
    ASSERT_GE(offsetBounds.MaxPoint().z, 7.0 - 1e-12);

    const SCCurveOnSurface curveOnSurface(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()),
                                        SCPolyline2d(
                                            {
                                                SCPoint2d{-2.0, -3.0},
                                                SCPoint2d{0.0, -1.0},
                                                SCPoint2d{2.0, 1.0},
                                            },
                                            SCPolylineClosure::Open));
    ASSERT_TRUE(curveOnSurface.IsValid());
    ASSERT_EQ(curveOnSurface.PointCount(), 3);
    ASSERT_TRUE(curveOnSurface.UvPointAt(1).AlmostEquals(SCPoint2d{0.0, -1.0}, 1e-12));
    ASSERT_TRUE(curveOnSurface.PointAt(1).AlmostEquals(planeSurface.PointAt(0.0, -1.0), 1e-12));
    const SCBox3d curveOnSurfaceBounds = curveOnSurface.Bounds();
    ASSERT_TRUE(curveOnSurfaceBounds.IsValid());
    EXPECT_NEAR(curveOnSurfaceBounds.MinPoint().z, 5.0, 1e-12);
    EXPECT_NEAR(curveOnSurfaceBounds.MaxPoint().z, 5.0, 1e-12);
    EXPECT_NEAR(Bounds(curveOnSurface).MinPoint().z, 5.0, 1e-12);
    EXPECT_NEAR(Length(curveOnSurface), std::sqrt(8.0) * 2.0, 1e-12);
    const auto curveOnSurfaceProjection = ProjectPointToCurveOnSurface(SCPoint3d{0.0, -1.0, 6.0}, curveOnSurface);
    ASSERT_TRUE(curveOnSurfaceProjection.success);
    ASSERT_TRUE(curveOnSurfaceProjection.IsValid());
    ASSERT_TRUE(curveOnSurfaceProjection.point.AlmostEquals(SCPoint3d{0.0, -1.0, 5.0}, 1e-12));
    EXPECT_NEAR(Distance(SCPoint3d{0.0, -1.0, 6.0}, curveOnSurface), 1.0, 1e-12);
    const auto curveOnSurfaceIntersection =
        Intersect(SCLine3d::FromOriginAndDirection(SCPoint3d{0.0, -1.0, 6.0}, SCVector3d{0.0, 0.0, -1.0}), curveOnSurface);
    ASSERT_TRUE(curveOnSurfaceIntersection.intersects);
    ASSERT_TRUE(curveOnSurfaceIntersection.IsValid());
    ASSERT_TRUE(curveOnSurfaceIntersection.point.AlmostEquals(SCPoint3d{0.0, -1.0, 5.0}, 1e-12));
    const auto planeCurveOnSurfaceIntersection = Intersect(supportPlane, curveOnSurface);
    ASSERT_TRUE(planeCurveOnSurfaceIntersection.intersects);
    ASSERT_TRUE(planeCurveOnSurfaceIntersection.IsValid());
    ASSERT_TRUE(planeCurveOnSurfaceIntersection.uv.AlmostEquals(SCPoint2d{-2.0, -3.0}, 1e-12));
    ASSERT_TRUE(planeCurveOnSurfaceIntersection.point.AlmostEquals(SCPoint3d{-2.0, -3.0, 5.0}, 1e-12));
    ASSERT_EQ(LocatePoint(SCPoint3d{0.0, -1.0, 5.0}, curveOnSurface), SCPointContainment3d::OnBoundary);
    ASSERT_EQ(LocatePoint(SCPoint3d{0.0, -1.0, 6.0}, curveOnSurface), SCPointContainment3d::Outside);

    const auto planeSurfaceProjection = ProjectPointToSurface(SCPoint3d{0.5, -1.0, 8.0}, planeSurface);
    ASSERT_TRUE(planeSurfaceProjection.success);
    ASSERT_TRUE(planeSurfaceProjection.IsValid());
    EXPECT_NEAR(planeSurfaceProjection.u, 0.5, 1e-12);
    EXPECT_NEAR(planeSurfaceProjection.v, -1.0, 1e-12);
    ASSERT_TRUE(planeSurfaceProjection.point.AlmostEquals(SCPoint3d{0.5, -1.0, 5.0}, 1e-12));

    const auto nurbsSurfaceProjection = ProjectPointToSurface(SCPoint3d{0.6, 1.4, 1.0}, nurbsSurface);
    ASSERT_TRUE(nurbsSurfaceProjection.success);
    ASSERT_TRUE(nurbsSurfaceProjection.IsValid());
    EXPECT_NEAR(nurbsSurfaceProjection.u, 0.3, 5e-2);
    EXPECT_NEAR(nurbsSurfaceProjection.v, 0.7, 5e-2);
    EXPECT_NEAR(nurbsSurfaceProjection.point.z, 0.0, 1e-12);

    const auto planeSurfaceIntersection =
        Intersect(SCLine3d::FromOriginAndDirection(SCPoint3d{0.5, -1.0, 8.0}, SCVector3d{0.0, 0.0, -1.0}), planeSurface);
    ASSERT_TRUE(planeSurfaceIntersection.intersects);
    ASSERT_TRUE(planeSurfaceIntersection.IsValid());
    EXPECT_NEAR(planeSurfaceIntersection.lineParameter, 3.0, 1e-12);
    EXPECT_NEAR(planeSurfaceIntersection.u, 0.5, 1e-12);
    EXPECT_NEAR(planeSurfaceIntersection.v, -1.0, 1e-12);
    ASSERT_TRUE(planeSurfaceIntersection.point.AlmostEquals(SCPoint3d{0.5, -1.0, 5.0}, 1e-12));

    const auto nurbsSurfaceIntersection =
        Intersect(SCLine3d::FromOriginAndDirection(SCPoint3d{0.6, 1.4, 1.0}, SCVector3d{0.0, 0.0, -1.0}), nurbsSurface);
    ASSERT_TRUE(nurbsSurfaceIntersection.intersects);
    ASSERT_TRUE(nurbsSurfaceIntersection.IsValid());
    EXPECT_NEAR(nurbsSurfaceIntersection.u, 0.3, 5e-2);
    EXPECT_NEAR(nurbsSurfaceIntersection.v, 0.7, 5e-2);
    EXPECT_NEAR(nurbsSurfaceIntersection.point.z, 0.0, 1e-12);

    EXPECT_NEAR(Distance(SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{1.0, 2.0, 2.0}), 3.0, 1e-12);
    EXPECT_NEAR(Distance(SCPoint3d{0.0, 2.0, 0.0},
                                       SCLine3d::FromOriginAndDirection(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, 0.0})),
                              2.0,
                              1e-12);
    EXPECT_NEAR(Distance(SCPoint3d{0.0, 0.0, 8.0}, supportPlane), 3.0, 1e-12);
    EXPECT_NEAR(Distance(SCPoint3d{0.5, -1.0, 8.0}, planeSurface), 3.0, 1e-12);
    EXPECT_NEAR(Distance(SCPoint3d{0.6, 1.4, 1.0}, nurbsSurface), 1.0, 5e-2);
    EXPECT_NEAR(
        Length(SCLineSegment3d::FromStartEnd(SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{0.0, 3.0, 4.0})), 5.0, 1e-12);

}
