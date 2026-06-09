#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <numbers>

#include "Geometry.h"
#include "support/GeometryTestSupport.h"

using Geometry::SCArcDirection;
using Geometry::SCArcSegment2d;
using Geometry::Area;
using Geometry::Bounds;
using Geometry::SCBox2d;
using Geometry::SCBox3d;
using Geometry::SCBrepBody;
using Geometry::SCBrepCoedge;
using Geometry::BrepConversionIssue3d;
using Geometry::SCBrepEdge;
using Geometry::SCBrepFace;
using Geometry::BrepHealing3d;
using Geometry::SCBrepLoop;
using Geometry::SCBrepShell;
using Geometry::SCBrepValidationIssue3d;
using Geometry::SCBrepVertex;
using Geometry::BuildSectionComponents;
using Geometry::BuildSectionTopology;
using Geometry::ClassifySectionContent;
using Geometry::ClosePlanarBoundaryLoops;
using Geometry::CloseSinglePlanarBoundaryLoop;
using Geometry::ComputeMeshShells;
using Geometry::ComputeTriangleAdjacency;
using Geometry::ComputeTriangleConnectedComponents;
using Geometry::ComputeTriangleNormals;
using Geometry::ComputeVertexNormals;
using Geometry::Contains;
using Geometry::ConvertSectionToTriangleMesh;
using Geometry::ConvertSectionToTriangleMeshes;
using Geometry::ConvertToPolyhedronBody;
using Geometry::ConvertToPolyhedronFace;
using Geometry::ConvertToTriangleMesh;
using Geometry::ISCCurve3d;
using Geometry::SCCurveOnSurface;
using Geometry::Distance;
using Geometry::ExtractBoundaryEdges;
using Geometry::ExtractBoundaryLoops;
using Geometry::ExtractNonManifoldEdges;
using Geometry::SCGeometryTolerance3d;
using Geometry::Heal;
using Geometry::Intersects;
using Geometry::SCIntervald;
using Geometry::IsClosedTriangleMesh;
using Geometry::IsConsistentlyOrientedTriangleMesh;
using Geometry::IsManifoldTriangleMesh;
using Geometry::SCLine3d;
using Geometry::SCLineCurve3d;
using Geometry::SCLineSegment2d;
using Geometry::LocatePoint;
using Geometry::MeshBoundaryEdge3d;
using Geometry::MeshBoundaryLoop3d;
using Geometry::MeshConversionIssue3d;
using Geometry::MeshHealing3d;
using Geometry::MeshNonManifoldEdge3d;
using Geometry::MeshRepairIssue3d;
using Geometry::MeshShell3d;
using Geometry::MeshTriangleAdjacency3d;
using Geometry::SCMeshValidationIssue3d;
using Geometry::SCNurbsCurve3d;
using Geometry::SCNurbsSurface;
using Geometry::SCOffsetSurface;
using Geometry::OrientTriangleMeshConsistently;
using Geometry::SCPlane;
using Geometry::SCPlaneSurface;
using Geometry::SCPoint2d;
using Geometry::SCPoint3d;
using Geometry::SCPointContainment2d;
using Geometry::SCPolygon2d;
using Geometry::PolyhedronBody;
using Geometry::PolyhedronFace3d;
using Geometry::PolyhedronHealing3d;
using Geometry::PolyhedronLoop3d;
using Geometry::SCPolyhedronSection3d;
using Geometry::SCPolyhedronValidationIssue3d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;
using Geometry::ProjectFaceToPolygon2d;
using Geometry::ProjectPointToBrepBody;
using Geometry::ProjectPointToBrepFace;
using Geometry::ProjectPointToPolyhedronBody;
using Geometry::ProjectPointToPolyhedronFace;
using Geometry::ProjectPointToSegment;
using Geometry::ProjectPointToSurface;
using Geometry::RebuildSectionBodies;
using Geometry::RebuildSectionBody;
using Geometry::RebuildSectionBrepBodies;
using Geometry::RebuildSectionBrepBody;
using Geometry::RebuildSectionBrepFaces;
using Geometry::RebuildSectionFaces;
using Geometry::SCRuledSurface;
using Geometry::Section;
using Geometry::SCSectionBodyRebuild3d;
using Geometry::SCSectionBodyRebuildIssue3d;
using Geometry::SCSectionBodySetRebuild3d;
using Geometry::SCSectionBrepBodyRebuild3d;
using Geometry::SCSectionBrepBodySetRebuild3d;
using Geometry::SCSectionBrepFaceRebuild3d;
using Geometry::SCSectionComponents3d;
using Geometry::SCSectionContentKind3d;
using Geometry::SCSectionFaceRebuild3d;
using Geometry::SCSectionFaceRebuildIssue3d;
using Geometry::SCSectionIssue3d;
using Geometry::SCSectionMeshConversion3d;
using Geometry::SCSectionMeshSetConversion3d;
using Geometry::SCSectionTopology3d;
using Geometry::SCSectionValidation3d;
using Geometry::SCSectionValidationIssue3d;
using Geometry::ISCSegment2d;
using Geometry::SCSegmentKind2;
using Geometry::SCSegmentProjection2d;
using Geometry::ISCSurface;
using Geometry::Tessellate;
using Geometry::SCTriangle3d;
using Geometry::TriangleAdjacency;
using Geometry::TriangleMesh;
using Geometry::TriangleMeshRepair3d;
using Geometry::TriangleNormal;
using Geometry::Validate;
using Geometry::SCVector2d;
using Geometry::SCVector3d;
using Geometry::VertexNormal;

using namespace Geometry;

TEST(GeometryTest, CoversCurrentCapabilities)
{
    const SCPoint2d a = SCPoint2d::FromXY(0.0, 0.0);
    const SCPoint2d b = SCPoint2d::FromXY(3.0, 4.0);
    const SCVector2d offset = b - a;

    GEOMETRY_TEST_ASSERT_NEAR(Distance(a, b), 5.0, 1e-12);
    ASSERT_EQ(offset, SCVector2d(SCVector2d{3.0, 4.0}));
    GEOMETRY_TEST_ASSERT_NEAR(offset.LengthSquared(), 25.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(offset.Length(), 5.0, 1e-12);
    ASSERT_EQ(a + offset, b);
    ASSERT_EQ(b - offset, a);

    const SCLineSegment2d line = SCLineSegment2d::FromEndpoints(a, b);
    ASSERT_TRUE(line.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(line.Length(), 5.0, 1e-12);
    const SCPoint2d lineMidpoint{1.5, 2.0};
    GEOMETRY_TEST_ASSERT_POINT_NEAR(line.PointAt(0.5), lineMidpoint, 1e-12);
    GEOMETRY_TEST_ASSERT_BOX_NEAR(line.Bounds(), SCBox2d::FromMinMax(SCPoint2d{0.0, 0.0}, SCPoint2d{3.0, 4.0}), 1e-12);
    ASSERT_EQ(line.DebugString().find("SCLineSegment2d{start="), 0);
    ASSERT_EQ(line.Kind(), SCSegmentKind2::Line);

    const SCArcSegment2d arc =
        SCArcSegment2d::FromCenterRadiusStartSweep(SCPoint2d{0.0, 0.0}, 1.0, 0.0, std::numbers::pi_v<double> * 0.5);
    ASSERT_TRUE(arc.IsValid());
    ASSERT_EQ(arc.Direction(), SCArcDirection::CounterClockwise);
    GEOMETRY_TEST_ASSERT_NEAR(arc.Length(), std::numbers::pi_v<double> * 0.5, 1e-12);
    const SCPoint2d arcStart{1.0, 0.0};
    const SCPoint2d arcEnd{0.0, 1.0};
    GEOMETRY_TEST_ASSERT_POINT_NEAR(arc.StartPoint(), arcStart, 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(arc.EndPoint(), arcEnd, 1e-12);
    GEOMETRY_TEST_ASSERT_BOX_NEAR(arc.Bounds(), SCBox2d::FromMinMax(SCPoint2d{0.0, 0.0}, SCPoint2d{1.0, 1.0}), 1e-12);
    ASSERT_EQ(arc.DebugString().find("SCArcSegment2d{center="), 0);
    ASSERT_EQ(arc.Kind(), SCSegmentKind2::Arc);

    std::unique_ptr<ISCSegment2d> lineSegment = line.Clone();
    std::unique_ptr<ISCSegment2d> arcSegment = arc.Clone();
    ASSERT_EQ(lineSegment->Kind(), SCSegmentKind2::Line);
    ASSERT_EQ(arcSegment->Kind(), SCSegmentKind2::Arc);
    GEOMETRY_TEST_ASSERT_NEAR(lineSegment->Length(), 5.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(arcSegment->Length(), std::numbers::pi_v<double> * 0.5, 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(lineSegment->StartPoint(), a, 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(arcSegment->EndPoint(), arcEnd, 1e-12);

    const auto projection = ProjectPointToSegment(SCPoint2d{3.0, 1.0}, a, b);
    const SCSegmentProjection2d expectedProjection{SCPoint2d{1.56, 2.08}, 0.52, 3.24, true};
    GEOMETRY_TEST_ASSERT_PROJECTION_NEAR(projection, expectedProjection, 1e-12);

    const SCBox2d boxA = SCBox2d::FromMinMax(SCPoint2d{0.0, 0.0}, SCPoint2d{2.0, 2.0});
    const SCBox2d boxB = SCBox2d::FromMinMax(SCPoint2d{1.0, 1.0}, SCPoint2d{3.0, 3.0});
    const SCBox2d boxC = SCBox2d::FromMinMax(SCPoint2d{3.1, 3.1}, SCPoint2d{4.0, 4.0});
    const SCBox2d invalidBox = SCBox2d::FromMinMax(SCPoint2d{2.0, 2.0}, SCPoint2d{1.0, 1.0});
    const SCBox2d expectedBox = SCBox2d::FromMinMax(SCPoint2d{0.0, 0.0}, SCPoint2d{2.0, 2.0});
    const SCPoint2d expectedCenter{1.0, 1.0};

    ASSERT_TRUE(boxA.IsValid());
    GEOMETRY_TEST_ASSERT_BOX_NEAR(boxA, expectedBox, 1e-12);
    ASSERT_FALSE(invalidBox.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(boxA.Width(), 2.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(boxA.Height(), 2.0, 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(boxA.Center(), expectedCenter, 1e-12);
    ASSERT_TRUE(Contains(boxA, SCPoint2d{1.0, 1.0}));
    ASSERT_TRUE(Intersects(boxA, boxB));
    ASSERT_FALSE(Intersects(boxA, boxC));
    ASSERT_FALSE(Contains(invalidBox, SCPoint2d{1.0, 1.0}));

    const SCPolyline2d openPath({SCPoint2d{0.0, 0.0}, SCPoint2d{3.0, 0.0}, SCPoint2d{3.0, 4.0}}, SCPolylineClosure::Open);
    ASSERT_TRUE(openPath.IsValid());
    ASSERT_FALSE(openPath.IsClosed());
    ASSERT_EQ(openPath.PointCount(), 3);
    ASSERT_EQ(openPath.SegmentCount(), 2);
    const SCPoint2d openMidPoint{3.0, 0.0};
    GEOMETRY_TEST_ASSERT_POINT_NEAR(openPath.PointAt(1), openMidPoint, 1e-12);
    GEOMETRY_TEST_ASSERT_BOX_NEAR(openPath.Bounds(), SCBox2d::FromMinMax(SCPoint2d{0.0, 0.0}, SCPoint2d{3.0, 4.0}), 1e-12);
    ASSERT_EQ(openPath.DebugString().find("SCPolyline2d{closure=Open"), 0);

    const SCPolyline2d outerRing({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}},
                               SCPolylineClosure::Closed);
    const SCPolyline2d holeRing({SCPoint2d{1.0, 1.0}, SCPoint2d{1.0, 3.0}, SCPoint2d{3.0, 3.0}, SCPoint2d{3.0, 1.0}},
                              SCPolylineClosure::Closed);
    const SCPolygon2d polygon(outerRing, {holeRing});
    ASSERT_TRUE(polygon.IsValid());
    ASSERT_EQ(polygon.HoleCount(), 1);
    ASSERT_EQ(polygon.PointCount(), 8);
    ASSERT_EQ(polygon.SegmentCount(), 8);
    GEOMETRY_TEST_ASSERT_POLYLINE_NEAR(polygon.OuterRing(), outerRing, 1e-12);
    GEOMETRY_TEST_ASSERT_POLYLINE_NEAR(polygon.HoleAt(0), holeRing, 1e-12);
    GEOMETRY_TEST_ASSERT_BOX_NEAR(polygon.Bounds(), SCBox2d::FromMinMax(SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 4.0}), 1e-12);
    ASSERT_EQ(polygon.DebugString().find("SCPolygon2d{holeCount=1"), 0);

    const SCLine3d line3 = SCLine3d::FromOriginAndDirection(SCPoint3d{1.0, 2.0, 3.0}, SCVector3d{2.0, 0.0, 0.0});
    const SCLineCurve3d lineCurve = SCLineCurve3d::FromLine(line3, SCIntervald{-2.0, 3.0});
    ASSERT_TRUE(lineCurve.IsValid());
    ASSERT_FALSE(lineCurve.IsClosed());
    ASSERT_FALSE(lineCurve.IsPeriodic());
    GEOMETRY_TEST_ASSERT_NEAR(lineCurve.StartParameter(), -2.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(lineCurve.EndParameter(), 3.0, 1e-12);
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

    const SCPlane supportPlane = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 5.0}, SCVector3d{0.0, 0.0, 1.0});
    const SCPlaneSurface planeSurface = SCPlaneSurface::FromPlane(supportPlane, SCIntervald{-2.0, 2.0}, SCIntervald{-3.0, 1.0});
    ASSERT_TRUE(planeSurface.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(planeSurface.StartU(), -2.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(planeSurface.EndU(), 2.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(planeSurface.StartV(), -3.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(planeSurface.EndV(), 1.0, 1e-12);
    const SCPoint3d planePoint = planeSurface.PointAt(0.5, -1.0);
    GEOMETRY_TEST_ASSERT_NEAR(supportPlane.SignedDistanceTo(planePoint), 0.0, 1e-12);
    const auto surfaceEval = planeSurface.Evaluate(0.25, 0.75, 1);
    ASSERT_TRUE(surfaceEval.IsValid());
    ASSERT_TRUE(surfaceEval.point.AlmostEquals(planeSurface.PointAt(0.25, 0.75), 1e-12));
    GEOMETRY_TEST_ASSERT_NEAR(Geometry::Dot(surfaceEval.derivativeU, surfaceEval.normal), 0.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(Geometry::Dot(surfaceEval.derivativeV, surfaceEval.normal), 0.0, 1e-12);
    const SCBox3d surfaceBounds = planeSurface.Bounds();
    ASSERT_TRUE(surfaceBounds.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(surfaceBounds.MinPoint().z, 5.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(surfaceBounds.MaxPoint().z, 5.0, 1e-12);
    std::unique_ptr<ISCSurface> surfaceClone = planeSurface.Clone();
    ASSERT_NE(surfaceClone, nullptr);
    ASSERT_TRUE(surfaceClone->IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(supportPlane.SignedDistanceTo(surfaceClone->PointAt(-2.0, -3.0)), 0.0, 1e-12);

    const SCNurbsCurve3d nurbsCurve(1,
                                  {
                                      SCPoint3d{0.0, 0.0, 0.0},
                                      SCPoint3d{1.0, 1.0, 0.0},
                                      SCPoint3d{2.0, 0.0, 0.0},
                                  },
                                  {0.0, 0.0, 0.5, 1.0, 1.0});
    ASSERT_TRUE(nurbsCurve.IsValid());
    ASSERT_FALSE(nurbsCurve.IsPeriodic());
    GEOMETRY_TEST_ASSERT_NEAR(nurbsCurve.StartParameter(), 0.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(nurbsCurve.EndParameter(), 1.0, 1e-12);
    ASSERT_TRUE(nurbsCurve.PointAt(0.5).AlmostEquals(SCPoint3d{1.0, 1.0, 0.0}, 1e-12));
    const auto nurbsCurveEval = nurbsCurve.Evaluate(0.25, 2);
    ASSERT_TRUE(nurbsCurveEval.IsValid());
    ASSERT_EQ(nurbsCurveEval.derivativeOrder, 2);
    const auto nurbsCurveProjection = ProjectPointToCurve(SCPoint3d{1.0, 0.0, 1.0}, nurbsCurve);
    ASSERT_TRUE(nurbsCurveProjection.success);
    ASSERT_TRUE(nurbsCurveProjection.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(nurbsCurveProjection.point.z, 0.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(Length(nurbsCurve, 64), std::sqrt(2.0) * 2.0, 5e-2);
    const SCBox3d nurbsCurveBounds = nurbsCurve.Bounds();
    ASSERT_TRUE(nurbsCurveBounds.IsValid());
    ASSERT_TRUE(nurbsCurveBounds.MinPoint().AlmostEquals(SCPoint3d{0.0, 0.0, 0.0}, 1e-12));
    ASSERT_TRUE(nurbsCurveBounds.MaxPoint().AlmostEquals(SCPoint3d{2.0, 1.0, 0.0}, 1e-12));
    std::unique_ptr<ISCCurve3d> nurbsCurveClone = nurbsCurve.Clone();
    ASSERT_NE(nurbsCurveClone, nullptr);
    ASSERT_TRUE(nurbsCurveClone->IsValid());
    ASSERT_TRUE(nurbsCurveClone->PointAt(0.5).AlmostEquals(SCPoint3d{1.0, 1.0, 0.0}, 1e-12));

    const SCNurbsSurface nurbsSurface(1,
                                    1,
                                    2,
                                    2,
                                    {
                                        SCPoint3d{0.0, 0.0, 0.0},
                                        SCPoint3d{2.0, 0.0, 0.0},
                                        SCPoint3d{0.0, 2.0, 0.0},
                                        SCPoint3d{2.0, 2.0, 0.0},
                                    },
                                    {0.0, 0.0, 1.0, 1.0},
                                    {0.0, 0.0, 1.0, 1.0});
    ASSERT_TRUE(nurbsSurface.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(nurbsSurface.StartU(), 0.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(nurbsSurface.EndU(), 1.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(nurbsSurface.StartV(), 0.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(nurbsSurface.EndV(), 1.0, 1e-12);
    ASSERT_TRUE(nurbsSurface.PointAt(0.5, 0.5).AlmostEquals(SCPoint3d{1.0, 1.0, 0.0}, 1e-12));
    const auto nurbsSurfaceEval = nurbsSurface.Evaluate(0.5, 0.5, 1);
    ASSERT_TRUE(nurbsSurfaceEval.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(nurbsSurfaceEval.normal.Normalized().z, 1.0, 1e-12);
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
    GEOMETRY_TEST_ASSERT_NEAR(lineCurveProjection.parameter, 0.25, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(Distance(SCPoint3d{0.5, 1.0, 0.0}, lowerRail), 1.0, 1e-12);
    const auto lineCurveIntersection =
        Intersect(SCLine3d::FromOriginAndDirection(SCPoint3d{0.5, 1.0, 0.0}, SCVector3d{0.0, -1.0, 0.0}), lowerRail);
    ASSERT_TRUE(lineCurveIntersection.intersects);
    ASSERT_TRUE(lineCurveIntersection.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(lineCurveIntersection.curveParameter, 0.25, 1e-12);
    const auto planeCurveIntersection =
        Intersect(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}), lowerRail);
    ASSERT_TRUE(planeCurveIntersection.intersects);
    ASSERT_TRUE(planeCurveIntersection.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(planeCurveIntersection.curveParameter, 0.0, 1e-12);
    ASSERT_TRUE(planeCurveIntersection.point.AlmostEquals(SCPoint3d{0.0, 0.0, 0.0}, 1e-12));
    ASSERT_EQ(LocatePoint(SCPoint3d{0.5, 0.0, 0.0}, lowerRail), SCPointContainment2d::OnBoundary);
    ASSERT_EQ(LocatePoint(SCPoint3d{0.5, 1.0, 0.0}, lowerRail), SCPointContainment2d::Outside);
    const SCRuledSurface ruledSurface = SCRuledSurface::FromCurves(lowerRail, upperRail);
    ASSERT_TRUE(ruledSurface.IsValid());
    ASSERT_NE(ruledSurface.FirstCurve(), nullptr);
    ASSERT_NE(ruledSurface.SecondCurve(), nullptr);
    ASSERT_TRUE(ruledSurface.PointAt(0.5, 0.5).AlmostEquals(SCPoint3d{1.0, 0.0, 1.0}, 1e-12));
    const auto ruledEval = ruledSurface.Evaluate(0.25, 0.25, 1);
    ASSERT_TRUE(ruledEval.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(ruledEval.normal.Length(), 4.0, 1e-12);

    const SCOffsetSurface offsetSurface = SCOffsetSurface::FromSurface(planeSurface, 2.0);
    ASSERT_TRUE(offsetSurface.IsValid());
    ASSERT_NE(offsetSurface.BaseSurface(), nullptr);
    GEOMETRY_TEST_ASSERT_NEAR(offsetSurface.OffsetDistance(), 2.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(offsetSurface.PointAt(0.0, 0.0).z, 7.0, 1e-12);
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
    GEOMETRY_TEST_ASSERT_NEAR(curveOnSurfaceBounds.MinPoint().z, 5.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(curveOnSurfaceBounds.MaxPoint().z, 5.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(Bounds(curveOnSurface).MinPoint().z, 5.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(Length(curveOnSurface), std::sqrt(8.0) * 2.0, 1e-12);
    const auto curveOnSurfaceProjection = ProjectPointToCurveOnSurface(SCPoint3d{0.0, -1.0, 6.0}, curveOnSurface);
    ASSERT_TRUE(curveOnSurfaceProjection.success);
    ASSERT_TRUE(curveOnSurfaceProjection.IsValid());
    ASSERT_TRUE(curveOnSurfaceProjection.point.AlmostEquals(SCPoint3d{0.0, -1.0, 5.0}, 1e-12));
    GEOMETRY_TEST_ASSERT_NEAR(Distance(SCPoint3d{0.0, -1.0, 6.0}, curveOnSurface), 1.0, 1e-12);
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
    ASSERT_EQ(LocatePoint(SCPoint3d{0.0, -1.0, 5.0}, curveOnSurface), SCPointContainment2d::OnBoundary);
    ASSERT_EQ(LocatePoint(SCPoint3d{0.0, -1.0, 6.0}, curveOnSurface), SCPointContainment2d::Outside);

    const auto planeSurfaceProjection = ProjectPointToSurface(SCPoint3d{0.5, -1.0, 8.0}, planeSurface);
    ASSERT_TRUE(planeSurfaceProjection.success);
    ASSERT_TRUE(planeSurfaceProjection.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(planeSurfaceProjection.u, 0.5, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(planeSurfaceProjection.v, -1.0, 1e-12);
    ASSERT_TRUE(planeSurfaceProjection.point.AlmostEquals(SCPoint3d{0.5, -1.0, 5.0}, 1e-12));

    const auto nurbsSurfaceProjection = ProjectPointToSurface(SCPoint3d{0.6, 1.4, 1.0}, nurbsSurface);
    ASSERT_TRUE(nurbsSurfaceProjection.success);
    ASSERT_TRUE(nurbsSurfaceProjection.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(nurbsSurfaceProjection.u, 0.3, 5e-2);
    GEOMETRY_TEST_ASSERT_NEAR(nurbsSurfaceProjection.v, 0.7, 5e-2);
    GEOMETRY_TEST_ASSERT_NEAR(nurbsSurfaceProjection.point.z, 0.0, 1e-12);

    const auto planeSurfaceIntersection =
        Intersect(SCLine3d::FromOriginAndDirection(SCPoint3d{0.5, -1.0, 8.0}, SCVector3d{0.0, 0.0, -1.0}), planeSurface);
    ASSERT_TRUE(planeSurfaceIntersection.intersects);
    ASSERT_TRUE(planeSurfaceIntersection.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(planeSurfaceIntersection.lineParameter, 3.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(planeSurfaceIntersection.u, 0.5, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(planeSurfaceIntersection.v, -1.0, 1e-12);
    ASSERT_TRUE(planeSurfaceIntersection.point.AlmostEquals(SCPoint3d{0.5, -1.0, 5.0}, 1e-12));

    const auto nurbsSurfaceIntersection =
        Intersect(SCLine3d::FromOriginAndDirection(SCPoint3d{0.6, 1.4, 1.0}, SCVector3d{0.0, 0.0, -1.0}), nurbsSurface);
    ASSERT_TRUE(nurbsSurfaceIntersection.intersects);
    ASSERT_TRUE(nurbsSurfaceIntersection.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(nurbsSurfaceIntersection.u, 0.3, 5e-2);
    GEOMETRY_TEST_ASSERT_NEAR(nurbsSurfaceIntersection.v, 0.7, 5e-2);
    GEOMETRY_TEST_ASSERT_NEAR(nurbsSurfaceIntersection.point.z, 0.0, 1e-12);

    GEOMETRY_TEST_ASSERT_NEAR(Distance(SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{1.0, 2.0, 2.0}), 3.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(Distance(SCPoint3d{0.0, 2.0, 0.0},
                                       SCLine3d::FromOriginAndDirection(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, 0.0})),
                              2.0,
                              1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(Distance(SCPoint3d{0.0, 0.0, 8.0}, supportPlane), 3.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(Distance(SCPoint3d{0.5, -1.0, 8.0}, planeSurface), 3.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(Distance(SCPoint3d{0.6, 1.4, 1.0}, nurbsSurface), 1.0, 5e-2);
    GEOMETRY_TEST_ASSERT_NEAR(
        Length(SCLineSegment3d::FromStartEnd(SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{0.0, 3.0, 4.0})), 5.0, 1e-12);

    const TriangleMesh mesh(
        {
            SCPoint3d{0.0, 0.0, 0.0},
            SCPoint3d{1.0, 0.0, 0.0},
            SCPoint3d{0.0, 1.0, 0.0},
            SCPoint3d{0.0, 0.0, 1.0},
        },
        {
            TriangleMesh::TriangleIndices{0, 1, 2},
            TriangleMesh::TriangleIndices{0, 1, 3},
        });
    ASSERT_FALSE(mesh.IsEmpty());
    ASSERT_TRUE(mesh.IsValid());
    ASSERT_EQ(mesh.VertexCount(), 4);
    ASSERT_EQ(mesh.TriangleCount(), 2);
    ASSERT_TRUE(mesh.VertexAt(3).AlmostEquals(SCPoint3d{0.0, 0.0, 1.0}, 1e-12));
    const SCTriangle3d firstTriangle = mesh.TriangleAt(0);
    ASSERT_TRUE(firstTriangle.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(firstTriangle.Area(), 0.5, 1e-12);
    const SCBox3d meshBounds = mesh.Bounds();
    ASSERT_TRUE(meshBounds.IsValid());
    ASSERT_TRUE(meshBounds.MinPoint().AlmostEquals(SCPoint3d{0.0, 0.0, 0.0}, 1e-12));
    ASSERT_TRUE(meshBounds.MaxPoint().AlmostEquals(SCPoint3d{1.0, 1.0, 1.0}, 1e-12));
    GEOMETRY_TEST_ASSERT_NEAR(mesh.SurfaceArea(), 1.0, 1e-12);
    const auto meshValidation = Validate(mesh);
    ASSERT_TRUE(meshValidation.valid);
    ASSERT_EQ(meshValidation.issue, SCMeshValidationIssue3d::None);
    const auto meshProjection = ProjectPointToTriangleMesh(SCPoint3d{1.0, 1.0, 3.0}, mesh);
    ASSERT_TRUE(meshProjection.success);
    ASSERT_TRUE(meshProjection.IsValid());
    ASSERT_TRUE(meshProjection.point.AlmostEquals(SCPoint3d{0.0, 0.0, 1.0}, 1e-12));
    GEOMETRY_TEST_ASSERT_NEAR(Distance(SCPoint3d{1.0, 1.0, 3.0}, mesh), std::sqrt(6.0), 1e-12);
    ASSERT_EQ(LocatePoint(SCPoint3d{1.0, 1.0, 0.0}, mesh), SCPointContainment2d::Outside);
    const auto meshLineIntersection =
        Intersect(SCLine3d::FromOriginAndDirection(SCPoint3d{1.0, 1.0, 3.0}, SCVector3d{0.0, 0.0, -1.0}), mesh);
    ASSERT_FALSE(meshLineIntersection.intersects);
    ASSERT_TRUE(meshLineIntersection.IsValid());
    ASSERT_TRUE(meshLineIntersection.triangleIndices.empty());
    ASSERT_TRUE(meshLineIntersection.points.empty());
    const SCVector3d firstTriangleNormal = TriangleNormal(mesh, 0);
    ASSERT_TRUE(firstTriangleNormal.AlmostEquals(SCVector3d{0.0, 0.0, 1.0}, 1e-12));
    const auto triangleNormals = ComputeTriangleNormals(mesh);
    ASSERT_EQ(triangleNormals.size(), mesh.TriangleCount());
    ASSERT_TRUE(triangleNormals[1].AlmostEquals(SCVector3d{0.0, -1.0, 0.0}, 1e-12));
    const SCVector3d sharedVertexNormal = VertexNormal(mesh, 0);
    GEOMETRY_TEST_ASSERT_NEAR(sharedVertexNormal.Length(), 1.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(sharedVertexNormal.x, 0.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(sharedVertexNormal.y, -std::sqrt(0.5), 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(sharedVertexNormal.z, std::sqrt(0.5), 1e-12);
    const auto vertexNormals = ComputeVertexNormals(mesh);
    ASSERT_EQ(vertexNormals.size(), mesh.VertexCount());
    ASSERT_TRUE(vertexNormals[2].AlmostEquals(SCVector3d{0.0, 0.0, 1.0}, 1e-12));
    const MeshTriangleAdjacency3d firstAdjacency = TriangleAdjacency(mesh, 0);
    ASSERT_TRUE(firstAdjacency.HasNeighbor(0));
    ASSERT_EQ(firstAdjacency.adjacentTriangles[0], 1);
    ASSERT_FALSE(firstAdjacency.HasNeighbor(1));
    ASSERT_FALSE(firstAdjacency.HasNeighbor(2));
    const auto adjacency = ComputeTriangleAdjacency(mesh);
    ASSERT_EQ(adjacency.size(), mesh.TriangleCount());
    ASSERT_TRUE(adjacency[1].HasNeighbor(0));
    ASSERT_EQ(adjacency[1].adjacentTriangles[0], 0);
    const std::vector<MeshBoundaryEdge3d> boundaryEdges = ExtractBoundaryEdges(mesh);
    ASSERT_EQ(boundaryEdges.size(), 4);
    ASSERT_TRUE(boundaryEdges[0].IsValid());
    const std::vector<MeshBoundaryLoop3d> boundaryLoops = ExtractBoundaryLoops(mesh);
    ASSERT_EQ(boundaryLoops.size(), 2);
    for (const MeshBoundaryLoop3d& loop : boundaryLoops)
    {
        ASSERT_TRUE(loop.IsValid());
        ASSERT_FALSE(loop.closed);
        ASSERT_EQ(loop.vertexIndices.size(), 3);
    }
    ASSERT_FALSE(IsClosedTriangleMesh(mesh));
    ASSERT_TRUE(IsManifoldTriangleMesh(mesh));
    ASSERT_FALSE(IsConsistentlyOrientedTriangleMesh(mesh));
    const TriangleMeshRepair3d repairedOpenMesh = OrientTriangleMeshConsistently(mesh);
    ASSERT_TRUE(repairedOpenMesh.success);
    ASSERT_EQ(repairedOpenMesh.issue, MeshRepairIssue3d::None);
    ASSERT_TRUE(repairedOpenMesh.mesh.IsValid());
    ASSERT_TRUE(IsConsistentlyOrientedTriangleMesh(repairedOpenMesh.mesh));
    ASSERT_TRUE(IsManifoldTriangleMesh(repairedOpenMesh.mesh));
    GEOMETRY_TEST_ASSERT_NEAR(repairedOpenMesh.mesh.SurfaceArea(), mesh.SurfaceArea(), 1e-12);
    const TriangleMeshRepair3d closedNonPlanarMesh = CloseSinglePlanarBoundaryLoop(mesh);
    ASSERT_FALSE(closedNonPlanarMesh.success);
    ASSERT_EQ(closedNonPlanarMesh.issue, MeshRepairIssue3d::UnsupportedBoundaryTopology);
    const auto meshComponents = ComputeTriangleConnectedComponents(mesh);
    ASSERT_EQ(meshComponents.size(), 1);
    ASSERT_EQ(meshComponents[0].size(), 2);
    const std::vector<MeshShell3d> openShells = ComputeMeshShells(mesh);
    ASSERT_EQ(openShells.size(), 1);
    ASSERT_TRUE(openShells[0].IsValid());
    ASSERT_FALSE(openShells[0].closed);
    ASSERT_TRUE(openShells[0].manifold);
    ASSERT_FALSE(openShells[0].consistentlyOriented);
    const TriangleMesh tetraMesh(
        {
            SCPoint3d{0.0, 0.0, 0.0},
            SCPoint3d{1.0, 0.0, 0.0},
            SCPoint3d{0.0, 1.0, 0.0},
            SCPoint3d{0.0, 0.0, 1.0},
        },
        {
            TriangleMesh::TriangleIndices{0, 2, 1},
            TriangleMesh::TriangleIndices{0, 1, 3},
            TriangleMesh::TriangleIndices{1, 2, 3},
            TriangleMesh::TriangleIndices{2, 0, 3},
        });
    ASSERT_TRUE(tetraMesh.IsValid());
    ASSERT_TRUE(ExtractBoundaryEdges(tetraMesh).empty());
    ASSERT_TRUE(ExtractBoundaryLoops(tetraMesh).empty());
    ASSERT_TRUE(IsClosedTriangleMesh(tetraMesh));
    ASSERT_TRUE(IsManifoldTriangleMesh(tetraMesh));
    ASSERT_TRUE(IsConsistentlyOrientedTriangleMesh(tetraMesh));
    GEOMETRY_TEST_ASSERT_NEAR(Volume(tetraMesh), 1.0 / 6.0, 1e-12);
    ASSERT_EQ(LocatePoint(SCPoint3d{0.1, 0.1, 0.1}, tetraMesh), SCPointContainment2d::Inside);
    ASSERT_EQ(LocatePoint(SCPoint3d{1.0, 1.0, 1.0}, tetraMesh), SCPointContainment2d::Outside);
    ASSERT_EQ(LocatePoint(SCPoint3d{0.0, 0.2, 0.2}, tetraMesh), SCPointContainment2d::OnBoundary);
    const TriangleMeshRepair3d repairedTetraMesh = OrientTriangleMeshConsistently(tetraMesh);
    ASSERT_TRUE(repairedTetraMesh.success);
    ASSERT_EQ(repairedTetraMesh.issue, MeshRepairIssue3d::None);
    ASSERT_TRUE(repairedTetraMesh.mesh.IsValid());
    ASSERT_TRUE(IsConsistentlyOrientedTriangleMesh(repairedTetraMesh.mesh));
    const std::vector<MeshShell3d> tetraShells = ComputeMeshShells(tetraMesh);
    ASSERT_EQ(tetraShells.size(), 1);
    ASSERT_TRUE(tetraShells[0].closed);
    ASSERT_TRUE(tetraShells[0].manifold);
    ASSERT_TRUE(tetraShells[0].consistentlyOriented);
    const TriangleMesh openTetraMesh(
        {
            SCPoint3d{0.0, 0.0, 0.0},
            SCPoint3d{1.0, 0.0, 0.0},
            SCPoint3d{0.0, 1.0, 0.0},
            SCPoint3d{0.0, 0.0, 1.0},
        },
        {
            TriangleMesh::TriangleIndices{0, 1, 3},
            TriangleMesh::TriangleIndices{1, 2, 3},
            TriangleMesh::TriangleIndices{2, 0, 3},
        });
    ASSERT_TRUE(openTetraMesh.IsValid());
    ASSERT_FALSE(IsClosedTriangleMesh(openTetraMesh));
    const TriangleMeshRepair3d closedOpenTetraMesh = CloseSinglePlanarBoundaryLoop(openTetraMesh);
    ASSERT_TRUE(closedOpenTetraMesh.success);
    ASSERT_EQ(closedOpenTetraMesh.issue, MeshRepairIssue3d::None);
    ASSERT_TRUE(closedOpenTetraMesh.mesh.IsValid());
    ASSERT_TRUE(IsClosedTriangleMesh(closedOpenTetraMesh.mesh));
    ASSERT_TRUE(IsManifoldTriangleMesh(closedOpenTetraMesh.mesh));
    ASSERT_TRUE(IsConsistentlyOrientedTriangleMesh(closedOpenTetraMesh.mesh));
    const MeshHealing3d healedOpenTetra = Heal(openTetraMesh);
    ASSERT_TRUE(healedOpenTetra.success);
    ASSERT_EQ(healedOpenTetra.issue, HealingIssue3d::None);
    ASSERT_TRUE(healedOpenTetra.mesh.IsValid());
    ASSERT_TRUE(IsClosedTriangleMesh(healedOpenTetra.mesh));
    const TriangleMesh disconnectedMesh(
        {
            SCPoint3d{0.0, 0.0, 0.0},
            SCPoint3d{1.0, 0.0, 0.0},
            SCPoint3d{0.0, 1.0, 0.0},
            SCPoint3d{3.0, 0.0, 0.0},
            SCPoint3d{4.0, 0.0, 0.0},
            SCPoint3d{3.0, 1.0, 0.0},
        },
        {
            TriangleMesh::TriangleIndices{0, 1, 2},
            TriangleMesh::TriangleIndices{3, 4, 5},
        });
    ASSERT_TRUE(disconnectedMesh.IsValid());
    const auto disconnectedComponents = ComputeTriangleConnectedComponents(disconnectedMesh);
    ASSERT_EQ(disconnectedComponents.size(), 2);
    ASSERT_EQ(disconnectedComponents[0].size(), 1);
    ASSERT_EQ(disconnectedComponents[1].size(), 1);
    const TriangleMeshRepair3d closedDisconnectedMesh = ClosePlanarBoundaryLoops(disconnectedMesh);
    ASSERT_TRUE(closedDisconnectedMesh.success);
    ASSERT_EQ(closedDisconnectedMesh.issue, MeshRepairIssue3d::None);
    ASSERT_TRUE(closedDisconnectedMesh.mesh.IsValid());
    ASSERT_TRUE(IsClosedTriangleMesh(closedDisconnectedMesh.mesh));
    ASSERT_TRUE(IsManifoldTriangleMesh(closedDisconnectedMesh.mesh));
    ASSERT_TRUE(IsConsistentlyOrientedTriangleMesh(closedDisconnectedMesh.mesh));
    const std::vector<MeshBoundaryLoop3d> disconnectedBoundaryLoops = ExtractBoundaryLoops(disconnectedMesh);
    ASSERT_EQ(disconnectedBoundaryLoops.size(), 2);
    ASSERT_TRUE(disconnectedBoundaryLoops[0].closed);
    ASSERT_TRUE(disconnectedBoundaryLoops[1].closed);
    const std::vector<MeshShell3d> disconnectedShells = ComputeMeshShells(disconnectedMesh);
    ASSERT_EQ(disconnectedShells.size(), 2);
    ASSERT_FALSE(disconnectedShells[0].closed);
    ASSERT_FALSE(disconnectedShells[1].closed);
    const TriangleMesh nonManifoldMesh(
        {
            SCPoint3d{0.0, 0.0, 0.0},
            SCPoint3d{1.0, 0.0, 0.0},
            SCPoint3d{0.0, 1.0, 0.0},
            SCPoint3d{0.0, 0.0, 1.0},
            SCPoint3d{0.0, -1.0, 0.0},
        },
        {
            TriangleMesh::TriangleIndices{0, 1, 2},
            TriangleMesh::TriangleIndices{0, 3, 1},
            TriangleMesh::TriangleIndices{0, 1, 4},
        });
    ASSERT_TRUE(nonManifoldMesh.IsValid());
    const auto nonManifoldEdges = ExtractNonManifoldEdges(nonManifoldMesh);
    ASSERT_EQ(nonManifoldEdges.size(), 1);
    ASSERT_TRUE(nonManifoldEdges[0].IsValid());
    ASSERT_EQ(nonManifoldEdges[0].incidentTriangles.size(), 3);
    ASSERT_FALSE(IsManifoldTriangleMesh(nonManifoldMesh));
    ASSERT_FALSE(IsConsistentlyOrientedTriangleMesh(nonManifoldMesh));
    const TriangleMeshRepair3d repairedNonManifoldMesh = OrientTriangleMeshConsistently(nonManifoldMesh);
    ASSERT_FALSE(repairedNonManifoldMesh.success);
    ASSERT_EQ(repairedNonManifoldMesh.issue, MeshRepairIssue3d::NonManifold);
    const auto nonManifoldComponents = ComputeTriangleConnectedComponents(nonManifoldMesh);
    ASSERT_EQ(nonManifoldComponents.size(), 1);
    ASSERT_EQ(nonManifoldComponents[0].size(), 3);
    const std::vector<MeshShell3d> nonManifoldShells = ComputeMeshShells(nonManifoldMesh);
    ASSERT_EQ(nonManifoldShells.size(), 1);
    ASSERT_FALSE(nonManifoldShells[0].closed);
    ASSERT_FALSE(nonManifoldShells[0].manifold);
    ASSERT_FALSE(nonManifoldShells[0].consistentlyOriented);
    const TriangleMesh movedMesh = mesh.Transformed(SCTransform3d::Translation(SCVector3d{2.0, -1.0, 3.0}));
    ASSERT_TRUE(movedMesh.IsValid());
    ASSERT_TRUE(movedMesh.VertexAt(0).AlmostEquals(SCPoint3d{2.0, -1.0, 3.0}, 1e-12));
    ASSERT_TRUE(movedMesh.VertexAt(3).AlmostEquals(SCPoint3d{2.0, -1.0, 4.0}, 1e-12));
    GEOMETRY_TEST_ASSERT_NEAR(movedMesh.SurfaceArea(), mesh.SurfaceArea(), 1e-12);

    const TriangleMesh invalidMesh(
        {
            SCPoint3d{0.0, 0.0, 0.0},
            SCPoint3d{1.0, 0.0, 0.0},
            SCPoint3d{1.0, 0.0, 0.0},
        },
        {
            TriangleMesh::TriangleIndices{0, 1, 2},
        });
    const auto invalidMeshValidation = Validate(invalidMesh);
    ASSERT_FALSE(invalidMeshValidation.valid);
    ASSERT_EQ(invalidMeshValidation.issue, SCMeshValidationIssue3d::DegenerateTriangle);

    const PolyhedronLoop3d outerLoop({
        SCPoint3d{0.0, 0.0, 0.0},
        SCPoint3d{2.0, 0.0, 0.0},
        SCPoint3d{2.0, 2.0, 0.0},
        SCPoint3d{0.0, 2.0, 0.0},
    });
    ASSERT_TRUE(outerLoop.IsValid());
    ASSERT_EQ(outerLoop.VertexCount(), 4);
    const PolyhedronFace3d face(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}), outerLoop);
    ASSERT_TRUE(face.IsValid());
    ASSERT_EQ(face.HoleCount(), 0);
    const SCBox3d faceBounds = face.Bounds();
    ASSERT_TRUE(faceBounds.IsValid());
    ASSERT_TRUE(faceBounds.MinPoint().AlmostEquals(SCPoint3d{0.0, 0.0, 0.0}, 1e-12));
    ASSERT_TRUE(faceBounds.MaxPoint().AlmostEquals(SCPoint3d{2.0, 2.0, 0.0}, 1e-12));
    const SCBox3d faceMeasureBounds = Bounds(face);
    ASSERT_TRUE(faceMeasureBounds.IsValid());
    ASSERT_TRUE(faceMeasureBounds.MaxPoint().AlmostEquals(SCPoint3d{2.0, 2.0, 0.0}, 1e-12));
    const PolyhedronBody body({face});
    ASSERT_TRUE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 1);
    const auto bodyValidation = Validate(body);
    ASSERT_TRUE(bodyValidation.valid);
    ASSERT_EQ(bodyValidation.issue, SCPolyhedronValidationIssue3d::None);
    const SCBox3d bodyBounds = body.Bounds();
    ASSERT_TRUE(bodyBounds.IsValid());
    ASSERT_TRUE(bodyBounds.MinPoint().AlmostEquals(SCPoint3d{0.0, 0.0, 0.0}, 1e-12));
    ASSERT_TRUE(bodyBounds.MaxPoint().AlmostEquals(SCPoint3d{2.0, 2.0, 0.0}, 1e-12));
    GEOMETRY_TEST_ASSERT_NEAR(Area(face), 4.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(Area(mesh), 1.0, 1e-12);

    const PolyhedronFace3d invalidFace(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}),
                                       PolyhedronLoop3d({
                                           SCPoint3d{0.0, 0.0, 0.0},
                                           SCPoint3d{1.0, 0.0, 1.0},
                                           SCPoint3d{0.0, 1.0, 0.0},
                                       }));
    const auto invalidBodyValidation = Validate(PolyhedronBody({invalidFace}));
    ASSERT_FALSE(invalidBodyValidation.valid);
    ASSERT_EQ(invalidBodyValidation.issue, SCPolyhedronValidationIssue3d::InvalidFace);

    const PolyhedronHealing3d healedBody = Heal(body);
    ASSERT_TRUE(healedBody.success);
    ASSERT_EQ(healedBody.body.FaceCount(), 1);

    const TriangleMesh surfaceMesh = Tessellate(planeSurface, 2, 3);
    ASSERT_TRUE(surfaceMesh.IsValid());
    ASSERT_EQ(surfaceMesh.VertexCount(), 12);
    ASSERT_EQ(surfaceMesh.TriangleCount(), 12);
    const auto surfaceMeshValidation = Validate(surfaceMesh);
    ASSERT_TRUE(surfaceMeshValidation.valid);
    ASSERT_FALSE(IsClosedTriangleMesh(surfaceMesh));
    for (const SCPoint3d& vertex : surfaceMesh.Vertices())
    {
        GEOMETRY_TEST_ASSERT_NEAR(supportPlane.SignedDistanceTo(vertex), 0.0, 1e-12);
    }
    GEOMETRY_TEST_ASSERT_NEAR(surfaceMesh.SurfaceArea(), 16.0, 1e-12);

    const auto faceMesh = ConvertToTriangleMesh(face);
    ASSERT_TRUE(faceMesh.success);
    ASSERT_EQ(faceMesh.issue, MeshConversionIssue3d::None);
    ASSERT_TRUE(faceMesh.mesh.IsValid());
    ASSERT_EQ(faceMesh.mesh.TriangleCount(), 2);
    GEOMETRY_TEST_ASSERT_NEAR(faceMesh.mesh.SurfaceArea(), 4.0, 1e-12);
    ASSERT_EQ(LocatePoint(SCPoint3d{1.0, 1.0, 0.0}, face), SCPointContainment2d::Inside);
    ASSERT_EQ(LocatePoint(SCPoint3d{2.0, 1.0, 0.0}, face), SCPointContainment2d::OnBoundary);
    ASSERT_EQ(LocatePoint(SCPoint3d{3.0, 1.0, 0.0}, face), SCPointContainment2d::Outside);
    const auto polyFaceProjection = ProjectPointToPolyhedronFace(SCPoint3d{1.0, 1.0, 3.0}, face);
    ASSERT_TRUE(polyFaceProjection.success);
    ASSERT_TRUE(polyFaceProjection.IsValid());
    ASSERT_TRUE(polyFaceProjection.onFace);
    ASSERT_TRUE(polyFaceProjection.point.AlmostEquals(SCPoint3d{1.0, 1.0, 0.0}, 1e-12));
    GEOMETRY_TEST_ASSERT_NEAR(Distance(SCPoint3d{1.0, 1.0, 3.0}, face), 3.0, 1e-12);
    const auto polyFaceLineIntersection =
        Intersect(SCLine3d::FromOriginAndDirection(SCPoint3d{1.0, 1.0, 3.0}, SCVector3d{0.0, 0.0, -1.0}), face);
    ASSERT_TRUE(polyFaceLineIntersection.intersects);
    ASSERT_TRUE(polyFaceLineIntersection.IsValid());
    ASSERT_TRUE(polyFaceLineIntersection.point.AlmostEquals(SCPoint3d{1.0, 1.0, 0.0}, 1e-12));

    const auto bodyMesh = ConvertToTriangleMesh(body);
    ASSERT_TRUE(bodyMesh.success);
    ASSERT_EQ(bodyMesh.issue, MeshConversionIssue3d::None);
    ASSERT_TRUE(bodyMesh.mesh.IsValid());
    ASSERT_EQ(bodyMesh.mesh.TriangleCount(), 2);
    const auto polyBodyProjection = ProjectPointToPolyhedronBody(SCPoint3d{3.0, 1.0, 1.0}, body);
    ASSERT_TRUE(polyBodyProjection.success);
    ASSERT_TRUE(polyBodyProjection.IsValid());
    ASSERT_EQ(polyBodyProjection.faceIndex, 0);
    ASSERT_TRUE(polyBodyProjection.projection.point.AlmostEquals(SCPoint3d{2.0, 1.0, 0.0}, 1e-12));
    GEOMETRY_TEST_ASSERT_NEAR(Distance(SCPoint3d{3.0, 1.0, 1.0}, body), std::sqrt(2.0), 1e-12);
    const auto polyBodyLineIntersection =
        Intersect(SCLine3d::FromOriginAndDirection(SCPoint3d{1.0, 1.0, 3.0}, SCVector3d{0.0, 0.0, -1.0}), body);
    ASSERT_TRUE(polyBodyLineIntersection.intersects);
    ASSERT_TRUE(polyBodyLineIntersection.IsValid());
    ASSERT_EQ(polyBodyLineIntersection.faceIndices.size(), 1);
    ASSERT_EQ(polyBodyLineIntersection.hits.size(), 1);
    ASSERT_EQ(polyBodyLineIntersection.faceIndices[0], 0);
    ASSERT_TRUE(polyBodyLineIntersection.hits[0].point.AlmostEquals(SCPoint3d{1.0, 1.0, 0.0}, 1e-12));
    ASSERT_EQ(LocatePoint(SCPoint3d{1.0, 1.0, 0.0}, body), SCPointContainment2d::OnBoundary);

    const PolyhedronFace3d holedFace(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}),
                                     outerLoop,
                                     {PolyhedronLoop3d({
                                         SCPoint3d{0.5, 0.5, 0.0},
                                         SCPoint3d{1.5, 0.5, 0.0},
                                         SCPoint3d{1.5, 1.5, 0.0},
                                         SCPoint3d{0.5, 1.5, 0.0},
                                     })});
    const auto projectedFace = ProjectFaceToPolygon2d(holedFace);
    ASSERT_TRUE(projectedFace.success);
    ASSERT_TRUE(projectedFace.polygon.IsValid());
    ASSERT_EQ(projectedFace.polygon.HoleCount(), 1);
    GEOMETRY_TEST_ASSERT_NEAR(projectedFace.origin.z, 0.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(Geometry::Dot(projectedFace.uAxis, projectedFace.vAxis), 0.0, 1e-12);

    const auto holedFaceMesh = ConvertToTriangleMesh(holedFace);
    ASSERT_TRUE(holedFaceMesh.success);
    ASSERT_EQ(holedFaceMesh.issue, MeshConversionIssue3d::None);
    ASSERT_TRUE(holedFaceMesh.mesh.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(holedFaceMesh.mesh.SurfaceArea(), projectedFace.polygon.Area(), 1e-12);

    const std::vector<SCBrepVertex> brepVertices{
        SCBrepVertex(SCPoint3d{0.0, 0.0, 5.0}),
        SCBrepVertex(SCPoint3d{2.0, 0.0, 5.0}),
        SCBrepVertex(SCPoint3d{2.0, 2.0, 5.0}),
        SCBrepVertex(SCPoint3d{0.0, 2.0, 5.0}),
    };
    const auto brepVertexProjection = ProjectPointToBrepVertex(SCPoint3d{1.0, 1.0, 6.0}, brepVertices[0]);
    ASSERT_TRUE(brepVertexProjection.success);
    ASSERT_TRUE(brepVertexProjection.IsValid());
    ASSERT_TRUE(brepVertexProjection.point.AlmostEquals(SCPoint3d{0.0, 0.0, 5.0}, 1e-12));
    GEOMETRY_TEST_ASSERT_NEAR(Distance(SCPoint3d{1.0, 1.0, 6.0}, brepVertices[0]), std::sqrt(3.0), 1e-12);
    const SCBox3d brepVertexBounds = Bounds(brepVertices[0]);
    ASSERT_TRUE(brepVertexBounds.IsValid());
    ASSERT_TRUE(brepVertexBounds.MinPoint().AlmostEquals(SCPoint3d{0.0, 0.0, 5.0}, 1e-12));
    ASSERT_TRUE(brepVertexBounds.MaxPoint().AlmostEquals(SCPoint3d{0.0, 0.0, 5.0}, 1e-12));
    const auto brepVertexLineIntersection =
        Intersect(SCLine3d::FromOriginAndDirection(SCPoint3d{0.0, 0.0, 6.0}, SCVector3d{0.0, 0.0, -1.0}), brepVertices[0]);
    ASSERT_TRUE(brepVertexLineIntersection.intersects);
    ASSERT_TRUE(brepVertexLineIntersection.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(brepVertexLineIntersection.lineParameter, 1.0, 1e-12);
    ASSERT_TRUE(brepVertexLineIntersection.point.AlmostEquals(SCPoint3d{0.0, 0.0, 5.0}, 1e-12));
    const auto brepVertexPlaneIntersection =
        Intersect(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 5.0}, SCVector3d{0.0, 0.0, 1.0}), brepVertices[0]);
    ASSERT_TRUE(brepVertexPlaneIntersection.intersects);
    ASSERT_TRUE(brepVertexPlaneIntersection.IsValid());
    ASSERT_TRUE(brepVertexPlaneIntersection.point.AlmostEquals(SCPoint3d{0.0, 0.0, 5.0}, 1e-12));
    const std::vector<SCBrepEdge> brepEdges{
        SCBrepEdge(
            std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                SCLine3d::FromOriginAndDirection(SCPoint3d{0.0, 0.0, 5.0}, SCVector3d{2.0, 0.0, 0.0}), SCIntervald{0.0, 1.0})),
            0,
            1),
        SCBrepEdge(
            std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                SCLine3d::FromOriginAndDirection(SCPoint3d{2.0, 0.0, 5.0}, SCVector3d{0.0, 2.0, 0.0}), SCIntervald{0.0, 1.0})),
            1,
            2),
        SCBrepEdge(
            std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                SCLine3d::FromOriginAndDirection(SCPoint3d{2.0, 2.0, 5.0}, SCVector3d{-2.0, 0.0, 0.0}), SCIntervald{0.0, 1.0})),
            2,
            3),
        SCBrepEdge(
            std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                SCLine3d::FromOriginAndDirection(SCPoint3d{0.0, 2.0, 5.0}, SCVector3d{0.0, -2.0, 0.0}), SCIntervald{0.0, 1.0})),
            3,
            0),
    };
    const SCBrepLoop brepOuterLoop({
        SCBrepCoedge(0, false),
        SCBrepCoedge(1, false),
        SCBrepCoedge(2, false),
        SCBrepCoedge(3, false),
    });
    ASSERT_TRUE(brepOuterLoop.IsValid());
    const SCBrepFace brepFace(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()),
                            brepOuterLoop,
                            {},
                            SCCurveOnSurface(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()),
                                           SCPolyline2d(
                                               {
                                                   SCPoint2d{0.0, 0.0},
                                                   SCPoint2d{2.0, 0.0},
                                                   SCPoint2d{2.0, 2.0},
                                                   SCPoint2d{0.0, 2.0},
                                               },
                                               SCPolylineClosure::Closed)));
    ASSERT_TRUE(brepFace.IsValid());
    const SCBox3d brepFaceBounds = brepFace.Bounds();
    ASSERT_TRUE(brepFaceBounds.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(brepFaceBounds.MinPoint().z, 5.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(brepFaceBounds.MaxPoint().z, 5.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(Bounds(brepFace).MinPoint().z, 5.0, 1e-12);
    const SCBrepShell brepShell({brepFace}, false);
    ASSERT_TRUE(brepShell.IsValid());
    const SCBox3d brepShellBounds = brepShell.Bounds();
    ASSERT_TRUE(brepShellBounds.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(brepShellBounds.MinPoint().z, 5.0, 1e-12);
    const SCBrepBody brepBody(brepVertices, brepEdges, {brepShell});
    ASSERT_FALSE(brepBody.IsEmpty());
    ASSERT_TRUE(brepBody.IsValid());
    ASSERT_EQ(brepBody.VertexCount(), 4);
    ASSERT_EQ(brepBody.EdgeCount(), 4);
    ASSERT_EQ(brepBody.ShellCount(), 1);
    ASSERT_EQ(brepBody.FaceCount(), 1);
    ASSERT_NE(brepBody.EdgeAt(0).Curve(), nullptr);
    const SCBox3d brepEdgeBounds = brepBody.EdgeAt(0).Bounds();
    ASSERT_TRUE(brepEdgeBounds.IsValid());
    ASSERT_TRUE(brepEdgeBounds.MinPoint().AlmostEquals(SCPoint3d{0.0, 0.0, 5.0}, 1e-12));
    ASSERT_TRUE(brepEdgeBounds.MaxPoint().AlmostEquals(SCPoint3d{2.0, 0.0, 5.0}, 1e-12));
    ASSERT_TRUE(Bounds(brepBody.EdgeAt(0)).MaxPoint().AlmostEquals(SCPoint3d{2.0, 0.0, 5.0}, 1e-12));
    const auto brepEdgeProjection = ProjectPointToBrepEdge(SCPoint3d{1.0, 1.0, 5.0}, brepBody.EdgeAt(0));
    ASSERT_TRUE(brepEdgeProjection.success);
    ASSERT_TRUE(brepEdgeProjection.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(brepEdgeProjection.parameter, 0.5, 1e-12);
    ASSERT_TRUE(brepEdgeProjection.point.AlmostEquals(SCPoint3d{1.0, 0.0, 5.0}, 1e-12));
    GEOMETRY_TEST_ASSERT_NEAR(Distance(SCPoint3d{1.0, 1.0, 5.0}, brepBody.EdgeAt(0)), 1.0, 1e-12);
    const auto brepEdgeLineIntersection =
        Intersect(SCLine3d::FromOriginAndDirection(SCPoint3d{1.0, -1.0, 5.0}, SCVector3d{0.0, 1.0, 0.0}), brepBody.EdgeAt(0));
    ASSERT_TRUE(brepEdgeLineIntersection.intersects);
    ASSERT_TRUE(brepEdgeLineIntersection.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(brepEdgeLineIntersection.edgeParameter, 0.5, 1e-12);
    ASSERT_TRUE(brepEdgeLineIntersection.point.AlmostEquals(SCPoint3d{1.0, 0.0, 5.0}, 1e-12));
    const auto brepEdgePlaneIntersection =
        Intersect(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 5.0}, SCVector3d{0.0, 0.0, 1.0}), brepBody.EdgeAt(0));
    ASSERT_TRUE(brepEdgePlaneIntersection.intersects);
    ASSERT_TRUE(brepEdgePlaneIntersection.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(brepEdgePlaneIntersection.edgeParameter, 0.0, 1e-12);
    ASSERT_TRUE(brepEdgePlaneIntersection.point.AlmostEquals(SCPoint3d{0.0, 0.0, 5.0}, 1e-12));
    const auto brepValidation = Validate(brepBody);
    ASSERT_TRUE(brepValidation.valid);
    ASSERT_EQ(brepValidation.issue, SCBrepValidationIssue3d::None);
    GEOMETRY_TEST_ASSERT_NEAR(Area(brepFace), 4.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(Bounds(brepBody).MinPoint().z, 5.0, 1e-12);
    const auto brepFaceLineIntersection =
        Intersect(SCLine3d::FromOriginAndDirection(SCPoint3d{1.0, 1.0, 8.0}, SCVector3d{0.0, 0.0, -1.0}), brepFace);
    ASSERT_TRUE(brepFaceLineIntersection.intersects);
    ASSERT_TRUE(brepFaceLineIntersection.IsValid());
    ASSERT_FALSE(brepFaceLineIntersection.onBoundary);
    GEOMETRY_TEST_ASSERT_NEAR(brepFaceLineIntersection.lineParameter, 3.0, 1e-12);
    ASSERT_TRUE(brepFaceLineIntersection.point.AlmostEquals(SCPoint3d{1.0, 1.0, 5.0}, 1e-12));
    ASSERT_EQ(LocatePoint(SCPoint3d{1.0, 1.0, 5.0}, brepFace), SCPointContainment2d::Inside);
    ASSERT_EQ(LocatePoint(SCPoint3d{2.0, 1.0, 5.0}, brepFace), SCPointContainment2d::OnBoundary);
    ASSERT_EQ(LocatePoint(SCPoint3d{3.0, 1.0, 5.0}, brepFace), SCPointContainment2d::Outside);
    const auto brepBodyLineIntersection =
        Intersect(SCLine3d::FromOriginAndDirection(SCPoint3d{1.0, 1.0, 8.0}, SCVector3d{0.0, 0.0, -1.0}), brepBody);
    ASSERT_TRUE(brepBodyLineIntersection.intersects);
    ASSERT_TRUE(brepBodyLineIntersection.IsValid());
    ASSERT_EQ(brepBodyLineIntersection.faceIndices.size(), 1);
    ASSERT_EQ(brepBodyLineIntersection.hits.size(), 1);
    ASSERT_EQ(brepBodyLineIntersection.faceIndices[0], 0);
    ASSERT_TRUE(brepBodyLineIntersection.hits[0].point.AlmostEquals(SCPoint3d{1.0, 1.0, 5.0}, 1e-12));
    ASSERT_EQ(LocatePoint(SCPoint3d{1.0, 1.0, 5.0}, brepBody), SCPointContainment2d::OnBoundary);
    ASSERT_EQ(LocatePoint(SCPoint3d{1.0, 1.0, 8.0}, brepBody), SCPointContainment2d::Outside);
    const auto brepFaceProjectionInside = ProjectPointToBrepFace(SCPoint3d{1.0, 1.0, 8.0}, brepFace);
    ASSERT_TRUE(brepFaceProjectionInside.success);
    ASSERT_TRUE(brepFaceProjectionInside.IsValid());
    ASSERT_TRUE(brepFaceProjectionInside.onTrimmedFace);
    ASSERT_FALSE(brepFaceProjectionInside.onBoundary);
    ASSERT_TRUE(brepFaceProjectionInside.point.AlmostEquals(SCPoint3d{1.0, 1.0, 5.0}, 1e-12));
    GEOMETRY_TEST_ASSERT_NEAR(Distance(SCPoint3d{1.0, 1.0, 8.0}, brepFace), 3.0, 1e-12);
    const auto brepFaceProjectionOutside = ProjectPointToBrepFace(SCPoint3d{3.0, 1.0, 5.0}, brepFace);
    ASSERT_TRUE(brepFaceProjectionOutside.success);
    ASSERT_TRUE(brepFaceProjectionOutside.IsValid());
    ASSERT_TRUE(brepFaceProjectionOutside.onTrimmedFace);
    ASSERT_TRUE(brepFaceProjectionOutside.onBoundary);
    ASSERT_TRUE(brepFaceProjectionOutside.point.AlmostEquals(SCPoint3d{2.0, 1.0, 5.0}, 1e-12));
    GEOMETRY_TEST_ASSERT_NEAR(Distance(SCPoint3d{3.0, 1.0, 5.0}, brepFace), 1.0, 1e-12);
    const auto brepBodyProjection = ProjectPointToBrepBody(SCPoint3d{3.0, 1.0, 5.0}, brepBody);
    ASSERT_TRUE(brepBodyProjection.success);
    ASSERT_TRUE(brepBodyProjection.IsValid());
    ASSERT_EQ(brepBodyProjection.faceIndex, 0);
    ASSERT_TRUE(brepBodyProjection.projection.point.AlmostEquals(SCPoint3d{2.0, 1.0, 5.0}, 1e-12));
    GEOMETRY_TEST_ASSERT_NEAR(Distance(SCPoint3d{3.0, 1.0, 5.0}, brepBody), 1.0, 1e-12);
    const auto brepFaceMesh = ConvertToTriangleMesh(brepFace);
    ASSERT_TRUE(brepFaceMesh.success);
    ASSERT_TRUE(brepFaceMesh.mesh.IsValid());
    ASSERT_EQ(brepFaceMesh.mesh.TriangleCount(), 2);
    GEOMETRY_TEST_ASSERT_NEAR(brepFaceMesh.mesh.SurfaceArea(), 4.0, 1e-12);
    const auto polyFaceFromBrep = ConvertToPolyhedronFace(brepFace);
    ASSERT_TRUE(polyFaceFromBrep.success);
    ASSERT_EQ(polyFaceFromBrep.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(polyFaceFromBrep.face.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(Area(polyFaceFromBrep.face), 4.0, 1e-12);
    const BrepHealing3d healedBrepBody = Heal(brepBody);
    ASSERT_TRUE(healedBrepBody.success);
    ASSERT_EQ(healedBrepBody.issue, HealingIssue3d::None);
    ASSERT_TRUE(healedBrepBody.body.IsValid());
    const SCBox3d brepBodyBounds = brepBody.Bounds();
    ASSERT_TRUE(brepBodyBounds.IsValid());
    ASSERT_TRUE(brepBodyBounds.MinPoint().AlmostEquals(SCPoint3d{0.0, 0.0, 5.0}, 1e-12));
    ASSERT_TRUE(brepBodyBounds.MaxPoint().AlmostEquals(SCPoint3d{2.0, 2.0, 5.0}, 1e-12));
    const auto polyBodyFromBrep = ConvertToPolyhedronBody(brepBody);
    ASSERT_TRUE(polyBodyFromBrep.success);
    ASSERT_EQ(polyBodyFromBrep.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(polyBodyFromBrep.body.IsValid());
    ASSERT_EQ(polyBodyFromBrep.body.FaceCount(), 1);
    GEOMETRY_TEST_ASSERT_NEAR(Volume(brepBody), 0.0, 1e-12);
    const auto brepMesh = ConvertToTriangleMesh(brepBody);
    ASSERT_TRUE(brepMesh.success);
    ASSERT_TRUE(brepMesh.mesh.IsValid());
    ASSERT_EQ(brepMesh.mesh.TriangleCount(), 2);
    GEOMETRY_TEST_ASSERT_NEAR(brepMesh.mesh.SurfaceArea(), 4.0, 1e-12);
    const SCPolyhedronSection3d brepSection =
        Section(brepBody, SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 5.0}, SCVector3d{0.0, 0.0, 1.0}));
    ASSERT_TRUE(brepSection.success);
    ASSERT_EQ(brepSection.polygons.size(), 1);
    GEOMETRY_TEST_ASSERT_NEAR(brepSection.polygons[0].Area(), 4.0, 1e-12);

    const SCBrepBody invalidBrepBody(brepVertices,
                                   brepEdges,
                                   {SCBrepShell({SCBrepFace(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()),
                                                        SCBrepLoop({SCBrepCoedge(9, false)}))})});
    const auto invalidBrepValidation = Validate(invalidBrepBody);
    ASSERT_FALSE(invalidBrepValidation.valid);
    ASSERT_EQ(invalidBrepValidation.issue, SCBrepValidationIssue3d::InvalidShell);

    const SCBrepBody invalidAdjacencyBody(brepVertices,
                                        brepEdges,
                                        {
                                            SCBrepShell(
                                                {
                                                    brepFace,
                                                    brepFace,
                                                    brepFace,
                                                },
                                                false),
                                        });
    const auto invalidAdjacencyValidation = Validate(invalidAdjacencyBody);
    ASSERT_FALSE(invalidAdjacencyValidation.valid);
    ASSERT_EQ(invalidAdjacencyValidation.issue, SCBrepValidationIssue3d::InvalidFaceAdjacency);

    const SCBrepFace brepFaceWithoutTrim(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), brepOuterLoop);
    const SCBrepBody brepBodyWithoutTrim(brepVertices, brepEdges, {SCBrepShell({brepFaceWithoutTrim}, false)});
    ASSERT_TRUE(brepBodyWithoutTrim.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(Area(brepFaceWithoutTrim), 0.0, 1e-12);
    const BrepHealing3d healedTrimmedBody = Heal(brepBodyWithoutTrim);
    ASSERT_TRUE(healedTrimmedBody.success);
    ASSERT_TRUE(healedTrimmedBody.body.IsValid());
    ASSERT_EQ(healedTrimmedBody.body.ShellCount(), 1);
    ASSERT_TRUE(healedTrimmedBody.body.ShellAt(0).FaceAt(0).OuterTrim().IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(Area(healedTrimmedBody.body.ShellAt(0).FaceAt(0)), 4.0, 1e-12);
    const auto healedTrimmedFaceMesh = ConvertToTriangleMesh(healedTrimmedBody.body.ShellAt(0).FaceAt(0));
    ASSERT_TRUE(healedTrimmedFaceMesh.success);
    ASSERT_TRUE(healedTrimmedFaceMesh.mesh.IsValid());
    const auto polyFaceFromUntrimmed = ConvertToPolyhedronFace(brepFaceWithoutTrim);
    ASSERT_FALSE(polyFaceFromUntrimmed.success);
    ASSERT_EQ(polyFaceFromUntrimmed.issue, BrepConversionIssue3d::InvalidTrim);
    const auto polyBodyFromUntrimmed = ConvertToPolyhedronBody(brepBodyWithoutTrim);
    ASSERT_TRUE(polyBodyFromUntrimmed.success);
    ASSERT_TRUE(polyBodyFromUntrimmed.body.IsValid());
    ASSERT_EQ(polyBodyFromUntrimmed.body.FaceCount(), 1);
    const auto healedTrimmedMesh = ConvertToTriangleMesh(healedTrimmedBody.body);
    ASSERT_TRUE(healedTrimmedMesh.success);
    ASSERT_TRUE(healedTrimmedMesh.mesh.IsValid());

    const SCBrepFace nurbsBrepFace(
        std::shared_ptr<ISCSurface>(nurbsSurface.Clone().release()),
        SCBrepLoop({SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)}),
        {},
        SCCurveOnSurface(std::shared_ptr<ISCSurface>(nurbsSurface.Clone().release()),
                       SCPolyline2d(
                           {
                               SCPoint2d{0.0, 0.0},
                               SCPoint2d{1.0, 0.0},
                               SCPoint2d{1.0, 1.0},
                               SCPoint2d{0.0, 1.0},
                           },
                           SCPolylineClosure::Closed)));
    ASSERT_TRUE(nurbsBrepFace.IsValid());
    const auto nurbsBrepFaceMesh = ConvertToTriangleMesh(nurbsBrepFace);
    ASSERT_TRUE(nurbsBrepFaceMesh.success);
    ASSERT_TRUE(nurbsBrepFaceMesh.mesh.IsValid());
    ASSERT_EQ(nurbsBrepFaceMesh.mesh.TriangleCount(), 2);
    const auto nurbsBrepFaceLineIntersection =
        Intersect(SCLine3d::FromOriginAndDirection(SCPoint3d{1.0, 1.0, 3.0}, SCVector3d{0.0, 0.0, -1.0}), nurbsBrepFace);
    ASSERT_TRUE(nurbsBrepFaceLineIntersection.intersects);
    ASSERT_TRUE(nurbsBrepFaceLineIntersection.IsValid());
    const auto nurbsBrepFaceProjection = ProjectPointToBrepFace(SCPoint3d{1.0, 1.0, 1.0}, nurbsBrepFace);
    ASSERT_TRUE(nurbsBrepFaceProjection.success);
    ASSERT_TRUE(nurbsBrepFaceProjection.IsValid());
    ASSERT_TRUE(nurbsBrepFaceProjection.onTrimmedFace);
    ASSERT_GE(nurbsBrepFaceProjection.point.z, 0.0);
    const double nurbsBrepFaceArea = Area(nurbsBrepFace);
    GEOMETRY_TEST_ASSERT_NEAR(nurbsBrepFaceArea, 4.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(nurbsBrepFaceArea, nurbsBrepFaceMesh.mesh.SurfaceArea(), 1e-12);

    const PolyhedronBody cubeBody({
        PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, -1.0}),
                         PolyhedronLoop3d({
                             SCPoint3d{0.0, 0.0, 0.0},
                             SCPoint3d{0.0, 1.0, 0.0},
                             SCPoint3d{1.0, 1.0, 0.0},
                             SCPoint3d{1.0, 0.0, 0.0},
                         })),
        PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 1.0}, SCVector3d{0.0, 0.0, 1.0}),
                         PolyhedronLoop3d({
                             SCPoint3d{0.0, 0.0, 1.0},
                             SCPoint3d{1.0, 0.0, 1.0},
                             SCPoint3d{1.0, 1.0, 1.0},
                             SCPoint3d{0.0, 1.0, 1.0},
                         })),
        PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, -1.0, 0.0}),
                         PolyhedronLoop3d({
                             SCPoint3d{0.0, 0.0, 0.0},
                             SCPoint3d{1.0, 0.0, 0.0},
                             SCPoint3d{1.0, 0.0, 1.0},
                             SCPoint3d{0.0, 0.0, 1.0},
                         })),
        PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{1.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
                         PolyhedronLoop3d({
                             SCPoint3d{1.0, 0.0, 0.0},
                             SCPoint3d{1.0, 1.0, 0.0},
                             SCPoint3d{1.0, 1.0, 1.0},
                             SCPoint3d{1.0, 0.0, 1.0},
                         })),
        PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 1.0, 0.0}, SCVector3d{0.0, 1.0, 0.0}),
                         PolyhedronLoop3d({
                             SCPoint3d{0.0, 1.0, 0.0},
                             SCPoint3d{0.0, 1.0, 1.0},
                             SCPoint3d{1.0, 1.0, 1.0},
                             SCPoint3d{1.0, 1.0, 0.0},
                         })),
        PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{-1.0, 0.0, 0.0}),
                         PolyhedronLoop3d({
                             SCPoint3d{0.0, 0.0, 0.0},
                             SCPoint3d{0.0, 0.0, 1.0},
                             SCPoint3d{0.0, 1.0, 1.0},
                             SCPoint3d{0.0, 1.0, 0.0},
                         })),
    });
    ASSERT_TRUE(cubeBody.IsValid());
    ASSERT_EQ(LocatePoint(SCPoint3d{0.5, 0.5, 0.5}, cubeBody), SCPointContainment2d::Inside);
    ASSERT_EQ(LocatePoint(SCPoint3d{1.5, 0.5, 0.5}, cubeBody), SCPointContainment2d::Outside);
    ASSERT_EQ(LocatePoint(SCPoint3d{1.0, 0.5, 0.5}, cubeBody), SCPointContainment2d::OnBoundary);
    const SCPolyhedronSection3d middleSection =
        Section(cubeBody, SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0}));
    ASSERT_TRUE(middleSection.success);
    ASSERT_EQ(middleSection.issue, SCSectionIssue3d::None);
    ASSERT_TRUE(middleSection.IsValid());
    const SCSectionValidation3d middleSectionValidation = Validate(middleSection);
    ASSERT_TRUE(middleSectionValidation.valid);
    ASSERT_EQ(middleSectionValidation.issue, SCSectionValidationIssue3d::None);
    const SCSectionTopology3d middleSectionTopology = BuildSectionTopology(middleSection);
    ASSERT_TRUE(middleSectionTopology.IsValid());
    ASSERT_EQ(middleSectionTopology.Count(), 1);
    ASSERT_EQ(middleSectionTopology.Roots().size(), 1);
    ASSERT_EQ(middleSectionTopology.Node(0).depth, 0);
    const SCSectionComponents3d middleSectionComponents = BuildSectionComponents(middleSection);
    ASSERT_TRUE(middleSectionComponents.IsValid());
    ASSERT_EQ(middleSectionComponents.components.size(), 1);
    ASSERT_EQ(middleSectionComponents.components[0].rootPolygonIndex, 0);
    ASSERT_EQ(middleSectionComponents.components[0].polygonIndices.size(), 1);
    ASSERT_EQ(middleSectionComponents.components[0].faceIndices.size(), 1);
    ASSERT_EQ(ClassifySectionContent(middleSection), SCSectionContentKind3d::Area);
    ASSERT_GE(middleSection.segments.size(), 4);
    ASSERT_EQ(middleSection.contours.size(), 1);
    ASSERT_TRUE(middleSection.contours[0].closed);
    ASSERT_EQ(middleSection.contours[0].points.size(), 4);
    ASSERT_EQ(middleSection.polygons.size(), 1);
    GEOMETRY_TEST_ASSERT_NEAR(middleSection.polygons[0].Area(), 1.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(middleSection.origin.z, 0.5, 1e-12);
    const SCSectionFaceRebuild3d rebuiltMiddleFaces = RebuildSectionFaces(middleSection);
    ASSERT_TRUE(rebuiltMiddleFaces.success);
    ASSERT_EQ(rebuiltMiddleFaces.issue, SCSectionFaceRebuildIssue3d::None);
    ASSERT_TRUE(rebuiltMiddleFaces.IsValid());
    ASSERT_EQ(rebuiltMiddleFaces.faces.size(), 1);
    const SCSectionBrepFaceRebuild3d rebuiltMiddleBrepFaces = RebuildSectionBrepFaces(middleSection);
    ASSERT_TRUE(rebuiltMiddleBrepFaces.success);
    ASSERT_TRUE(rebuiltMiddleBrepFaces.IsValid());
    ASSERT_EQ(rebuiltMiddleBrepFaces.faces.size(), 1);
    ASSERT_TRUE(rebuiltMiddleBrepFaces.faces[0].OuterTrim().IsValid());
    const auto rebuiltMiddleMesh = ConvertToTriangleMesh(rebuiltMiddleFaces.faces[0]);
    ASSERT_TRUE(rebuiltMiddleMesh.success);
    GEOMETRY_TEST_ASSERT_NEAR(rebuiltMiddleMesh.mesh.SurfaceArea(), 1.0, 1e-12);
    const SCSectionBodyRebuild3d rebuiltMiddleBody = RebuildSectionBody(middleSection);
    ASSERT_TRUE(rebuiltMiddleBody.success);
    ASSERT_EQ(rebuiltMiddleBody.issue, SCSectionBodyRebuildIssue3d::None);
    ASSERT_TRUE(rebuiltMiddleBody.IsValid());
    ASSERT_EQ(rebuiltMiddleBody.body.FaceCount(), 1);
    const SCSectionBrepBodyRebuild3d rebuiltMiddleBrepBody = RebuildSectionBrepBody(middleSection);
    ASSERT_TRUE(rebuiltMiddleBrepBody.success);
    ASSERT_TRUE(rebuiltMiddleBrepBody.IsValid());
    ASSERT_EQ(rebuiltMiddleBrepBody.body.FaceCount(), 1);
    const SCSectionBrepBodySetRebuild3d rebuiltMiddleBrepBodies = RebuildSectionBrepBodies(middleSection);
    ASSERT_TRUE(rebuiltMiddleBrepBodies.success);
    ASSERT_TRUE(rebuiltMiddleBrepBodies.IsValid());
    ASSERT_EQ(rebuiltMiddleBrepBodies.bodies.size(), 1);
    ASSERT_EQ(rebuiltMiddleBrepBodies.rootPolygonIndices.size(), 1);
    const SCSectionBodySetRebuild3d rebuiltMiddleBodies = RebuildSectionBodies(middleSection);
    ASSERT_TRUE(rebuiltMiddleBodies.success);
    ASSERT_TRUE(rebuiltMiddleBodies.IsValid());
    ASSERT_EQ(rebuiltMiddleBodies.bodies.size(), 1);
    ASSERT_EQ(rebuiltMiddleBodies.rootPolygonIndices.size(), 1);
    ASSERT_EQ(rebuiltMiddleBodies.rootPolygonIndices[0], 0);
    const SCSectionMeshConversion3d middleSectionMesh = ConvertSectionToTriangleMesh(middleSection);
    ASSERT_TRUE(middleSectionMesh.success);
    ASSERT_TRUE(middleSectionMesh.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(middleSectionMesh.mesh.SurfaceArea(), 1.0, 1e-12);
    const SCSectionMeshSetConversion3d middleSectionMeshes = ConvertSectionToTriangleMeshes(middleSection);
    ASSERT_TRUE(middleSectionMeshes.success);
    ASSERT_TRUE(middleSectionMeshes.IsValid());
    ASSERT_EQ(middleSectionMeshes.meshes.size(), 1);
    ASSERT_EQ(middleSectionMeshes.rootPolygonIndices.size(), 1);
    ASSERT_EQ(middleSectionMeshes.rootPolygonIndices[0], 0);

    const SCPolyhedronSection3d disjointSection =
        Section(cubeBody, SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 2.0}, SCVector3d{0.0, 0.0, 1.0}));
    ASSERT_TRUE(disjointSection.success);
    ASSERT_EQ(disjointSection.issue, SCSectionIssue3d::None);
    ASSERT_TRUE(disjointSection.segments.empty());
    ASSERT_TRUE(disjointSection.contours.empty());
    ASSERT_TRUE(disjointSection.polygons.empty());
    ASSERT_EQ(ClassifySectionContent(disjointSection), SCSectionContentKind3d::Empty);

    const SCPolyhedronSection3d coplanarSection =
        Section(cubeBody, SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    ASSERT_TRUE(coplanarSection.success);
    ASSERT_EQ(coplanarSection.issue, SCSectionIssue3d::None);
    ASSERT_TRUE(coplanarSection.IsValid());
    ASSERT_EQ(coplanarSection.contours.size(), 1);
    ASSERT_EQ(coplanarSection.polygons.size(), 1);
    GEOMETRY_TEST_ASSERT_NEAR(coplanarSection.polygons[0].Area(), 1.0, 1e-12);
    const SCSectionFaceRebuild3d rebuiltCoplanarFaces = RebuildSectionFaces(coplanarSection);
    ASSERT_TRUE(rebuiltCoplanarFaces.success);
    ASSERT_EQ(rebuiltCoplanarFaces.faces.size(), 1);

    const SCPolyhedronSection3d edgeOnlySection =
        Section(cubeBody, SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{1.0, 1.0, 0.0}));
    ASSERT_TRUE(edgeOnlySection.success);
    ASSERT_EQ(edgeOnlySection.issue, SCSectionIssue3d::None);
    ASSERT_TRUE(edgeOnlySection.IsValid());
    ASSERT_EQ(edgeOnlySection.segments.size(), 1);
    ASSERT_EQ(edgeOnlySection.contours.size(), 1);
    ASSERT_FALSE(edgeOnlySection.contours[0].closed);
    ASSERT_EQ(edgeOnlySection.contours[0].points.size(), 2);
    ASSERT_TRUE(edgeOnlySection.polygons.empty());
    const SCSectionTopology3d edgeOnlyTopology = BuildSectionTopology(edgeOnlySection);
    ASSERT_TRUE(edgeOnlyTopology.IsValid());
    ASSERT_TRUE(edgeOnlyTopology.IsEmpty());
    const SCSectionComponents3d edgeOnlyComponents = BuildSectionComponents(edgeOnlySection);
    ASSERT_TRUE(edgeOnlyComponents.IsValid());
    ASSERT_TRUE(edgeOnlyComponents.components.empty());
    ASSERT_EQ(ClassifySectionContent(edgeOnlySection), SCSectionContentKind3d::Curve);
    const SCSectionFaceRebuild3d rebuiltEdgeOnlyFaces = RebuildSectionFaces(edgeOnlySection);
    ASSERT_TRUE(rebuiltEdgeOnlyFaces.success);
    ASSERT_TRUE(rebuiltEdgeOnlyFaces.faces.empty());
    const SCSectionBrepFaceRebuild3d rebuiltEdgeOnlyBrepFaces = RebuildSectionBrepFaces(edgeOnlySection);
    ASSERT_TRUE(rebuiltEdgeOnlyBrepFaces.success);
    ASSERT_TRUE(rebuiltEdgeOnlyBrepFaces.faces.empty());
    const SCSectionBodyRebuild3d rebuiltEdgeOnlyBody = RebuildSectionBody(edgeOnlySection);
    ASSERT_TRUE(rebuiltEdgeOnlyBody.success);
    ASSERT_TRUE(rebuiltEdgeOnlyBody.body.IsEmpty());
    const SCSectionBrepBodyRebuild3d rebuiltEdgeOnlyBrepBody = RebuildSectionBrepBody(edgeOnlySection);
    ASSERT_TRUE(rebuiltEdgeOnlyBrepBody.success);
    ASSERT_TRUE(rebuiltEdgeOnlyBrepBody.body.IsEmpty());
    const SCSectionBrepBodySetRebuild3d rebuiltEdgeOnlyBrepBodies = RebuildSectionBrepBodies(edgeOnlySection);
    ASSERT_TRUE(rebuiltEdgeOnlyBrepBodies.success);
    ASSERT_TRUE(rebuiltEdgeOnlyBrepBodies.bodies.empty());
    const SCSectionBodySetRebuild3d rebuiltEdgeOnlyBodies = RebuildSectionBodies(edgeOnlySection);
    ASSERT_TRUE(rebuiltEdgeOnlyBodies.success);
    ASSERT_TRUE(rebuiltEdgeOnlyBodies.bodies.empty());
    const SCSectionMeshConversion3d edgeOnlySectionMesh = ConvertSectionToTriangleMesh(edgeOnlySection);
    ASSERT_TRUE(edgeOnlySectionMesh.success);
    ASSERT_TRUE(edgeOnlySectionMesh.mesh.IsEmpty());
    const SCSectionMeshSetConversion3d edgeOnlySectionMeshes = ConvertSectionToTriangleMeshes(edgeOnlySection);
    ASSERT_TRUE(edgeOnlySectionMeshes.success);
    ASSERT_TRUE(edgeOnlySectionMeshes.meshes.empty());

    SCPolyhedronSection3d nestedSection{};
    nestedSection.success = true;
    nestedSection.origin = SCPoint3d{0.0, 0.0, 0.5};
    nestedSection.uAxis = SCVector3d{1.0, 0.0, 0.0};
    nestedSection.vAxis = SCVector3d{0.0, 1.0, 0.0};
    nestedSection.polygons.push_back(SCPolygon2d(SCPolyline2d(
        {
            SCPoint2d{0.0, 0.0},
            SCPoint2d{4.0, 0.0},
            SCPoint2d{4.0, 4.0},
            SCPoint2d{0.0, 4.0},
        },
        SCPolylineClosure::Closed)));
    nestedSection.polygons.push_back(SCPolygon2d(SCPolyline2d(
        {
            SCPoint2d{1.0, 1.0},
            SCPoint2d{1.0, 3.0},
            SCPoint2d{3.0, 3.0},
            SCPoint2d{3.0, 1.0},
        },
        SCPolylineClosure::Closed)));
    const SCSectionFaceRebuild3d rebuiltMergedFaces = RebuildSectionFaces(nestedSection);
    ASSERT_TRUE(rebuiltMergedFaces.success);
    ASSERT_TRUE(rebuiltMergedFaces.IsValid());
    ASSERT_EQ(rebuiltMergedFaces.faces.size(), 1);
    ASSERT_EQ(rebuiltMergedFaces.mappings.size(), 1);
    ASSERT_EQ(rebuiltMergedFaces.mappings[0].outerPolygonIndex, 0);
    ASSERT_EQ(rebuiltMergedFaces.mappings[0].holePolygonIndices.size(), 1);
    ASSERT_EQ(rebuiltMergedFaces.mappings[0].holePolygonIndices[0], 1);
    ASSERT_EQ(rebuiltMergedFaces.faces[0].HoleCount(), 1);
    const SCSectionTopology3d nestedTopology = BuildSectionTopology(nestedSection);
    ASSERT_TRUE(nestedTopology.IsValid());
    ASSERT_EQ(nestedTopology.Count(), 2);
    ASSERT_EQ(nestedTopology.Roots().size(), 1);
    ASSERT_EQ(nestedTopology.ParentOf(1), 0);
    ASSERT_EQ(nestedTopology.Node(0).depth, 0);
    ASSERT_EQ(nestedTopology.Node(1).depth, 1);
    const SCSectionComponents3d nestedComponents = BuildSectionComponents(nestedSection);
    ASSERT_TRUE(nestedComponents.IsValid());
    ASSERT_EQ(nestedComponents.components.size(), 1);
    ASSERT_EQ(nestedComponents.components[0].polygonIndices.size(), 2);
    ASSERT_EQ(nestedComponents.components[0].faceIndices.size(), 1);
    const auto rebuiltMergedMesh = ConvertToTriangleMesh(rebuiltMergedFaces.faces[0]);
    ASSERT_TRUE(rebuiltMergedMesh.success);
    GEOMETRY_TEST_ASSERT_NEAR(rebuiltMergedMesh.mesh.SurfaceArea(), 12.0, 1e-12);
    const SCSectionBodyRebuild3d rebuiltMergedBody = RebuildSectionBody(nestedSection);
    ASSERT_TRUE(rebuiltMergedBody.success);
    ASSERT_EQ(rebuiltMergedBody.body.FaceCount(), 1);
    const SCSectionBrepFaceRebuild3d rebuiltMergedBrepFaces = RebuildSectionBrepFaces(nestedSection);
    ASSERT_TRUE(rebuiltMergedBrepFaces.success);
    ASSERT_EQ(rebuiltMergedBrepFaces.faces.size(), 1);
    ASSERT_EQ(rebuiltMergedBrepFaces.faces[0].HoleCount(), 1);
    const SCSectionBrepBodyRebuild3d rebuiltMergedBrepBody = RebuildSectionBrepBody(nestedSection);
    ASSERT_TRUE(rebuiltMergedBrepBody.success);
    ASSERT_EQ(rebuiltMergedBrepBody.body.FaceCount(), 1);
    const SCSectionBrepBodySetRebuild3d rebuiltMergedBrepBodies = RebuildSectionBrepBodies(nestedSection);
    ASSERT_TRUE(rebuiltMergedBrepBodies.success);
    ASSERT_EQ(rebuiltMergedBrepBodies.bodies.size(), 1);
    ASSERT_EQ(rebuiltMergedBrepBodies.rootPolygonIndices.size(), 1);
    const SCSectionBodySetRebuild3d rebuiltMergedBodies = RebuildSectionBodies(nestedSection);
    ASSERT_TRUE(rebuiltMergedBodies.success);
    ASSERT_EQ(rebuiltMergedBodies.bodies.size(), 1);
    ASSERT_EQ(rebuiltMergedBodies.rootPolygonIndices.size(), 1);
    ASSERT_EQ(rebuiltMergedBodies.rootPolygonIndices[0], 0);
    const SCSectionMeshConversion3d rebuiltMergedSectionMesh = ConvertSectionToTriangleMesh(nestedSection);
    ASSERT_TRUE(rebuiltMergedSectionMesh.success);
    GEOMETRY_TEST_ASSERT_NEAR(rebuiltMergedSectionMesh.mesh.SurfaceArea(), 12.0, 1e-12);
    const SCSectionMeshSetConversion3d rebuiltMergedSectionMeshes = ConvertSectionToTriangleMeshes(nestedSection);
    ASSERT_TRUE(rebuiltMergedSectionMeshes.success);
    ASSERT_EQ(rebuiltMergedSectionMeshes.meshes.size(), 1);
    ASSERT_EQ(rebuiltMergedSectionMeshes.rootPolygonIndices.size(), 1);
    ASSERT_EQ(rebuiltMergedSectionMeshes.rootPolygonIndices[0], 0);
    ASSERT_EQ(ClassifySectionContent(nestedSection), SCSectionContentKind3d::Area);

    SCPolyhedronSection3d disjointAreaSection{};
    disjointAreaSection.success = true;
    disjointAreaSection.origin = SCPoint3d{0.0, 0.0, 0.5};
    disjointAreaSection.uAxis = SCVector3d{1.0, 0.0, 0.0};
    disjointAreaSection.vAxis = SCVector3d{0.0, 1.0, 0.0};
    disjointAreaSection.polygons.push_back(SCPolygon2d(SCPolyline2d(
        {
            SCPoint2d{0.0, 0.0},
            SCPoint2d{1.0, 0.0},
            SCPoint2d{1.0, 1.0},
            SCPoint2d{0.0, 1.0},
        },
        SCPolylineClosure::Closed)));
    disjointAreaSection.polygons.push_back(SCPolygon2d(SCPolyline2d(
        {
            SCPoint2d{3.0, 0.0},
            SCPoint2d{4.0, 0.0},
            SCPoint2d{4.0, 1.0},
            SCPoint2d{3.0, 1.0},
        },
        SCPolylineClosure::Closed)));
    const SCSectionTopology3d disjointAreaTopology = BuildSectionTopology(disjointAreaSection);
    ASSERT_TRUE(disjointAreaTopology.IsValid());
    ASSERT_EQ(disjointAreaTopology.Roots().size(), 2);
    const SCSectionComponents3d disjointAreaComponents = BuildSectionComponents(disjointAreaSection);
    ASSERT_TRUE(disjointAreaComponents.IsValid());
    ASSERT_EQ(disjointAreaComponents.components.size(), 2);
    ASSERT_EQ(disjointAreaComponents.components[0].polygonIndices.size(), 1);
    ASSERT_EQ(disjointAreaComponents.components[1].polygonIndices.size(), 1);
    const SCSectionBodySetRebuild3d rebuiltDisjointBodies = RebuildSectionBodies(disjointAreaSection);
    ASSERT_TRUE(rebuiltDisjointBodies.success);
    ASSERT_TRUE(rebuiltDisjointBodies.IsValid());
    ASSERT_EQ(rebuiltDisjointBodies.bodies.size(), 2);
    ASSERT_EQ(rebuiltDisjointBodies.bodies[0].FaceCount(), 1);
    ASSERT_EQ(rebuiltDisjointBodies.bodies[1].FaceCount(), 1);
    ASSERT_EQ(rebuiltDisjointBodies.rootPolygonIndices.size(), 2);
    ASSERT_EQ(rebuiltDisjointBodies.rootPolygonIndices[0], 0);
    ASSERT_EQ(rebuiltDisjointBodies.rootPolygonIndices[1], 1);
    const SCSectionBrepBodySetRebuild3d rebuiltDisjointBrepBodies = RebuildSectionBrepBodies(disjointAreaSection);
    ASSERT_TRUE(rebuiltDisjointBrepBodies.success);
    ASSERT_EQ(rebuiltDisjointBrepBodies.bodies.size(), 2);
    ASSERT_EQ(rebuiltDisjointBrepBodies.rootPolygonIndices.size(), 2);
    ASSERT_EQ(rebuiltDisjointBrepBodies.rootPolygonIndices[0], 0);
    ASSERT_EQ(rebuiltDisjointBrepBodies.rootPolygonIndices[1], 1);
    const SCSectionMeshSetConversion3d rebuiltDisjointMeshes = ConvertSectionToTriangleMeshes(disjointAreaSection);
    ASSERT_TRUE(rebuiltDisjointMeshes.success);
    ASSERT_TRUE(rebuiltDisjointMeshes.IsValid());
    ASSERT_EQ(rebuiltDisjointMeshes.meshes.size(), 2);
    ASSERT_EQ(rebuiltDisjointMeshes.rootPolygonIndices.size(), 2);
    ASSERT_EQ(rebuiltDisjointMeshes.rootPolygonIndices[0], 0);
    ASSERT_EQ(rebuiltDisjointMeshes.rootPolygonIndices[1], 1);

    SCPolyhedronSection3d mixedSection = edgeOnlySection;
    mixedSection.polygons.push_back(middleSection.polygons[0]);
    ASSERT_EQ(ClassifySectionContent(mixedSection), SCSectionContentKind3d::Mixed);

    SCPolyhedronSection3d invalidSection = middleSection;
    invalidSection.uAxis = SCVector3d{0.0, 0.0, 0.0};
    const SCSectionValidation3d invalidSectionValidation = Validate(invalidSection);
    ASSERT_FALSE(invalidSectionValidation.valid);
    ASSERT_EQ(invalidSectionValidation.issue, SCSectionValidationIssue3d::InvalidBasis);
}



