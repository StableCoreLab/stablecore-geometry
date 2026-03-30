#include <gtest/gtest.h>
#include <cassert>
#include <cmath>
#include <memory>
#include <numbers>

#include "sdk/Geometry.h"
#include "support/GTestCompat.h"
#include "support/GeometryTestSupport.h"

using geometry::sdk::Contains;
using geometry::sdk::Distance;
using geometry::sdk::Area;
using geometry::sdk::ArcSegment2d;
using geometry::sdk::Box2d;
using geometry::sdk::Intersects;
using geometry::sdk::LineSegment2d;
using geometry::sdk::Line3d;
using geometry::sdk::LineCurve3d;
using geometry::sdk::MeshBoundaryEdge3d;
using geometry::sdk::MeshBoundaryLoop3d;
using geometry::sdk::MeshNonManifoldEdge3d;
using geometry::sdk::MeshRepairIssue3d;
using geometry::sdk::MeshShell3d;
using geometry::sdk::MeshTriangleAdjacency3d;
using geometry::sdk::MeshValidationIssue3d;
using geometry::sdk::MeshConversionIssue3d;
using geometry::sdk::Plane;
using geometry::sdk::PlaneSurface;
using geometry::sdk::Point2d;
using geometry::sdk::Point3d;
using geometry::sdk::PolyhedronBody;
using geometry::sdk::PolyhedronFace3d;
using geometry::sdk::PolyhedronLoop3d;
using geometry::sdk::SectionFaceRebuild3d;
using geometry::sdk::SectionFaceRebuildIssue3d;
using geometry::sdk::SectionBodyRebuild3d;
using geometry::sdk::SectionBodyRebuildIssue3d;
using geometry::sdk::SectionBodySetRebuild3d;
using geometry::sdk::SectionContentKind3d;
using geometry::sdk::SectionComponents3d;
using geometry::sdk::SectionMeshConversion3d;
using geometry::sdk::SectionMeshSetConversion3d;
using geometry::sdk::SectionTopology3d;
using geometry::sdk::SectionValidation3d;
using geometry::sdk::SectionValidationIssue3d;
using geometry::sdk::PolyhedronSection3d;
using geometry::sdk::PolyhedronValidationIssue3d;
using geometry::sdk::Polygon2d;
using geometry::sdk::ProjectFaceToPolygon2d;
using geometry::sdk::ProjectPointToSegment;
using geometry::sdk::Polyline2d;
using geometry::sdk::PolylineClosure;
using geometry::sdk::Segment2d;
using geometry::sdk::Surface;
using geometry::sdk::Tessellate;
using geometry::sdk::TriangleAdjacency;
using geometry::sdk::TriangleNormal;
using geometry::sdk::ComputeTriangleAdjacency;
using geometry::sdk::ComputeTriangleNormals;
using geometry::sdk::ComputeTriangleConnectedComponents;
using geometry::sdk::ComputeMeshShells;
using geometry::sdk::ExtractBoundaryEdges;
using geometry::sdk::ExtractBoundaryLoops;
using geometry::sdk::ExtractNonManifoldEdges;
using geometry::sdk::ConvertToTriangleMesh;
using geometry::sdk::Curve3d;
using geometry::sdk::TriangleMesh;
using geometry::sdk::TriangleMeshRepair3d;
using geometry::sdk::Triangle3d;
using geometry::sdk::Intervald;
using geometry::sdk::Section;
using geometry::sdk::SectionIssue3d;
using geometry::sdk::RebuildSectionFaces;
using geometry::sdk::RebuildSectionBody;
using geometry::sdk::RebuildSectionBodies;
using geometry::sdk::BuildSectionTopology;
using geometry::sdk::ConvertSectionToTriangleMesh;
using geometry::sdk::ConvertSectionToTriangleMeshes;
using geometry::sdk::ClassifySectionContent;
using geometry::sdk::BuildSectionComponents;
using geometry::sdk::BrepHealing3d;
using geometry::sdk::BrepBody;
using geometry::sdk::BrepCoedge;
using geometry::sdk::BrepEdge;
using geometry::sdk::BrepFace;
using geometry::sdk::BrepLoop;
using geometry::sdk::BrepShell;
using geometry::sdk::BrepValidationIssue3d;
using geometry::sdk::BrepVertex;
using geometry::sdk::CurveOnSurface;
using geometry::sdk::Heal;
using geometry::sdk::MeshHealing3d;
using geometry::sdk::NurbsCurve3d;
using geometry::sdk::NurbsSurface;
using geometry::sdk::OffsetSurface;
using geometry::sdk::PolyhedronHealing3d;
using geometry::sdk::RuledSurface;
using geometry::sdk::Validate;
using geometry::sdk::Vector2d;
using geometry::sdk::Vector3d;
using geometry::sdk::VertexNormal;
using geometry::sdk::ComputeVertexNormals;
using geometry::sdk::IsClosedTriangleMesh;
using geometry::sdk::IsConsistentlyOrientedTriangleMesh;
using geometry::sdk::IsManifoldTriangleMesh;
using geometry::sdk::ClosePlanarBoundaryLoops;
using geometry::sdk::OrientTriangleMeshConsistently;
using geometry::sdk::CloseSinglePlanarBoundaryLoop;

TEST(SdkTest, CoversCurrentCapabilities)
{
    const Point2d a = Point2d::FromXY(0.0, 0.0);
    const Point2d b = Point2d::FromXY(3.0, 4.0);
    const Vector2d offset = b - a;

    GEOMETRY_TEST_ASSERT_NEAR(Distance(a, b), 5.0, 1e-12);
    assert(offset == Vector2d(Vector2d{3.0, 4.0}));
    GEOMETRY_TEST_ASSERT_NEAR(offset.LengthSquared(), 25.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(offset.Length(), 5.0, 1e-12);
    assert(a + offset == b);
    assert(b - offset == a);

    const LineSegment2d line = LineSegment2d::FromEndpoints(a, b);
    assert(line.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(line.Length(), 5.0, 1e-12);
    const Point2d lineMidpoint{1.5, 2.0};
    GEOMETRY_TEST_ASSERT_POINT_NEAR(line.PointAt(0.5), lineMidpoint, 1e-12);
    GEOMETRY_TEST_ASSERT_BOX_NEAR(
        line.Bounds(),
        Box2d::FromMinMax(Point2d{0.0, 0.0}, Point2d{3.0, 4.0}),
        1e-12);
    assert(line.DebugString().find("LineSegment2d{start=") == 0);
    assert(line.Kind() == geometry::SegmentKind2::Line);

    const ArcSegment2d arc = ArcSegment2d::FromCenterRadiusStartSweep(
        Point2d{0.0, 0.0},
        1.0,
        0.0,
        std::numbers::pi_v<double> * 0.5);
    assert(arc.IsValid());
    assert(arc.Direction() == geometry::ArcDirection::CounterClockwise);
    GEOMETRY_TEST_ASSERT_NEAR(arc.Length(), std::numbers::pi_v<double> * 0.5, 1e-12);
    const Point2d arcStart{1.0, 0.0};
    const Point2d arcEnd{0.0, 1.0};
    GEOMETRY_TEST_ASSERT_POINT_NEAR(arc.StartPoint(), arcStart, 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(arc.EndPoint(), arcEnd, 1e-12);
    GEOMETRY_TEST_ASSERT_BOX_NEAR(
        arc.Bounds(),
        Box2d::FromMinMax(Point2d{0.0, 0.0}, Point2d{1.0, 1.0}),
        1e-12);
    assert(arc.DebugString().find("ArcSegment2d{center=") == 0);
    assert(arc.Kind() == geometry::SegmentKind2::Arc);

    std::unique_ptr<Segment2d> lineSegment = line.Clone();
    std::unique_ptr<Segment2d> arcSegment = arc.Clone();
    assert(lineSegment->Kind() == geometry::SegmentKind2::Line);
    assert(arcSegment->Kind() == geometry::SegmentKind2::Arc);
    GEOMETRY_TEST_ASSERT_NEAR(lineSegment->Length(), 5.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(arcSegment->Length(), std::numbers::pi_v<double> * 0.5, 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(lineSegment->StartPoint(), a, 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(arcSegment->EndPoint(), arcEnd, 1e-12);

    const auto projection = ProjectPointToSegment(Point2d{3.0, 1.0}, a, b);
    const geometry::sdk::SegmentProjection2d expectedProjection{
        Point2d{1.56, 2.08},
        0.52,
        3.24,
        true};
    GEOMETRY_TEST_ASSERT_PROJECTION_NEAR(
        projection,
        expectedProjection,
        1e-12);

    const Box2d boxA = Box2d::FromMinMax(Point2d{0.0, 0.0}, Point2d{2.0, 2.0});
    const Box2d boxB = Box2d::FromMinMax(Point2d{1.0, 1.0}, Point2d{3.0, 3.0});
    const Box2d boxC = Box2d::FromMinMax(Point2d{3.1, 3.1}, Point2d{4.0, 4.0});
    const Box2d invalidBox = Box2d::FromMinMax(Point2d{2.0, 2.0}, Point2d{1.0, 1.0});
    const Box2d expectedBox = Box2d::FromMinMax(Point2d{0.0, 0.0}, Point2d{2.0, 2.0});
    const Point2d expectedCenter{1.0, 1.0};

    assert(boxA.IsValid());
    GEOMETRY_TEST_ASSERT_BOX_NEAR(boxA, expectedBox, 1e-12);
    assert(!invalidBox.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(boxA.Width(), 2.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(boxA.Height(), 2.0, 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(boxA.Center(), expectedCenter, 1e-12);
    assert(Contains(boxA, Point2d{1.0, 1.0}));
    assert(Intersects(boxA, boxB));
    assert(!Intersects(boxA, boxC));
    assert(!Contains(invalidBox, Point2d{1.0, 1.0}));

    const Polyline2d openPath(
        {Point2d{0.0, 0.0}, Point2d{3.0, 0.0}, Point2d{3.0, 4.0}},
        PolylineClosure::Open);
    assert(openPath.IsValid());
    assert(!openPath.IsClosed());
    assert(openPath.PointCount() == 3);
    assert(openPath.SegmentCount() == 2);
    const Point2d openMidPoint{3.0, 0.0};
    GEOMETRY_TEST_ASSERT_POINT_NEAR(openPath.PointAt(1), openMidPoint, 1e-12);
    GEOMETRY_TEST_ASSERT_BOX_NEAR(
        openPath.Bounds(),
        Box2d::FromMinMax(Point2d{0.0, 0.0}, Point2d{3.0, 4.0}),
        1e-12);
    assert(openPath.DebugString().find("Polyline2d{closure=Open") == 0);

    const Polyline2d outerRing(
        {Point2d{0.0, 0.0}, Point2d{4.0, 0.0}, Point2d{4.0, 4.0}, Point2d{0.0, 4.0}},
        PolylineClosure::Closed);
    const Polyline2d holeRing(
        {Point2d{1.0, 1.0}, Point2d{1.0, 3.0}, Point2d{3.0, 3.0}, Point2d{3.0, 1.0}},
        PolylineClosure::Closed);
    const Polygon2d polygon(outerRing, {holeRing});
    assert(polygon.IsValid());
    assert(polygon.HoleCount() == 1);
    assert(polygon.PointCount() == 8);
    assert(polygon.SegmentCount() == 8);
    GEOMETRY_TEST_ASSERT_POLYLINE_NEAR(polygon.OuterRing(), outerRing, 1e-12);
    GEOMETRY_TEST_ASSERT_POLYLINE_NEAR(polygon.HoleAt(0), holeRing, 1e-12);
    GEOMETRY_TEST_ASSERT_BOX_NEAR(
        polygon.Bounds(),
        Box2d::FromMinMax(Point2d{0.0, 0.0}, Point2d{4.0, 4.0}),
        1e-12);
    assert(polygon.DebugString().find("Polygon2d{holeCount=1") == 0);

    const Line3d line3 = Line3d::FromOriginAndDirection(Point3d{1.0, 2.0, 3.0}, Vector3d{2.0, 0.0, 0.0});
    const LineCurve3d lineCurve = LineCurve3d::FromLine(line3, Intervald{-2.0, 3.0});
    assert(lineCurve.IsValid());
    assert(!lineCurve.IsClosed());
    assert(!lineCurve.IsPeriodic());
    GEOMETRY_TEST_ASSERT_NEAR(lineCurve.StartT(), -2.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(lineCurve.EndT(), 3.0, 1e-12);
    assert(lineCurve.PointAt(0.5).AlmostEquals(Point3d{2.0, 2.0, 3.0}, 1e-12));
    const auto lineEval = lineCurve.Evaluate(1.5, 2);
    assert(lineEval.IsValid());
    assert(lineEval.derivativeOrder == 2);
    assert(lineEval.point.AlmostEquals(Point3d{4.0, 2.0, 3.0}, 1e-12));
    assert(lineEval.firstDerivative.AlmostEquals(Vector3d{2.0, 0.0, 0.0}, 1e-12));
    assert(lineEval.secondDerivative.AlmostEquals(Vector3d{0.0, 0.0, 0.0}, 1e-12));
    const geometry::Box3d lineBounds = lineCurve.Bounds();
    assert(lineBounds.IsValid());
    assert(lineBounds.MinPoint().AlmostEquals(Point3d{-3.0, 2.0, 3.0}, 1e-12));
    assert(lineBounds.MaxPoint().AlmostEquals(Point3d{7.0, 2.0, 3.0}, 1e-12));
    std::unique_ptr<Curve3d> lineClone = lineCurve.Clone();
    assert(lineClone && lineClone->IsValid());
    assert(lineClone->PointAt(-2.0).AlmostEquals(Point3d{-3.0, 2.0, 3.0}, 1e-12));

    const Plane supportPlane = Plane::FromPointAndNormal(Point3d{0.0, 0.0, 5.0}, Vector3d{0.0, 0.0, 1.0});
    const PlaneSurface planeSurface = PlaneSurface::FromPlane(
        supportPlane,
        Intervald{-2.0, 2.0},
        Intervald{-3.0, 1.0});
    assert(planeSurface.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(planeSurface.StartU(), -2.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(planeSurface.EndU(), 2.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(planeSurface.StartV(), -3.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(planeSurface.EndV(), 1.0, 1e-12);
    const Point3d planePoint = planeSurface.PointAt(0.5, -1.0);
    GEOMETRY_TEST_ASSERT_NEAR(supportPlane.SignedDistanceTo(planePoint), 0.0, 1e-12);
    const auto surfaceEval = planeSurface.Evaluate(0.25, 0.75, 1);
    assert(surfaceEval.IsValid());
    assert(surfaceEval.point.AlmostEquals(planeSurface.PointAt(0.25, 0.75), 1e-12));
    GEOMETRY_TEST_ASSERT_NEAR(geometry::Dot(surfaceEval.derivativeU, surfaceEval.normal), 0.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(geometry::Dot(surfaceEval.derivativeV, surfaceEval.normal), 0.0, 1e-12);
    const geometry::Box3d surfaceBounds = planeSurface.Bounds();
    assert(surfaceBounds.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(surfaceBounds.MinPoint().z, 5.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(surfaceBounds.MaxPoint().z, 5.0, 1e-12);
    std::unique_ptr<Surface> surfaceClone = planeSurface.Clone();
    assert(surfaceClone && surfaceClone->IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(supportPlane.SignedDistanceTo(surfaceClone->PointAt(-2.0, -3.0)), 0.0, 1e-12);

    const NurbsCurve3d nurbsCurve(
        1,
        {
            Point3d{0.0, 0.0, 0.0},
            Point3d{1.0, 1.0, 0.0},
            Point3d{2.0, 0.0, 0.0},
        },
        {0.0, 0.0, 0.5, 1.0, 1.0});
    assert(nurbsCurve.IsValid());
    assert(!nurbsCurve.IsPeriodic());
    GEOMETRY_TEST_ASSERT_NEAR(nurbsCurve.StartT(), 0.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(nurbsCurve.EndT(), 1.0, 1e-12);
    assert(nurbsCurve.PointAt(0.5).AlmostEquals(Point3d{1.0, 1.0, 0.0}, 1e-12));
    const auto nurbsCurveEval = nurbsCurve.Evaluate(0.25, 2);
    assert(nurbsCurveEval.IsValid());
    assert(nurbsCurveEval.derivativeOrder == 2);
    GEOMETRY_TEST_ASSERT_NEAR(geometry::sdk::Length(nurbsCurve, 64), std::sqrt(2.0) * 2.0, 5e-2);
    const geometry::Box3d nurbsCurveBounds = nurbsCurve.Bounds();
    assert(nurbsCurveBounds.IsValid());
    assert(nurbsCurveBounds.MinPoint().AlmostEquals(Point3d{0.0, 0.0, 0.0}, 1e-12));
    assert(nurbsCurveBounds.MaxPoint().AlmostEquals(Point3d{2.0, 1.0, 0.0}, 1e-12));
    std::unique_ptr<Curve3d> nurbsCurveClone = nurbsCurve.Clone();
    assert(nurbsCurveClone && nurbsCurveClone->IsValid());
    assert(nurbsCurveClone->PointAt(0.5).AlmostEquals(Point3d{1.0, 1.0, 0.0}, 1e-12));

    const NurbsSurface nurbsSurface(
        1,
        1,
        2,
        2,
        {
            Point3d{0.0, 0.0, 0.0},
            Point3d{2.0, 0.0, 0.0},
            Point3d{0.0, 2.0, 0.0},
            Point3d{2.0, 2.0, 0.0},
        },
        {0.0, 0.0, 1.0, 1.0},
        {0.0, 0.0, 1.0, 1.0});
    assert(nurbsSurface.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(nurbsSurface.StartU(), 0.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(nurbsSurface.EndU(), 1.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(nurbsSurface.StartV(), 0.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(nurbsSurface.EndV(), 1.0, 1e-12);
    assert(nurbsSurface.PointAt(0.5, 0.5).AlmostEquals(Point3d{1.0, 1.0, 0.0}, 1e-12));
    const auto nurbsSurfaceEval = nurbsSurface.Evaluate(0.5, 0.5, 1);
    assert(nurbsSurfaceEval.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(nurbsSurfaceEval.normal.Normalized().z, 1.0, 1e-12);
    const geometry::Box3d nurbsSurfaceBounds = nurbsSurface.Bounds();
    assert(nurbsSurfaceBounds.IsValid());
    assert(nurbsSurfaceBounds.MinPoint().AlmostEquals(Point3d{0.0, 0.0, 0.0}, 1e-12));
    assert(nurbsSurfaceBounds.MaxPoint().AlmostEquals(Point3d{2.0, 2.0, 0.0}, 1e-12));
    std::unique_ptr<Surface> nurbsSurfaceClone = nurbsSurface.Clone();
    assert(nurbsSurfaceClone && nurbsSurfaceClone->IsValid());
    assert(nurbsSurfaceClone->PointAt(0.25, 0.75).AlmostEquals(Point3d{0.5, 1.5, 0.0}, 1e-12));

    const LineCurve3d lowerRail = LineCurve3d::FromLine(
        Line3d::FromOriginAndDirection(Point3d{0.0, 0.0, 0.0}, Vector3d{2.0, 0.0, 0.0}),
        Intervald{0.0, 1.0});
    const LineCurve3d upperRail = LineCurve3d::FromLine(
        Line3d::FromOriginAndDirection(Point3d{0.0, 0.0, 2.0}, Vector3d{2.0, 0.0, 0.0}),
        Intervald{0.0, 1.0});
    const RuledSurface ruledSurface = RuledSurface::FromCurves(lowerRail, upperRail);
    assert(ruledSurface.IsValid());
    assert(ruledSurface.FirstCurve() != nullptr);
    assert(ruledSurface.SecondCurve() != nullptr);
    assert(ruledSurface.PointAt(0.5, 0.5).AlmostEquals(Point3d{1.0, 0.0, 1.0}, 1e-12));
    const auto ruledEval = ruledSurface.Evaluate(0.25, 0.25, 1);
    assert(ruledEval.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(ruledEval.normal.Length(), 4.0, 1e-12);

    const OffsetSurface offsetSurface = OffsetSurface::FromSurface(planeSurface, 2.0);
    assert(offsetSurface.IsValid());
    assert(offsetSurface.BaseSurface() != nullptr);
    GEOMETRY_TEST_ASSERT_NEAR(offsetSurface.OffsetDistance(), 2.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(offsetSurface.PointAt(0.0, 0.0).z, 7.0, 1e-12);
    const geometry::Box3d offsetBounds = offsetSurface.Bounds();
    assert(offsetBounds.IsValid());
    assert(offsetBounds.MinPoint().z <= 3.0 + 1e-12);
    assert(offsetBounds.MaxPoint().z >= 7.0 - 1e-12);

    const CurveOnSurface curveOnSurface(
        std::shared_ptr<Surface>(planeSurface.Clone().release()),
        Polyline2d(
            {
                Point2d{-2.0, -3.0},
                Point2d{0.0, -1.0},
                Point2d{2.0, 1.0},
            },
            PolylineClosure::Open));
    assert(curveOnSurface.IsValid());
    assert(curveOnSurface.PointCount() == 3);
    assert(curveOnSurface.UvPointAt(1).AlmostEquals(Point2d{0.0, -1.0}, 1e-12));
    assert(curveOnSurface.PointAt(1).AlmostEquals(planeSurface.PointAt(0.0, -1.0), 1e-12));
    const geometry::Box3d curveOnSurfaceBounds = curveOnSurface.Bounds();
    assert(curveOnSurfaceBounds.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(curveOnSurfaceBounds.MinPoint().z, 5.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(curveOnSurfaceBounds.MaxPoint().z, 5.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(geometry::sdk::Bounds(curveOnSurface).MinPoint().z, 5.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(geometry::sdk::Length(curveOnSurface), std::sqrt(8.0) * 2.0, 1e-12);

    GEOMETRY_TEST_ASSERT_NEAR(geometry::sdk::Distance(Point3d{0.0, 0.0, 0.0}, Point3d{1.0, 2.0, 2.0}), 3.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(
        geometry::sdk::Distance(Point3d{0.0, 2.0, 0.0}, Line3d::FromOriginAndDirection(Point3d{0.0, 0.0, 0.0}, Vector3d{1.0, 0.0, 0.0})),
        2.0,
        1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(
        geometry::sdk::Distance(Point3d{0.0, 0.0, 8.0}, supportPlane),
        3.0,
        1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(
        geometry::sdk::Length(geometry::LineSegment3d::FromStartEnd(Point3d{0.0, 0.0, 0.0}, Point3d{0.0, 3.0, 4.0})),
        5.0,
        1e-12);

    const TriangleMesh mesh(
        {
            Point3d{0.0, 0.0, 0.0},
            Point3d{1.0, 0.0, 0.0},
            Point3d{0.0, 1.0, 0.0},
            Point3d{0.0, 0.0, 1.0},
        },
        {
            TriangleMesh::TriangleIndices{0, 1, 2},
            TriangleMesh::TriangleIndices{0, 1, 3},
        });
    assert(!mesh.IsEmpty());
    assert(mesh.IsValid());
    assert(mesh.VertexCount() == 4);
    assert(mesh.TriangleCount() == 2);
    assert(mesh.VertexAt(3).AlmostEquals(Point3d{0.0, 0.0, 1.0}, 1e-12));
    const Triangle3d firstTriangle = mesh.TriangleAt(0);
    assert(firstTriangle.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(firstTriangle.Area(), 0.5, 1e-12);
    const geometry::Box3d meshBounds = mesh.Bounds();
    assert(meshBounds.IsValid());
    assert(meshBounds.MinPoint().AlmostEquals(Point3d{0.0, 0.0, 0.0}, 1e-12));
    assert(meshBounds.MaxPoint().AlmostEquals(Point3d{1.0, 1.0, 1.0}, 1e-12));
    GEOMETRY_TEST_ASSERT_NEAR(mesh.SurfaceArea(), 1.0, 1e-12);
    const auto meshValidation = Validate(mesh);
    assert(meshValidation.valid);
    assert(meshValidation.issue == MeshValidationIssue3d::None);
    const Vector3d firstTriangleNormal = TriangleNormal(mesh, 0);
    assert(firstTriangleNormal.AlmostEquals(Vector3d{0.0, 0.0, 1.0}, 1e-12));
    const auto triangleNormals = ComputeTriangleNormals(mesh);
    assert(triangleNormals.size() == mesh.TriangleCount());
    assert(triangleNormals[1].AlmostEquals(Vector3d{0.0, -1.0, 0.0}, 1e-12));
    const Vector3d sharedVertexNormal = VertexNormal(mesh, 0);
    GEOMETRY_TEST_ASSERT_NEAR(sharedVertexNormal.Length(), 1.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(sharedVertexNormal.x, 0.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(sharedVertexNormal.y, -std::sqrt(0.5), 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(sharedVertexNormal.z, std::sqrt(0.5), 1e-12);
    const auto vertexNormals = ComputeVertexNormals(mesh);
    assert(vertexNormals.size() == mesh.VertexCount());
    assert(vertexNormals[2].AlmostEquals(Vector3d{0.0, 0.0, 1.0}, 1e-12));
    const MeshTriangleAdjacency3d firstAdjacency = TriangleAdjacency(mesh, 0);
    assert(firstAdjacency.HasNeighbor(0));
    assert(firstAdjacency.adjacentTriangles[0] == 1);
    assert(!firstAdjacency.HasNeighbor(1));
    assert(!firstAdjacency.HasNeighbor(2));
    const auto adjacency = ComputeTriangleAdjacency(mesh);
    assert(adjacency.size() == mesh.TriangleCount());
    assert(adjacency[1].HasNeighbor(0));
    assert(adjacency[1].adjacentTriangles[0] == 0);
    const std::vector<MeshBoundaryEdge3d> boundaryEdges = ExtractBoundaryEdges(mesh);
    assert(boundaryEdges.size() == 4);
    assert(boundaryEdges[0].IsValid());
    const std::vector<MeshBoundaryLoop3d> boundaryLoops = ExtractBoundaryLoops(mesh);
    assert(boundaryLoops.size() == 1);
    assert(boundaryLoops[0].IsValid());
    assert(boundaryLoops[0].closed);
    assert(boundaryLoops[0].vertexIndices.size() == 4);
    assert(!IsClosedTriangleMesh(mesh));
    assert(IsManifoldTriangleMesh(mesh));
    assert(!IsConsistentlyOrientedTriangleMesh(mesh));
    const TriangleMeshRepair3d repairedOpenMesh = OrientTriangleMeshConsistently(mesh);
    assert(repairedOpenMesh.success);
    assert(repairedOpenMesh.issue == MeshRepairIssue3d::None);
    assert(repairedOpenMesh.mesh.IsValid());
    assert(IsConsistentlyOrientedTriangleMesh(repairedOpenMesh.mesh));
    assert(IsManifoldTriangleMesh(repairedOpenMesh.mesh));
    GEOMETRY_TEST_ASSERT_NEAR(repairedOpenMesh.mesh.SurfaceArea(), mesh.SurfaceArea(), 1e-12);
    const TriangleMeshRepair3d closedNonPlanarMesh = CloseSinglePlanarBoundaryLoop(mesh);
    assert(!closedNonPlanarMesh.success);
    assert(closedNonPlanarMesh.issue == MeshRepairIssue3d::NonPlanarBoundary);
    const auto meshComponents = ComputeTriangleConnectedComponents(mesh);
    assert(meshComponents.size() == 1);
    assert(meshComponents[0].size() == 2);
    const std::vector<MeshShell3d> openShells = ComputeMeshShells(mesh);
    assert(openShells.size() == 1);
    assert(openShells[0].IsValid());
    assert(!openShells[0].closed);
    assert(openShells[0].manifold);
    assert(!openShells[0].consistentlyOriented);
    const TriangleMesh tetraMesh(
        {
            Point3d{0.0, 0.0, 0.0},
            Point3d{1.0, 0.0, 0.0},
            Point3d{0.0, 1.0, 0.0},
            Point3d{0.0, 0.0, 1.0},
        },
        {
            TriangleMesh::TriangleIndices{0, 2, 1},
            TriangleMesh::TriangleIndices{0, 1, 3},
            TriangleMesh::TriangleIndices{1, 2, 3},
            TriangleMesh::TriangleIndices{2, 0, 3},
        });
    assert(tetraMesh.IsValid());
    assert(ExtractBoundaryEdges(tetraMesh).empty());
    assert(ExtractBoundaryLoops(tetraMesh).empty());
    assert(IsClosedTriangleMesh(tetraMesh));
    assert(IsManifoldTriangleMesh(tetraMesh));
    assert(IsConsistentlyOrientedTriangleMesh(tetraMesh));
    GEOMETRY_TEST_ASSERT_NEAR(geometry::sdk::Volume(tetraMesh), 1.0 / 6.0, 1e-12);
    const TriangleMeshRepair3d repairedTetraMesh = OrientTriangleMeshConsistently(tetraMesh);
    assert(repairedTetraMesh.success);
    assert(repairedTetraMesh.issue == MeshRepairIssue3d::None);
    assert(repairedTetraMesh.mesh.IsValid());
    assert(IsConsistentlyOrientedTriangleMesh(repairedTetraMesh.mesh));
    const std::vector<MeshShell3d> tetraShells = ComputeMeshShells(tetraMesh);
    assert(tetraShells.size() == 1);
    assert(tetraShells[0].closed);
    assert(tetraShells[0].manifold);
    assert(tetraShells[0].consistentlyOriented);
    const TriangleMesh openTetraMesh(
        {
            Point3d{0.0, 0.0, 0.0},
            Point3d{1.0, 0.0, 0.0},
            Point3d{0.0, 1.0, 0.0},
            Point3d{0.0, 0.0, 1.0},
        },
        {
            TriangleMesh::TriangleIndices{0, 1, 3},
            TriangleMesh::TriangleIndices{1, 2, 3},
            TriangleMesh::TriangleIndices{2, 0, 3},
        });
    assert(openTetraMesh.IsValid());
    assert(!IsClosedTriangleMesh(openTetraMesh));
    const TriangleMeshRepair3d closedOpenTetraMesh = CloseSinglePlanarBoundaryLoop(openTetraMesh);
    assert(closedOpenTetraMesh.success);
    assert(closedOpenTetraMesh.issue == MeshRepairIssue3d::None);
    assert(closedOpenTetraMesh.mesh.IsValid());
    assert(IsClosedTriangleMesh(closedOpenTetraMesh.mesh));
    assert(IsManifoldTriangleMesh(closedOpenTetraMesh.mesh));
    assert(IsConsistentlyOrientedTriangleMesh(closedOpenTetraMesh.mesh));
    const MeshHealing3d healedOpenTetra = Heal(openTetraMesh);
    assert(healedOpenTetra.success);
    assert(healedOpenTetra.issue == geometry::sdk::HealingIssue3d::None);
    assert(healedOpenTetra.mesh.IsValid());
    assert(IsClosedTriangleMesh(healedOpenTetra.mesh));
    const TriangleMesh disconnectedMesh(
        {
            Point3d{0.0, 0.0, 0.0},
            Point3d{1.0, 0.0, 0.0},
            Point3d{0.0, 1.0, 0.0},
            Point3d{3.0, 0.0, 0.0},
            Point3d{4.0, 0.0, 0.0},
            Point3d{3.0, 1.0, 0.0},
        },
        {
            TriangleMesh::TriangleIndices{0, 1, 2},
            TriangleMesh::TriangleIndices{3, 4, 5},
        });
    assert(disconnectedMesh.IsValid());
    const auto disconnectedComponents = ComputeTriangleConnectedComponents(disconnectedMesh);
    assert(disconnectedComponents.size() == 2);
    assert(disconnectedComponents[0].size() == 1);
    assert(disconnectedComponents[1].size() == 1);
    const TriangleMeshRepair3d closedDisconnectedMesh = ClosePlanarBoundaryLoops(disconnectedMesh);
    assert(closedDisconnectedMesh.success);
    assert(closedDisconnectedMesh.issue == MeshRepairIssue3d::None);
    assert(closedDisconnectedMesh.mesh.IsValid());
    assert(IsClosedTriangleMesh(closedDisconnectedMesh.mesh));
    assert(IsManifoldTriangleMesh(closedDisconnectedMesh.mesh));
    assert(IsConsistentlyOrientedTriangleMesh(closedDisconnectedMesh.mesh));
    const std::vector<MeshBoundaryLoop3d> disconnectedBoundaryLoops = ExtractBoundaryLoops(disconnectedMesh);
    assert(disconnectedBoundaryLoops.size() == 2);
    assert(disconnectedBoundaryLoops[0].closed);
    assert(disconnectedBoundaryLoops[1].closed);
    const std::vector<MeshShell3d> disconnectedShells = ComputeMeshShells(disconnectedMesh);
    assert(disconnectedShells.size() == 2);
    assert(!disconnectedShells[0].closed);
    assert(!disconnectedShells[1].closed);
    const TriangleMesh nonManifoldMesh(
        {
            Point3d{0.0, 0.0, 0.0},
            Point3d{1.0, 0.0, 0.0},
            Point3d{0.0, 1.0, 0.0},
            Point3d{0.0, 0.0, 1.0},
            Point3d{0.0, -1.0, 0.0},
        },
        {
            TriangleMesh::TriangleIndices{0, 1, 2},
            TriangleMesh::TriangleIndices{0, 3, 1},
            TriangleMesh::TriangleIndices{0, 1, 4},
        });
    assert(nonManifoldMesh.IsValid());
    const auto nonManifoldEdges = ExtractNonManifoldEdges(nonManifoldMesh);
    assert(nonManifoldEdges.size() == 1);
    assert(nonManifoldEdges[0].IsValid());
    assert(nonManifoldEdges[0].incidentTriangles.size() == 3);
    assert(!IsManifoldTriangleMesh(nonManifoldMesh));
    assert(!IsConsistentlyOrientedTriangleMesh(nonManifoldMesh));
    const TriangleMeshRepair3d repairedNonManifoldMesh = OrientTriangleMeshConsistently(nonManifoldMesh);
    assert(!repairedNonManifoldMesh.success);
    assert(repairedNonManifoldMesh.issue == MeshRepairIssue3d::NonManifold);
    const auto nonManifoldComponents = ComputeTriangleConnectedComponents(nonManifoldMesh);
    assert(nonManifoldComponents.size() == 1);
    assert(nonManifoldComponents[0].size() == 3);
    const std::vector<MeshShell3d> nonManifoldShells = ComputeMeshShells(nonManifoldMesh);
    assert(nonManifoldShells.size() == 1);
    assert(!nonManifoldShells[0].closed);
    assert(!nonManifoldShells[0].manifold);
    assert(!nonManifoldShells[0].consistentlyOriented);
    const TriangleMesh movedMesh = mesh.Transformed(geometry::Transform3d::Translation(Vector3d{2.0, -1.0, 3.0}));
    assert(movedMesh.IsValid());
    assert(movedMesh.VertexAt(0).AlmostEquals(Point3d{2.0, -1.0, 3.0}, 1e-12));
    assert(movedMesh.VertexAt(3).AlmostEquals(Point3d{2.0, -1.0, 4.0}, 1e-12));
    GEOMETRY_TEST_ASSERT_NEAR(movedMesh.SurfaceArea(), mesh.SurfaceArea(), 1e-12);

    const TriangleMesh invalidMesh(
        {
            Point3d{0.0, 0.0, 0.0},
            Point3d{1.0, 0.0, 0.0},
            Point3d{1.0, 0.0, 0.0},
        },
        {
            TriangleMesh::TriangleIndices{0, 1, 2},
        });
    const auto invalidMeshValidation = Validate(invalidMesh);
    assert(!invalidMeshValidation.valid);
    assert(invalidMeshValidation.issue == MeshValidationIssue3d::DegenerateTriangle);

    const PolyhedronLoop3d outerLoop(
        {
            Point3d{0.0, 0.0, 0.0},
            Point3d{2.0, 0.0, 0.0},
            Point3d{2.0, 2.0, 0.0},
            Point3d{0.0, 2.0, 0.0},
        });
    assert(outerLoop.IsValid());
    assert(outerLoop.VertexCount() == 4);
    const PolyhedronFace3d face(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}),
        outerLoop);
    assert(face.IsValid());
    assert(face.HoleCount() == 0);
    const geometry::Box3d faceBounds = face.Bounds();
    assert(faceBounds.IsValid());
    assert(faceBounds.MinPoint().AlmostEquals(Point3d{0.0, 0.0, 0.0}, 1e-12));
    assert(faceBounds.MaxPoint().AlmostEquals(Point3d{2.0, 2.0, 0.0}, 1e-12));
    const PolyhedronBody body({face});
    assert(body.IsValid());
    assert(body.FaceCount() == 1);
    const auto bodyValidation = Validate(body);
    assert(bodyValidation.valid);
    assert(bodyValidation.issue == PolyhedronValidationIssue3d::None);
    const geometry::Box3d bodyBounds = body.Bounds();
    assert(bodyBounds.IsValid());
    assert(bodyBounds.MinPoint().AlmostEquals(Point3d{0.0, 0.0, 0.0}, 1e-12));
    assert(bodyBounds.MaxPoint().AlmostEquals(Point3d{2.0, 2.0, 0.0}, 1e-12));
    GEOMETRY_TEST_ASSERT_NEAR(geometry::sdk::Area(face), 4.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(geometry::sdk::Area(mesh), 1.0, 1e-12);

    const PolyhedronFace3d invalidFace(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}),
        PolyhedronLoop3d(
            {
                Point3d{0.0, 0.0, 0.0},
                Point3d{1.0, 0.0, 1.0},
                Point3d{0.0, 1.0, 0.0},
            }));
    const auto invalidBodyValidation = Validate(PolyhedronBody({invalidFace}));
    assert(!invalidBodyValidation.valid);
    assert(invalidBodyValidation.issue == PolyhedronValidationIssue3d::InvalidFace);

    const PolyhedronHealing3d healedBody = Heal(body);
    assert(healedBody.success);
    assert(healedBody.body.FaceCount() == 1);

    const TriangleMesh surfaceMesh = Tessellate(planeSurface, 2, 3);
    assert(surfaceMesh.IsValid());
    assert(surfaceMesh.VertexCount() == 12);
    assert(surfaceMesh.TriangleCount() == 12);
    const auto surfaceMeshValidation = Validate(surfaceMesh);
    assert(surfaceMeshValidation.valid);
    assert(!IsClosedTriangleMesh(surfaceMesh));
    for (const Point3d& vertex : surfaceMesh.Vertices())
    {
        GEOMETRY_TEST_ASSERT_NEAR(supportPlane.SignedDistanceTo(vertex), 0.0, 1e-12);
    }
    GEOMETRY_TEST_ASSERT_NEAR(surfaceMesh.SurfaceArea(), 16.0, 1e-12);

    const auto faceMesh = ConvertToTriangleMesh(face);
    assert(faceMesh.success);
    assert(faceMesh.issue == MeshConversionIssue3d::None);
    assert(faceMesh.mesh.IsValid());
    assert(faceMesh.mesh.TriangleCount() == 2);
    GEOMETRY_TEST_ASSERT_NEAR(faceMesh.mesh.SurfaceArea(), 4.0, 1e-12);

    const auto bodyMesh = ConvertToTriangleMesh(body);
    assert(bodyMesh.success);
    assert(bodyMesh.issue == MeshConversionIssue3d::None);
    assert(bodyMesh.mesh.IsValid());
    assert(bodyMesh.mesh.TriangleCount() == 2);

    const PolyhedronFace3d holedFace(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}),
        outerLoop,
        {PolyhedronLoop3d(
            {
                Point3d{0.5, 0.5, 0.0},
                Point3d{1.5, 0.5, 0.0},
                Point3d{1.5, 1.5, 0.0},
                Point3d{0.5, 1.5, 0.0},
            })});
    const auto projectedFace = ProjectFaceToPolygon2d(holedFace);
    assert(projectedFace.success);
    assert(projectedFace.polygon.IsValid());
    assert(projectedFace.polygon.HoleCount() == 1);
    GEOMETRY_TEST_ASSERT_NEAR(projectedFace.origin.z, 0.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(geometry::Dot(projectedFace.uAxis, projectedFace.vAxis), 0.0, 1e-12);

    const auto holedFaceMesh = ConvertToTriangleMesh(holedFace);
    assert(holedFaceMesh.success);
    assert(holedFaceMesh.issue == MeshConversionIssue3d::None);
    assert(holedFaceMesh.mesh.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(holedFaceMesh.mesh.SurfaceArea(), Area(projectedFace.polygon), 1e-12);

    const std::vector<BrepVertex> brepVertices{
        BrepVertex(Point3d{0.0, 0.0, 5.0}),
        BrepVertex(Point3d{2.0, 0.0, 5.0}),
        BrepVertex(Point3d{2.0, 2.0, 5.0}),
        BrepVertex(Point3d{0.0, 2.0, 5.0}),
    };
    const std::vector<BrepEdge> brepEdges{
        BrepEdge(
            std::make_shared<LineCurve3d>(LineCurve3d::FromLine(
                Line3d::FromOriginAndDirection(Point3d{0.0, 0.0, 5.0}, Vector3d{2.0, 0.0, 0.0}),
                Intervald{0.0, 1.0})),
            0,
            1),
        BrepEdge(
            std::make_shared<LineCurve3d>(LineCurve3d::FromLine(
                Line3d::FromOriginAndDirection(Point3d{2.0, 0.0, 5.0}, Vector3d{0.0, 2.0, 0.0}),
                Intervald{0.0, 1.0})),
            1,
            2),
        BrepEdge(
            std::make_shared<LineCurve3d>(LineCurve3d::FromLine(
                Line3d::FromOriginAndDirection(Point3d{2.0, 2.0, 5.0}, Vector3d{-2.0, 0.0, 0.0}),
                Intervald{0.0, 1.0})),
            2,
            3),
        BrepEdge(
            std::make_shared<LineCurve3d>(LineCurve3d::FromLine(
                Line3d::FromOriginAndDirection(Point3d{0.0, 2.0, 5.0}, Vector3d{0.0, -2.0, 0.0}),
                Intervald{0.0, 1.0})),
            3,
            0),
    };
    const BrepLoop brepOuterLoop({
        BrepCoedge(0, false),
        BrepCoedge(1, false),
        BrepCoedge(2, false),
        BrepCoedge(3, false),
    });
    assert(brepOuterLoop.IsValid());
    const BrepFace brepFace(
        std::shared_ptr<Surface>(planeSurface.Clone().release()),
        brepOuterLoop,
        {},
        CurveOnSurface(
            std::shared_ptr<Surface>(planeSurface.Clone().release()),
            Polyline2d(
                {
                    Point2d{-2.0, -3.0},
                    Point2d{0.0, -3.0},
                    Point2d{0.0, -1.0},
                    Point2d{-2.0, -1.0},
                },
                PolylineClosure::Closed)));
    assert(brepFace.IsValid());
    const geometry::Box3d brepFaceBounds = brepFace.Bounds();
    assert(brepFaceBounds.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(brepFaceBounds.MinPoint().z, 5.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(brepFaceBounds.MaxPoint().z, 5.0, 1e-12);
    const BrepShell brepShell({brepFace}, false);
    assert(brepShell.IsValid());
    const geometry::Box3d brepShellBounds = brepShell.Bounds();
    assert(brepShellBounds.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(brepShellBounds.MinPoint().z, 5.0, 1e-12);
    const BrepBody brepBody(brepVertices, brepEdges, {brepShell});
    assert(!brepBody.IsEmpty());
    assert(brepBody.IsValid());
    assert(brepBody.VertexCount() == 4);
    assert(brepBody.EdgeCount() == 4);
    assert(brepBody.ShellCount() == 1);
    assert(brepBody.FaceCount() == 1);
    assert(brepBody.EdgeAt(0).Curve() != nullptr);
    const geometry::Box3d brepEdgeBounds = brepBody.EdgeAt(0).Bounds();
    assert(brepEdgeBounds.IsValid());
    assert(brepEdgeBounds.MinPoint().AlmostEquals(Point3d{0.0, 0.0, 5.0}, 1e-12));
    assert(brepEdgeBounds.MaxPoint().AlmostEquals(Point3d{2.0, 0.0, 5.0}, 1e-12));
    const auto brepValidation = Validate(brepBody);
    assert(brepValidation.valid);
    assert(brepValidation.issue == BrepValidationIssue3d::None);
    GEOMETRY_TEST_ASSERT_NEAR(geometry::sdk::Area(brepFace), 4.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(geometry::sdk::Bounds(brepBody).MinPoint().z, 5.0, 1e-12);
    const auto brepFaceMesh = ConvertToTriangleMesh(brepFace);
    assert(brepFaceMesh.success);
    assert(brepFaceMesh.mesh.IsValid());
    assert(brepFaceMesh.mesh.TriangleCount() == 2);
    GEOMETRY_TEST_ASSERT_NEAR(brepFaceMesh.mesh.SurfaceArea(), 4.0, 1e-12);
    const BrepHealing3d healedBrepBody = Heal(brepBody);
    assert(healedBrepBody.success);
    assert(healedBrepBody.issue == geometry::sdk::HealingIssue3d::None);
    assert(healedBrepBody.body.IsValid());
    const geometry::Box3d brepBodyBounds = brepBody.Bounds();
    assert(brepBodyBounds.IsValid());
    assert(brepBodyBounds.MinPoint().AlmostEquals(Point3d{0.0, 0.0, 5.0}, 1e-12));
    assert(brepBodyBounds.MaxPoint().AlmostEquals(Point3d{2.0, 2.0, 5.0}, 1e-12));
    GEOMETRY_TEST_ASSERT_NEAR(geometry::sdk::Volume(brepBody), 0.0, 1e-12);
    const auto brepMesh = ConvertToTriangleMesh(brepBody);
    assert(brepMesh.success);
    assert(brepMesh.mesh.IsValid());
    assert(brepMesh.mesh.TriangleCount() == 2);
    GEOMETRY_TEST_ASSERT_NEAR(brepMesh.mesh.SurfaceArea(), 4.0, 1e-12);
    const PolyhedronSection3d brepSection = Section(
        brepBody,
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 5.0}, Vector3d{0.0, 0.0, 1.0}));
    assert(brepSection.success);
    assert(brepSection.polygons.size() == 1);
    GEOMETRY_TEST_ASSERT_NEAR(Area(brepSection.polygons[0]), 4.0, 1e-12);

    const BrepBody invalidBrepBody(
        brepVertices,
        brepEdges,
        {BrepShell({BrepFace(std::shared_ptr<Surface>(planeSurface.Clone().release()), BrepLoop({BrepCoedge(9, false)}))})});
    const auto invalidBrepValidation = Validate(invalidBrepBody);
    assert(!invalidBrepValidation.valid);
    assert(invalidBrepValidation.issue == BrepValidationIssue3d::InvalidShell);

    const BrepFace brepFaceWithoutTrim(
        std::shared_ptr<Surface>(planeSurface.Clone().release()),
        brepOuterLoop);
    const BrepBody brepBodyWithoutTrim(brepVertices, brepEdges, {BrepShell({brepFaceWithoutTrim}, false)});
    assert(brepBodyWithoutTrim.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(geometry::sdk::Area(brepFaceWithoutTrim), 0.0, 1e-12);
    const BrepHealing3d healedTrimmedBody = Heal(brepBodyWithoutTrim);
    assert(healedTrimmedBody.success);
    assert(healedTrimmedBody.body.IsValid());
    assert(healedTrimmedBody.body.ShellCount() == 1);
    assert(healedTrimmedBody.body.ShellAt(0).FaceAt(0).OuterTrim().IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(
        geometry::sdk::Area(healedTrimmedBody.body.ShellAt(0).FaceAt(0)),
        4.0,
        1e-12);
    const auto healedTrimmedFaceMesh = ConvertToTriangleMesh(healedTrimmedBody.body.ShellAt(0).FaceAt(0));
    assert(healedTrimmedFaceMesh.success);
    assert(healedTrimmedFaceMesh.mesh.IsValid());
    const auto healedTrimmedMesh = ConvertToTriangleMesh(healedTrimmedBody.body);
    assert(healedTrimmedMesh.success);
    assert(healedTrimmedMesh.mesh.IsValid());

    const PolyhedronBody cubeBody(
        {
            PolyhedronFace3d(
                Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, -1.0}),
                PolyhedronLoop3d(
                    {
                        Point3d{0.0, 0.0, 0.0},
                        Point3d{0.0, 1.0, 0.0},
                        Point3d{1.0, 1.0, 0.0},
                        Point3d{1.0, 0.0, 0.0},
                    })),
            PolyhedronFace3d(
                Plane::FromPointAndNormal(Point3d{0.0, 0.0, 1.0}, Vector3d{0.0, 0.0, 1.0}),
                PolyhedronLoop3d(
                    {
                        Point3d{0.0, 0.0, 1.0},
                        Point3d{1.0, 0.0, 1.0},
                        Point3d{1.0, 1.0, 1.0},
                        Point3d{0.0, 1.0, 1.0},
                    })),
            PolyhedronFace3d(
                Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, -1.0, 0.0}),
                PolyhedronLoop3d(
                    {
                        Point3d{0.0, 0.0, 0.0},
                        Point3d{1.0, 0.0, 0.0},
                        Point3d{1.0, 0.0, 1.0},
                        Point3d{0.0, 0.0, 1.0},
                    })),
            PolyhedronFace3d(
                Plane::FromPointAndNormal(Point3d{1.0, 0.0, 0.0}, Vector3d{1.0, 0.0, 0.0}),
                PolyhedronLoop3d(
                    {
                        Point3d{1.0, 0.0, 0.0},
                        Point3d{1.0, 1.0, 0.0},
                        Point3d{1.0, 1.0, 1.0},
                        Point3d{1.0, 0.0, 1.0},
                    })),
            PolyhedronFace3d(
                Plane::FromPointAndNormal(Point3d{0.0, 1.0, 0.0}, Vector3d{0.0, 1.0, 0.0}),
                PolyhedronLoop3d(
                    {
                        Point3d{0.0, 1.0, 0.0},
                        Point3d{0.0, 1.0, 1.0},
                        Point3d{1.0, 1.0, 1.0},
                        Point3d{1.0, 1.0, 0.0},
                    })),
            PolyhedronFace3d(
                Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{-1.0, 0.0, 0.0}),
                PolyhedronLoop3d(
                    {
                        Point3d{0.0, 0.0, 0.0},
                        Point3d{0.0, 0.0, 1.0},
                        Point3d{0.0, 1.0, 1.0},
                        Point3d{0.0, 1.0, 0.0},
                    })),
        });
    assert(cubeBody.IsValid());
    const PolyhedronSection3d middleSection = Section(
        cubeBody,
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.5}, Vector3d{0.0, 0.0, 1.0}));
    assert(middleSection.success);
    assert(middleSection.issue == SectionIssue3d::None);
    assert(middleSection.IsValid());
    const SectionValidation3d middleSectionValidation = Validate(middleSection);
    assert(middleSectionValidation.valid);
    assert(middleSectionValidation.issue == SectionValidationIssue3d::None);
    const SectionTopology3d middleSectionTopology = BuildSectionTopology(middleSection);
    assert(middleSectionTopology.IsValid());
    assert(middleSectionTopology.Count() == 1);
    assert(middleSectionTopology.Roots().size() == 1);
    assert(middleSectionTopology.Node(0).depth == 0);
    const SectionComponents3d middleSectionComponents = BuildSectionComponents(middleSection);
    assert(middleSectionComponents.IsValid());
    assert(middleSectionComponents.components.size() == 1);
    assert(middleSectionComponents.components[0].rootPolygonIndex == 0);
    assert(middleSectionComponents.components[0].polygonIndices.size() == 1);
    assert(middleSectionComponents.components[0].faceIndices.size() == 1);
    assert(ClassifySectionContent(middleSection) == SectionContentKind3d::Area);
    assert(middleSection.segments.size() >= 4);
    assert(middleSection.contours.size() == 1);
    assert(middleSection.contours[0].closed);
    assert(middleSection.contours[0].points.size() == 4);
    assert(middleSection.polygons.size() == 1);
    GEOMETRY_TEST_ASSERT_NEAR(Area(middleSection.polygons[0]), 1.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(middleSection.origin.z, 0.5, 1e-12);
    const SectionFaceRebuild3d rebuiltMiddleFaces = RebuildSectionFaces(middleSection);
    assert(rebuiltMiddleFaces.success);
    assert(rebuiltMiddleFaces.issue == SectionFaceRebuildIssue3d::None);
    assert(rebuiltMiddleFaces.IsValid());
    assert(rebuiltMiddleFaces.faces.size() == 1);
    const auto rebuiltMiddleMesh = ConvertToTriangleMesh(rebuiltMiddleFaces.faces[0]);
    assert(rebuiltMiddleMesh.success);
    GEOMETRY_TEST_ASSERT_NEAR(rebuiltMiddleMesh.mesh.SurfaceArea(), 1.0, 1e-12);
    const SectionBodyRebuild3d rebuiltMiddleBody = RebuildSectionBody(middleSection);
    assert(rebuiltMiddleBody.success);
    assert(rebuiltMiddleBody.issue == SectionBodyRebuildIssue3d::None);
    assert(rebuiltMiddleBody.IsValid());
    assert(rebuiltMiddleBody.body.FaceCount() == 1);
    const SectionBodySetRebuild3d rebuiltMiddleBodies = RebuildSectionBodies(middleSection);
    assert(rebuiltMiddleBodies.success);
    assert(rebuiltMiddleBodies.IsValid());
    assert(rebuiltMiddleBodies.bodies.size() == 1);
    assert(rebuiltMiddleBodies.rootPolygonIndices.size() == 1);
    assert(rebuiltMiddleBodies.rootPolygonIndices[0] == 0);
    const SectionMeshConversion3d middleSectionMesh = ConvertSectionToTriangleMesh(middleSection);
    assert(middleSectionMesh.success);
    assert(middleSectionMesh.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(middleSectionMesh.mesh.SurfaceArea(), 1.0, 1e-12);
    const SectionMeshSetConversion3d middleSectionMeshes = ConvertSectionToTriangleMeshes(middleSection);
    assert(middleSectionMeshes.success);
    assert(middleSectionMeshes.IsValid());
    assert(middleSectionMeshes.meshes.size() == 1);
    assert(middleSectionMeshes.rootPolygonIndices.size() == 1);
    assert(middleSectionMeshes.rootPolygonIndices[0] == 0);

    const PolyhedronSection3d disjointSection = Section(
        cubeBody,
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 2.0}, Vector3d{0.0, 0.0, 1.0}));
    assert(disjointSection.success);
    assert(disjointSection.issue == SectionIssue3d::None);
    assert(disjointSection.segments.empty());
    assert(disjointSection.contours.empty());
    assert(disjointSection.polygons.empty());
    assert(ClassifySectionContent(disjointSection) == SectionContentKind3d::Empty);

    const PolyhedronSection3d coplanarSection = Section(
        cubeBody,
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}));
    assert(coplanarSection.success);
    assert(coplanarSection.issue == SectionIssue3d::None);
    assert(coplanarSection.IsValid());
    assert(coplanarSection.contours.size() == 1);
    assert(coplanarSection.polygons.size() == 1);
    GEOMETRY_TEST_ASSERT_NEAR(Area(coplanarSection.polygons[0]), 1.0, 1e-12);
    const SectionFaceRebuild3d rebuiltCoplanarFaces = RebuildSectionFaces(coplanarSection);
    assert(rebuiltCoplanarFaces.success);
    assert(rebuiltCoplanarFaces.faces.size() == 1);

    const PolyhedronSection3d edgeOnlySection = Section(
        cubeBody,
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{1.0, 1.0, 0.0}));
    assert(edgeOnlySection.success);
    assert(edgeOnlySection.issue == SectionIssue3d::None);
    assert(edgeOnlySection.IsValid());
    assert(edgeOnlySection.segments.size() == 1);
    assert(edgeOnlySection.contours.size() == 1);
    assert(!edgeOnlySection.contours[0].closed);
    assert(edgeOnlySection.contours[0].points.size() == 2);
    assert(edgeOnlySection.polygons.empty());
    const SectionTopology3d edgeOnlyTopology = BuildSectionTopology(edgeOnlySection);
    assert(edgeOnlyTopology.IsValid());
    assert(edgeOnlyTopology.IsEmpty());
    const SectionComponents3d edgeOnlyComponents = BuildSectionComponents(edgeOnlySection);
    assert(edgeOnlyComponents.IsValid());
    assert(edgeOnlyComponents.components.empty());
    assert(ClassifySectionContent(edgeOnlySection) == SectionContentKind3d::Curve);
    const SectionFaceRebuild3d rebuiltEdgeOnlyFaces = RebuildSectionFaces(edgeOnlySection);
    assert(rebuiltEdgeOnlyFaces.success);
    assert(rebuiltEdgeOnlyFaces.faces.empty());
    const SectionBodyRebuild3d rebuiltEdgeOnlyBody = RebuildSectionBody(edgeOnlySection);
    assert(rebuiltEdgeOnlyBody.success);
    assert(rebuiltEdgeOnlyBody.body.IsEmpty());
    const SectionBodySetRebuild3d rebuiltEdgeOnlyBodies = RebuildSectionBodies(edgeOnlySection);
    assert(rebuiltEdgeOnlyBodies.success);
    assert(rebuiltEdgeOnlyBodies.bodies.empty());
    const SectionMeshConversion3d edgeOnlySectionMesh = ConvertSectionToTriangleMesh(edgeOnlySection);
    assert(edgeOnlySectionMesh.success);
    assert(edgeOnlySectionMesh.mesh.IsEmpty());
    const SectionMeshSetConversion3d edgeOnlySectionMeshes = ConvertSectionToTriangleMeshes(edgeOnlySection);
    assert(edgeOnlySectionMeshes.success);
    assert(edgeOnlySectionMeshes.meshes.empty());

    PolyhedronSection3d nestedSection{};
    nestedSection.success = true;
    nestedSection.origin = Point3d{0.0, 0.0, 0.5};
    nestedSection.uAxis = Vector3d{1.0, 0.0, 0.0};
    nestedSection.vAxis = Vector3d{0.0, 1.0, 0.0};
    nestedSection.polygons.push_back(Polygon2d(
        Polyline2d(
            {
                Point2d{0.0, 0.0},
                Point2d{4.0, 0.0},
                Point2d{4.0, 4.0},
                Point2d{0.0, 4.0},
            },
            PolylineClosure::Closed)));
    nestedSection.polygons.push_back(Polygon2d(
        Polyline2d(
            {
                Point2d{1.0, 1.0},
                Point2d{1.0, 3.0},
                Point2d{3.0, 3.0},
                Point2d{3.0, 1.0},
            },
            PolylineClosure::Closed)));
    const SectionFaceRebuild3d rebuiltMergedFaces = RebuildSectionFaces(nestedSection);
    assert(rebuiltMergedFaces.success);
    assert(rebuiltMergedFaces.IsValid());
    assert(rebuiltMergedFaces.faces.size() == 1);
    assert(rebuiltMergedFaces.mappings.size() == 1);
    assert(rebuiltMergedFaces.mappings[0].outerPolygonIndex == 0);
    assert(rebuiltMergedFaces.mappings[0].holePolygonIndices.size() == 1);
    assert(rebuiltMergedFaces.mappings[0].holePolygonIndices[0] == 1);
    assert(rebuiltMergedFaces.faces[0].HoleCount() == 1);
    const SectionTopology3d nestedTopology = BuildSectionTopology(nestedSection);
    assert(nestedTopology.IsValid());
    assert(nestedTopology.Count() == 2);
    assert(nestedTopology.Roots().size() == 1);
    assert(nestedTopology.ParentOf(1) == 0);
    assert(nestedTopology.Node(0).depth == 0);
    assert(nestedTopology.Node(1).depth == 1);
    const SectionComponents3d nestedComponents = BuildSectionComponents(nestedSection);
    assert(nestedComponents.IsValid());
    assert(nestedComponents.components.size() == 1);
    assert(nestedComponents.components[0].polygonIndices.size() == 2);
    assert(nestedComponents.components[0].faceIndices.size() == 1);
    const auto rebuiltMergedMesh = ConvertToTriangleMesh(rebuiltMergedFaces.faces[0]);
    assert(rebuiltMergedMesh.success);
    GEOMETRY_TEST_ASSERT_NEAR(rebuiltMergedMesh.mesh.SurfaceArea(), 12.0, 1e-12);
    const SectionBodyRebuild3d rebuiltMergedBody = RebuildSectionBody(nestedSection);
    assert(rebuiltMergedBody.success);
    assert(rebuiltMergedBody.body.FaceCount() == 1);
    const SectionBodySetRebuild3d rebuiltMergedBodies = RebuildSectionBodies(nestedSection);
    assert(rebuiltMergedBodies.success);
    assert(rebuiltMergedBodies.bodies.size() == 1);
    assert(rebuiltMergedBodies.rootPolygonIndices.size() == 1);
    assert(rebuiltMergedBodies.rootPolygonIndices[0] == 0);
    const SectionMeshConversion3d rebuiltMergedSectionMesh = ConvertSectionToTriangleMesh(nestedSection);
    assert(rebuiltMergedSectionMesh.success);
    GEOMETRY_TEST_ASSERT_NEAR(rebuiltMergedSectionMesh.mesh.SurfaceArea(), 12.0, 1e-12);
    const SectionMeshSetConversion3d rebuiltMergedSectionMeshes = ConvertSectionToTriangleMeshes(nestedSection);
    assert(rebuiltMergedSectionMeshes.success);
    assert(rebuiltMergedSectionMeshes.meshes.size() == 1);
    assert(rebuiltMergedSectionMeshes.rootPolygonIndices.size() == 1);
    assert(rebuiltMergedSectionMeshes.rootPolygonIndices[0] == 0);
    assert(ClassifySectionContent(nestedSection) == SectionContentKind3d::Area);

    PolyhedronSection3d disjointAreaSection{};
    disjointAreaSection.success = true;
    disjointAreaSection.origin = Point3d{0.0, 0.0, 0.5};
    disjointAreaSection.uAxis = Vector3d{1.0, 0.0, 0.0};
    disjointAreaSection.vAxis = Vector3d{0.0, 1.0, 0.0};
    disjointAreaSection.polygons.push_back(Polygon2d(
        Polyline2d(
            {
                Point2d{0.0, 0.0},
                Point2d{1.0, 0.0},
                Point2d{1.0, 1.0},
                Point2d{0.0, 1.0},
            },
            PolylineClosure::Closed)));
    disjointAreaSection.polygons.push_back(Polygon2d(
        Polyline2d(
            {
                Point2d{3.0, 0.0},
                Point2d{4.0, 0.0},
                Point2d{4.0, 1.0},
                Point2d{3.0, 1.0},
            },
            PolylineClosure::Closed)));
    const SectionTopology3d disjointAreaTopology = BuildSectionTopology(disjointAreaSection);
    assert(disjointAreaTopology.IsValid());
    assert(disjointAreaTopology.Roots().size() == 2);
    const SectionComponents3d disjointAreaComponents = BuildSectionComponents(disjointAreaSection);
    assert(disjointAreaComponents.IsValid());
    assert(disjointAreaComponents.components.size() == 2);
    assert(disjointAreaComponents.components[0].polygonIndices.size() == 1);
    assert(disjointAreaComponents.components[1].polygonIndices.size() == 1);
    const SectionBodySetRebuild3d rebuiltDisjointBodies = RebuildSectionBodies(disjointAreaSection);
    assert(rebuiltDisjointBodies.success);
    assert(rebuiltDisjointBodies.IsValid());
    assert(rebuiltDisjointBodies.bodies.size() == 2);
    assert(rebuiltDisjointBodies.bodies[0].FaceCount() == 1);
    assert(rebuiltDisjointBodies.bodies[1].FaceCount() == 1);
    assert(rebuiltDisjointBodies.rootPolygonIndices.size() == 2);
    assert(rebuiltDisjointBodies.rootPolygonIndices[0] == 0);
    assert(rebuiltDisjointBodies.rootPolygonIndices[1] == 1);
    const SectionMeshSetConversion3d rebuiltDisjointMeshes = ConvertSectionToTriangleMeshes(disjointAreaSection);
    assert(rebuiltDisjointMeshes.success);
    assert(rebuiltDisjointMeshes.IsValid());
    assert(rebuiltDisjointMeshes.meshes.size() == 2);
    assert(rebuiltDisjointMeshes.rootPolygonIndices.size() == 2);
    assert(rebuiltDisjointMeshes.rootPolygonIndices[0] == 0);
    assert(rebuiltDisjointMeshes.rootPolygonIndices[1] == 1);

    PolyhedronSection3d mixedSection = edgeOnlySection;
    mixedSection.polygons.push_back(middleSection.polygons[0]);
    assert(ClassifySectionContent(mixedSection) == SectionContentKind3d::Mixed);

    PolyhedronSection3d invalidSection = middleSection;
    invalidSection.uAxis = Vector3d{0.0, 0.0, 0.0};
    const SectionValidation3d invalidSectionValidation = Validate(invalidSection);
    assert(!invalidSectionValidation.valid);
    assert(invalidSectionValidation.issue == SectionValidationIssue3d::InvalidBasis);
}


