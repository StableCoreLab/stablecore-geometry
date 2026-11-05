#include "Geometry.h"

#include <array>
#include <memory>
#include <numbers>

#if defined(SCGEOMETRY_EXPECT_SHARED) != defined(GEOMETRY_USE_DLL)
#error "The installed SCGeometry target publishes the wrong DLL import definition."
#endif

int main()
{
    using namespace Geometry;
    const bool exportedDataAvailable = kPi > 3.0 && kPi < 4.0 && kTwoPi == 2.0 * kPi &&
        kArcBoundsCriticalAngles == std::array<double, 4>{0.0, 0.5 * kPi, kPi, 1.5 * kPi};
    const SCLineSegment2d first{{0.0, 0.0}, {10.0, 0.0}};
    const SCLineSegment2d second{{2.0, 0.0}, {8.0, 0.0}};
    const auto projection = ClassifyParallelSegmentProjection(
        first, second, SCParallelSegmentProjectionTolerance2d{});
    const SCBox2d box = SCBox2d::FromMinMax({0.0, 0.0}, {10.0, 10.0});
    const SCPolyline2d path({{2.0, 2.0}, {8.0, 8.0}}, SCPolylineClosure::Open);
    const SCPolygon2d polygon(SCPolyline2d(
        {{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}}, SCPolylineClosure::Closed));
    const SCMultiPolygon2d polygonSet({polygon});
    const auto polygonTopology = BuildPolygonTopology(polygonSet);
    const bool topologySizeAvailable = polygonTopology.IsValid() && polygonTopology.Size() == 1 &&
        SCSectionTopology3d{}.Size() == 0;
    const bool polygonToleranceAvailable = polygon.IsValid(1e-12) && polygon.Area(1e-12) == 100.0 &&
        polygon.Perimeter(1e-12) == 40.0 && polygon.Centroid(1e-12).IsValid() &&
        polygon.Bounds(1e-12).IsValid() && polygonSet.IsValid(1e-12) && polygonSet.Bounds(1e-12).IsValid();
    const auto searched = SearchPolygons(SCMultiPolyline2d({polygon.OuterRing()}),
                                         SCSearchPolyOptions2d{1e-12});
    const bool searchCandidateToleranceAvailable = searched.IsSuccess() &&
        !searched.candidates.empty() && searched.candidates.front().IsValid(1e-12);
    const SCOffsetOptions2d offsetOptions;
    const bool polygonOffsetToleranceAvailable = Offset(polygon, 1.0, offsetOptions, 1e-12).IsValid(1e-12) &&
        !OffsetToMultiPolygon(polygon, 1.0, offsetOptions, 1e-12).IsEmpty() &&
        Offset(polygonSet, 1.0, offsetOptions, 1e-12).IsValid(1e-12) &&
        Offset(path, 1.0, offsetOptions, 1e-12).IsValid() &&
        Offset(SCMultiPolyline2d{path}, 1.0, offsetOptions, 1e-12).IsValid();
    const auto area = QueryPolygonPositiveAreaIntersection(polygon, polygon);
    const SCPathGeometryTolerance pathTolerance{1e-8, 1e-8, 1e-12};
    const std::array<SCPoint3d, 3> planePoints{{{0,0,0}, {1,0,0}, {0,1,0}}};
    const auto fittedPlane = FitPlane(planePoints, pathTolerance);
    const auto arc = CreateArcSegment3d({2,0,0}, {0,2,0}, {0,0,0}, {0,0,1},
                                        2.0, std::numbers::pi_v<double> / 2.0, pathTolerance);
    const SCPlaneFrame3d xy{{0,0,0}, {1,0,0}, {0,1,0}, {0,0,1}};
    const auto projectedArc = arc.success ? ProjectArcToFrame(arc.arc, xy, pathTolerance)
                                          : SCArcProjectionResult{};
    const auto mappedOffset = OffsetWithMapping(path, 0.0, {4.0}, pathTolerance);
    const SCLineSegment2d cornerExit{{10.0, 0.0}, {10.0, 1.0}};
    const auto corner = EvaluateCornerGeometry(first, cornerExit, pathTolerance);
    const auto legacyOffset = Offset(first, 1.0);
    const SCArcSegment2d planarArc({0.0, 0.0}, 1.0, 0.0, std::numbers::pi_v<double> / 2.0);
    const SCArcSegment2d directionalArc({0.0, 0.0}, 1.0, 0.0, std::numbers::pi_v<double> / 2.0,
                                       SCArcDirection2d::CounterClockwise);
    const ISCSegment2d& directionalSegment = directionalArc;
    const bool dimensionedEnumsAvailable = directionalSegment.Kind() == SCSegmentKind2d::Arc &&
        directionalArc.Direction() == SCArcDirection2d::CounterClockwise;
    const SCArcSegment2d overlapFirst({0.0, 0.0}, 1.0, 0.0, 1.5 * std::numbers::pi_v<double>);
    const SCArcSegment2d overlapSecond({0.0, 0.0}, 1.0, std::numbers::pi_v<double>,
                                       1.5 * std::numbers::pi_v<double>);
    const auto allOverlaps = IntersectAll(overlapFirst, overlapSecond);
    const bool allOverlapsAvailable = allOverlaps.size() == 2 &&
        allOverlaps[0].kind == SCIntersectionKind2d::Overlap &&
        allOverlaps[1].kind == SCIntersectionKind2d::Overlap;
    const bool segmentParameterAvailable =
        ParameterAtLength(static_cast<const ISCSegment2d&>(first), 5.0, true) == 0.5 &&
        ParameterAtLength(static_cast<const ISCSegment2d&>(planarArc), planarArc.Length() * 0.5) == 0.5 &&
        ParameterAtLength(static_cast<const ISCSegment2d&>(first), 5.0, true, 1e-12) == 0.5;
    const SCLineSegment2d tinyLine({0.0, 0.0}, {1e-10, 0.0});
    const bool shortProjectionAvailable =
        ProjectPointToLineSegment({5e-11, 1e-11}, tinyLine, true, 1e-15).point.AlmostEquals({5e-11, 0.0}, 1e-15) &&
        ProjectPointToSegment({5e-11, 1e-11}, tinyLine.startPoint, tinyLine.endPoint, true, 1e-15)
            .point.AlmostEquals({5e-11, 0.0}, 1e-15) &&
        ProjectPointToSegment({5e-11, 1e-11}, static_cast<const ISCSegment2d&>(tinyLine), true, 1e-15)
            .point.AlmostEquals({5e-11, 0.0}, 1e-15);
    const SCArcSegment2d tinyArc({0.0, 0.0}, 1e-10, 0.0, std::numbers::pi_v<double> * 0.5);
    const SCArcSegment2d shortSweepArc({0.0, 0.0}, 1e6, 0.0, 1e-10);
    const bool shortSweepArcToleranceAvailable = shortSweepArc.IsValid(1e-9) &&
        shortSweepArc.Length(1e-9) > 0.0 && shortSweepArc.Bounds(1e-9).IsValid();
    const bool arcProjectionAvailable =
        ProjectPointToArcSegment({1e-10, -1e-16}, tinyArc, true, 1e-15)
            .point.AlmostEquals(tinyArc.StartPoint(), 1e-15) &&
        ProjectPointToSegment({1e-10, -1e-16}, static_cast<const ISCSegment2d&>(tinyArc), true, 1e-15)
            .point.AlmostEquals(tinyArc.StartPoint(), 1e-15);
    const bool segmentSamplingAvailable =
        tinyLine.PointAt(0.5, 1e-15).AlmostEquals({5e-11, 0.0}, 1e-15) &&
        tinyLine.PointAtLength(5e-11, false, 1e-15).AlmostEquals({5e-11, 0.0}, 1e-15) &&
        tinyLine.Bounds(1e-15).IsValid() &&
        tinyArc.PointAt(0.5, 1e-15).IsValid() &&
        tinyArc.PointAtLength(tinyArc.Length() * 0.5, false, 1e-15).IsValid();
    const bool segmentDirectionsAvailable =
        TangentAt(tinyLine, 0.5, 1e-15) == SCVector2d{1.0, 0.0} &&
        NormalAt(tinyLine, 0.5, 1e-15) == SCVector2d{0.0, 1.0} &&
        TangentAt(tinyArc, 0.0, 1e-15).AlmostEquals({0.0, 1.0}, 1e-15) &&
        NormalAt(tinyArc, 0.0, 1e-15).AlmostEquals({-1.0, 0.0}, 1e-15) &&
        TangentAt(static_cast<const ISCSegment2d&>(tinyLine), 0.5, 1e-15) == SCVector2d{1.0, 0.0} &&
        NormalAt(static_cast<const ISCSegment2d&>(tinyArc), 0.0, 1e-15)
            .AlmostEquals({-1.0, 0.0}, 1e-15);
    const SCLineSegment3d tinySegment3d{{0.0, 0.0, 0.0}, {1e-200, 0.0, 0.0}};
    const bool tinySegmentSamplingAvailable =
        tinySegment3d.PointAtLength(5e-201, false, 1e-210) == SCPoint3d{5e-201, 0.0, 0.0};
    const bool primitiveOffsetToleranceAvailable = Offset(first, 1.0, 1e-12).IsValid(1e-12) &&
        Offset(planarArc, 0.1, 1e-12).IsValid();
    const SCPolyline2d tinyPath({{0.0, 0.0}, {1e-10, 0.0}, {1e-10, 1e-10}});
    const SCMultiPolyline2d tinyPaths({tinyPath});
    const bool subPolylineToleranceAvailable =
        SubPolyline(tinyPath, 5e-11, 15e-11, 1e-15).IsValid(1e-15);
    const bool polylineSamplingToleranceAvailable =
        tinyPath.PointAtLength(5e-11, false, 1e-15).AlmostEquals({5e-11, 0.0}, 1e-15) &&
        tinyPath.PointAt(0.25, 1e-15).AlmostEquals({5e-11, 0.0}, 1e-15) &&
        !tinyPath.HasNonLineSegments() &&
        tinyPath.Length(1e-15) > 0.0 && tinyPath.LengthAt(0.5, 1e-15) > 0.0 &&
        tinyPath.ParameterAtLength(5e-11, false, 1e-15) > 0.0 &&
        tinyPaths.IsValid(1e-15) &&
        tinyPaths.Bounds(1e-15).IsValid();
    const auto normalized = Normalize(path);
    const SCNurbsCurve3d nurbsCurve(1, {{0, 0, 0}, {2, 0, 0}}, {0, 0, 1, 1});
    const SCNurbsSurface nurbsSurface(1, 1, 2, 2,
                                      {{0, 0, 0}, {2, 0, 0}, {0, 2, 0}, {2, 2, 0}},
                                      {0, 0, 1, 1}, {0, 0, 1, 1});
    const bool nurbsAvailable = nurbsCurve.IsValid() && nurbsSurface.IsValid() &&
        nurbsCurve.PointAt(0.5).AlmostEquals({1, 0, 0}) &&
        nurbsSurface.PointAt(0.5, 0.5).AlmostEquals({1, 1, 0});
    const auto curveSurface = std::make_shared<SCPlaneSurface>(SCPlaneSurface::FromPlane(
        SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1}), {-2, 2}, {-2, 2}));
    const SCCurveOnSurface composedCurve(curveSurface, SCPolyline2d({{0, 0}, {1, 0}}));
    const SCPolylineCurve2d uvAdapter(SCPolyline2d({{0,0},{1,0}}));
    const ISCCurve2d& uvBase = uvAdapter;
    if (!uvBase.IsValid() || !uvBase.Clone()->PointAt(0.5).AlmostEquals({0.5,0}) ||
        !composedCurve.UvGeometry().PointAt(0.5).AlmostEquals({0.5,0})) return 31;
    if (composedCurve.Evaluate(0.5, 1, SCGeometryTolerance3d{}).derivativeOrder != 1) return 32;
    const SCCurveOnSurface clonedUvCurve(curveSurface, uvBase);
    if (!clonedUvCurve.IsValid() || clonedUvCurve.UvPieceCount() != 1 ||
        clonedUvCurve.UvParameterAt(0, 0.5) != 0.5 ||
        !clonedUvCurve.Clone()->PointAt(0.5).AlmostEquals({0.5,0,0})) return 32;
    const ISCCurve3d& genericCurve = composedCurve;
    const bool genericCurveAvailable = genericCurve.IsValid() &&
        genericCurve.PointAt(0.5).AlmostEquals({0.5, 0, 0}) &&
        Length(composedCurve, SCGeometryTolerance3d{}) > 0.0;
    constexpr double tinySide = 1e-10;
    const TriangleMesh tinyTetrahedron(
        {{0, 0, 0}, {tinySide, 0, 0}, {0, tinySide, 0}, {0, 0, tinySide}},
        {{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {2, 0, 3}});
    const bool tinyVolumeAvailable = IsClosedTriangleMesh(tinyTetrahedron, 1e-15) &&
        IsManifoldTriangleMesh(tinyTetrahedron, 1e-15) &&
        IsConsistentlyOrientedTriangleMesh(tinyTetrahedron, 1e-15) &&
        ComputeTriangleConnectedComponents(tinyTetrahedron, 1e-15).size() == 1 &&
        ComputeMeshShells(tinyTetrahedron, 1e-15).size() == 1 &&
        ComputeTriangleAdjacency(tinyTetrahedron, 1e-15).size() == 4 &&
        TriangleAdjacency(tinyTetrahedron, 0, 1e-15).HasNeighbor(0) &&
        ExtractNonManifoldEdges(tinyTetrahedron, 1e-15).empty() &&
        ExtractBoundaryEdges(tinyTetrahedron, 1e-15).empty() && Volume(tinyTetrahedron, 1e-15) > 0.0;
    const TriangleMesh tinyOpenTriangle(
        {{0, 0, 0}, {tinySide, 0, 0}, {0, tinySide, 0}}, {{0, 1, 2}});
    const bool tinyBoundaryAvailable = ExtractBoundaryLoops(tinyOpenTriangle, 1e-15).size() == 1;
    const std::array<const ISCCurve3d*, 1> snapCurves{&genericCurve};
    const auto snappedCurve = SnapPointToCurves3d({0.5, 0.1, 0.0}, snapCurves, 0.2);
    const bool curveSnapAvailable = snappedCurve.snapped && snappedCurve.segmentIndex == 0 &&
        snappedCurve.point.AlmostEquals({0.5, 0.0, 0.0});
    const SCBrepEdge sampleEdge(std::shared_ptr<ISCCurve3d>(genericCurve.Clone()), 0, 1);
    const SCBrepFace sampleFace(curveSurface, SCBrepLoop({SCBrepCoedge(0)}));
    const SCBrepShell sampleShell({sampleFace});
    const bool topologyReferencesAvailable = sampleEdge.IsValid(2) && !sampleEdge.IsValid(1) &&
        sampleFace.IsValidWithEdgeCount(1) && !sampleFace.IsValidWithEdgeCount(0) &&
        sampleShell.IsValid(1) && !sampleShell.IsValid(0);
    const SCOffsetSurface offsetSurface(curveSurface, 0.2);
    const auto offsetEval = offsetSurface.Evaluate(0.0, 0.0, 1);
    const bool offsetAvailable = offsetEval.IsValid() &&
        offsetEval.point.AlmostEquals({0.0, 0.0, 0.2});
    return exportedDataAvailable && projection == SCParallelSegmentProjectionRelation2d::PositiveLengthIntersection &&
                   Contains(box, path) && Intersects(box, path) && polygonToleranceAvailable &&
                   searchCandidateToleranceAvailable && polygonOffsetToleranceAvailable &&
                   primitiveOffsetToleranceAvailable && segmentParameterAvailable && shortProjectionAvailable &&
                   arcProjectionAvailable && shortSweepArcToleranceAvailable &&
                   segmentSamplingAvailable && segmentDirectionsAvailable && dimensionedEnumsAvailable && allOverlapsAvailable &&
                   tinySegmentSamplingAvailable &&
                   subPolylineToleranceAvailable && polylineSamplingToleranceAvailable && area.success &&
                   area.hasPositiveAreaIntersection && fittedPlane.success && arc.success &&
                   projectedArc.success && mappedOffset.success && corner.success &&
                   legacyOffset.IsValid() && planarArc.AlmostEquals(planarArc, kDefaultEpsilon, kDefaultAngleEpsilon) &&
                   normalized.IsValid() && nurbsAvailable && genericCurveAvailable && curveSnapAvailable &&
                   topologyReferencesAvailable && topologySizeAvailable && tinyVolumeAvailable && tinyBoundaryAvailable &&
                   offsetAvailable
               ? 0
               : 1;
}
