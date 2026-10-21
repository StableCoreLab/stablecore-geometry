#include <gtest/gtest.h>

#include "Geometry.h"
#include "Support/Fixtures3d.h"

using namespace Geometry;

TEST(GeometryTest, CoversPolyhedronAndBrepCapabilities)
{
    const SCPlane supportPlane = Geometry::Test::BuildSupportPlaneAtZ5();
    const SCPlaneSurface planeSurface = Geometry::Test::BuildFinitePlaneSurfaceAtZ5();
    const SCNurbsSurface nurbsSurface = Geometry::Test::BuildUnitNurbsSurface();
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
    EXPECT_NEAR(Area(face), 4.0, 1e-12);

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
        EXPECT_NEAR(supportPlane.SignedDistanceTo(vertex), 0.0, 1e-12);
    }
    EXPECT_NEAR(surfaceMesh.SurfaceArea(), 16.0, 1e-12);

    const auto faceMesh = ConvertToTriangleMesh(face);
    ASSERT_TRUE(faceMesh.success);
    ASSERT_EQ(faceMesh.issue, MeshConversionIssue3d::None);
    ASSERT_TRUE(faceMesh.mesh.IsValid());
    ASSERT_EQ(faceMesh.mesh.TriangleCount(), 2);
    EXPECT_NEAR(faceMesh.mesh.SurfaceArea(), 4.0, 1e-12);
    ASSERT_EQ(LocatePoint(SCPoint3d{1.0, 1.0, 0.0}, face), SCPointContainment3d::Inside);
    ASSERT_EQ(LocatePoint(SCPoint3d{2.0, 1.0, 0.0}, face), SCPointContainment3d::OnBoundary);
    ASSERT_EQ(LocatePoint(SCPoint3d{3.0, 1.0, 0.0}, face), SCPointContainment3d::Outside);
    const auto polyFaceProjection = ProjectPointToPolyhedronFace(SCPoint3d{1.0, 1.0, 3.0}, face);
    ASSERT_TRUE(polyFaceProjection.success);
    ASSERT_TRUE(polyFaceProjection.IsValid());
    ASSERT_TRUE(polyFaceProjection.onFace);
    ASSERT_TRUE(polyFaceProjection.point.AlmostEquals(SCPoint3d{1.0, 1.0, 0.0}, 1e-12));
    EXPECT_NEAR(Distance(SCPoint3d{1.0, 1.0, 3.0}, face), 3.0, 1e-12);
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
    EXPECT_NEAR(Distance(SCPoint3d{3.0, 1.0, 1.0}, body), std::sqrt(2.0), 1e-12);
    const auto polyBodyLineIntersection =
        Intersect(SCLine3d::FromOriginAndDirection(SCPoint3d{1.0, 1.0, 3.0}, SCVector3d{0.0, 0.0, -1.0}), body);
    ASSERT_TRUE(polyBodyLineIntersection.intersects);
    ASSERT_TRUE(polyBodyLineIntersection.IsValid());
    ASSERT_EQ(polyBodyLineIntersection.faceIndices.size(), 1);
    ASSERT_EQ(polyBodyLineIntersection.hits.size(), 1);
    ASSERT_EQ(polyBodyLineIntersection.faceIndices[0], 0);
    ASSERT_TRUE(polyBodyLineIntersection.hits[0].point.AlmostEquals(SCPoint3d{1.0, 1.0, 0.0}, 1e-12));
    ASSERT_EQ(LocatePoint(SCPoint3d{1.0, 1.0, 0.0}, body), SCPointContainment3d::OnBoundary);

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
    EXPECT_NEAR(projectedFace.origin.z, 0.0, 1e-12);
    EXPECT_NEAR(Geometry::Dot(projectedFace.uAxis, projectedFace.vAxis), 0.0, 1e-12);

    const auto holedFaceMesh = ConvertToTriangleMesh(holedFace);
    ASSERT_TRUE(holedFaceMesh.success);
    ASSERT_EQ(holedFaceMesh.issue, MeshConversionIssue3d::None);
    ASSERT_TRUE(holedFaceMesh.mesh.IsValid());
    EXPECT_NEAR(holedFaceMesh.mesh.SurfaceArea(), projectedFace.polygon.Area(), 1e-12);

    const std::vector<SCBrepVertex> brepVertices = Geometry::Test::BuildPlanarBrepVerticesAtZ5();
    const auto brepVertexProjection = ProjectPointToBrepVertex(SCPoint3d{1.0, 1.0, 6.0}, brepVertices[0]);
    ASSERT_TRUE(brepVertexProjection.success);
    ASSERT_TRUE(brepVertexProjection.IsValid());
    ASSERT_TRUE(brepVertexProjection.point.AlmostEquals(SCPoint3d{0.0, 0.0, 5.0}, 1e-12));
    EXPECT_NEAR(Distance(SCPoint3d{1.0, 1.0, 6.0}, brepVertices[0]), std::sqrt(3.0), 1e-12);
    const SCBox3d brepVertexBounds = Bounds(brepVertices[0]);
    ASSERT_TRUE(brepVertexBounds.IsValid());
    ASSERT_TRUE(brepVertexBounds.MinPoint().AlmostEquals(SCPoint3d{0.0, 0.0, 5.0}, 1e-12));
    ASSERT_TRUE(brepVertexBounds.MaxPoint().AlmostEquals(SCPoint3d{0.0, 0.0, 5.0}, 1e-12));
    const auto brepVertexLineIntersection =
        Intersect(SCLine3d::FromOriginAndDirection(SCPoint3d{0.0, 0.0, 6.0}, SCVector3d{0.0, 0.0, -1.0}), brepVertices[0]);
    ASSERT_TRUE(brepVertexLineIntersection.intersects);
    ASSERT_TRUE(brepVertexLineIntersection.IsValid());
    EXPECT_NEAR(brepVertexLineIntersection.lineParameter, 1.0, 1e-12);
    ASSERT_TRUE(brepVertexLineIntersection.point.AlmostEquals(SCPoint3d{0.0, 0.0, 5.0}, 1e-12));
    const auto brepVertexPlaneIntersection =
        Intersect(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 5.0}, SCVector3d{0.0, 0.0, 1.0}), brepVertices[0]);
    ASSERT_TRUE(brepVertexPlaneIntersection.intersects);
    ASSERT_TRUE(brepVertexPlaneIntersection.IsValid());
    ASSERT_TRUE(brepVertexPlaneIntersection.point.AlmostEquals(SCPoint3d{0.0, 0.0, 5.0}, 1e-12));
    const std::vector<SCBrepEdge> brepEdges = Geometry::Test::BuildPlanarBrepEdgesAtZ5();
    const SCBrepLoop brepOuterLoop = Geometry::Test::BuildPlanarBrepOuterLoop();
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
    EXPECT_NEAR(brepFaceBounds.MinPoint().z, 5.0, 1e-12);
    EXPECT_NEAR(brepFaceBounds.MaxPoint().z, 5.0, 1e-12);
    EXPECT_NEAR(Bounds(brepFace).MinPoint().z, 5.0, 1e-12);
    const SCBrepShell brepShell({brepFace}, false);
    ASSERT_TRUE(brepShell.IsValid());
    const SCBox3d brepShellBounds = brepShell.Bounds();
    ASSERT_TRUE(brepShellBounds.IsValid());
    EXPECT_NEAR(brepShellBounds.MinPoint().z, 5.0, 1e-12);
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
    EXPECT_NEAR(brepEdgeProjection.parameter, 0.5, 1e-12);
    ASSERT_TRUE(brepEdgeProjection.point.AlmostEquals(SCPoint3d{1.0, 0.0, 5.0}, 1e-12));
    EXPECT_NEAR(Distance(SCPoint3d{1.0, 1.0, 5.0}, brepBody.EdgeAt(0)), 1.0, 1e-12);
    const auto brepEdgeLineIntersection =
        Intersect(SCLine3d::FromOriginAndDirection(SCPoint3d{1.0, -1.0, 5.0}, SCVector3d{0.0, 1.0, 0.0}), brepBody.EdgeAt(0));
    ASSERT_TRUE(brepEdgeLineIntersection.intersects);
    ASSERT_TRUE(brepEdgeLineIntersection.IsValid());
    EXPECT_NEAR(brepEdgeLineIntersection.edgeParameter, 0.5, 1e-12);
    ASSERT_TRUE(brepEdgeLineIntersection.point.AlmostEquals(SCPoint3d{1.0, 0.0, 5.0}, 1e-12));
    const auto brepEdgePlaneIntersection =
        Intersect(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 5.0}, SCVector3d{0.0, 0.0, 1.0}), brepBody.EdgeAt(0));
    ASSERT_TRUE(brepEdgePlaneIntersection.intersects);
    ASSERT_TRUE(brepEdgePlaneIntersection.IsValid());
    EXPECT_NEAR(brepEdgePlaneIntersection.edgeParameter, 0.0, 1e-12);
    ASSERT_TRUE(brepEdgePlaneIntersection.point.AlmostEquals(SCPoint3d{0.0, 0.0, 5.0}, 1e-12));
    const auto brepValidation = Validate(brepBody);
    ASSERT_TRUE(brepValidation.valid);
    ASSERT_EQ(brepValidation.issue, SCBrepValidationIssue3d::None);
    EXPECT_NEAR(Area(brepFace), 4.0, 1e-12);
    EXPECT_NEAR(Bounds(brepBody).MinPoint().z, 5.0, 1e-12);
    const auto brepFaceLineIntersection =
        Intersect(SCLine3d::FromOriginAndDirection(SCPoint3d{1.0, 1.0, 8.0}, SCVector3d{0.0, 0.0, -1.0}), brepFace);
    ASSERT_TRUE(brepFaceLineIntersection.intersects);
    ASSERT_TRUE(brepFaceLineIntersection.IsValid());
    ASSERT_FALSE(brepFaceLineIntersection.onBoundary);
    EXPECT_NEAR(brepFaceLineIntersection.lineParameter, 3.0, 1e-12);
    ASSERT_TRUE(brepFaceLineIntersection.point.AlmostEquals(SCPoint3d{1.0, 1.0, 5.0}, 1e-12));
    ASSERT_EQ(LocatePoint(SCPoint3d{1.0, 1.0, 5.0}, brepFace), SCPointContainment3d::Inside);
    ASSERT_EQ(LocatePoint(SCPoint3d{2.0, 1.0, 5.0}, brepFace), SCPointContainment3d::OnBoundary);
    ASSERT_EQ(LocatePoint(SCPoint3d{3.0, 1.0, 5.0}, brepFace), SCPointContainment3d::Outside);
    const auto brepBodyLineIntersection =
        Intersect(SCLine3d::FromOriginAndDirection(SCPoint3d{1.0, 1.0, 8.0}, SCVector3d{0.0, 0.0, -1.0}), brepBody);
    ASSERT_TRUE(brepBodyLineIntersection.intersects);
    ASSERT_TRUE(brepBodyLineIntersection.IsValid());
    ASSERT_EQ(brepBodyLineIntersection.faceIndices.size(), 1);
    ASSERT_EQ(brepBodyLineIntersection.hits.size(), 1);
    ASSERT_EQ(brepBodyLineIntersection.faceIndices[0], 0);
    ASSERT_TRUE(brepBodyLineIntersection.hits[0].point.AlmostEquals(SCPoint3d{1.0, 1.0, 5.0}, 1e-12));
    ASSERT_EQ(LocatePoint(SCPoint3d{1.0, 1.0, 5.0}, brepBody), SCPointContainment3d::OnBoundary);
    ASSERT_EQ(LocatePoint(SCPoint3d{1.0, 1.0, 8.0}, brepBody), SCPointContainment3d::Outside);
    const auto brepFaceProjectionInside = ProjectPointToBrepFace(SCPoint3d{1.0, 1.0, 8.0}, brepFace);
    ASSERT_TRUE(brepFaceProjectionInside.success);
    ASSERT_TRUE(brepFaceProjectionInside.IsValid());
    ASSERT_TRUE(brepFaceProjectionInside.onTrimmedFace);
    ASSERT_FALSE(brepFaceProjectionInside.onBoundary);
    ASSERT_TRUE(brepFaceProjectionInside.point.AlmostEquals(SCPoint3d{1.0, 1.0, 5.0}, 1e-12));
    EXPECT_NEAR(Distance(SCPoint3d{1.0, 1.0, 8.0}, brepFace), 3.0, 1e-12);
    const auto brepFaceProjectionOutside = ProjectPointToBrepFace(SCPoint3d{3.0, 1.0, 5.0}, brepFace);
    ASSERT_TRUE(brepFaceProjectionOutside.success);
    ASSERT_TRUE(brepFaceProjectionOutside.IsValid());
    ASSERT_TRUE(brepFaceProjectionOutside.onTrimmedFace);
    ASSERT_TRUE(brepFaceProjectionOutside.onBoundary);
    ASSERT_TRUE(brepFaceProjectionOutside.point.AlmostEquals(SCPoint3d{2.0, 1.0, 5.0}, 1e-12));
    EXPECT_NEAR(Distance(SCPoint3d{3.0, 1.0, 5.0}, brepFace), 1.0, 1e-12);
    const auto brepBodyProjection = ProjectPointToBrepBody(SCPoint3d{3.0, 1.0, 5.0}, brepBody);
    ASSERT_TRUE(brepBodyProjection.success);
    ASSERT_TRUE(brepBodyProjection.IsValid());
    ASSERT_EQ(brepBodyProjection.faceIndex, 0);
    ASSERT_TRUE(brepBodyProjection.projection.point.AlmostEquals(SCPoint3d{2.0, 1.0, 5.0}, 1e-12));
    EXPECT_NEAR(Distance(SCPoint3d{3.0, 1.0, 5.0}, brepBody), 1.0, 1e-12);
    const auto brepFaceMesh = ConvertToTriangleMesh(brepFace);
    ASSERT_TRUE(brepFaceMesh.success);
    ASSERT_TRUE(brepFaceMesh.mesh.IsValid());
    ASSERT_EQ(brepFaceMesh.mesh.TriangleCount(), 2);
    EXPECT_NEAR(brepFaceMesh.mesh.SurfaceArea(), 4.0, 1e-12);
    const auto polyFaceFromBrep = ConvertToPolyhedronFace(brepFace);
    ASSERT_TRUE(polyFaceFromBrep.success);
    ASSERT_EQ(polyFaceFromBrep.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(polyFaceFromBrep.face.IsValid());
    EXPECT_NEAR(Area(polyFaceFromBrep.face), 4.0, 1e-12);
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
    EXPECT_NEAR(Volume(brepBody), 0.0, 1e-12);
    const auto brepMesh = ConvertToTriangleMesh(brepBody);
    ASSERT_TRUE(brepMesh.success);
    ASSERT_TRUE(brepMesh.mesh.IsValid());
    ASSERT_EQ(brepMesh.mesh.TriangleCount(), 2);
    EXPECT_NEAR(brepMesh.mesh.SurfaceArea(), 4.0, 1e-12);
    const SCPolyhedronSection3d brepSection =
        Section(brepBody, SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 5.0}, SCVector3d{0.0, 0.0, 1.0}));
    ASSERT_TRUE(brepSection.success);
    ASSERT_EQ(brepSection.polygons.size(), 1);
    EXPECT_NEAR(brepSection.polygons[0].Area(), 4.0, 1e-12);

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
    EXPECT_NEAR(Area(brepFaceWithoutTrim), 0.0, 1e-12);
    const BrepHealing3d healedTrimmedBody = Heal(brepBodyWithoutTrim);
    ASSERT_TRUE(healedTrimmedBody.success);
    ASSERT_TRUE(healedTrimmedBody.body.IsValid());
    ASSERT_EQ(healedTrimmedBody.body.ShellCount(), 1);
    ASSERT_TRUE(healedTrimmedBody.body.ShellAt(0).FaceAt(0).OuterTrim().IsValid());
    EXPECT_NEAR(Area(healedTrimmedBody.body.ShellAt(0).FaceAt(0)), 4.0, 1e-12);
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
    EXPECT_NEAR(nurbsBrepFaceArea, 4.0, 1e-12);
    EXPECT_NEAR(nurbsBrepFaceArea, nurbsBrepFaceMesh.mesh.SurfaceArea(), 1e-12);

}
